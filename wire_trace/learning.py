"""Stage 2 - learning-based trajectory adaptation.

1. DMP (Dynamic Movement Primitives, Ijspeert et al. 2013) learned from one or more
   demonstrations. The learned primitive can be re-generated for a *new start / goal*
   (small variations of the wire), at any duration, without re-teaching.

2. ILC (Iterative Learning Control, P-type with Q-filter). After every autonomous trial the
   measured tool path is compared with the desired path; the error is fed back into a
   learned position offset that is added to the reference on the next trial:
       c_{k+1}(t) = Q( c_k(t) + L * e_k(t + lead) )
   After a few trials the robot follows the wire more accurately than the plain feedback
   controller, which lets you raise the speed and still have zero contact "in one go".

3. Rigid registration (Kabsch): if the wire stand is moved/rotated, touch 3+ reference
   points on the stand before and after; the whole trajectory is transformed.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .so3 import gaussian_filter1d_edge, quat_interp, quat_mul, rotmat_to_quat
from .trajectory import Demo, ProcessParams, Trajectory, process_demo


# ---------------------------------------------------------------------------
# DMP
# ---------------------------------------------------------------------------
@dataclass
class DMP:
    n_basis: int = 200
    alpha_z: float = 48.0
    beta_z: float = 12.0
    alpha_x: float = 4.0
    ridge: float = 1e-8
    w: np.ndarray | None = None        # (D, n_basis)
    y0: np.ndarray | None = None
    g: np.ndarray | None = None
    tau: float = 1.0

    def _basis(self, x: np.ndarray) -> np.ndarray:
        c = np.exp(-self.alpha_x * np.linspace(0, 1, self.n_basis))
        d = np.abs(np.diff(c))
        h = 1.0 / np.maximum(np.append(d, d[-1]) ** 2, 1e-12)
        psi = np.exp(-h[None, :] * (x[:, None] - c[None, :]) ** 2)
        return psi

    def fit(self, demos: list[tuple[np.ndarray, np.ndarray]]):
        """demos: list of (t, y) with y shaped (N, D). Each demo is time-normalised to [0, 1]
        (tau = its duration); all demos share the same weights (least squares over all)."""
        Phi_all, F_all = [], []
        y0s, gs, taus = [], [], []
        for t, y in demos:
            t = np.asarray(t, float) - t[0]
            tau = t[-1]
            yd = np.gradient(y, t, axis=0)
            ydd = np.gradient(yd, t, axis=0)
            x = np.exp(-self.alpha_x * t / tau)
            y0, g = y[0], y[-1]
            f = (tau ** 2) * ydd - self.alpha_z * (self.beta_z * (g - y) - tau * yd)
            psi = self._basis(x)
            scale = x[:, None] * psi / psi.sum(axis=1, keepdims=True)
            # f = (g - y0) * scale @ w  -> divide out (g - y0) per dim (guard tiny)
            Phi_all.append(scale)
            F_all.append(f / np.where(np.abs(g - y0) < 1e-3, 1.0, (g - y0))[None, :])
            y0s.append(y0); gs.append(g); taus.append(tau)
        Phi = np.concatenate(Phi_all)
        F = np.concatenate(F_all)
        A = Phi.T @ Phi + self.ridge * np.eye(self.n_basis)
        self.w = np.linalg.solve(A, Phi.T @ F).T          # (D, n_basis)
        self.y0 = np.mean(y0s, axis=0)
        self.g = np.mean(gs, axis=0)
        self.tau = float(np.mean(taus))
        return self

    def rollout(self, y0=None, g=None, tau=None, dt=0.002):
        y0 = self.y0 if y0 is None else np.asarray(y0, float)
        g = self.g if g is None else np.asarray(g, float)
        tau = self.tau if tau is None else float(tau)
        amp = np.where(np.abs(self.g - self.y0) < 1e-3, 1.0, (g - y0))
        n = int(np.ceil(tau / dt)) + 1
        y, z, x = y0.copy(), np.zeros_like(y0), 1.0
        Y, T = [y.copy()], [0.0]
        for i in range(1, n):
            psi = self._basis(np.array([x]))[0]
            f = (self.w @ psi) / psi.sum() * x * amp
            zd = (self.alpha_z * (self.beta_z * (g - y) - z) + f) / tau
            yd = z / tau
            z = z + zd * dt
            y = y + yd * dt
            x = x + (-self.alpha_x * x / tau) * dt
            Y.append(y.copy()); T.append(i * dt)
        return np.array(T), np.array(Y)


def learn_dmp_trajectory(trajs: list[Trajectory], start=None, goal=None, n_basis=200,
                         params: ProcessParams | None = None, verbose=True) -> Trajectory:
    """Fit a DMP to processed demo trajectories, roll it out for (start, goal) and re-time it
    with the normal time-parameterisation pipeline."""
    demos = [(tr.t, tr.pos) for tr in trajs]
    dmp = DMP(n_basis=n_basis).fit(demos)
    t, y = dmp.rollout(start, goal)
    phase = t / t[-1]
    ref = trajs[0]
    ref_phase = (ref.t - ref.t[0]) / ref.duration
    quat = quat_interp(ref_phase, ref.quat, phase)
    q = np.stack([np.interp(phase, ref_phase, ref.q_ref[:, k]) for k in range(ref.q_ref.shape[1])], axis=1)
    demo = Demo(t=t, q=q, pos=y, quat=quat, joint_names=ref.joint_names,
                meta={"dmp": True, "n_demos": len(trajs), "n_basis": n_basis})
    p = params or ProcessParams(**{k: v for k, v in ref.meta.get("params", {}).items()
                                   if k in ProcessParams.__dataclass_fields__})
    p = replace(p, smooth_sigma=0.0, ori_smooth_sigma=0.0, trim_start_end=0.0,
                offset_xyz=(0.0, 0.0, 0.0), raw_interp="linear")
    out = process_demo(demo, p, verbose=verbose)
    out.meta["stage2"] = "dmp"
    return out


# ---------------------------------------------------------------------------
# ILC
# ---------------------------------------------------------------------------
@dataclass
class ILCParams:
    gain: float = 0.7            # L  (0 < L <= 1)
    lead: int = 5                # samples of phase lead (compensates robot lag)
    q_sigma: float = 4.0         # Q-filter Gaussian sigma in samples (robustness)
    max_offset: float = 0.008    # m clamp on the learned correction


def tracking_error_on_traj_time(traj: Trajectory, log_tau: np.ndarray, log_pos: np.ndarray) -> np.ndarray:
    """Desired-minus-actual tool position, evaluated on the trajectory's own time grid."""
    tau = np.asarray(log_tau, float)
    pos = np.asarray(log_pos, float)
    # keep the last sample for each tau value (tau stalls while the reference waits)
    _, idx = np.unique(tau[::-1], return_index=True)
    idx = len(tau) - 1 - idx
    idx.sort()
    tau_u, pos_u = tau[idx], pos[idx]
    act = np.stack([np.interp(traj.t, tau_u, pos_u[:, k]) for k in range(3)], axis=1)
    e = traj.pos - act
    e[traj.t > tau_u.max()] = 0.0          # part of the path this trial never reached: no info
    return e


def ilc_update(traj: Trajectory, log_tau, log_pos, p: ILCParams = ILCParams()) -> tuple[Trajectory, dict]:
    e = tracking_error_on_traj_time(traj, log_tau, log_pos)
    covered = traj.t <= float(np.max(log_tau))
    if p.lead > 0:
        e = np.concatenate([e[p.lead:], np.repeat(e[-1:], p.lead, axis=0)])
    c = traj.pos_offset + p.gain * e
    c = gaussian_filter1d_edge(c, p.q_sigma)
    n = np.linalg.norm(c, axis=1, keepdims=True)
    c = np.where(n > p.max_offset, c * p.max_offset / np.maximum(n, 1e-12), c)
    c[0] = 0.0      # never move the start point
    new = Trajectory(t=traj.t, pos=traj.pos, quat=traj.quat, vel=traj.vel, omega=traj.omega,
                     q_ref=traj.q_ref, s=traj.s, joint_names=traj.joint_names, pos_offset=c,
                     meta={**traj.meta, "ilc_iter": int(traj.meta.get("ilc_iter", 0)) + 1})
    en = np.linalg.norm(e[covered], axis=1)
    stats = {"coverage": float(covered.mean()), "iter": new.meta["ilc_iter"], "rms_err_mm": float(np.sqrt(np.mean(en ** 2)) * 1000),
             "max_err_mm": float(en.max() * 1000), "max_offset_mm": float(np.linalg.norm(c, axis=1).max() * 1000)}
    return new, stats


# ---------------------------------------------------------------------------
# Rigid registration (stand moved)
# ---------------------------------------------------------------------------
def kabsch(A: np.ndarray, B: np.ndarray):
    """Least-squares R, t with B ~= R A + t (A, B: (N, 3), N >= 3, non-collinear)."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def transform_trajectory(traj: Trajectory, R: np.ndarray, t: np.ndarray) -> Trajectory:
    qR = rotmat_to_quat(R)
    quat = quat_mul(np.broadcast_to(qR, traj.quat.shape), traj.quat)
    return Trajectory(t=traj.t, pos=traj.pos @ R.T + t, quat=quat, vel=traj.vel @ R.T,
                      omega=traj.omega @ R.T, q_ref=traj.q_ref, s=traj.s,
                      joint_names=traj.joint_names, pos_offset=traj.pos_offset @ R.T,
                      meta={**traj.meta, "registered": True})
