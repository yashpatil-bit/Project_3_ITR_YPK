"""Demo recording format, trajectory processing (smoothing, interpolation, time
parameterisation) and the Trajectory container used by the replay node.

Pure numpy/scipy: runs on any laptop, no ROS needed.

Pipeline (process_demo):
    raw demo (t, q, pos, quat) recorded at 20-100 Hz while hand-guiding the arm
      1. drop near-duplicate samples (operator pauses)
      2. resample by arc length (1 mm) and Gaussian-smooth the 3D path
      3. fit a cubic spline p(s) through the smoothed path
      4. smooth orientation (tangent-space Gaussian), SLERP between samples
      5. optional constant clearance offset
      6. time-parameterise along s: v(s) <= min(v_max, sqrt(a_lat/kappa), w_max/|dtheta/ds|),
         tangential accel <= a_max (forward/backward pass) -> trapezoid-like profile that
         automatically slows down in tight bends
      7. sample at the controller rate -> Trajectory(t, pos, quat, vel, omega, q_ref)
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np
from scipy.interpolate import CubicSpline

from .so3 import (
    gaussian_filter1d_edge,
    quat_conj,
    quat_interp,
    quat_log,
    quat_make_continuous,
    quat_mul,
    smooth_quaternions,
)


# ---------------------------------------------------------------------------
# Demonstrations
# ---------------------------------------------------------------------------
@dataclass
class Demo:
    t: np.ndarray            # (N,) seconds
    q: np.ndarray            # (N, n_joints) rad
    pos: np.ndarray          # (N, 3) m, tool point in base_link
    quat: np.ndarray         # (N, 4) xyzw, tool orientation in base_link
    joint_names: list
    meta: dict = field(default_factory=dict)

    def save(self, path: str) -> None:
        np.savez(
            path,
            t=self.t, q=self.q, pos=self.pos, quat=self.quat,
            joint_names=np.array(self.joint_names),
            meta=json.dumps(self.meta),
        )

    @staticmethod
    def load(path: str) -> "Demo":
        d = np.load(path, allow_pickle=False)
        return Demo(
            t=d["t"], q=d["q"], pos=d["pos"], quat=d["quat"],
            joint_names=[str(x) for x in d["joint_names"]],
            meta=json.loads(str(d["meta"])),
        )


# ---------------------------------------------------------------------------
# Trajectory container
# ---------------------------------------------------------------------------
@dataclass
class Trajectory:
    t: np.ndarray            # (M,)
    pos: np.ndarray          # (M, 3)
    quat: np.ndarray         # (M, 4)
    vel: np.ndarray          # (M, 3) linear velocity feed-forward
    omega: np.ndarray        # (M, 3) angular velocity feed-forward (base frame)
    q_ref: np.ndarray        # (M, n) joint config from the demo at the same path point
    s: np.ndarray            # (M,) arc length
    joint_names: list
    pos_offset: Optional[np.ndarray] = None   # (M, 3) learned correction (Stage 2 / ILC)
    meta: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.pos_offset is None:
            self.pos_offset = np.zeros_like(self.pos)

    @property
    def duration(self) -> float:
        return float(self.t[-1] - self.t[0])

    @property
    def length(self) -> float:
        return float(self.s[-1] - self.s[0])

    def sample(self, tau: float):
        """Interpolated (pos_cmd, quat, vel, omega, q_ref) at trajectory time tau.

        pos_cmd already includes the learned pos_offset.
        """
        tau = float(np.clip(tau, self.t[0], self.t[-1]))
        i = int(np.clip(np.searchsorted(self.t, tau, side="right") - 1, 0, len(self.t) - 2))
        dt = self.t[i + 1] - self.t[i]
        u = 0.0 if dt <= 0 else (tau - self.t[i]) / dt

        def lerp(a):
            return (1 - u) * a[i] + u * a[i + 1]

        q0, q1 = self.quat[i], self.quat[i + 1]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        qq = (1 - u) * q0 + u * q1          # nlerp is fine for 10 ms spacing
        qq /= np.linalg.norm(qq)
        return lerp(self.pos) + lerp(self.pos_offset), qq, lerp(self.vel), lerp(self.omega), lerp(self.q_ref)

    def index(self, tau: float) -> int:
        return int(np.clip(np.searchsorted(self.t, tau, side="right") - 1, 0, len(self.t) - 1))

    def contour_error(self, pos: np.ndarray, tau: float, window_s: float = 1.0) -> float:
        """Distance from pos to the DESIRED path (no learned offset), searched in a time
        window around tau. This is what matters for touching the wire; lag *along* the
        path is harmless."""
        i = self.index(tau)
        dt = max(float(np.median(np.diff(self.t[: min(len(self.t), 50)]))), 1e-4)
        w = max(2, int(window_s / dt))
        a, b = max(0, i - w), min(len(self.t), i + w + 1)
        P = self.pos[a:b]
        if len(P) < 2:
            return float(np.linalg.norm(P[0] - pos))
        A, B = P[:-1], P[1:]
        AB = B - A
        L2 = np.maximum(np.sum(AB * AB, axis=1), 1e-18)
        u = np.clip(np.sum((pos - A) * AB, axis=1) / L2, 0.0, 1.0)
        d = np.linalg.norm(A + u[:, None] * AB - pos, axis=1)
        return float(d.min())

    def reversed(self) -> "Trajectory":
        T = self.t[-1]
        return Trajectory(
            t=(T - self.t[::-1]).copy(),
            pos=self.pos[::-1].copy(),
            quat=self.quat[::-1].copy(),
            vel=-self.vel[::-1],
            omega=-self.omega[::-1],
            q_ref=self.q_ref[::-1].copy(),
            s=(self.s[-1] - self.s[::-1]).copy(),
            joint_names=list(self.joint_names),
            pos_offset=self.pos_offset[::-1].copy(),
            meta={**self.meta, "reversed": not self.meta.get("reversed", False)},
        )

    def save(self, path: str) -> None:
        np.savez(
            path,
            t=self.t, pos=self.pos, quat=self.quat, vel=self.vel, omega=self.omega,
            q_ref=self.q_ref, s=self.s, pos_offset=self.pos_offset,
            joint_names=np.array(self.joint_names), meta=json.dumps(self.meta),
        )

    @staticmethod
    def load(path: str) -> "Trajectory":
        d = np.load(path, allow_pickle=False)
        return Trajectory(
            t=d["t"], pos=d["pos"], quat=d["quat"], vel=d["vel"], omega=d["omega"],
            q_ref=d["q_ref"], s=d["s"], joint_names=[str(x) for x in d["joint_names"]],
            pos_offset=d["pos_offset"] if "pos_offset" in d.files else None,
            meta=json.loads(str(d["meta"])),
        )


# ---------------------------------------------------------------------------
# Processing parameters
# ---------------------------------------------------------------------------
@dataclass
class ProcessParams:
    min_step: float = 0.0003         # m, drop samples closer than this (operator pauses)
    ds: float = 0.001                # m, arc-length resolution of the processed path
    smooth_sigma: float = 0.003      # m, Gaussian smoothing width along the path (0 = off)
    ori_smooth_sigma: float = 0.005  # m, orientation smoothing width along the path
    max_smoothing_dev: float = 0.0015  # m, warn if smoothing moved the path more than this
    offset_xyz: tuple = (0.0, 0.0, 0.0)  # m, constant clearance offset in base frame
    v_max: float = 0.05              # m/s   (start 0.05, go up to ~0.15+ once confident)
    a_max: float = 0.10              # m/s^2 tangential
    a_lat_max: float = 0.10          # m/s^2 centripetal -> slows down in tight bends
    w_max: float = 0.5               # rad/s orientation rate limit
    profile_smooth: float = 0.01     # m, smoothing of the speed profile (reduces jerk)
    curvature_sigma: float = 0.008   # m, curvature for speed planning is computed on a path
                                     #    smoothed this much (ignores hand tremor; geometry untouched)
    dt: float = 0.01                 # s, output sample time (= controller period)
    trim_start_end: float = 0.0      # m, cut this much arc length from both ends (0 = keep)
    raw_interp: str = "linear"       # "linear" for dense free-drive demos, "spline" for sparse
                                     #  waypoints (teleop_teach.py) -> curved, not a polyline


def _dedupe(pos: np.ndarray, min_step: float) -> np.ndarray:
    keep = [0]
    for i in range(1, len(pos)):
        if np.linalg.norm(pos[i] - pos[keep[-1]]) >= min_step:
            keep.append(i)
    if keep[-1] != len(pos) - 1:
        if np.linalg.norm(pos[-1] - pos[keep[-1]]) > 1e-9:
            keep.append(len(pos) - 1)
        else:
            keep[-1] = len(pos) - 1
    return np.asarray(keep)


def _arclength(pos: np.ndarray) -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))])


def path_deviation(path_a: np.ndarray, path_b: np.ndarray) -> np.ndarray:
    """For every point in a: distance to the closest point of polyline-sampled b."""
    out = np.empty(len(path_a))
    for i, p in enumerate(path_a):
        out[i] = np.min(np.linalg.norm(path_b - p, axis=1))
    return out


def speed_profile(s: np.ndarray, kappa: np.ndarray, dtheta_ds: np.ndarray, p: ProcessParams) -> np.ndarray:
    """Velocity limit along arc length + forward/backward acceleration passes."""
    v = np.full(len(s), p.v_max)
    with np.errstate(divide="ignore"):
        v = np.minimum(v, np.sqrt(p.a_lat_max / np.maximum(kappa, 1e-9)))
        v = np.minimum(v, p.w_max / np.maximum(np.abs(dtheta_ds), 1e-9))
    if p.profile_smooth > 0:
        # smooth the *limit* with a min-preserving trick: smooth then never exceed raw limit
        sig = p.profile_smooth / max(np.mean(np.diff(s)), 1e-9)
        v = np.minimum(v, gaussian_filter1d_edge(v, sig))
    v[0] = 0.0
    v[-1] = 0.0
    ds = np.diff(s)
    for i in range(1, len(s)):                       # forward (accel)
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * p.a_max * ds[i - 1]))
    for i in range(len(s) - 2, -1, -1):              # backward (decel)
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * p.a_max * ds[i]))
    return v


def process_demo(demo: Demo, p: ProcessParams = ProcessParams(), verbose: bool = True) -> Trajectory:
    pos_raw = np.asarray(demo.pos, dtype=float)
    quat_raw = quat_make_continuous(demo.quat)
    q_raw = np.asarray(demo.q, dtype=float)

    keep = _dedupe(pos_raw, p.min_step)
    pos_raw, quat_raw, q_raw = pos_raw[keep], quat_raw[keep], q_raw[keep]
    if len(pos_raw) < 4:
        raise ValueError("Demo has fewer than 4 distinct samples - did the arm actually move?")

    s_raw = _arclength(pos_raw)
    L = s_raw[-1]

    # --- 2. uniform arc-length resample of raw data, then smooth ---------------
    n1 = max(int(np.ceil(L / p.ds)) + 1, 8)
    s1 = np.linspace(0.0, L, n1)
    if p.raw_interp == "spline":
        pos1 = CubicSpline(s_raw, pos_raw, axis=0, bc_type="natural")(s1)
    else:
        pos1 = np.stack([np.interp(s1, s_raw, pos_raw[:, k]) for k in range(3)], axis=1)
    q1 = np.stack([np.interp(s1, s_raw, q_raw[:, k]) for k in range(q_raw.shape[1])], axis=1)
    quat1 = quat_interp(s_raw, quat_raw, s1)

    if p.smooth_sigma > 0:
        pos_s = gaussian_filter1d_edge(pos1, p.smooth_sigma / (s1[1] - s1[0]))
        # pin endpoints exactly so the robot starts where it was taught
        pos_s[0], pos_s[-1] = pos1[0], pos1[-1]
    else:
        pos_s = pos1
    dev = path_deviation(pos_s, pos1)
    if verbose:
        print(f"[process] raw samples={len(demo.t)}, distinct={len(keep)}, path length={L*100:.1f} cm")
        print(f"[process] smoothing deviation: max={dev.max()*1000:.2f} mm, mean={dev.mean()*1000:.2f} mm")
    if dev.max() > p.max_smoothing_dev:
        print(f"[process] WARNING: smoothing moved the path by {dev.max()*1000:.2f} mm "
              f"(> {p.max_smoothing_dev*1000:.1f} mm). Reduce smooth_sigma or re-teach more steadily.")

    if p.ori_smooth_sigma > 0:
        quat1 = smooth_quaternions(quat1, p.ori_smooth_sigma / (s1[1] - s1[0]))

    # --- 3. spline through the smoothed path, re-parameterised by its own arc length
    s2_knots = _arclength(pos_s)
    keep2 = np.concatenate([[True], np.diff(s2_knots) > 1e-9])
    s2_knots, pos_s, quat1, q1 = s2_knots[keep2], pos_s[keep2], quat1[keep2], q1[keep2]
    spline = CubicSpline(s2_knots, pos_s, axis=0, bc_type="natural")
    L2 = s2_knots[-1]

    a, b = p.trim_start_end, L2 - p.trim_start_end
    if b <= a:
        raise ValueError("trim_start_end too large for this path")
    n2 = max(int(np.ceil((b - a) / p.ds)) + 1, 8)
    s2 = np.linspace(a, b, n2)
    pos2 = spline(s2) + np.asarray(p.offset_xyz, dtype=float)
    d1 = spline(s2, 1)
    d2 = spline(s2, 2)
    speed_s = np.linalg.norm(d1, axis=1)
    kappa = np.linalg.norm(np.cross(d1, d2), axis=1) / np.maximum(speed_s, 1e-9) ** 3
    if p.curvature_sigma > 0:
        ds2 = s2[1] - s2[0]
        pk = gaussian_filter1d_edge(pos2, p.curvature_sigma / ds2)
        g1 = np.gradient(pk, ds2, axis=0)
        g2 = np.gradient(g1, ds2, axis=0)
        kappa_plan = np.linalg.norm(np.cross(g1, g2), axis=1) / np.maximum(np.linalg.norm(g1, axis=1), 1e-9) ** 3
    else:
        kappa_plan = kappa
    quat2 = quat_interp(s2_knots, quat1, s2)
    qref2 = np.stack([np.interp(s2, s2_knots, q1[:, k]) for k in range(q1.shape[1])], axis=1)

    # orientation change per metre of path
    dth = np.linalg.norm(quat_log(quat_mul(quat2[1:], quat_conj(quat2[:-1]))), axis=1)
    dtheta_ds = np.concatenate([dth / np.diff(s2), [dth[-1] / (s2[-1] - s2[-2])]])
    dtheta_ds = gaussian_filter1d_edge(dtheta_ds, 3.0)

    # --- 6. time parameterisation ----------------------------------------------
    v = speed_profile(s2, kappa_plan, dtheta_ds, p)
    v_mid = 0.5 * (v[1:] + v[:-1])
    dt_seg = np.diff(s2) / np.maximum(v_mid, 1e-4)
    t2 = np.concatenate([[0.0], np.cumsum(dt_seg)])
    T = t2[-1]

    # --- 7. resample at controller rate ---------------------------------------
    t_out = np.arange(0.0, T + 1e-9, p.dt)
    if t_out[-1] < T:
        t_out = np.append(t_out, T)
    # exact s(t), sdot(t): constant acceleration inside every arc-length segment
    seg = np.clip(np.searchsorted(t2, t_out, side="right") - 1, 0, len(s2) - 2)
    tau_in = t_out - t2[seg]
    a_seg = (v[1:] ** 2 - v[:-1] ** 2) / (2.0 * np.maximum(np.diff(s2), 1e-12))
    s_out = np.minimum(s2[seg] + v[seg] * tau_in + 0.5 * a_seg[seg] * tau_in ** 2, s2[seg + 1])
    sdot = np.maximum(v[seg] + a_seg[seg] * tau_in, 0.0)
    sdot[-1] = 0.0
    pos_out = spline(s_out) + np.asarray(p.offset_xyz, dtype=float)
    vel_out = spline(s_out, 1) * sdot[:, None]
    quat_out = quat_interp(s2, quat2, s_out)
    dq = quat_log(quat_mul(quat_out[1:], quat_conj(quat_out[:-1])))
    omega_out = np.zeros_like(pos_out)
    omega_out[1:-1] = 0.5 * (dq[1:] + dq[:-1]) / np.maximum(np.diff(t_out)[:-1, None], 1e-9)
    qref_out = np.stack([np.interp(s_out, s2, qref2[:, k]) for k in range(qref2.shape[1])], axis=1)

    meta = {
        "source": demo.meta,
        "params": {k: (list(v_) if isinstance(v_, tuple) else v_) for k, v_ in asdict(p).items()},
        "length_m": float(s_out[-1] - s_out[0]),
        "duration_s": float(T),
        "max_smoothing_dev_m": float(dev.max()),
        "min_bend_radius_m": float(1.0 / max(kappa_plan.max(), 1e-9)),
        "peak_speed_mps": float(np.max(np.linalg.norm(vel_out, axis=1))),
    }
    if verbose:
        print(f"[process] tightest bend radius={meta['min_bend_radius_m']*100:.1f} cm, "
              f"peak speed={meta['peak_speed_mps']*100:.1f} cm/s, duration={T:.2f} s")
    return Trajectory(
        t=t_out, pos=pos_out, quat=quat_out, vel=vel_out, omega=omega_out,
        q_ref=qref_out, s=s_out, joint_names=list(demo.joint_names), meta=meta,
    )


def retime(traj: Trajectory, p: ProcessParams) -> Trajectory:
    """Re-run time parameterisation on an existing trajectory (e.g. after learning) with new
    speed limits, keeping the geometry (incl. pos_offset)."""
    fake = Demo(t=traj.t, q=traj.q_ref, pos=traj.pos + traj.pos_offset, quat=traj.quat,
                joint_names=traj.joint_names, meta=traj.meta.get("source", {}))
    p2 = ProcessParams(**{**asdict(p), "smooth_sigma": 0.0, "ori_smooth_sigma": 0.0,
                          "offset_xyz": (0.0, 0.0, 0.0), "trim_start_end": 0.0})
    return process_demo(fake, p2, verbose=False)
