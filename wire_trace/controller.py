"""Cartesian trajectory tracking with damped-least-squares IK -> joint velocity commands.

Pure numpy, independent of ROS, so the exact same code runs in the offline
simulator (scripts/simulate_replay.py, tests/) and on the real HEAL arm
(scripts/replay.py).

Control law at every tick (dt = 10 ms):
    (p_d, R_d, v_d, w_d) = trajectory(tau)
    v = k * s * v_d + Kp * (p_d - p)                 linear twist   (feed-forward + feedback)
    w = k * s * w_d + Ko * log(R_d R^T)              angular twist
    qdot = J^T (J J^T + lambda^2 I)^-1 [v; w]        DLS-IK, lambda grows near singularities
    qdot <- clamp(|qdot_i| <= qdot_max, |dqdot_i| <= qddot_max*dt)   (direction preserving)
    tau  <- tau + k * s * dt

s = user speed scale (0.1 = 10 % speed), k in [k_min, 1] = automatic slow-down when the
tracking error grows: the reference waits for the robot instead of the robot cutting a
corner into the wire.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from .so3 import quat_error
from .trajectory import Trajectory


@dataclass
class ControllerParams:
    kp_pos: float = 3.0            # 1/s  position feedback gain
    kp_ori: float = 2.0            # 1/s  orientation feedback gain
    damping: float = 0.02          # DLS lambda away from singularities
    damping_max: float = 0.10      # DLS lambda at a singularity
    sigma_thresh: float = 0.04     # start increasing lambda below this singular value
    qdot_max: Sequence[float] = (0.6, 0.6, 0.6, 0.8, 0.8, 0.8)    # rad/s per joint
    qddot_max: float = 3.0         # rad/s^2 per joint (slew limit)
    qddot_stop: float = 3.0        # rad/s^2 used when ramping to zero on stop
    speed_scale: float = 0.1       # 0.1 = 10 % of the processed trajectory speed
    err_soft: float = 0.0015       # m   start slowing the reference above this path error
    err_abort: float = 0.006       # m   abort if the probe is this far from the desired path
    lag_soft: float = 0.010        # m   along-path lag tolerated before slowing the reference
    lag_abort: float = 0.030       # m   abort if the robot lags the reference by more than this
    ori_err_abort: float = 0.20    # rad abort above this orientation error
    k_min: float = 0.2             # minimum reference-speed factor when slowing down
    start_tol: float = 0.005       # m   must be this close to the start pose to begin
    done_tol: float = 0.0015       # m   final position tolerance
    settle_time: float = 1.5       # s   max extra time after tau reaches T
    joint_margin: float = 0.05     # rad keep away from URDF joint limits
    workspace_min: Sequence[float] = (-1.0, -1.0, -0.05)   # m, base_link box for the tool point
    workspace_max: Sequence[float] = (1.0, 1.0, 1.2)


@dataclass
class StepResult:
    qdot: np.ndarray
    status: str                     # "running" | "done" | "abort"
    reason: str = ""
    info: dict = field(default_factory=dict)


def dls(J: np.ndarray, twist: np.ndarray, lam0: float, lam_max: float, sigma_thresh: float):
    """Damped least squares with singular-value-based adaptive damping."""
    sv = np.linalg.svd(J, compute_uv=False)
    smin = float(sv[-1]) if J.shape[0] <= J.shape[1] else float(sv.min())
    if smin < sigma_thresh:
        lam2 = lam0 ** 2 + (1.0 - (smin / sigma_thresh) ** 2) * lam_max ** 2
    else:
        lam2 = lam0 ** 2
    A = J @ J.T + lam2 * np.eye(J.shape[0])
    return J.T @ np.linalg.solve(A, twist), smin, float(np.sqrt(lam2))


def limit_qdot(qdot: np.ndarray, last: np.ndarray, qdot_max: np.ndarray, dqdot: float):
    """Direction-preserving velocity clamp and per-tick change clamp."""
    scale = np.max(np.abs(qdot) / qdot_max)
    if scale > 1.0:
        qdot = qdot / scale
    delta = qdot - last
    s2 = np.max(np.abs(delta)) / max(dqdot, 1e-12)
    if s2 > 1.0:
        delta = delta / s2
    return last + delta


class TrackingController:
    def __init__(
        self,
        traj: Trajectory,
        params: ControllerParams,
        joint_lower: Optional[np.ndarray] = None,
        joint_upper: Optional[np.ndarray] = None,
    ):
        self.traj = traj
        self.p = params
        self.n = traj.q_ref.shape[1]
        self.qdot_max = np.resize(np.asarray(params.qdot_max, dtype=float), self.n)
        self.lo = None if joint_lower is None else np.asarray(joint_lower, float) + params.joint_margin
        self.hi = None if joint_upper is None else np.asarray(joint_upper, float) - params.joint_margin
        self.ws_min = np.asarray(params.workspace_min, float)
        self.ws_max = np.asarray(params.workspace_max, float)
        self.reset()

    def reset(self):
        self.tau = float(self.traj.t[0])
        self.last_qdot = np.zeros(self.n)
        self.started = False
        self.t_over = 0.0
        self.done = False

    # ------------------------------------------------------------------
    def check_start(self, pos: np.ndarray, quat: np.ndarray) -> tuple[bool, str]:
        p0, q0, *_ = self.traj.sample(self.traj.t[0])
        e = float(np.linalg.norm(p0 - pos))
        eo = float(np.linalg.norm(quat_error(q0, quat)))
        if e > self.p.start_tol:
            return False, f"tool is {e*1000:.1f} mm from trajectory start (tol {self.p.start_tol*1000:.1f} mm) - run go_to_start first"
        if eo > 0.1:
            return False, f"tool orientation is {np.degrees(eo):.1f} deg off the start orientation - run go_to_start first"
        return True, ""

    def step(self, q: np.ndarray, pos: np.ndarray, quat: np.ndarray, J: np.ndarray, dt: float) -> StepResult:
        p = self.p
        q = np.asarray(q, float)
        T = float(self.traj.t[-1])

        p_cmd, q_d, v_d, w_d, q_ref = self.traj.sample(self.tau)
        e_pos = p_cmd - pos                        # feedback towards the (possibly ILC-shifted) command
        e_ori = quat_error(q_d, quat)
        # safety uses the distance to the DESIRED path (contour error), not the time-based error
        en = self.traj.contour_error(pos, self.tau)
        e_track = float(np.linalg.norm(e_pos))
        eon = float(np.linalg.norm(e_ori))
        info = dict(tau=self.tau, p_d=p_cmd, q_d=q_d, e_pos=en, e_track=e_track, e_ori=eon, k=1.0,
                    sigma_min=np.nan, lam=np.nan)

        # ---------------- safety checks ----------------
        if en > p.err_abort:
            return StepResult(np.zeros(self.n), "abort", f"probe {en*1000:.1f} mm off the path (> {p.err_abort*1000:.1f} mm)", info)
        if e_track > p.lag_abort:
            return StepResult(np.zeros(self.n), "abort", f"robot lags the reference by {e_track*1000:.0f} mm", info)
        if eon > p.ori_err_abort:
            return StepResult(np.zeros(self.n), "abort", f"orientation error {np.degrees(eon):.1f} deg too large", info)
        if np.any(pos < self.ws_min) or np.any(pos > self.ws_max):
            return StepResult(np.zeros(self.n), "abort", f"tool left workspace box: {np.round(pos, 3)}", info)
        if self.lo is not None and (np.any(q < self.lo) or np.any(q > self.hi)):
            bad = np.where((q < self.lo) | (q > self.hi))[0].tolist()
            return StepResult(np.zeros(self.n), "abort", f"joint(s) {bad} inside joint-limit margin", info)

        # ---------------- reference progression ----------------
        # slow the reference down when the probe drifts off the path or falls far behind
        en_k = max(en, e_track - p.lag_soft)
        if en_k > p.err_soft:
            k = 1.0 - (en_k - p.err_soft) / max(p.err_abort - p.err_soft, 1e-9)
            k = float(np.clip(k, p.k_min, 1.0))
        else:
            k = 1.0
        info["k"] = k
        ff = k * p.speed_scale if self.tau < T else 0.0

        twist = np.concatenate([ff * v_d + p.kp_pos * e_pos, ff * w_d + p.kp_ori * e_ori])
        qdot, smin, lam = dls(J, twist, p.damping, p.damping_max, p.sigma_thresh)
        info["sigma_min"], info["lam"] = smin, lam
        qdot = limit_qdot(qdot, self.last_qdot, self.qdot_max, p.qddot_max * dt)

        # do not drive further into a joint limit
        if self.lo is not None:
            q_next = q + qdot * dt
            outward = ((q_next < self.lo) & (qdot < 0)) | ((q_next > self.hi) & (qdot > 0))
            if np.any(outward):
                return StepResult(np.zeros(self.n), "abort", f"joint(s) {np.where(outward)[0].tolist()} about to hit limit", info)

        self.last_qdot = qdot
        if self.tau < T:
            self.tau = min(T, self.tau + ff * dt)
        else:
            self.t_over += dt
            if e_track < p.done_tol or self.t_over > p.settle_time:
                self.done = True
                return StepResult(qdot, "done", "", info)
        return StepResult(qdot, "running", "", info)

    def stop_step(self, dt: float) -> np.ndarray:
        """Ramp the last command to zero (call every tick after done/abort until all ~0)."""
        step = self.p.qddot_stop * dt
        mag = np.max(np.abs(self.last_qdot))
        if mag <= step:
            self.last_qdot = np.zeros(self.n)
        else:
            self.last_qdot = self.last_qdot * (1.0 - step / mag)
        return self.last_qdot.copy()
