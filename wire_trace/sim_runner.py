"""Run the real TrackingController against the SimArm (offline dry-run / tests)."""
from __future__ import annotations

import numpy as np

from .controller import ControllerParams, TrackingController
from .runlog import RunLog
from .sim_arm import SimArm
from .trajectory import Trajectory


def place_at_start(arm: SimArm, traj: Trajectory):
    p0, q0, *_ = traj.sample(traj.t[0])
    arm.q = traj.q_ref[0].copy()          # same IK branch as the demo
    arm.ik_step_to(p0, q0)


def simulate(traj: Trajectory, arm: SimArm, cp: ControllerParams, dt: float = 0.01,
             max_time: float = 600.0, disturbance=None) -> tuple[RunLog, str, str]:
    ctrl = TrackingController(traj, cp, arm.lower, arm.upper)
    log = RunLog(meta={"speed_scale": cp.speed_scale, "sim": True})
    pos, quat = arm.fk(arm.measured_q())
    ok, why = ctrl.check_start(pos, quat)
    if not ok:
        log.meta.update(status="abort", reason=why)
        return log, "abort", why
    t = 0.0
    status, reason = "running", ""
    while t < max_time:
        q = arm.measured_q()
        pos, quat = arm.fk(q)
        J = arm.jacobian(q)
        r = ctrl.step(q, pos, quat, J, dt)
        log.add(t=t, tau=r.info["tau"], q=q, qdot_cmd=r.qdot, pos=pos, quat=quat, p_d=r.info["p_d"],
                q_d=r.info["q_d"], e_pos=r.info["e_pos"], e_track=r.info["e_track"], e_ori=r.info["e_ori"], k=r.info["k"],
                sigma_min=r.info["sigma_min"])
        if r.status != "running":
            status, reason = r.status, r.reason
            break
        cmd = r.qdot
        if disturbance is not None:
            cmd = cmd + disturbance(t)
        arm.apply(cmd, dt)
        t += dt
    # ramp down
    for _ in range(200):
        z = ctrl.stop_step(dt)
        arm.apply(z, dt)
        if not np.any(z):
            break
    log.meta.update(status=status, reason=reason)
    return log, status, reason
