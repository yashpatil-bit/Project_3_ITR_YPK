#!/usr/bin/env python3
"""Move the arm (slowly) to the start pose of a processed trajectory.

Phase 1: quintic joint-space move from the current joints to the joint configuration
         that was recorded at the start of the demo (same IK branch as the demo).
Phase 2: short Cartesian DLS alignment so the probe sits exactly on the trajectory start.

!! Joint-space motion does NOT know where the wire is. Before running, put the probe
!! close to the start by hand (free-drive) or make sure the straight-ish joint motion is
!! clear of the wire stand. Buddy keeps a hand on the E-stop.

    python3 scripts/go_to_start.py data/processed/demo_01_v5.npz
    python3 scripts/go_to_start.py data/processed/demo_01_v5.npz --end    # go to the END (before a --reverse run)
"""
import argparse
import time

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import load_config
from wire_trace.controller import dls, limit_qdot
from wire_trace.quintic import QuinticJointSpacePlanner
from wire_trace.ros_common import RobotIO, confirm
from wire_trace.so3 import quat_error
from wire_trace.trajectory import Trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("--end", action="store_true", help="go to the end of the trajectory instead")
    ap.add_argument("--max-joint-speed", type=float, default=0.25, help="rad/s peak in phase 1")
    ap.add_argument("--config", default=None)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    traj = Trajectory.load(a.traj)
    idx = -1 if a.end else 0
    q_goal = traj.q_ref[idx]
    p_goal, quat_goal = traj.pos[idx] + traj.pos_offset[idx], traj.quat[idx]

    rclpy.init()
    node = Node("wire_trace_go_to_start")
    io = RobotIO(node, cfg)
    rate = float(cfg["robot"]["rate_hz"])
    dt = 1.0 / rate
    try:
        io.wait_ready(urdf_file=a.urdf)
        if list(traj.joint_names) != list(io.kin.joint_names):
            raise RuntimeError(f"trajectory joints {traj.joint_names} != robot chain {io.kin.joint_names}")
        q0 = io.q()
        p0, _ = io.kin.fk(q0)
        dq = q_goal - q0
        T = max(3.0, 1.875 * float(np.max(np.abs(dq))) / a.max_joint_speed)
        print(f"Joint change [deg]: {np.round(np.degrees(dq), 1)}")
        print(f"Tool moves {np.linalg.norm(p_goal - p0)*100:.1f} cm (straight-line distance) in {T:.1f} s")
        if np.any(q_goal < io.kin.lower) or np.any(q_goal > io.kin.upper):
            raise RuntimeError("goal joint configuration outside URDF limits")
        if not a.yes and not confirm("Path clear of the wire? Buddy on E-stop? [Enter = go, n = cancel] "):
            print("Cancelled.")
            return
        planner = QuinticJointSpacePlanner(q0, q_goal, T)
        qdot_max = np.full(io.kin.n, max(a.max_joint_speed * 1.5, 0.1))
        last = np.zeros(io.kin.n)
        t0 = time.monotonic()
        next_tick = t0
        phase = 1
        t_align = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.001, next_tick - now))
                continue
            next_tick += dt
            if io.stale():
                raise RuntimeError("joint_states stale - stopping")
            q = io.q()
            if phase == 1:
                t = now - t0
                qd, qdd, _ = planner.evaluate(t)
                cmd = qdd + 2.0 * (qd - q)
                if t >= T and np.max(np.abs(q_goal - q)) < 0.01:
                    phase, t_align = 2, now
                    print("Phase 1 done, fine alignment ...")
                elif t >= T + 3.0:
                    raise RuntimeError(f"joint goal not reached (err {np.degrees(np.max(np.abs(q_goal - q))):.2f} deg)")
            else:
                p, qq = io.kin.fk(q)
                e = np.concatenate([1.5 * (p_goal - p), 1.5 * quat_error(quat_goal, qq)])
                cmd, _, _ = dls(io.kin.jacobian(q), e, 0.02, 0.1, 0.04)
                ep, eo = np.linalg.norm(p_goal - p), np.linalg.norm(quat_error(quat_goal, qq))
                if ep < 0.0005 and eo < 0.01:
                    print(f"At {'end' if a.end else 'start'}: pos err {ep*1000:.2f} mm, ori err {np.degrees(eo):.2f} deg")
                    break
                if now - t_align > 5.0:
                    print(f"Alignment timeout: pos err {ep*1000:.2f} mm, ori err {np.degrees(eo):.2f} deg")
                    break
            cmd = limit_qdot(cmd, last, qdot_max, 2.0 * dt)
            last = cmd
            io.publish_qdot(cmd)
    except (RuntimeError, KeyboardInterrupt) as e:
        print(f"STOP: {e}")
    finally:
        io.publish_zero(10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
