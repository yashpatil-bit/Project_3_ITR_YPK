#!/usr/bin/env python3
"""Move the arm to a joint configuration with a smooth quintic joint-space motion.

    python3 scripts/move_joints.py --home                       # all joints to 0 deg (HEAL home pose)
    python3 scripts/move_joints.py --deg 0 20 30 -40 80 0       # six joint angles in degrees
    python3 scripts/move_joints.py --file data/wire_start_q.npy # start pose saved by find_pose.py

Needs ./heal_server + the velocity_control launch.
!! Joint-space motion does not know about obstacles. The arm moves along the straight line
!! in JOINT space, so the tool may swing. Remove the probe from the wire first, keep the area
!! clear, buddy on the E-stop. Peak joint speed 0.12 rad/s by default.
"""
import argparse
import time

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import load_config
from wire_trace.controller import limit_qdot
from wire_trace.quintic import QuinticJointSpacePlanner
from wire_trace.ros_common import RobotIO, confirm


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--home", action="store_true")
    g.add_argument("--deg", type=float, nargs=6)
    g.add_argument("--file", help=".npy with 6 joint angles in rad")
    ap.add_argument("--max-joint-speed", type=float, default=0.12, help="rad/s peak")
    ap.add_argument("--config", default=None)
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    rclpy.init()
    node = Node("wire_trace_move_joints")
    io = RobotIO(node, cfg)
    try:
        io.wait_ready()
        if a.home:
            q_goal = np.zeros(io.kin.n)
        elif a.deg is not None:
            q_goal = np.radians(a.deg)
        else:
            q_goal = np.asarray(np.load(a.file), float).reshape(-1)
        if np.any(q_goal < io.kin.lower + 0.05) or np.any(q_goal > io.kin.upper - 0.05):
            raise RuntimeError(f"goal {np.round(np.degrees(q_goal), 1)} deg is outside / too close to the joint limits")
        q0 = io.q()
        dq = q_goal - q0
        T = max(4.0, 1.875 * float(np.max(np.abs(dq))) / a.max_joint_speed)
        p0, _ = io.kin.fk(q0)
        p1, _ = io.kin.fk(q_goal)
        print(f"now  (deg): {np.round(np.degrees(q0), 1)}")
        print(f"goal (deg): {np.round(np.degrees(q_goal), 1)}")
        print(f"tool tip moves from {np.round(p0, 3)} to {np.round(p1, 3)} m in {T:.1f} s")
        if not a.yes and not confirm("Probe OFF the wire, path clear, buddy on E-stop? [Enter = go, n = cancel] "):
            print("Cancelled.")
            return
        planner = QuinticJointSpacePlanner(q0, q_goal, T)
        dt = 1.0 / float(cfg["robot"]["rate_hz"])
        qdot_max = np.full(io.kin.n, a.max_joint_speed * 1.3)
        last = np.zeros(io.kin.n)
        t0 = time.monotonic()
        next_tick = t0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.001, next_tick - now))
                continue
            next_tick += dt
            if io.stale():
                raise RuntimeError("joint_states stale - robot in error? check T1")
            q = io.q()
            t = now - t0
            qd, qdd, _ = planner.evaluate(t)
            err = qd - q
            if np.max(np.abs(err)) > 0.15:
                raise RuntimeError(f"robot not following (joint error {np.degrees(np.max(np.abs(err))):.1f} deg)")
            cmd = limit_qdot(qdd + 1.5 * err, last, qdot_max, 0.6 * dt)
            last = cmd
            io.publish_qdot(cmd)
            if t > T and np.max(np.abs(q_goal - q)) < 0.005:
                print(f"Reached goal. joints (deg) = {np.round(np.degrees(q), 1)}")
                break
            if t > T + 5.0:
                print(f"Stopped near goal, error {np.degrees(np.max(np.abs(q_goal - q))):.2f} deg")
                break
    except (RuntimeError, KeyboardInterrupt) as e:
        print(f"STOP: {e}")
    finally:
        io.publish_zero(10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
