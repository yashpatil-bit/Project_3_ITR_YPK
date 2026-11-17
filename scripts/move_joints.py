#!/usr/bin/env python3
"""Move the arm to a joint configuration SAFELY.

    python3 scripts/move_joints.py --home                       # all joints to 0 deg (HEAL home pose)
    python3 scripts/move_joints.py --file data/wire_start_q.npy # start pose saved by find_pose.py
    python3 scripts/move_joints.py --deg 0 20 30 -40 80 0       # six joint angles in degrees

With --file / --deg the arm goes in two stages (approach from above):
  1. smooth joint-space move to a pose --approach metres (default 0.15) ABOVE the goal
  2. straight vertical descent in Cartesian space at 1 cm/s down to the goal
Before moving, the whole joint-space path is simulated with the real kinematics: if any
part of the arm would come closer than --min-clear (default 0.08 m) to the platform the
script refuses to move. Needs ./heal_server + the velocity_control launch.
Keep the wire box AWAY until the arm has reached the goal. Buddy on the E-stop.
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


def ik(kin, p_t, q_t, q, lo, hi, iters=200):
    for _ in range(iters):
        q = np.clip(q, lo + 0.02, hi - 0.02)
        p, qq = kin.fk(q)
        e = np.concatenate([p_t - p, quat_error(q_t, qq)])
        if np.linalg.norm(e[:3]) < 3e-4 and np.linalg.norm(e[3:]) < 3e-3:
            return q, True
        J = kin.jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + 0.02 ** 2 * np.eye(6), e)
        n = np.max(np.abs(dq))
        if n > 0.2:
            dq *= 0.2 / n
        q = q + dq
    return q, False


def lowest_point(kin, q):
    """Lowest z among the arm's frames (ignoring the base-most ones) and the tool point."""
    zs = [kin.fk(q)[0][2]]
    if hasattr(kin, "frame_positions"):
        zs += list(kin.frame_positions(q)[3:, 2])
    return float(min(zs))


def path_clearance(kin, q0, q1, T):
    plan = QuinticJointSpacePlanner(q0, q1, T)
    return min(lowest_point(kin, plan.evaluate(t)[0]) for t in np.linspace(0, T, 60))


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--home", action="store_true")
    g.add_argument("--deg", type=float, nargs=6)
    g.add_argument("--file", help=".npy with 6 joint angles in rad")
    ap.add_argument("--approach", type=float, default=0.15, help="approach from this far above the goal [m] (0 = off)")
    ap.add_argument("--min-clear", type=float, default=0.08, help="minimum height of any arm part above the platform [m]")
    ap.add_argument("--max-joint-speed", type=float, default=0.12, help="rad/s peak")
    ap.add_argument("--config", default=None)
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    rclpy.init()
    node = Node("wire_trace_move_joints")
    io = RobotIO(node, cfg)
    dt = 1.0 / float(cfg["robot"]["rate_hz"])
    try:
        io.wait_ready()
        kin, lo, hi = io.kin, io.kin.lower, io.kin.upper
        if a.home:
            q_goal = np.zeros(kin.n)
        elif a.deg is not None:
            q_goal = np.radians(a.deg)
        else:
            q_goal = np.asarray(np.load(a.file), float).reshape(-1)
        if np.any(q_goal < lo + 0.05) or np.any(q_goal > hi - 0.05):
            raise RuntimeError(f"goal {np.round(np.degrees(q_goal), 1)} deg is outside / too close to the joint limits")
        q0 = io.q()

        # ---- plan: optional approach pose above the goal --------------------------------
        p_goal, quat_goal = kin.fk(q_goal)
        use_approach = (not a.home) and a.approach > 0
        if use_approach:
            q_app, ok = ik(kin, p_goal + np.array([0.0, 0.0, a.approach]), quat_goal, q_goal.copy(), lo, hi)
            if not ok:
                raise RuntimeError("no joint solution for the approach pose above the goal; try --approach 0.10")
        else:
            q_app = q_goal
        T = max(4.0, 1.875 * float(np.max(np.abs(q_app - q0))) / a.max_joint_speed)
        clear = path_clearance(kin, q0, q_app, T)
        p0, _ = kin.fk(q0)
        p_app, _ = kin.fk(q_app)
        print(f"now      (deg): {np.round(np.degrees(q0), 1)}")
        if use_approach:
            print(f"approach (deg): {np.round(np.degrees(q_app), 1)}   tool at {np.round(p_app, 3)} m")
        print(f"goal     (deg): {np.round(np.degrees(q_goal), 1)}   tool at {np.round(p_goal, 3)} m")
        print(f"stage 1: joint move {T:.1f} s, lowest arm point on the way {clear*100:.1f} cm above the platform")
        if use_approach:
            print(f"stage 2: straight descent of {a.approach*100:.0f} cm at 1 cm/s")
        if clear < a.min_clear:
            raise RuntimeError(f"path would bring the arm to {clear*100:.1f} cm above the platform "
                               f"(< {a.min_clear*100:.0f} cm). Not moving. Tell Claude.")
        if lowest_point(kin, q_goal) < a.min_clear * 0.5:
            raise RuntimeError("goal pose itself is too close to the platform. Not moving.")
        if not a.yes and not confirm("Wire box AWAY, path clear, buddy on E-stop? [Enter = go, n = cancel] "):
            print("Cancelled.")
            return

        # ---- stage 1: quintic joint move ---------------------------------------------
        planner = QuinticJointSpacePlanner(q0, q_app, T)
        qdot_max = np.full(kin.n, a.max_joint_speed * 1.3)
        last = np.zeros(kin.n)
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
                raise RuntimeError("joint_states stale or fake zeros - robot in error? check T1")
            q = io.q()
            t = now - t0
            qd, qdd, _ = planner.evaluate(t)
            err = qd - q
            if np.max(np.abs(err)) > 0.15:
                raise RuntimeError(f"robot not following (joint error {np.degrees(np.max(np.abs(err))):.1f} deg) - blocked or stopped?")
            if lowest_point(kin, q) < a.min_clear * 0.8:
                raise RuntimeError("arm unexpectedly close to the platform - stopping")
            cmd = limit_qdot(qdd + 1.5 * err, last, qdot_max, 0.6 * dt)
            last = cmd
            io.publish_qdot(cmd)
            if t > T and np.max(np.abs(q_app - q)) < 0.005:
                break
            if t > T + 5.0:
                print(f"stage 1 ended near target, error {np.degrees(np.max(np.abs(q_app - q))):.2f} deg")
                break
        io.publish_zero(3)
        print(f"stage 1 done. joints (deg) = {np.round(np.degrees(io.q()), 1)}")

        # ---- stage 2: straight Cartesian descent to the goal --------------------------
        if use_approach:
            t_start = time.monotonic()
            last = np.zeros(kin.n)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                if io.stale():
                    raise RuntimeError("joint_states stale or fake zeros - robot in error? check T1")
                q = io.q()
                p, qq = kin.fk(q)
                e_p, e_o = p_goal - p, quat_error(quat_goal, qq)
                if np.linalg.norm(e_p) < 0.0008 and np.linalg.norm(e_o) < 0.01:
                    break
                if time.monotonic() - t_start > a.approach / 0.01 + 10.0:
                    print(f"stage 2 timeout, remaining error {np.linalg.norm(e_p)*1000:.1f} mm")
                    break
                v = 2.0 * e_p
                if np.linalg.norm(v) > 0.01:
                    v *= 0.01 / np.linalg.norm(v)
                w = 2.0 * e_o
                if np.linalg.norm(w) > 0.1:
                    w *= 0.1 / np.linalg.norm(w)
                cmd, _, _ = dls(kin.jacobian(q), np.concatenate([v, w]), 0.02, 0.1, 0.04)
                cmd = limit_qdot(cmd, last, np.full(kin.n, 0.15), 0.5 * dt)
                last = cmd
                io.publish_qdot(cmd)
                time.sleep(dt)
        q = io.q()
        print(f"Reached goal. joints (deg) = {np.round(np.degrees(q), 1)}   tool at {np.round(kin.fk(q)[0], 3)} m")
    except (RuntimeError, KeyboardInterrupt) as e:
        print(f"STOP: {e}")
    finally:
        io.publish_zero(10)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
