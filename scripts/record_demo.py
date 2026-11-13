#!/usr/bin/env python3
"""TEACH phase: record a demonstration while the arm is hand-guided in gravity-compensation
(free-drive) mode. Publishes NOTHING to the robot - it only listens to /joint_states.

    python3 scripts/record_demo.py --name demo_01
      Enter  -> start recording   (probe at the start of the wire)
      Enter  -> stop and save     (probe at the end of the wire)
      q+Enter-> quit without saving

Saved to data/demos/<name>.npz  (t, q, tool pos, tool quat, effort, joint names).
"""
import argparse
import time

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import data_dir, load_config
from wire_trace.ros_common import KeyReader, RobotIO
from wire_trace.trajectory import Demo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=time.strftime("demo_%Y%m%d_%H%M%S"))
    ap.add_argument("--config", default=None)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--rate", type=float, default=None, help="sampling rate Hz (default from config)")
    a = ap.parse_args()
    cfg = load_config(a.config)
    rate = a.rate or float(cfg["recording"]["rate_hz"])

    rclpy.init()
    node = Node("wire_trace_recorder")
    io = RobotIO(node, cfg, need_command=False)
    io.wait_ready(urdf_file=a.urdf)
    keys = KeyReader()

    print("\n=== TEACH MODE ===")
    print("1. Robot must be in free-drive / gravity-compensation mode (./free_drive).")
    print("2. Buddy: hand on the E-stop.  Operator: place probe at the START of the wire.")
    print("3. Press Enter to START, guide slowly and steadily (2-5 mm clearance), Enter to STOP.")
    print("   (type q + Enter to abort without saving)\n")

    rec = False
    T, Q, P, R, E = [], [], [], [], []
    t_start = None
    last_print = 0.0
    period = 1.0 / rate
    next_tick = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            k = keys.pop()
            if k is not None:
                if k.lower() == "q":
                    print("Aborted, nothing saved.")
                    return
                if not rec:
                    rec = True
                    t_start = time.monotonic()
                    print(">>> RECORDING ... press Enter to stop")
                else:
                    break
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.001, next_tick - now))
                continue
            next_tick += period
            if not rec:
                continue
            if io.stale():
                print("WARNING: /joint_states stale - is the robot driver running?")
                continue
            q = io.q()
            p, qq = io.kin.fk(q)
            T.append(now - t_start); Q.append(q); P.append(p); R.append(qq); E.append(io.effort())
            if now - last_print > 1.0 and len(P) > 2:
                last_print = now
                Pa = np.array(P)
                L = np.sum(np.linalg.norm(np.diff(Pa, axis=0), axis=1))
                v = np.linalg.norm(Pa[-1] - Pa[-min(len(Pa), int(rate)) ]) / 1.0
                warn = "  <-- slow down" if v > 0.10 else ""
                print(f"  t={T[-1]:5.1f}s  samples={len(T):5d}  path={L*100:6.1f} cm  speed~{v*100:4.1f} cm/s{warn}")
    except KeyboardInterrupt:
        print("Ctrl-C: saving what was recorded")
    finally:
        if len(T) > 5:
            path = data_dir("demos", a.name + ".npz")
            demo = Demo(t=np.array(T), q=np.array(Q), pos=np.array(P), quat=np.array(R),
                        joint_names=io.kin.joint_names,
                        meta={"rate_hz": rate, "tool_xyz": cfg["robot"]["tool_xyz"],
                              "tool_rpy": cfg["robot"]["tool_rpy"], "recorded": time.ctime()})
            demo.save(path)
            np.save(path.replace(".npz", "_effort.npy"), np.array(E))
            L = np.sum(np.linalg.norm(np.diff(demo.pos, axis=0), axis=1))
            print(f"\nSaved {len(T)} samples, {T[-1]:.1f} s, path {L*100:.1f} cm -> {path}")
            print(f"Next: python3 scripts/process_demo.py {path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
