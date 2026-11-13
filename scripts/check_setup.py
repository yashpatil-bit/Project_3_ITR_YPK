#!/usr/bin/env python3
"""Day-1 diagnostics. Run while ./heal_server (or ./free_drive) is running.

Checks: /joint_states arrives (and its rate), URDF arrives, KDL chain builds, joint names
match the config, joint limits, current tool pose, and whether anything is subscribed
to the velocity command topic. Moves nothing.

    python3 scripts/check_setup.py            # one-shot report
    python3 scripts/check_setup.py --watch    # keep printing the tool pose at 2 Hz
"""
import argparse
import time

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import load_config
from wire_trace.ros_common import RobotIO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--urdf", default=None, help="URDF file if /robot_description is not published")
    ap.add_argument("--watch", action="store_true")
    a = ap.parse_args()
    cfg = load_config(a.config)
    rclpy.init()
    node = Node("wire_trace_check_setup")
    io = RobotIO(node, cfg, need_command=False)
    ok = True
    try:
        print("Topics visible:")
        time.sleep(1.0)
        for name, types in sorted(node.get_topic_names_and_types()):
            print(f"   {name:45s} {types}")
        io.wait_ready(timeout=10.0, urdf_file=a.urdf)
        print(f"\n[OK] KDL chain {cfg['robot']['base_link']} -> {cfg['robot']['tip_link']} "
              f"(+ tool {cfg['robot']['tool_xyz']}) with joints {io.kin.joint_names}")
        print(f"     lower limits: {np.round(io.kin.lower, 3)}")
        print(f"     upper limits: {np.round(io.kin.upper, 3)}")
        print(f"     vel limits  : {io.kin.vel_limit}")

        # joint_states rate
        n0, t0, last = 0, time.monotonic(), None
        while time.monotonic() - t0 < 2.0:
            rclpy.spin_once(node, timeout_sec=0.01)
            if io._js is not last:
                n0 += 1
                last = io._js
        rate = n0 / 2.0
        print(f"[{'OK' if rate > 50 else 'WARN'}] /joint_states rate ~ {rate:.0f} Hz")
        if rate < 50:
            ok = False
        print(f"     effort field present: {len(io._js.effort) == len(io._js.name)}")

        cmd_topic = cfg["robot"]["velocity_command_topic"]
        subs = node.count_subscribers(cmd_topic)
        print(f"[{'OK' if subs > 0 else 'INFO'}] {subs} subscriber(s) on {cmd_topic} "
              f"({'velocity controller running' if subs else 'start: ros2 launch addverb_cobot_control velocity_control.launch.py'})")

        q = io.q()
        p, qq = io.kin.fk(q)
        J = io.kin.jacobian(q)
        sv = np.linalg.svd(J, compute_uv=False)
        print(f"\nq [rad]     = {np.round(q, 4)}")
        print(f"tool pos [m]= {np.round(p, 4)}   quat xyzw = {np.round(qq, 4)}")
        print(f"Jacobian singular values = {np.round(sv, 3)}  (smallest < 0.04 => near singularity)")

        while a.watch and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            p, qq = io.kin.fk(io.q())
            print(f"tool pos = {np.round(p, 4)}  quat = {np.round(qq, 3)}  stale={io.stale()}")
            time.sleep(0.5)
    except RuntimeError as e:
        ok = False
        print(f"[FAIL] {e}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("\nRESULT:", "ready" if ok else "fix the items above")


if __name__ == "__main__":
    main()
