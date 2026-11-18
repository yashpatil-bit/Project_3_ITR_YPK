#!/usr/bin/env python3
"""Print and SAVE the robot's current pose (joint angles + tool coordinates), with a
singularity / joint-limit check. Moves nothing.

Typical use: move the arm by hand in ./free_drive to where you want it (e.g. around the probe
handle), switch to ./heal_server + velocity launch (the arm holds that pose), then:

    python3 scripts/save_pose.py --name grip_pose

Later, come back to exactly that pose (joint-space move -> no singularity problems):

    python3 scripts/move_joints.py --file data/poses/grip_pose.npy --approach 0.10
"""
import argparse
import os

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import data_dir, load_config
from wire_trace.ros_common import RobotIO
from wire_trace.so3 import quat_to_rotmat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="pose")
    ap.add_argument("--config", default=None)
    a = ap.parse_args()
    cfg = load_config(a.config)
    rclpy.init()
    node = Node("wire_trace_save_pose")
    io = RobotIO(node, cfg, need_command=False)
    try:
        io.wait_ready()
        qs = []
        for _ in range(20):                      # average a few samples
            rclpy.spin_once(node, timeout_sec=0.02)
            qs.append(io.q())
        q = np.mean(qs, axis=0)
        p, qq = io.kin.fk(q)
        R = quat_to_rotmat(qq)
        smin = float(np.linalg.svd(io.kin.jacobian(q), compute_uv=False)[-1])
        margin = np.minimum(q - io.kin.lower, io.kin.upper - q)
        print(f"joints (deg)        : {np.round(np.degrees(q), 1).tolist()}")
        print(f"tool point (m)      : x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f}   (robot.tip_link + tool offset)")
        print(f"tool axes           : x={np.round(R[:, 0], 2)}  y={np.round(R[:, 1], 2)}  z={np.round(R[:, 2], 2)}")
        print(f"singularity check   : sigma_min = {smin:.3f}  "
              f"({'OK' if smin > 0.06 else 'CLOSE TO A SINGULARITY - avoid Cartesian moves here' if smin > 0.03 else 'SINGULAR - move away'})")
        worst = int(np.argmin(margin))
        print(f"joint-limit margin  : {np.degrees(margin.min()):.0f} deg (joint {worst + 1})"
              f"{'' if margin.min() > np.radians(15) else '  <-- close to a joint limit'}")
        os.makedirs(data_dir("poses"), exist_ok=True)
        path = data_dir("poses", a.name + ".npy")
        np.save(path, q)
        print(f"saved -> {path}")
        print(f"return here later:  python3 scripts/move_joints.py --file {os.path.relpath(path)} --approach 0.10")
    except RuntimeError as e:
        print(f"[FAIL] {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
