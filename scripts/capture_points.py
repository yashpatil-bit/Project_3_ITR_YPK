#!/usr/bin/env python3
"""Capture tool-tip positions (Enter = save current point, q = finish). Moves nothing.

Used for
  * measuring the wire stand / workspace box  (set controller.workspace_min/max)
  * Stage 2: reference points on the stand before/after moving it (stage2/register_stand.py)
  * Stage 2: new start / goal points for the DMP (stage2/learn_dmp.py --start-goal)

    python3 scripts/capture_points.py --out data/ref_points_A.npy
"""
import argparse

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import load_config
from wire_trace.ros_common import KeyReader, RobotIO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", default=None)
    ap.add_argument("--urdf", default=None)
    a = ap.parse_args()
    cfg = load_config(a.config)
    rclpy.init()
    node = Node("wire_trace_capture_points")
    io = RobotIO(node, cfg, need_command=False)
    io.wait_ready(urdf_file=a.urdf)
    keys = KeyReader()
    pts = []
    print("Free-drive the probe to a point, press Enter to capture it, q + Enter to finish.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            k = keys.pop()
            if k is None:
                continue
            if k.lower() == "q":
                break
            # average 20 samples to beat encoder noise
            acc = []
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.01)
                acc.append(io.kin.fk(io.q())[0])
            p = np.mean(acc, axis=0)
            pts.append(p)
            print(f"  point {len(pts)}: {np.round(p, 4)}")
    except KeyboardInterrupt:
        pass
    finally:
        if pts:
            np.save(a.out, np.array(pts))
            P = np.array(pts)
            print(f"Saved {len(pts)} points -> {a.out}")
            print(f"bounding box min {np.round(P.min(0), 3)} max {np.round(P.max(0), 3)}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
