#!/usr/bin/env python3
"""Visualise trajectories in RViz BEFORE executing (brief: "Visualize"). Publishes
nav_msgs/Path topics in the base_link frame (latched):

  /wire_trace/planned   processed trajectory (incl. learned offset)
  /wire_trace/demo      raw demonstration           (--demo)
  /wire_trace/actual    measured tool path of a run (--run)

    python3 scripts/publish_path_rviz.py data/processed/demo_01_v5.npz --demo data/demos/demo_01.npz
    rviz2   ->  Fixed Frame: base_link, Add -> By topic -> /wire_trace/* (Path)  + RobotModel + TF
"""
import argparse

import _path  # noqa: F401
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from wire_trace.config import load_config
from wire_trace.ros_common import LATCHED_QOS
from wire_trace.trajectory import Demo, Trajectory


def to_path(node, frame, pos, quat=None, step=1):
    msg = Path()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    for i in range(0, len(pos), step):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos[i])
        if quat is not None:
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = map(float, quat[i])
        else:
            ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("--demo", default=None)
    ap.add_argument("--run", default=None, help="data/runs/<name>.npz")
    ap.add_argument("--config", default=None)
    a = ap.parse_args()
    cfg = load_config(a.config)
    frame = cfg["robot"]["base_link"]
    rclpy.init()
    node = Node("wire_trace_rviz_paths")
    tr = Trajectory.load(a.traj)
    msgs = {"/wire_trace/planned": to_path(node, frame, tr.pos + tr.pos_offset, tr.quat, step=2)}
    if a.demo:
        d = Demo.load(a.demo)
        msgs["/wire_trace/demo"] = to_path(node, frame, d.pos, d.quat)
    if a.run:
        r = np.load(a.run)
        msgs["/wire_trace/actual"] = to_path(node, frame, r["pos"], r["quat"], step=2)
    pubs = {k: node.create_publisher(Path, k, LATCHED_QOS) for k in msgs}
    for k, m in msgs.items():
        pubs[k].publish(m)
        print(f"published {k} ({len(m.poses)} poses)")

    def tick():
        for k, m in msgs.items():
            m.header.stamp = node.get_clock().now().to_msg()
            pubs[k].publish(m)

    node.create_timer(1.0, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
