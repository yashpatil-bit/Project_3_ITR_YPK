#!/usr/bin/env python3
"""Print HEAL's kinematic chain (base_link -> tool tip) compactly: one line per joint with
origin xyz/rpy, axis and limits. Moves nothing. Paste the output to Claude so the kinematics
can be modelled offline.

    python3 scripts/dump_chain.py
"""
import json

import _path  # noqa: F401
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF

from wire_trace.config import load_config
from wire_trace.ros_common import RobotIO


def main():
    cfg = load_config()
    rclpy.init()
    node = Node("wire_trace_dump_chain")
    io = RobotIO(node, cfg, need_command=False)
    io.wait_ready()
    robot = URDF.from_xml_string(io._urdf)
    names = robot.get_chain(cfg["robot"]["base_link"], cfg["robot"]["tip_link"], links=False)
    print("CHAIN_BEGIN")
    for jn in names:
        j = robot.joint_map[jn]
        o = j.origin
        lim = j.limit
        print(json.dumps({
            "name": jn, "type": j.type,
            "xyz": [round(v, 6) for v in (o.xyz if o is not None and o.xyz is not None else [0, 0, 0])],
            "rpy": [round(v, 6) for v in (o.rpy if o is not None and o.rpy is not None else [0, 0, 0])],
            "axis": [round(v, 6) for v in (j.axis or [0, 0, 1])] if j.type != "fixed" else None,
            "lower": getattr(lim, "lower", None), "upper": getattr(lim, "upper", None),
            "velocity": getattr(lim, "velocity", None),
        }))
    print("CHAIN_END")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
