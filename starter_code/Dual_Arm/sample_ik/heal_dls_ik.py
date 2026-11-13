#!/usr/bin/env python3
# heal_dls_ik.py
from __future__ import annotations

import os
import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor

# ------------------------------------------------------------------
# Make the project root (RC-DS) importable so "import utils.XXX" works
# This file is assumed to be: RC-DS/Dual_Arm/heal_dls_ik.py
# ------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))      # .../RC-DS/Dual_Arm
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)                   # .../RC-DS
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from utils.dls_velocity_commander import DLSVelocityCommander


def main(args=None):
    rclpy.init(args=args)

    node = DLSVelocityCommander(
        robot_id="heal",
        base_link="base_link",
        tip_link="end-effector",
        # HEAL joint names as they appear in /joint_states
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        # Simple test target in front of the robot
        target_pos=[0.3, 0.5, 0.3],
        target_quat=[0.0, 0.0, 0.0, 1.0],
        joint_state_topic="/joint_states",
        velocity_command_topic="/velocity_controller/commands",
        robot_description_topic="/robot_description",
        ee_pose_topic=None,          # FK from KDL only
        ee_pose_is_stamped=False,
        max_cartesian_vel=0.04,
        max_angular_vel=0.04,
        dt=0.01,
        damping=0.1,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero_velocity()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
