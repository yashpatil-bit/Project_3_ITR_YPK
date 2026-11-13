#!/usr/bin/env python3
"""
ROS2 equivalent of your ROS1 KDL FK node.

What it does:
- Subscribes:   /joint_states   (sensor_msgs/JointState)
- Reads URDF:   from /robot_description topic (std_msgs/String) OR from this node's
               'robot_description' parameter (fallback).
- Builds KDL chain: base_link -> tip_link
- Computes FK each joint_state callback
- Publishes:    /end_effector_pose  (geometry_msgs/PoseStamped)

Notes for your cobot setup:
- In your system, /robot_description is a latched TRANSIENT_LOCAL topic from controller_manager.
- This node subscribes with TRANSIENT_LOCAL QoS so it receives the latched URDF immediately.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class KDLForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__("kdl_forward_kinematics_node")

        # ----------------------------
        # Parameters (edit if needed)
        # ----------------------------
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "end-effector")          # <-- your earlier guess was wrong in TF; check below
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("pose_topic", "/end_effector_pose")

        self.base_link = self.get_parameter("base_link").value
        self.tip_link = self.get_parameter("tip_link").value

        joint_states_topic = self.get_parameter("joint_states_topic").value
        robot_description_topic = self.get_parameter("robot_description_topic").value
        pose_topic = self.get_parameter("pose_topic").value

        # ----------------------------
        # State
        # ----------------------------
        self._kdl_ready = False
        self.tree = None
        self.chain = None
        self.n_joints = 0
        self.joint_names = []
        self.fk_solver = None

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        # Subscribe to joint states (we can subscribe immediately; we just won't compute FK until KDL is ready)
        self.create_subscription(JointState, joint_states_topic, self.joint_state_callback, 10)

        # Subscribe to /robot_description (TRANSIENT_LOCAL so we get latched URDF)
        robot_desc_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._robot_desc_sub = self.create_subscription(
            String, robot_description_topic, self.robot_description_callback, robot_desc_qos
        )

        self.get_logger().info(
            f"Started. Waiting for robot_description to build KDL chain {self.base_link} -> {self.tip_link} "
            f"and joint_states on {joint_states_topic}."
        )

    # ----------------------------
    # URDF -> KDL setup
    # ----------------------------
    def robot_description_callback(self, msg: String):
        """Build KDL chain once when URDF arrives."""
        if self._kdl_ready:
            return

        urdf_xml = msg.data.strip()
        if not urdf_xml or "<robot" not in urdf_xml:
            self.get_logger().error("Received /robot_description but it doesn't look like URDF XML.")
            return

        try:
            robot = URDF.from_xml_string(urdf_xml)
            ok, tree = treeFromUrdfModel(robot)
            if not ok:
                self.get_logger().error("Failed to construct KDL tree from URDF.")
                return

            chain = tree.getChain(self.base_link, self.tip_link)
            n = chain.getNrOfJoints()

            if n == 0:
                # This is the MOST common issue: tip_link name is not in URDF / not connected to base_link.
                self.get_logger().error(
                    f"KDL chain has 0 joints. Check base_link='{self.base_link}' and tip_link='{self.tip_link}'.\n"
                    "Tip: run:\n"
                    "  ros2 run tf2_tools view_frames\n"
                    "and see the actual end-effector frame name (you had 'end-effector' in your TF graph)."
                )
                return

            self.tree = tree
            self.chain = chain
            self.n_joints = int(n)
            self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

            # Build joint order from the KDL chain (skip fixed joints)
            self.joint_names = []
            for i in range(self.chain.getNrOfSegments()):
                seg = self.chain.getSegment(i)
                j = seg.getJoint()
                if j.getTypeName() != "None":
                    self.joint_names.append(j.getName())

            self._kdl_ready = True
            self.get_logger().info(
                f"KDL chain ready with {self.n_joints} joints. Joint order: {self.joint_names}"
            )

            # We only need the URDF once; stop subscribing to reduce noise
            self.destroy_subscription(self._robot_desc_sub)

        except Exception as e:
            self.get_logger().error(f"Exception creating KDL from URDF: {e}")

    # ----------------------------
    # Joint states -> FK -> Pose
    # ----------------------------
    def joint_state_callback(self, msg: JointState):
        if not self._kdl_ready:
            return  # wait until URDF/KDL is ready

        if len(msg.name) == 0 or len(msg.position) == 0:
            return

        # Fill q in chain joint order
        q = kdl.JntArray(self.n_joints)
        name_to_index = {n: i for i, n in enumerate(msg.name)}

        for i, joint_name in enumerate(self.joint_names):
            idx = name_to_index.get(joint_name, None)
            if idx is None:
                # If the joint name doesn't exist in JointState, default to 0 and warn occasionally
                q[i] = 0.0
                # (no throttle helper in rclpy; keep it light)
                if (self.get_clock().now().nanoseconds // 1_000_000_000) % 2 == 0:
                    self.get_logger().warn(f"Joint '{joint_name}' not found in JointState; defaulting to 0.0")
            else:
                q[i] = float(msg.position[idx])

        end_effector_frame = kdl.Frame()
        ret = self.fk_solver.JntToCart(q, end_effector_frame)
        if ret < 0:
            self.get_logger().error("Failed to compute forward kinematics (JntToCart returned < 0)")
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.base_link

        pose_msg.pose.position.x = float(end_effector_frame.p[0])
        pose_msg.pose.position.y = float(end_effector_frame.p[1])
        pose_msg.pose.position.z = float(end_effector_frame.p[2])

        qx, qy, qz, qw = end_effector_frame.M.GetQuaternion()
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)

        self.pose_pub.publish(pose_msg)


def main():
    rclpy.init()
    node = KDLForwardKinematicsNode()
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
