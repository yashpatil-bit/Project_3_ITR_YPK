#!/usr/bin/env python3
"""
dual_going_home.py

Run HEAL and Franka "go home" motions in parallel using a QuinticJointSpacePlanner
for each arm.

- HEAL:
    JointState:  /joint_states
    Command:     /velocity_controller/commands
    Joints:      joint1..joint6
    Home:        all zeros

- Franka:
    JointState:  /NS_1/franka/joint_states
    Command:     /NS_1/joint_velocity_controller/commands
    Joints:      fr3_joint1..fr3_joint7
    Home:        [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
"""

from __future__ import annotations

import os
import sys
from typing import Optional, Dict, List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ---------------------------------------------------------------------------
# Make sure we can import from RC-DS/utils even when running this script directly
# ---------------------------------------------------------------------------
PROJECT_ROOT = os.path.dirname(
    os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )
)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from utils.quintic_planner import QuinticJointSpacePlanner  # type: ignore


# ===========================================================================
# Base class for "go home" via quintic planning
# ===========================================================================

class BaseGoHome(Node):
    def __init__(
        self,
        node_name: str,
        *,
        joint_state_topic: str,
        command_topic: str,
        joint_names: List[str],
        q_home: np.ndarray,
        total_time: float,
        rate_hz: float,
    ):
        super().__init__(node_name)

        self.joint_state_topic = joint_state_topic
        self.command_topic = command_topic
        self.joint_names = joint_names
        self.n_joints = len(self.joint_names)

        self.q_home = q_home.astype(float).reshape(self.n_joints)

        self.total_time = float(total_time)
        self.dt = 1.0 / float(rate_hz)
        self.planner: Optional[QuinticJointSpacePlanner] = None
        self.t0: Optional[float] = None

        self._latest_js: Optional[JointState] = None
        self._name_to_idx: Dict[str, int] = {}
        self._q_start: Optional[np.ndarray] = None
        self._done = False
        self._settle_start: Optional[float] = None

        # Tail-correction + termination parameters
        self.kp = 2.0            # proportional gain for tail correction (rad/s per rad)
        self.q_tol = 0.01        # rad
        self.settle_time = 0.25  # seconds within tolerance before declaring done
        self.vmax = 0.49         # rad/s, safety clamp

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(Float64MultiArray, self.command_topic, qos)
        self._timer = self.create_timer(self.dt, self._on_timer)

        self.get_logger().info(
            f"{node_name} started.\n"
            f"  joint_state_topic = {self.joint_state_topic}\n"
            f"  command_topic     = {self.command_topic}\n"
            f"  joint_names       = {self.joint_names}\n"
            f"  q_home            = {self.q_home.tolist()}\n"
            f"  total_time        = {self.total_time} s\n"
            f"  rate              = {rate_hz} Hz"
        )

    # ------------------------------------------------------------------
    # Callbacks / helpers
    # ------------------------------------------------------------------

    def joint_state_callback(self, msg: JointState):
        self._latest_js = msg
        self._name_to_idx = {n: i for i, n in enumerate(msg.name)}

    def _extract_q_in_order(self, msg: JointState) -> np.ndarray:
        q = np.zeros(self.n_joints, dtype=float)
        missing = []
        for i, name in enumerate(self.joint_names):
            idx = self._name_to_idx.get(name, None)
            if idx is None:
                missing.append(name)
                q[i] = 0.0
            else:
                q[i] = float(msg.position[idx])

        if missing:
            self.get_logger().warn(
                f"Missing joints in JointState: {missing}. "
                "Using 0.0 for those entries."
            )
        return q

    def _maybe_init_planner(self, now_sec: float):
        if self.planner is not None or self._latest_js is None:
            return

        q_start = self._extract_q_in_order(self._latest_js)
        self._q_start = q_start.copy()

        self.planner = QuinticJointSpacePlanner(
            q_start=self._q_start,
            q_goal=self.q_home,
            duration=self.total_time,
        )
        self.t0 = now_sec

        self.get_logger().info(
            "Initialized QuinticJointSpacePlanner.\n"
            f"  q_start = {self._q_start}\n"
            f"  q_goal  = {self.q_home}\n"
            f"  T       = {self.total_time}"
        )

    def _publish_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * self.n_joints
        self.cmd_pub.publish(msg)

    def is_done(self) -> bool:
        return self._done

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _on_timer(self):
        if self._done:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        self._maybe_init_planner(now)

        if self.planner is None or self.t0 is None:
            return

        t = now - self.t0

        # Need current joint position for error/tail correction
        if self._latest_js is None:
            return
        q_cur = self._extract_q_in_order(self._latest_js)

        if t < self.total_time:
            # Follow planned quintic trajectory
            q_plan, qd_plan, _ = self.planner.evaluate(t)
            qd_cmd = qd_plan
        else:
            # Tail correction after nominal duration
            err = self.q_home - q_cur
            qd_cmd = self.kp * err

        # Clamp velocities for safety
        qd_cmd = np.clip(qd_cmd, -self.vmax, self.vmax)

        msg = Float64MultiArray()
        msg.data = qd_cmd.tolist()
        self.cmd_pub.publish(msg)

        # Tolerance-based termination with settle time
        max_err = float(np.max(np.abs(self.q_home - q_cur)))
        if max_err <= self.q_tol:
            if self._settle_start is None:
                self._settle_start = now
            elif (now - self._settle_start) >= self.settle_time:
                self._done = True
                self.get_logger().info(
                    f"[{self.get_name()}] Converged: max|q_home - q|={max_err:.4f} <= {self.q_tol}. Stopping."
                )
        else:
            # Reset settle timer if we leave the tolerance band
            self._settle_start = None


# ===========================================================================
# HEAL + Franka concrete subclasses
# ===========================================================================

class HealGoHome(BaseGoHome):
    def __init__(self):
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        q_home = np.zeros(len(joint_names), dtype=float)

        super().__init__(
            node_name="heal_go_home",
            joint_state_topic="/joint_states",
            command_topic="/velocity_controller/commands",
            joint_names=joint_names,
            q_home=q_home,
            total_time=10.0,   # seconds
            rate_hz=200.0,
        )


class FrankaGoHome(BaseGoHome):
    def __init__(self):
        joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        # Standard Franka "home" as per your controllers.yaml
        q_home = np.array(
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            dtype=float,
        )

        super().__init__(
            node_name="franka_go_home",
            joint_state_topic="/NS_1/franka/joint_states",
            command_topic="/NS_1/joint_velocity_controller/commands",
            joint_names=joint_names,
            q_home=q_home,
            total_time=8.0,   # seconds
            rate_hz=200.0,
        )


# ===========================================================================
# main
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)

    heal_node = HealGoHome()
    franka_node = FrankaGoHome()

    executor = MultiThreadedExecutor()
    executor.add_node(heal_node)
    executor.add_node(franka_node)

    try:
        # Spin until both nodes report done
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            if heal_node.is_done() and franka_node.is_done():
                break
    except KeyboardInterrupt:
        pass
    finally:
        for node in (heal_node, franka_node):
            try:
                node._publish_zero()
            except Exception:
                pass
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
