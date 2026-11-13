"""Small rclpy helpers shared by the scripts in scripts/ (ROS 2 Humble)."""
from __future__ import annotations

import sys
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .kdl_kinematics import KDLKinematics

LATCHED_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class RobotIO:
    """Joint-state cache + URDF/KDL + velocity publisher, attached to an existing node."""

    def __init__(self, node: Node, cfg: dict, need_command: bool = True):
        r = cfg["robot"]
        self.node = node
        self.cfg = cfg
        self.cmd_names = list(r["joint_names"])
        self.timeout = float(r.get("joint_state_timeout", 0.1))
        self._js: Optional[JointState] = None
        self._js_wall = 0.0
        self._urdf: Optional[str] = None
        self.kin: Optional[KDLKinematics] = None
        node.create_subscription(JointState, r["joint_state_topic"], self._js_cb, qos_profile_sensor_data)
        self._urdf_sub = node.create_subscription(String, r["robot_description_topic"], self._urdf_cb, LATCHED_QOS)
        self.pub = node.create_publisher(Float64MultiArray, r["velocity_command_topic"], 1) if need_command else None

    # -- callbacks --------------------------------------------------------
    def _js_cb(self, msg: JointState):
        self._js = msg
        self._js_wall = time.monotonic()

    def _urdf_cb(self, msg: String):
        if self._urdf is None and "<robot" in msg.data:
            self._urdf = msg.data

    # -- setup ------------------------------------------------------------
    def wait_ready(self, timeout: float = 10.0, urdf_file: Optional[str] = None) -> None:
        if urdf_file:
            with open(urdf_file) as f:
                self._urdf = f.read()
        t0 = time.monotonic()
        while rclpy.ok() and (self._urdf is None or self._js is None):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                missing = []
                if self._urdf is None:
                    missing.append(f"URDF on {self.cfg['robot']['robot_description_topic']} (or pass --urdf file)")
                if self._js is None:
                    missing.append(f"JointState on {self.cfg['robot']['joint_state_topic']}")
                raise RuntimeError("Timed out waiting for: " + "; ".join(missing))
        r = self.cfg["robot"]
        self.kin = KDLKinematics(self._urdf, r["base_link"], r["tip_link"],
                                 r.get("tool_xyz", [0, 0, 0]), r.get("tool_rpy", [0, 0, 0]))
        missing = [n for n in self.kin.joint_names if n not in self._js.name]
        if missing:
            raise RuntimeError(f"KDL joints {missing} not present in /joint_states names {list(self._js.name)}")
        bad = [n for n in self.cmd_names if n not in self.kin.joint_names]
        if bad:
            raise RuntimeError(f"config robot.joint_names {bad} not in KDL chain {self.kin.joint_names}")

    # -- state ------------------------------------------------------------
    def stale(self) -> bool:
        return self._js is None or (time.monotonic() - self._js_wall) > self.timeout

    def q(self) -> np.ndarray:
        """Joint positions in KDL chain order."""
        js = self._js
        idx = {n: i for i, n in enumerate(js.name)}
        return np.array([js.position[idx[n]] for n in self.kin.joint_names], float)

    def effort(self) -> np.ndarray:
        js = self._js
        if js is None or len(js.effort) != len(js.name):
            return np.full(self.kin.n, np.nan)
        idx = {n: i for i, n in enumerate(js.name)}
        return np.array([js.effort[idx[n]] for n in self.kin.joint_names], float)

    def publish_qdot(self, qdot_chain: np.ndarray) -> None:
        """qdot in KDL chain order -> Float64MultiArray in robot.joint_names order."""
        if self.pub is None:
            return
        cidx = {n: i for i, n in enumerate(self.kin.joint_names)}
        msg = Float64MultiArray()
        msg.data = [float(qdot_chain[cidx[n]]) for n in self.cmd_names]
        self.pub.publish(msg)

    def publish_zero(self, repeats: int = 5) -> None:
        if self.pub is None:
            return
        msg = Float64MultiArray()
        msg.data = [0.0] * len(self.cmd_names)
        for _ in range(repeats):
            self.pub.publish(msg)
            time.sleep(0.005)


class KeyReader:
    """Background stdin line reader (Enter-terminated commands) for interactive scripts."""

    def __init__(self):
        self.lines: list[str] = []
        self._lock = threading.Lock()
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def _run(self):
        for line in sys.stdin:
            with self._lock:
                self.lines.append(line.strip())

    def pop(self) -> Optional[str]:
        with self._lock:
            return self.lines.pop(0) if self.lines else None


def confirm(prompt: str) -> bool:
    try:
        ans = input(prompt)
    except EOFError:
        return False
    return ans.strip().lower() in ("", "y", "yes")
