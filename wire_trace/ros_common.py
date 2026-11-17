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
        self.cmd_log: list = []
        self.n_clamped = 0

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
        if self.readings_fake():
            raise RuntimeError("all joints read exactly 0.0 - HEAL is probably in an ERROR state (fake readings). "
                               "Restart: ./base_rigid (go home), ./heal_server, then the velocity launch. See INSTRUCTIONS H.")
        missing = [n for n in self.kin.joint_names if n not in self._js.name]
        if missing:
            raise RuntimeError(f"KDL joints {missing} not present in /joint_states names {list(self._js.name)}")
        bad = [n for n in self.cmd_names if n not in self.kin.joint_names]
        if bad:
            raise RuntimeError(f"config robot.joint_names {bad} not in KDL chain {self.kin.joint_names}")

    # -- state ------------------------------------------------------------
    def readings_fake(self) -> bool:
        """True if every joint reads EXACTLY 0.0. HEAL publishes that after a robot error, even
        though the arm is somewhere else. Real encoder readings at home are never exactly 0."""
        js = self._js
        return js is not None and len(js.position) > 0 and all(float(v) == 0.0 for v in js.position)

    def stale(self) -> bool:
        """True if joint states are old OR look like the fake all-zero readings of an error state.
        Every moving script stops when this is True."""
        if self._js is None or (time.monotonic() - self._js_wall) > self.timeout:
            return True
        return self.readings_fake()

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
        """qdot in KDL chain order -> Float64MultiArray in robot.joint_names order.

        Last line of defence: every joint is limited to robot.qdot_hard_limit (direction
        preserving), whatever the calling script asked for. HEAL's own safety stops the robot
        with "Large Velocity for Joint" above its threshold.
        """
        if self.pub is None:
            return
        qdot_chain = np.asarray(qdot_chain, dtype=float)
        lim = float(self.cfg["robot"].get("qdot_hard_limit", 0.25))
        peak = float(np.max(np.abs(qdot_chain))) if qdot_chain.size else 0.0
        if peak > lim:
            qdot_chain = qdot_chain * (lim / peak)
            self.n_clamped += 1
        self.cmd_log.append((time.monotonic(), *qdot_chain.tolist()))
        if len(self.cmd_log) > 200000:
            self.cmd_log = self.cmd_log[-100000:]
        cidx = {n: i for i, n in enumerate(self.kin.joint_names)}
        msg = Float64MultiArray()
        msg.data = [float(qdot_chain[cidx[n]]) for n in self.cmd_names]
        self.pub.publish(msg)

    def save_cmd_log(self, path: str) -> None:
        """Save every published joint-velocity command (t, qdot...) for debugging."""
        if self.cmd_log:
            np.save(path, np.array(self.cmd_log))

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
