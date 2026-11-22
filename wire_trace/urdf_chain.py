"""Pure-numpy forward kinematics / Jacobian for a serial URDF chain (no ROS, no KDL).

The HEAL chain (base_link -> rq_tool_tip) is stored in config/heal_chain.json (from
scripts/dump_chain.py on the lab PC), so placement searches and simulations can run on any
laptop with the robot's REAL geometry and joint limits.
"""
from __future__ import annotations

import json
import os

import numpy as np

from .so3 import rotmat_to_quat

DEFAULT_CHAIN = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "heal_chain.json")


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K


class ChainKinematics:
    """Same interface as wire_trace.kdl_kinematics.KDLKinematics (fk, jacobian, frame_positions,
    lower, upper, joint_names, n)."""

    def __init__(self, chain_file: str = DEFAULT_CHAIN, tool_xyz=(0, 0, 0), tool_rpy=(0, 0, 0)):
        with open(chain_file) as f:
            self.joints = json.load(f)
        self.tool = np.eye(4)
        self.tool[:3, :3] = _rpy(*tool_rpy)
        self.tool[:3, 3] = tool_xyz
        mov = [j for j in self.joints if j["type"] != "fixed"]
        self.joint_names = [j["name"] for j in mov]
        self.n = len(mov)
        self.lower = np.array([j["lower"] for j in mov], float)
        self.upper = np.array([j["upper"] for j in mov], float)
        self.vel_limit = np.array([j["velocity"] for j in mov], float)
        self._origin = []
        for j in self.joints:
            T = np.eye(4)
            T[:3, :3] = _rpy(*j["rpy"])
            T[:3, 3] = j["xyz"]
            self._origin.append(T)

    def _frames(self, q):
        """Returns list of (T_after_joint, axis_world, origin_world, movable)."""
        T = np.eye(4)
        out = []
        k = 0
        for j, To in zip(self.joints, self._origin):
            T = T @ To
            if j["type"] != "fixed":
                ax_w = T[:3, :3] @ np.asarray(j["axis"], float)
                org = T[:3, 3].copy()
                R = np.eye(4)
                R[:3, :3] = _axis_angle(j["axis"], q[k])
                T = T @ R
                out.append((T.copy(), ax_w / np.linalg.norm(ax_w), org, True))
                k += 1
            else:
                out.append((T.copy(), None, None, False))
        return out, T @ self.tool

    def fk(self, q):
        _, T = self._frames(np.asarray(q, float))
        return T[:3, 3].copy(), rotmat_to_quat(T[:3, :3])

    def jacobian(self, q):
        frames, T = self._frames(np.asarray(q, float))
        pe = T[:3, 3]
        J = np.zeros((6, self.n))
        k = 0
        for _, ax, org, mov in frames:
            if mov:
                J[:3, k] = np.cross(ax, pe - org)
                J[3:, k] = ax
                k += 1
        return J

    def frame_positions(self, q):
        frames, T = self._frames(np.asarray(q, float))
        pts = [F[:3, 3] for F, *_ in frames]
        if not np.allclose(self.tool, np.eye(4)):
            pts.append(T[:3, 3])
        return np.array(pts)
