"""KDL forward kinematics + Jacobian for the HEAL arm, built from the URDF that the robot
publishes on /robot_description (same approach as the starter code), plus an optional
fixed tool offset so we control the *probe tip*, not the flange.

Requires (lab PC, ROS 2 Humble):  python3-pykdl, ros-humble-kdl-parser-py (kdl_parser_py),
urdf_parser_py.
"""
from __future__ import annotations

from typing import Sequence

import numpy as np
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF


def _rpy_to_kdl(rpy):
    return kdl.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2]))


class KDLKinematics:
    def __init__(self, urdf_xml: str, base_link: str, tip_link: str,
                 tool_xyz: Sequence[float] = (0, 0, 0), tool_rpy: Sequence[float] = (0, 0, 0)):
        robot = URDF.from_xml_string(urdf_xml)
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            raise RuntimeError("Failed to build KDL tree from URDF")
        chain = tree.getChain(base_link, tip_link)
        if chain.getNrOfJoints() == 0:
            raise RuntimeError(f"KDL chain {base_link} -> {tip_link} has 0 joints. "
                               "Check link names with: ros2 run tf2_tools view_frames")
        if any(abs(v) > 0 for v in list(tool_xyz) + list(tool_rpy)):
            chain.addSegment(kdl.Segment("probe_tip", kdl.Joint(),
                                         kdl.Frame(_rpy_to_kdl(tool_rpy), kdl.Vector(*map(float, tool_xyz)))))
        self.chain = chain
        self.n = chain.getNrOfJoints()
        self.joint_names = []
        for i in range(chain.getNrOfSegments()):
            j = chain.getSegment(i).getJoint()
            if j.getTypeName() not in ("None", "Fixed"):
                self.joint_names.append(j.getName())
        self.fk_solver = kdl.ChainFkSolverPos_recursive(chain)
        self.jac_solver = kdl.ChainJntToJacSolver(chain)

        lo, hi, vel = [], [], []
        for jn in self.joint_names:
            j = robot.joint_map.get(jn)
            lim = getattr(j, "limit", None) if j is not None else None
            lo.append(getattr(lim, "lower", None) if lim is not None and lim.lower is not None else -np.pi * 2)
            hi.append(getattr(lim, "upper", None) if lim is not None and lim.upper is not None else np.pi * 2)
            vel.append(getattr(lim, "velocity", None) if lim is not None and lim.velocity else np.nan)
        self.lower = np.array(lo, float)
        self.upper = np.array(hi, float)
        self.vel_limit = np.array(vel, float)

    def _jnt(self, q):
        a = kdl.JntArray(self.n)
        for i in range(self.n):
            a[i] = float(q[i])
        return a

    def fk(self, q):
        f = kdl.Frame()
        self.fk_solver.JntToCart(self._jnt(q), f)
        x, y, z, w = f.M.GetQuaternion()
        qq = np.array([x, y, z, w], float)
        return np.array([f.p.x(), f.p.y(), f.p.z()], float), qq / np.linalg.norm(qq)

    def jacobian(self, q):
        J = kdl.Jacobian(self.n)
        self.jac_solver.JntToJac(self._jnt(q), J)
        return np.array([[J[i, j] for j in range(self.n)] for i in range(6)], float)
