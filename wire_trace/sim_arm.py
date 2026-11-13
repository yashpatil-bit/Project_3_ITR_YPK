"""Tiny kinematic simulator of a generic 6-DOF arm (UR5-like DH parameters).

It is NOT the HEAL arm - it only exists so the whole pipeline (processing, DLS-IK
tracking, safety, logging, Stage 2 learning) can be tested on a laptop with no
robot and no ROS. On the real robot the kinematics come from the URDF via KDL.
"""
from __future__ import annotations

import numpy as np

from .so3 import rotmat_to_quat

# standard DH: (d, a, alpha)
UR5_DH = [
    (0.089159, 0.0, np.pi / 2),
    (0.0, -0.425, 0.0),
    (0.0, -0.39225, 0.0),
    (0.10915, 0.0, np.pi / 2),
    (0.09465, 0.0, -np.pi / 2),
    (0.0823, 0.0, 0.0),
]


def _dh(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1.0]])


class SimArm:
    joint_names = [f"joint{i}" for i in range(1, 7)]

    def __init__(self, q0=None, tool_offset=(0.0, 0.0, 0.10), lag=0.02, noise=0.0, seed=0):
        self.dh = UR5_DH
        self.q = np.array(q0 if q0 is not None else [0.0, -1.2, 1.4, -1.8, -1.57, 0.0], float)
        self.tool = np.eye(4)
        self.tool[:3, 3] = tool_offset
        self.lag = lag                       # first-order velocity response time constant
        self.noise = noise                   # joint encoder noise (rad)
        self.qdot = np.zeros(6)
        self.rng = np.random.default_rng(seed)
        self.lower = np.full(6, -2 * np.pi)
        self.upper = np.full(6, 2 * np.pi)

    def frames(self, q):
        T = np.eye(4)
        Ts = [T]
        for qi, (d, a, al) in zip(q, self.dh):
            T = T @ _dh(qi, d, a, al)
            Ts.append(T)
        return Ts

    def fk(self, q=None):
        q = self.q if q is None else q
        T = self.frames(q)[-1] @ self.tool
        return T[:3, 3].copy(), rotmat_to_quat(T[:3, :3])

    def jacobian(self, q=None):
        q = self.q if q is None else q
        Ts = self.frames(q)
        pe = (Ts[-1] @ self.tool)[:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            z = Ts[i][:3, 2]
            o = Ts[i][:3, 3]
            J[:3, i] = np.cross(z, pe - o)
            J[3:, i] = z
        return J

    def measured_q(self):
        if self.noise > 0:
            return self.q + self.rng.normal(0, self.noise, 6)
        return self.q.copy()

    def apply(self, qdot_cmd, dt):
        a = dt / max(self.lag, dt) if self.lag > 0 else 1.0
        self.qdot = self.qdot + a * (np.asarray(qdot_cmd) - self.qdot)
        self.q = self.q + self.qdot * dt

    def ik_step_to(self, pos, quat, iters=200):
        """Simple numerical IK used to place the simulated robot at a pose."""
        from .so3 import quat_error
        for _ in range(iters):
            p, qq = self.fk()
            e = np.concatenate([pos - p, quat_error(quat, qq)])
            if np.linalg.norm(e) < 1e-7:
                break
            J = self.jacobian()
            self.q = self.q + J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
        self.qdot[:] = 0
