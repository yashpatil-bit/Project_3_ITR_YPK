"""Synthetic buzz-wire + noisy 'human' demonstration, for offline testing only."""
from __future__ import annotations

import numpy as np

from .sim_arm import SimArm
from .so3 import quat_exp, quat_mul, quat_normalize
from .trajectory import Demo


def wire_path(n=600, center=(-0.55, -0.10, 0.30), width=0.40, height=0.06, depth=0.03, wiggles=2):
    """A vertical (x-z plane) wavy wire ~40 cm long with a small out-of-plane twist."""
    u = np.linspace(0, 1, n)
    cx, cy, cz = center
    x = cx + 0.0 * u
    y = cy + width * (u - 0.5)
    z = cz + height * np.sin(2 * np.pi * wiggles * u) * np.sin(np.pi * u)
    x = x + depth * np.sin(2 * np.pi * 1.5 * u)
    return np.stack([x, y, z], axis=1)


def human_demo(arm: SimArm, wire: np.ndarray, rate=50.0, mean_speed=0.03, tremor=0.0008,
               seed=1, base_quat=None, pause_at=(0.3, 0.7)) -> Demo:
    """Walk along the wire with uneven speed, tremor and a couple of pauses; record like
    scripts/record_demo.py would (joint angles + FK pose)."""
    rng = np.random.default_rng(seed)
    seg = np.linalg.norm(np.diff(wire, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    L = s[-1]
    if base_quat is None:
        _, base_quat = arm.fk()
    dt = 1.0 / rate
    t, sp = 0.0, 0.0
    T, Q, P, R = [], [], [], []
    trem = np.zeros(3)
    paused = {p: False for p in pause_at}
    p0 = np.array([np.interp(0, s, wire[:, k]) for k in range(3)])
    arm.ik_step_to(p0, base_quat)
    while sp < L:
        frac = sp / L
        speed = mean_speed * (0.6 + 0.8 * abs(np.sin(7 * frac + 1.3)))
        for pa in pause_at:
            if not paused[pa] and frac > pa:
                paused[pa] = True
                for _ in range(int(0.8 * rate)):      # 0.8 s pause
                    p, qq = arm.fk()
                    T.append(t); Q.append(arm.measured_q()); P.append(p); R.append(qq)
                    t += dt
        sp = min(L, sp + speed * dt)
        target = np.array([np.interp(sp, s, wire[:, k]) for k in range(3)])
        trem = 0.9 * trem + rng.normal(0, tremor * 0.44, 3)
        twist = quat_exp(rng.normal(0, 0.002, 3) + 0.05 * np.array([0, 0, np.sin(4 * frac)]))
        arm.ik_step_to(target + trem, quat_normalize(quat_mul(twist, base_quat)), iters=30)
        p, qq = arm.fk()
        T.append(t); Q.append(arm.measured_q()); P.append(p); R.append(qq)
        t += dt
    return Demo(t=np.array(T), q=np.array(Q), pos=np.array(P), quat=np.array(R),
                joint_names=list(arm.joint_names), meta={"synthetic": True, "rate_hz": rate})


def serpentine_wire(center=(-0.50, -0.05, 0.25), n_loops=3, width=0.30, height=0.12, bend_r=0.02, ds=0.001):
    """Planar wire like the lab's buzz-wire: vertical straight runs joined by U-bends
    (alternating top/bottom), in the y-z plane. Returns (points (N,3), tangent angle phi (N,)).
    phi = direction of travel in the y-z plane (0 = +y, pi/2 = +z)."""
    cy, cz = center[1] - width / 2, center[2]
    pts, phis = [], []
    pitch = width / (2 * n_loops)
    r = min(bend_r, pitch / 2)
    y, z = cy, cz - height / 2
    # lead-in going right
    def add_line(p0, p1, phi):
        L = np.linalg.norm(np.subtract(p1, p0))
        for u in np.linspace(0, 1, max(2, int(L / ds)), endpoint=False):
            pts.append(np.add(p0, u * np.subtract(p1, p0))); phis.append(phi)

    def add_arc(cyz, r, a0, a1):
        n = max(4, int(abs(a1 - a0) * r / ds))
        for a in np.linspace(a0, a1, n, endpoint=False):
            pts.append((cyz[0] + r * np.cos(a), cyz[1] + r * np.sin(a)))
            phis.append(a + (np.pi / 2 if a1 > a0 else -np.pi / 2))

    add_line((y, z), (y + pitch / 2 - r, z), 0.0)
    y += pitch / 2 - r
    add_arc((y, z + r), r, -np.pi / 2, 0.0)            # turn up
    y, z = y + r, z + r
    up = True
    for i in range(2 * n_loops - 1):
        z_end = cz + height / 2 - r if up else cz - height / 2 + r
        add_line((y, z), (y, z_end), np.pi / 2 if up else -np.pi / 2)
        z = z_end
        if up:
            add_arc((y + r, z), r, np.pi, 0.0)             # over the top, now going down
        else:
            add_arc((y + r, z), r, -np.pi, 0.0)            # under the bottom, now going up
        y += 2 * r
        up = not up
    z_end = cz + height / 2 - r if up else cz - height / 2 + r
    add_line((y, z), (y, z_end), np.pi / 2 if up else -np.pi / 2)
    P = np.array(pts)
    xyz = np.column_stack([np.full(len(P), center[0]), P[:, 0], P[:, 1]])
    return xyz, np.unwrap(np.array(phis))


def ring_demo(arm: SimArm, rate=50.0, mean_speed=0.02, tremor=0.0005, seed=4) -> tuple[Demo, np.ndarray]:
    """Hand-guided demo of the lab wire: probe handle points along -x (into the wire plane),
    the ring rolls about that axis to stay perpendicular to the wire."""
    wire, phi = serpentine_wire()
    rng = np.random.default_rng(seed)
    base = quat_exp(np.array([0.0, -np.pi / 2, 0.0]))     # tool z -> world -x
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(wire, axis=0), axis=1))])
    dt = 1.0 / rate
    T, Q, P, R = [], [], [], []
    sp, t, trem = 0.0, 0.0, np.zeros(3)
    q_goal = quat_mul(quat_exp(np.array([-phi[0], 0, 0])), base)
    arm.q = np.array([0.0, -1.0, 1.6, -0.6, 1.57, 0.0])
    arm.ik_step_to(wire[0], q_goal, iters=500)
    while sp < s[-1]:
        sp = min(s[-1], sp + mean_speed * dt * (0.6 + 0.8 * rng.random()))
        p = np.array([np.interp(sp, s, wire[:, k]) for k in range(3)])
        a = np.interp(sp, s, phi)
        trem = 0.9 * trem + rng.normal(0, tremor * 0.44, 3)
        qd = quat_mul(quat_exp(np.array([-(a - phi[0]), 0, 0]) + rng.normal(0, 0.003, 3)), q_goal)
        arm.ik_step_to(p + trem, quat_normalize(qd), iters=30)
        pp, qq = arm.fk()
        T.append(t); Q.append(arm.measured_q()); P.append(pp); R.append(qq)
        t += dt
    return Demo(t=np.array(T), q=np.array(Q), pos=np.array(P), quat=np.array(R),
                joint_names=list(arm.joint_names), meta={"synthetic": "serpentine"}), wire
