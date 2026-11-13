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
