"""Rotation / quaternion helpers (numpy only, quaternions are [x, y, z, w])."""
from __future__ import annotations

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n = np.where(n < 1e-12, 1.0, n)
    out = q / n
    return out


def quat_conj(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return np.concatenate([-q[..., :3], q[..., 3:4]], axis=-1)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    x1, y1, z1, w1 = np.moveaxis(a, -1, 0)
    x2, y2, z2, w2 = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        axis=-1,
    )


def quat_log(q: np.ndarray) -> np.ndarray:
    """Unit quaternion -> rotation vector (angle * axis), shortest arc."""
    q = quat_normalize(q)
    q = np.where(q[..., 3:4] < 0.0, -q, q)
    v = q[..., :3]
    s = np.linalg.norm(v, axis=-1, keepdims=True)
    w = np.clip(q[..., 3:4], -1.0, 1.0)
    angle = 2.0 * np.arctan2(s, w)
    scale = np.where(s < 1e-12, 2.0, angle / np.where(s < 1e-12, 1.0, s))
    return v * scale


def quat_exp(r: np.ndarray) -> np.ndarray:
    """Rotation vector -> unit quaternion."""
    r = np.asarray(r, dtype=float)
    angle = np.linalg.norm(r, axis=-1, keepdims=True)
    half = 0.5 * angle
    k = np.where(angle < 1e-12, 0.5, np.sin(half) / np.where(angle < 1e-12, 1.0, angle))
    return np.concatenate([r * k, np.cos(half)], axis=-1)


def quat_error(q_target: np.ndarray, q_current: np.ndarray) -> np.ndarray:
    """Rotation vector e such that exp(e) * q_current = q_target (world frame)."""
    return quat_log(quat_mul(q_target, quat_conj(q_current)))


def quat_make_continuous(qs: np.ndarray) -> np.ndarray:
    """Flip signs so consecutive quaternions are in the same hemisphere."""
    qs = quat_normalize(np.array(qs, dtype=float, copy=True))
    for i in range(1, len(qs)):
        if np.dot(qs[i], qs[i - 1]) < 0.0:
            qs[i] = -qs[i]
    return qs


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    if np.dot(q0, q1) < 0.0:
        q1 = -q1
    d = quat_log(quat_mul(q1, quat_conj(q0)))
    return quat_normalize(quat_mul(quat_exp(u * d), q0))


def quat_interp(s_knots: np.ndarray, qs: np.ndarray, s_query: np.ndarray) -> np.ndarray:
    """Piecewise SLERP of a quaternion sequence defined at monotonic s_knots."""
    s_knots = np.asarray(s_knots, dtype=float)
    s_query = np.atleast_1d(np.asarray(s_query, dtype=float))
    qs = quat_make_continuous(qs)
    idx = np.clip(np.searchsorted(s_knots, s_query, side="right") - 1, 0, len(s_knots) - 2)
    out = np.empty((len(s_query), 4))
    for k, (i, s) in enumerate(zip(idx, s_query)):
        ds = s_knots[i + 1] - s_knots[i]
        u = 0.0 if ds <= 0 else float(np.clip((s - s_knots[i]) / ds, 0.0, 1.0))
        out[k] = slerp(qs[i], qs[i + 1], u)
    return quat_make_continuous(out)


def smooth_quaternions(qs: np.ndarray, sigma_samples: float) -> np.ndarray:
    """Gaussian smoothing of a quaternion sequence in the tangent space of a running mean.

    Works well for the small rotation changes seen when hand-guiding a probe.
    """
    qs = quat_make_continuous(qs)
    if sigma_samples <= 0 or len(qs) < 3:
        return qs
    # express every quat relative to the first, smooth rotation vectors, map back
    ref = qs[0]
    rv = quat_log(quat_mul(qs, quat_conj(np.broadcast_to(ref, qs.shape))))
    rv_s = gaussian_filter1d_edge(rv, sigma_samples)
    return quat_make_continuous(quat_mul(quat_exp(rv_s), np.broadcast_to(ref, qs.shape)))


def gaussian_filter1d_edge(x: np.ndarray, sigma: float) -> np.ndarray:
    """Gaussian filter along axis 0 with edge (nearest) padding. numpy only."""
    x = np.asarray(x, dtype=float)
    if sigma <= 0:
        return x.copy()
    r = max(1, int(round(4 * sigma)))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma) ** 2)
    k /= k.sum()
    pad = np.concatenate([np.repeat(x[:1], r, axis=0), x, np.repeat(x[-1:], r, axis=0)], axis=0)
    if x.ndim == 1:
        return np.convolve(pad, k, mode="valid")
    return np.stack([np.convolve(pad[:, j], k, mode="valid") for j in range(x.shape[1])], axis=1)


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return quat_normalize(np.array([x, y, z, w]))


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
