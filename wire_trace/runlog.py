"""Per-run logging + evaluation metrics (shared by the real replay node and the simulator)."""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RunLog:
    meta: dict = field(default_factory=dict)
    rows: dict = field(default_factory=lambda: {k: [] for k in RunLog.FIELDS})
    contacts: list = field(default_factory=list)       # wall times (s since start) of contact events

    FIELDS = ("t", "tau", "q", "qdot_cmd", "pos", "quat", "p_d", "q_d", "e_pos", "e_track", "e_ori", "k", "sigma_min", "effort")

    def add(self, **kw):
        for k in self.FIELDS:
            v = kw.get(k, np.nan)
            self.rows[k].append(np.asarray(v, dtype=float))

    def arrays(self) -> dict:
        out = {}
        for k, v in self.rows.items():
            if len(v) == 0:
                out[k] = np.zeros(0)
                continue
            try:
                out[k] = np.stack(v)
            except ValueError:          # ragged (e.g. effort missing on some ticks)
                out[k] = np.array([np.nan] * len(v))
        return out

    def save(self, directory: str, name: str | None = None) -> str:
        os.makedirs(directory, exist_ok=True)
        name = name or time.strftime("run_%Y%m%d_%H%M%S")
        path = os.path.join(directory, name)
        arr = self.arrays()
        summary = summarize(arr, self.meta, self.contacts)
        np.savez(path + ".npz", contacts=np.array(self.contacts, dtype=float),
                 meta=json.dumps(self.meta), **arr)
        with open(path + ".json", "w") as f:
            json.dump(summary, f, indent=2)
        return path


def summarize(arr: dict, meta: dict, contacts) -> dict:
    t = arr.get("t", np.zeros(0))
    out = dict(meta)
    if len(t) == 0:
        out.update(duration_s=0.0, success=False)
        return out
    e = arr["e_pos"]
    qd = arr["qdot_cmd"]
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.01
    # smoothness: RMS of commanded joint jerk (d^2 qdot / dt^2) and accel
    if len(qd) > 3:
        acc = np.diff(qd, axis=0) / dt
        jerk = np.diff(acc, axis=0) / dt
        rms_acc = float(np.sqrt(np.nanmean(acc ** 2)))
        rms_jerk = float(np.sqrt(np.nanmean(jerk ** 2)))
    else:
        rms_acc = rms_jerk = float("nan")
    out.update(
        duration_s=float(t[-1] - t[0]),
        max_pos_err_mm=float(np.nanmax(e) * 1000),
        mean_pos_err_mm=float(np.nanmean(e) * 1000),
        max_ori_err_deg=float(np.degrees(np.nanmax(arr["e_ori"]))),
        rms_joint_acc=rms_acc,
        rms_joint_jerk=rms_jerk,
        n_contacts=len(contacts),
        contact_times_s=[float(c) for c in contacts],
    )
    out["success"] = bool(meta.get("status") == "done" and len(contacts) == 0)
    return out


def repeatability(paths: list[np.ndarray], n: int = 400) -> dict:
    """Paths = list of (N_i, 3) actual tool paths. Resample each by arc length to n points
    and report the spread between runs."""
    if len(paths) < 2:
        return {}
    rs = []
    for p in paths:
        s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        u = np.linspace(0, s[-1], n)
        rs.append(np.stack([np.interp(u, s, p[:, k]) for k in range(3)], axis=1))
    rs = np.stack(rs)
    mean = rs.mean(axis=0)
    dev = np.linalg.norm(rs - mean, axis=2)
    return {"max_dev_from_mean_mm": float(dev.max() * 1000), "mean_dev_from_mean_mm": float(dev.mean() * 1000)}
