#!/usr/bin/env python3
"""EVALUATION: table of all logged runs (time, success, contacts, errors, smoothness) and
repeatability across runs. Offline, no robot needed.

    python3 scripts/evaluate_runs.py                       # every run in data/runs
    python3 scripts/evaluate_runs.py --tag final           # only runs whose tag starts with "final"
    python3 scripts/evaluate_runs.py data/runs/final_*.json

Writes data/runs/summary.md (paste it into the README / slides).
"""
import argparse
import glob
import json
import os

import _path  # noqa: F401
import numpy as np

from wire_trace.config import data_dir
from wire_trace.runlog import repeatability


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--include-dry", action="store_true")
    a = ap.parse_args()
    files = a.files or sorted(glob.glob(os.path.join(data_dir("runs"), "*.json")))
    rows = []
    for f in files:
        with open(f) as fh:
            s = json.load(fh)
        if a.tag and not str(s.get("tag", "")).startswith(a.tag):
            continue
        if s.get("dry_run") and not a.include_dry:
            continue
        s["_file"] = os.path.splitext(f)[0]
        rows.append(s)
    if not rows:
        print("No runs found.")
        return

    hdr = "| run | speed | status | time [s] | contacts | max err [mm] | mean err [mm] | RMS jerk | success |"
    lines = [hdr, "|" + "---|" * 9]
    for s in rows:
        lines.append(
            f"| {os.path.basename(s['_file'])} | {s.get('speed_scale', 0):.2f} | {s.get('status')} "
            f"| {s.get('duration_s', 0):.2f} | {s.get('n_contacts', 0)} | {s.get('max_pos_err_mm', float('nan')):.2f} "
            f"| {s.get('mean_pos_err_mm', float('nan')):.2f} | {s.get('rms_joint_jerk', float('nan')):.2f} "
            f"| {'YES' if s.get('success') else 'no'} |")
    ok = [s for s in rows if s.get("success")]
    lines.append("")
    lines.append(f"**Success rate:** {len(ok)}/{len(rows)}")
    if ok:
        times = np.array([s["duration_s"] for s in ok])
        lines.append(f"**Successful run time:** best {times.min():.2f} s, mean {times.mean():.2f} s, std {times.std():.2f} s")
        paths = []
        for s in ok:
            d = np.load(s["_file"] + ".npz", allow_pickle=False)
            paths.append(d["pos"])
        rep = repeatability(paths)
        if rep:
            lines.append(f"**Repeatability (successful runs):** max deviation from mean path "
                         f"{rep['max_dev_from_mean_mm']:.2f} mm, mean {rep['mean_dev_from_mean_mm']:.2f} mm")
    out = "\n".join(lines)
    print(out)
    with open(os.path.join(data_dir("runs"), "summary.md"), "w") as f:
        f.write(out + "\n")
    print(f"\n-> {os.path.join(data_dir('runs'), 'summary.md')}")


if __name__ == "__main__":
    main()
