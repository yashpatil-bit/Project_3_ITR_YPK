#!/usr/bin/env python3
"""Print one block of text with everything needed to debug remotely: code version, key config
values, recorded demos, processed trajectories and the latest run summaries.
Copy the whole output and paste it into the chat.

    python3 scripts/report.py            # last 3 runs
    python3 scripts/report.py --runs 10
"""
import argparse
import glob
import json
import os
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def sh(cmd):
    try:
        return subprocess.run(cmd, cwd=ROOT, shell=True, capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception as e:
        return f"({e})"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=3)
    a = ap.parse_args()
    print("=" * 20, "WIRE-TRACE REPORT", "=" * 20)
    print("code version :", sh("git log --oneline -1"), "| branch:", sh("git rev-parse --abbrev-ref HEAD"))
    changed = sh("git status --short")
    print("local changes:", changed.replace("\n", ", ") if changed else "none")
    try:
        import yaml
        cfg = yaml.safe_load(open(os.path.join(ROOT, "config", "heal_wire_trace.yaml")))
        r, p, c = cfg["robot"], cfg["processing"], cfg["controller"]
        print(f"config       : tip_link={r['tip_link']} tool_xyz={r['tool_xyz']} joints={r['joint_names']}")
        print(f"               v_max={p['v_max']} a_max={p['a_max']} a_lat={p['a_lat_max']} w_max={p['w_max']} "
              f"kp_pos={c['kp_pos']} kp_ori={c['kp_ori']} qdot_max={c['qdot_max']}")
        print(f"               workspace_min={c['workspace_min']} workspace_max={c['workspace_max']}")
    except Exception as e:
        print("config       : could not read:", e)
    for folder in ("demos", "processed"):
        files = sorted(glob.glob(os.path.join(ROOT, "data", folder, "*.npz")), key=os.path.getmtime)
        print(f"{folder:13s}: " + (", ".join(os.path.basename(f) for f in files[-8:]) if files else "none"))
    runs = sorted(glob.glob(os.path.join(ROOT, "data", "runs", "*.json")), key=os.path.getmtime)[-a.runs:]
    print(f"last {len(runs)} runs:")
    for f in runs:
        s = json.load(open(f))
        print(f"  {os.path.basename(f)[:-5]}: {s.get('status')} {s.get('reason', '')} | traj={os.path.basename(str(s.get('traj')))} "
              f"speed={s.get('speed_scale')} rev={s.get('reverse')} dry={s.get('dry_run')} | time={s.get('duration_s', 0):.2f}s "
              f"maxerr={s.get('max_pos_err_mm', float('nan')):.2f}mm meanerr={s.get('mean_pos_err_mm', float('nan')):.2f}mm "
              f"ori={s.get('max_ori_err_deg', float('nan')):.1f}deg contacts={s.get('n_contacts')}")
    print("=" * 59)


if __name__ == "__main__":
    main()
