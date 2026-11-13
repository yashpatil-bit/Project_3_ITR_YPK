#!/usr/bin/env python3
"""STAGE 2 (bonus) - Iterative Learning Control: the robot learns the wire over trials.

Loop (each trial is a normal replay; this script runs offline in between):

    python3 scripts/replay.py data/processed/demo_01_v10.npz --speed 0.5 --tag ilc0
    python3 stage2/ilc_update.py data/processed/demo_01_v10.npz data/runs/ilc0_<time>.npz
        -> data/processed/demo_01_v10_ilc1.npz
    python3 scripts/go_to_start.py data/processed/demo_01_v10_ilc1.npz
    python3 scripts/replay.py data/processed/demo_01_v10_ilc1.npz --speed 0.5 --tag ilc1
    python3 stage2/ilc_update.py data/processed/demo_01_v10_ilc1.npz data/runs/ilc1_<time>.npz
    ...   (3-5 iterations, then raise --speed and repeat)

The final *_ilcN.npz is executed "in one go" for the demo.
Update only from runs that completed cleanly (status done); aborted runs only update the
part of the path they covered.
"""
import argparse
import json
import os
import re

import _path  # noqa: F401
import numpy as np

from wire_trace.learning import ILCParams, ilc_update
from wire_trace.trajectory import Trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj", help="trajectory that was executed")
    ap.add_argument("run", help="data/runs/<name>.npz of that execution")
    ap.add_argument("--gain", type=float, default=ILCParams.gain)
    ap.add_argument("--lead", type=int, default=ILCParams.lead)
    ap.add_argument("--q-sigma", type=float, default=ILCParams.q_sigma)
    ap.add_argument("--max-offset", type=float, default=ILCParams.max_offset)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    traj = Trajectory.load(a.traj)
    run = np.load(a.run, allow_pickle=False)
    meta = json.loads(str(run["meta"]))
    if meta.get("traj") and os.path.abspath(meta["traj"]) != os.path.abspath(a.traj):
        print(f"WARNING: run was recorded with {meta['traj']}, not {a.traj}")
    if meta.get("reverse"):
        raise SystemExit("Use forward runs for ILC (reverse runs have a different time axis).")
    if meta.get("dry_run"):
        raise SystemExit("Dry runs contain no real tracking error.")
    if meta.get("status") != "done":
        print(f"NOTE: run status '{meta.get('status')}' ({meta.get('reason')}) - only the covered part is updated.")

    new, st = ilc_update(traj, run["tau"], run["pos"],
                         ILCParams(gain=a.gain, lead=a.lead, q_sigma=a.q_sigma, max_offset=a.max_offset))
    base = re.sub(r"_ilc\d+$", "", os.path.splitext(a.traj)[0])
    out = a.out or f"{base}_ilc{st['iter']}.npz"
    new.meta["stage2"] = "ilc"
    new.save(out)
    print(f"ILC iteration {st['iter']}: covered {st['coverage']*100:.0f}% of the path, "
          f"time-aligned error rms {st['rms_err_mm']:.2f} mm / max {st['max_err_mm']:.2f} mm, "
          f"learned offset max {st['max_offset_mm']:.2f} mm")
    print(f"-> {out}")


if __name__ == "__main__":
    main()
