#!/usr/bin/env python3
"""STAGE 2 (bonus) - adapt a learned trajectory when the wire stand has been moved/rotated.

1. Before moving the stand: capture >= 3 well-separated reference points on the stand
       python3 scripts/capture_points.py --out data/ref_A.npy
2. Move the stand, capture the SAME points in the SAME order
       python3 scripts/capture_points.py --out data/ref_B.npy
3. Transform the trajectory (Kabsch least-squares rigid fit)
       python3 stage2/register_stand.py data/processed/demo_01_v10_ilc3.npz data/ref_A.npy data/ref_B.npy

Prints the fit residual - if it is > ~1 mm the points were not captured consistently.
"""
import argparse
import os

import _path  # noqa: F401
import numpy as np

from wire_trace.learning import kabsch, transform_trajectory
from wire_trace.trajectory import Trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("ref_before")
    ap.add_argument("ref_after")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    A, B = np.load(a.ref_before), np.load(a.ref_after)
    if A.shape != B.shape or len(A) < 3:
        raise SystemExit("need the same >= 3 points before and after")
    R, t = kabsch(A, B)
    res = np.linalg.norm(A @ R.T + t - B, axis=1)
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    print(f"stand moved by {np.linalg.norm(t)*100:.1f} cm, rotated {ang:.1f} deg; "
          f"fit residual max {res.max()*1000:.2f} mm")
    new = transform_trajectory(Trajectory.load(a.traj), R, t)
    out = a.out or os.path.splitext(a.traj)[0] + "_reg.npz"
    new.save(out)
    print(f"-> {out}   (joint reference q_ref is unchanged: go_to_start aligns in Cartesian space)")


if __name__ == "__main__":
    main()
