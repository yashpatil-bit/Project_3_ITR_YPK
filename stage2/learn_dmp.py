#!/usr/bin/env python3
"""STAGE 2 (bonus) - learning from demonstration with a Dynamic Movement Primitive.

Learn ONE primitive from several demos (averaging out hand tremor), then generate a new
trajectory for a (slightly) different start / goal without re-teaching.

    # learn from 3 demos, reproduce the mean demo
    python3 stage2/learn_dmp.py data/demos/demo_01.npz data/demos/demo_02.npz data/demos/demo_03.npz

    # generalise: new start/goal captured with scripts/capture_points.py (2 points: start, end)
    python3 stage2/learn_dmp.py data/demos/demo_0*.npz --start-goal data/new_start_goal.npy --vmax 0.08

Output: data/processed/dmp_<n>demos[_sg].npz - run it with go_to_start.py / replay.py as usual.
"""
import argparse
import os

import _path  # noqa: F401
import numpy as np

from wire_trace.config import data_dir, load_config, process_params
from wire_trace.learning import learn_dmp_trajectory
from wire_trace.plotting import plot_processing
from wire_trace.trajectory import Demo, path_deviation, process_demo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("demos", nargs="+")
    ap.add_argument("--start-goal", default=None, help=".npy with 2 points [[start],[goal]] in base_link")
    ap.add_argument("--n-basis", type=int, default=200)
    ap.add_argument("--vmax", type=float, default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    cfg = load_config(a.config)
    p = process_params(cfg, v_max=a.vmax)
    demos = [Demo.load(f) for f in a.demos]
    trajs = [process_demo(d, p, verbose=False) for d in demos]
    start = goal = None
    if a.start_goal:
        sg = np.load(a.start_goal)
        start, goal = sg[0], sg[-1]
        print(f"new start {np.round(start, 4)} (was {np.round(trajs[0].pos[0], 4)})")
        print(f"new goal  {np.round(goal, 4)} (was {np.round(trajs[0].pos[-1], 4)})")
    traj = learn_dmp_trajectory(trajs, start, goal, n_basis=a.n_basis, params=p)
    if start is None:
        for i, tr in enumerate(trajs):
            d = path_deviation(traj.pos[::3], tr.pos)
            print(f"DMP vs demo {i+1}: max {d.max()*1000:.2f} mm, mean {d.mean()*1000:.2f} mm")
    else:
        print("NOTE: the joint configuration (q_ref) is borrowed from demo 1 - go_to_start will do a "
              "final Cartesian alignment to the new start.")
    out = a.out or data_dir("processed", f"dmp_{len(demos)}demos{'_sg' if a.start_goal else ''}.npz")
    traj.save(out)
    plot_processing(demos[0], traj, os.path.splitext(out)[0] + ".png")
    print(f"-> {out}")


if __name__ == "__main__":
    main()
