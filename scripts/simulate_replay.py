#!/usr/bin/env python3
"""OFFLINE end-to-end rehearsal (no robot, no ROS): synthetic wire -> noisy human demo ->
processing -> DLS-IK replay on a simulated 6-DOF arm -> metrics -> (optional) Stage 2 ILC.

Use it to understand / tune the pipeline at home before lab time.

    python3 scripts/simulate_replay.py
    python3 scripts/simulate_replay.py --vmax 0.15 --speed 1.0 --ilc 4
"""
import argparse
import os

import _path  # noqa: F401

from wire_trace.config import data_dir
from wire_trace.controller import ControllerParams
from wire_trace.learning import ILCParams, ilc_update
from wire_trace.plotting import plot_processing, plot_run
from wire_trace.runlog import summarize
from wire_trace.sim_arm import SimArm
from wire_trace.sim_runner import place_at_start, simulate
from wire_trace.synthetic import human_demo, wire_path
from wire_trace.trajectory import ProcessParams, path_deviation, process_demo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vmax", type=float, default=0.10)
    ap.add_argument("--accel", type=float, default=0.3)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--ilc", type=int, default=0, help="number of ILC learning trials")
    ap.add_argument("--lag", type=float, default=0.03, help="simulated robot velocity lag [s]")
    a = ap.parse_args()
    out = data_dir("sim")
    wire = wire_path()
    demo = human_demo(SimArm(), wire)
    traj = process_demo(demo, ProcessParams(v_max=a.vmax, a_max=a.accel, a_lat_max=a.accel))
    plot_processing(demo, traj, os.path.join(out, "sim_processing.png"))
    cp = ControllerParams(speed_scale=a.speed, qdot_max=(1.5,) * 6)

    for it in range(a.ilc + 1):
        arm = SimArm(noise=2e-5, lag=a.lag, seed=it)
        place_at_start(arm, traj)
        log, status, reason = simulate(traj, arm, cp)
        arr = log.arrays()
        s = summarize(arr, log.meta, [])
        wd = path_deviation(arr["pos"][::4], wire)
        print(f"[trial {it}] {status} {reason} time={s['duration_s']:.2f}s  tracking err max={s['max_pos_err_mm']:.2f}mm "
              f"mean={s['mean_pos_err_mm']:.2f}mm  dist-to-wire max={wd.max()*1000:.2f}mm")
        arr["contacts"] = []
        plot_run(arr, os.path.join(out, f"sim_run_{it}.png"), f"sim trial {it}")
        if it < a.ilc:
            traj, st = ilc_update(traj, arr["tau"], arr["pos"], ILCParams())
            print(f"          ILC update -> {st}")
    print(f"plots in {out}")


if __name__ == "__main__":
    main()
