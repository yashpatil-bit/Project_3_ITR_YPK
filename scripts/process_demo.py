#!/usr/bin/env python3
"""PROCESS phase (offline, no robot needed): smooth + interpolate + time-parameterise a demo.

    python3 scripts/process_demo.py data/demos/demo_01.npz                 # uses config/processing
    python3 scripts/process_demo.py data/demos/demo_01.npz --vmax 0.10 --amax 0.2 --alat 0.2
    python3 scripts/process_demo.py data/demos/demo_01.npz --out data/processed/fast.npz

Writes data/processed/<demo>_v<cm/s>.npz and a PNG preview next to it.
"""
import argparse
import os

import _path  # noqa: F401

from wire_trace.config import data_dir, load_config, process_params
from wire_trace.plotting import plot_processing
from wire_trace.trajectory import Demo, process_demo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("demo")
    ap.add_argument("--config", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--vmax", type=float, default=None, help="m/s")
    ap.add_argument("--amax", type=float, default=None, help="m/s^2 tangential")
    ap.add_argument("--alat", type=float, default=None, help="m/s^2 centripetal (bend speed)")
    ap.add_argument("--wmax", type=float, default=None, help="rad/s")
    ap.add_argument("--smooth", type=float, default=None, help="path smoothing sigma [m]")
    ap.add_argument("--offset", type=float, nargs=3, default=None, help="clearance offset xyz [m]")
    ap.add_argument("--trim", type=float, default=None, help="cut this arc length [m] from both ends")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()

    cfg = load_config(a.config)
    p = process_params(cfg, v_max=a.vmax, a_max=a.amax, a_lat_max=a.alat, w_max=a.wmax,
                       smooth_sigma=a.smooth, offset_xyz=tuple(a.offset) if a.offset else None,
                       trim_start_end=a.trim)
    demo = Demo.load(a.demo)
    traj = process_demo(demo, p)
    base = os.path.splitext(os.path.basename(a.demo))[0]
    out = a.out or data_dir("processed", f"{base}_v{int(round(p.v_max * 100))}.npz")
    traj.save(out)
    print(f"Saved processed trajectory -> {out}")
    if not a.no_plot:
        png = os.path.splitext(out)[0] + ".png"
        plot_processing(demo, traj, png)
        print(f"Preview -> {png}")
    print("Next (robot): python3 scripts/go_to_start.py", out)
    print("              python3 scripts/replay.py", out, "--speed 0.1")


if __name__ == "__main__":
    main()
