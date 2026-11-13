"""Matplotlib previews (saved as PNG, no display needed)."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def plot_processing(demo, traj, out_png: str):
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(2, 2, 1, projection="3d")
    ax.plot(*demo.pos.T, ".", ms=1.5, color="0.6", label="raw demo")
    ax.plot(*(traj.pos + traj.pos_offset).T, "-", lw=1.5, color="C0", label="processed")
    ax.scatter(*traj.pos[0], color="g", s=40, label="start")
    ax.scatter(*traj.pos[-1], color="r", s=40, label="end")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("Tool path (base_link)")
    try:
        ax.set_box_aspect(np.ptp(np.vstack([demo.pos, traj.pos]), axis=0) + 1e-3)
    except Exception:
        pass

    ax = fig.add_subplot(2, 2, 2)
    sp = np.linalg.norm(traj.vel, axis=1)
    ax.plot(traj.t, sp * 100, label="|v| planned")
    ax.set_xlabel("t [s]"); ax.set_ylabel("speed [cm/s]"); ax.grid(True)
    ax.set_title(f"Speed profile: {traj.duration:.2f} s for {traj.length*100:.1f} cm (at speed 1.0)")

    ax = fig.add_subplot(2, 2, 3)
    raw_speed = np.linalg.norm(np.gradient(demo.pos, demo.t, axis=0), axis=1)
    ax.plot(demo.t, raw_speed * 100, color="0.6")
    ax.set_xlabel("t [s]"); ax.set_ylabel("speed [cm/s]"); ax.grid(True)
    ax.set_title("Demo hand speed (raw, noisy)")

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(traj.t, np.degrees(traj.q_ref))
    ax.set_xlabel("t [s]"); ax.set_ylabel("joint [deg]"); ax.grid(True)
    ax.set_title("Joint angles of the demo along the path")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def plot_run(run: dict, out_png: str, title: str = ""):
    fig, axs = plt.subplots(2, 2, figsize=(14, 8))
    t = run["t"]
    ax = axs[0, 0]
    ax.plot(t, run["e_pos"] * 1000)
    for c in run.get("contacts", []):
        ax.axvline(c, color="r", ls="--")
    ax.set_ylabel("position error [mm]"); ax.set_xlabel("t [s]"); ax.grid(True)
    ax.set_title("Tracking error (red = contact)")
    ax = axs[0, 1]
    ax.plot(t, run["qdot_cmd"])
    ax.set_ylabel("qdot cmd [rad/s]"); ax.set_xlabel("t [s]"); ax.grid(True)
    ax.set_title("Joint velocity commands")
    ax = axs[1, 0]
    ax.plot(run["p_d"][:, 1], run["p_d"][:, 2], label="desired")
    ax.plot(run["pos"][:, 1], run["pos"][:, 2], label="actual")
    ax.set_xlabel("y [m]"); ax.set_ylabel("z [m]"); ax.axis("equal"); ax.grid(True); ax.legend()
    ax.set_title("Path (y-z view)")
    ax = axs[1, 1]
    ax.plot(t, run["k"], label="reference speed factor k")
    ax.plot(t, np.clip(run["sigma_min"], 0, 1), label="min singular value")
    ax.set_xlabel("t [s]"); ax.grid(True); ax.legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)
