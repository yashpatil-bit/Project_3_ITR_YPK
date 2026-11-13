#!/usr/bin/env python3
"""REPLAY / EXECUTE phase: track a processed trajectory with DLS-IK joint-velocity control.

Prerequisites (see README): ./heal_server running, velocity controller launched, robot at
the trajectory start (scripts/go_to_start.py). Buddy on the E-stop.

    python3 scripts/replay.py data/processed/demo_01_v5.npz --dry-run      # robot does NOT move
    python3 scripts/replay.py data/processed/demo_01_v5.npz --speed 0.1    # 10 % speed (ALWAYS first)
    python3 scripts/replay.py data/processed/demo_01_v5.npz --speed 0.5
    python3 scripts/replay.py data/processed/demo_01_v5.npz --speed 1.0 --tag trial1
    python3 scripts/replay.py data/processed/demo_01_v5.npz --reverse      # trace back end -> start

--dry-run integrates the commanded joint velocities on a *virtual* copy of the robot using
the real URDF kinematics, starting from the real joint state. It checks reachability,
singularities, joint limits and the workspace box without publishing anything.

Every run is logged to data/runs/<tag>_<time>.npz/.json (time, errors, contacts, ...).
"""
import argparse
import time

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from wire_trace.config import controller_params, data_dir, load_config
from wire_trace.controller import TrackingController
from wire_trace.plotting import plot_run
from wire_trace.ros_common import RobotIO, confirm
from wire_trace.runlog import RunLog
from wire_trace.trajectory import Trajectory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("--speed", type=float, default=None, help="speed scale (default config: 0.1)")
    ap.add_argument("--reverse", action="store_true", help="run the trajectory end -> start")
    ap.add_argument("--dry-run", action="store_true", help="virtual robot, publish nothing")
    ap.add_argument("--stop-on-contact", action="store_true")
    ap.add_argument("--tag", default="run")
    ap.add_argument("--config", default=None)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    a = ap.parse_args()

    cfg = load_config(a.config)
    cp = controller_params(cfg, speed_scale=a.speed)
    if cp.speed_scale > 0.25 and not a.dry_run:
        print(f"NOTE: speed scale {cp.speed_scale}. Did this trajectory already run cleanly at 0.1-0.2?")
    traj = Trajectory.load(a.traj)
    if a.reverse:
        traj = traj.reversed()
    rate = float(cfg["robot"]["rate_hz"])
    dt = 1.0 / rate

    rclpy.init()
    node = Node("wire_trace_replay")
    io = RobotIO(node, cfg, need_command=not a.dry_run)
    contacts = []
    contact_state = {"on": False}
    t_run0 = {"t": None}

    def contact_cb(msg: Bool):
        if msg.data and not contact_state["on"] and t_run0["t"] is not None:
            contacts.append(time.monotonic() - t_run0["t"])
            node.get_logger().warn(f"CONTACT at t={contacts[-1]:.2f}s")
        contact_state["on"] = bool(msg.data)

    node.create_subscription(Bool, cfg["contact"]["topic"], contact_cb, 10)

    log = RunLog(meta={"traj": a.traj, "speed_scale": cp.speed_scale, "reverse": a.reverse,
                       "dry_run": a.dry_run, "tag": a.tag, "rate_hz": rate,
                       "planned_duration_s": traj.duration / max(cp.speed_scale, 1e-9),
                       "ilc_iter": traj.meta.get("ilc_iter", 0), "stage2": traj.meta.get("stage2", "")})
    status, reason = "abort", "not started"
    ctrl = None
    try:
        io.wait_ready(urdf_file=a.urdf)
        if list(traj.joint_names) != list(io.kin.joint_names):
            raise RuntimeError(f"trajectory joints {traj.joint_names} != robot chain {io.kin.joint_names}")
        ctrl = TrackingController(traj, cp, io.kin.lower, io.kin.upper)
        q_virtual = io.q()
        p, qq = io.kin.fk(q_virtual)
        ok, why = ctrl.check_start(p, qq)
        if not ok:
            raise RuntimeError(why)
        print(f"Trajectory: {traj.length*100:.1f} cm, planned time {traj.duration/cp.speed_scale:.2f} s "
              f"at speed {cp.speed_scale}{' (DRY RUN)' if a.dry_run else ''}")
        if not a.dry_run and not a.yes:
            if not confirm("Buddy on E-stop, robot in sight, area clear? [Enter = GO, n = cancel] "):
                raise RuntimeError("cancelled by user")
            for c in (3, 2, 1):
                print(f"  {c}...")
                time.sleep(0.5)

        t0 = time.monotonic()
        t_run0["t"] = t0
        next_tick = t0
        status = "running"
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.0005, next_tick - now))
                continue
            if now - next_tick > 5 * dt:
                node.get_logger().warn(f"control loop late by {(now-next_tick)*1000:.0f} ms")
                next_tick = now
            next_tick += dt

            if a.dry_run:
                q = q_virtual
            else:
                if io.stale():
                    status, reason = "abort", "joint_states stale"
                    break
                q = io.q()
            pos, quat = io.kin.fk(q)
            J = io.kin.jacobian(q)
            r = ctrl.step(q, pos, quat, J, dt)
            log.add(t=now - t0, tau=r.info["tau"], q=q, qdot_cmd=r.qdot, pos=pos, quat=quat,
                    p_d=r.info["p_d"], q_d=r.info["q_d"], e_pos=r.info["e_pos"], e_track=r.info["e_track"], e_ori=r.info["e_ori"],
                    k=r.info["k"], sigma_min=r.info["sigma_min"], effort=io.effort())
            if r.status != "running":
                status, reason = r.status, r.reason
                break
            if a.stop_on_contact and contacts:
                status, reason = "abort", "contact detected"
                break
            if a.dry_run:
                q_virtual = q_virtual + r.qdot * dt
            else:
                io.publish_qdot(r.qdot)
    except KeyboardInterrupt:
        status, reason = "abort", "Ctrl-C"
    except RuntimeError as e:
        status, reason = "abort", str(e)
    finally:
        # smooth ramp to zero, then hard zeros (the velocity controller keeps the LAST command!)
        if ctrl is not None and not a.dry_run:
            for _ in range(int(rate)):
                z = ctrl.stop_step(dt)
                io.publish_qdot(z)
                time.sleep(dt)
                if not np.any(z):
                    break
        if not a.dry_run:
            io.publish_zero(10)
        t_end = log.rows["t"][-1] if log.rows["t"] else 0.0
        log.contacts = contacts
        log.meta.update(status=status, reason=reason)
        if log.rows["t"]:
            path = log.save(data_dir("runs"), f"{a.tag}_{time.strftime('%Y%m%d_%H%M%S')}")
            try:
                arr = log.arrays()
                arr["contacts"] = contacts
                plot_run(arr, path + ".png", f"{a.tag}: {status} {reason}")
            except Exception as e:  # plotting must never break the safety path
                print(f"(plot failed: {e})")
            e = np.array([float(x) for x in log.rows["e_pos"]])
            print("\n================ RUN SUMMARY ================")
            print(f" status   : {status.upper()} {reason}")
            print(f" time     : {float(t_end):.2f} s")
            print(f" pos err  : max {np.nanmax(e)*1000:.2f} mm, mean {np.nanmean(e)*1000:.2f} mm")
            print(f" contacts : {len(contacts)} {['%.2f s' % c for c in contacts]}")
            print(f" log      : {path}.npz / .json / .png")
            print("=============================================")
        else:
            print(f"STOPPED before start: {reason}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
