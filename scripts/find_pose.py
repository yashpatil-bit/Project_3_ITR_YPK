#!/usr/bin/env python3
"""Where should the wire box go? Searches wire placements with HEAL's REAL kinematics
(URDF from /robot_description) and reports the ones where the whole wire can be traced
comfortably: inside joint limits with margin, away from singularities, arm above the table.
Moves nothing.

Model of the task: the probe handle points straight out of the gripper (tool z) and
perpendicular into the wire plane; the wire plane FACES the robot (its normal points from
the robot base to the wire centre); the ring rolls about the handle axis to stay
perpendicular to the wire, about 180 deg per U-bend.

    python3 scripts/find_pose.py                         # defaults for our wire (30 x 15 cm)
    python3 scripts/find_pose.py --offset 0.10           # ring centre 10 cm beyond the fingertips
    python3 scripts/find_pose.py --height 0.22 --width 0.30 --wire-height 0.15

Coordinates: base_link. +y = the direction the arm reaches in its home pose (away from the
robot), +z = up, +x = to your RIGHT when you stand behind the robot looking along +y.
Output: ranked placements + data/wire_start_q.npy (joint angles of the best start pose, for
scripts/move_joints.py --file).
"""
import argparse
import time

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import data_dir, load_config
from wire_trace.kdl_kinematics import KDLKinematics
from wire_trace.ros_common import RobotIO
from wire_trace.so3 import quat_error, rotmat_to_quat
from wire_trace.synthetic import serpentine_wire


def wire_poses(center, yaw, width, height, n_loops, roll0, step):
    """Ring-centre poses along a serpentine wire whose plane faces the robot base."""
    pts, phi = serpentine_wire(center=(0.0, 0.0, 0.0), n_loops=n_loops, width=width, height=height,
                               bend_r=0.02, ds=0.001)
    # sample by a combined metric: path length + 5 cm per radian of ring roll, so the
    # U-bends (fast roll) get as many checks as the straight parts
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1) + 0.05 * np.abs(np.diff(phi))
    metric = np.concatenate([[0.0], np.cumsum(seg)])
    idx = np.unique(np.searchsorted(metric, np.arange(0.0, metric[-1], step)))
    idx = idx[idx < len(pts)]
    if idx[-1] != len(pts) - 1:
        idx = np.append(idx, len(pts) - 1)
    d = np.array([np.sin(yaw), np.cos(yaw), 0.0])          # wire-plane normal = handle direction
    zup = np.array([0.0, 0.0, 1.0])
    u = np.cross(zup, d)                                    # in-plane horizontal
    poses = []
    for i in idx:
        p = center + pts[i, 1] * u + pts[i, 2] * zup
        a = phi[i] + roll0
        x = np.cos(a) * u + np.sin(a) * zup
        y = np.cross(d, x)
        R = np.column_stack([x, y, d])
        poses.append((p, rotmat_to_quat(R)))
    return poses


def ik_point(kin, p_t, q_t, q, iters=60, lam=0.02):
    for _ in range(iters):
        p, qq = kin.fk(q)
        e = np.concatenate([p_t - p, quat_error(q_t, qq)])
        if np.linalg.norm(e[:3]) < 3e-4 and np.linalg.norm(e[3:]) < 3e-3:
            return q, True
        J = kin.jacobian(q)
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        n = np.max(np.abs(dq))
        if n > 0.2:
            dq *= 0.2 / n
        q = q + dq
    return q, False


def evaluate(kin, poses, q_start, lo, hi, table_z, max_jump=0.5):
    """Track the whole wire from q_start. Returns (score, info) or (None, reason)."""
    q = q_start.copy()
    margins, smins, travel = [], [], 0.0
    qs = []
    for k, (p_t, q_t) in enumerate(poses):
        q_new, ok = ik_point(kin, p_t, q_t, q, iters=60 if k == 0 else 25)
        if not ok:
            return None, f"IK lost at point {k}/{len(poses)}"
        if k > 0 and np.max(np.abs(q_new - q)) > max_jump:
            return None, "configuration jump"
        travel += float(np.sum(np.abs(q_new - q)))
        q = q_new
        m = float(np.min(np.minimum(q - lo, hi - q)))
        if m < 0.05:
            return None, "joint limit"
        margins.append(m)
        smins.append(float(np.linalg.svd(kin.jacobian(q), compute_uv=False)[-1]))
        if hasattr(kin, "frame_positions"):
            fp = kin.frame_positions(q)
            if np.min(fp[2:, 2]) < table_z:
                return None, "arm too low (table)"
        qs.append(q.copy())
    score = min(margins) + 2.0 * min(smins) - 0.02 * travel
    return score, dict(margin=min(margins), smin=min(smins), travel=travel, q0=qs[0], qs=np.array(qs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--offset", type=float, default=0.09, help="ring centre beyond the fingertips [m]")
    ap.add_argument("--width", type=float, default=0.30, help="wire length left-right [m]")
    ap.add_argument("--wire-height", type=float, default=0.15, help="wire height top-bottom [m]")
    ap.add_argument("--loops", type=int, default=3, help="number of up-down loops")
    ap.add_argument("--height", type=float, nargs="*", default=[0.15, 0.22, 0.30, 0.38],
                    help="candidate wire-centre heights above the robot base mount [m]")
    ap.add_argument("--dist", type=float, nargs="*", default=[0.35, 0.45, 0.55, 0.65],
                    help="candidate horizontal distances base -> wire centre [m]")
    ap.add_argument("--yaw", type=float, nargs="*", default=list(range(-150, 181, 30)),
                    help="candidate directions [deg] around the base, 0 = +y (home reach direction)")
    ap.add_argument("--table-z", type=float, default=0.03, help="keep arm frames above this z [m]")
    ap.add_argument("--step", type=float, default=0.01, help="check every N metres along the wire")
    ap.add_argument("--config", default=None)
    a = ap.parse_args()
    cfg = load_config(a.config)

    rclpy.init()
    node = Node("wire_trace_find_pose")
    io = RobotIO(node, cfg, need_command=False)
    io.wait_ready()
    r = cfg["robot"]
    kin = KDLKinematics(io._urdf, r["base_link"], r["tip_link"], (0.0, 0.0, a.offset), (0.0, 0.0, 0.0))
    lo, hi = io.kin.lower, io.kin.upper
    q_now = io.q()
    rng = np.random.default_rng(0)
    seeds = [q_now, np.zeros(6)] + [rng.uniform(lo * 0.8, hi * 0.8) for _ in range(10)]

    t0 = time.time()

    def search(cands, step, n_seeds, label, max_jump):
        out = []
        for n, (yaw_d, dist, h, roll0_d) in enumerate(cands, 1):
            yaw = np.radians(yaw_d)
            c = np.array([dist * np.sin(yaw), dist * np.cos(yaw), h])
            poses = wire_poses(c, yaw, a.width, a.wire_height, a.loops, np.radians(roll0_d), step)
            best, tried = None, 0
            for s0 in seeds:
                q0, ok = ik_point(kin, poses[0][0], poses[0][1], s0.copy(), iters=80)
                if not ok:
                    continue
                tried += 1
                sc, info = evaluate(kin, poses, q0, lo, hi, a.table_z, max_jump)
                if sc is not None and (best is None or sc > best[0]):
                    best = (sc, info)
                if tried >= n_seeds:
                    break
            if best is not None:
                out.append((best[0], yaw_d, dist, h, roll0_d, best[1]))
            if n % 100 == 0:
                print(f"  [{label}] {n}/{len(cands)} checked, {len(out)} feasible ({time.time()-t0:.0f} s)")
        out.sort(key=lambda x: -x[0])
        return out

    cands = [(y, d, h, r0) for y in a.yaw for d in a.dist for h in a.height for r0 in (0, 90, 180, 270)]
    print(f"Searching {len(cands)} placements (ring offset {a.offset*100:.0f} cm, "
          f"wire {a.width*100:.0f} x {a.wire_height*100:.0f} cm). Coarse pass ...")
    coarse = search(cands, 0.025, 2, "coarse", 1.0)
    print(f"Coarse pass: {len(coarse)} feasible. Fine check of the best 15 ...")
    results = search([(y, d, h, r0) for _, y, d, h, r0, _ in coarse[:15]], a.step, 3, "fine", 0.5)
    print(f"Done in {time.time()-t0:.0f} s")
    node.destroy_node()
    rclpy.shutdown()

    if not results:
        print("\nNo feasible placement found. Try other --offset / --height / --dist values, or tell Claude.")
        return
    results.sort(key=lambda x: -x[0])
    print(f"\n{len(results)} feasible placements. Best 8 (higher score = more margin):")
    print(" rank  dir(deg)  dist  height  wire centre x,y,z [m]     roll0  joint-margin  sigma_min  joints at start (deg)")
    for i, (sc, yaw_d, dist, h, roll0_d, info) in enumerate(results[:8]):
        yaw = np.radians(yaw_d)
        c = (dist * np.sin(yaw), dist * np.cos(yaw), h)
        print(f" {i+1:4d}  {yaw_d:7.0f}  {dist:4.2f}  {h:5.2f}   ({c[0]:+.2f},{c[1]:+.2f},{c[2]:+.2f})        {roll0_d:4d}   "
              f"{np.degrees(info['margin']):6.0f} deg    {info['smin']:.3f}    {np.round(np.degrees(info['q0'])).astype(int).tolist()}")
    sc, yaw_d, dist, h, roll0_d, info = results[0]
    np.save(data_dir("wire_start_q.npy"), info["q0"])
    np.save(data_dir("wire_path_q.npy"), info["qs"])
    yaw = np.radians(yaw_d)
    print(f"\nBEST: put the WIRE CENTRE {dist*100:.0f} cm from the robot base axis, direction {yaw_d:.0f} deg "
          f"(0 = straight along +y, positive = towards +x / your right),")
    print(f"      at {h*100:.0f} cm above the robot's mounting surface, with the wire plane FACING the robot")
    print("      (perpendicular to the line from the robot to the wire centre).")
    print(f"      = x {dist*np.sin(yaw)*100:+.0f} cm, y {dist*np.cos(yaw)*100:+.0f} cm from the base centre.")
    print(f"Start joint angles saved -> data/wire_start_q.npy  ({np.round(np.degrees(info['q0']), 1).tolist()} deg)")


if __name__ == "__main__":
    main()
