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
from collections import Counter

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import data_dir, load_config
from wire_trace.kdl_kinematics import KDLKinematics
from wire_trace.ros_common import RobotIO
from wire_trace.so3 import quat_error, rotmat_to_quat
from wire_trace.synthetic import serpentine_wire


def wire_poses(center, yaw, width, height, n_loops, roll0, step, mode="roll", pitch=0.0):
    """Ring-centre poses along a serpentine wire whose plane faces the robot base.

    mode "roll":  the ring rolls about the handle to stay exactly perpendicular to the wire
                  (about 180 deg of wrist roll per U-bend).
    mode "fixed": constant orientation. With roll0 = 45 deg the ring is never more than 45 deg
                  away from perpendicular, which our loop (about 3 cm around a 4 mm wire)
                  tolerates, so the wrist does not need to roll at all.
    pitch: handle tilted downwards by this angle [rad] (0 = horizontal).
    """
    pts, phi = serpentine_wire(center=(0.0, 0.0, 0.0), n_loops=n_loops, width=width, height=height,
                               bend_r=0.02, ds=0.001)
    w_roll = 0.05 if mode == "roll" else 0.0
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1) + w_roll * np.abs(np.diff(phi))
    metric = np.concatenate([[0.0], np.cumsum(seg)])
    idx = np.unique(np.searchsorted(metric, np.arange(0.0, metric[-1], step)))
    idx = idx[idx < len(pts)]
    if idx[-1] != len(pts) - 1:
        idx = np.append(idx, len(pts) - 1)
    radial = np.array([np.sin(yaw), np.cos(yaw), 0.0])     # wire-plane normal (robot -> wire)
    zup = np.array([0.0, 0.0, 1.0])
    u = np.cross(zup, radial)                               # in-plane horizontal
    d = np.cos(pitch) * radial - np.sin(pitch) * zup        # handle direction = tool z
    v = np.cross(d, u)
    poses = []
    for i in idx:
        p = center + pts[i, 1] * u + pts[i, 2] * zup
        a = (phi[i] + roll0) if mode == "roll" else roll0
        x = np.cos(a) * u + np.sin(a) * v
        y = np.cross(d, x)
        R = np.column_stack([x, y, d])
        poses.append((p, rotmat_to_quat(R)))
    return poses


def ik_point(kin, p_t, q_t, q, iters=60, lam=0.02, lo=None, hi=None):
    for _ in range(iters):
        if lo is not None:
            q = np.clip(q, lo + 0.02, hi - 0.02)
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
        q_new, ok = ik_point(kin, p_t, q_t, q, iters=60 if k == 0 else 25, lo=lo, hi=hi)
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
            if np.min(fp[3:, 2]) < table_z:
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
    ap.add_argument("--dist", type=float, nargs="*", default=[0.35, 0.45, 0.55, 0.65, 0.75],
                    help="candidate horizontal distances base -> wire centre [m]")
    ap.add_argument("--yaw", type=float, nargs="*", default=[0, 90, 180, -90],
                    help="candidate directions [deg] around the base, 0 = +y (home reach direction)")
    ap.add_argument("--mode", nargs="*", default=["fixed", "roll"], choices=["fixed", "roll"],
                    help="fixed = constant probe orientation (ring tilted 45 deg), roll = full ring roll")
    ap.add_argument("--pitch", type=float, nargs="*", default=[0, 20, 40],
                    help="candidate downward tilts of the handle [deg]")
    ap.add_argument("--table-z", type=float, default=0.0, help="keep arm frames above this z [m]")
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

    # diagnostics: where are the chain frames at home?
    if hasattr(kin, "frame_positions"):
        print("chain frame heights at the current pose [m]:", np.round(kin.frame_positions(q_now)[:, 2], 3).tolist())
    print(f"joint limits (deg): low {np.round(np.degrees(lo)).astype(int).tolist()} high {np.round(np.degrees(hi)).astype(int).tolist()}")

    t0 = time.time()
    reasons = Counter()

    def search(cands, step, n_seeds, label, max_jump):
        out = []
        for n, cand in enumerate(cands, 1):
            yaw_d, dist, h, mode, pitch_d, roll0_d = cand
            yaw = np.radians(yaw_d)
            c = np.array([dist * np.sin(yaw), dist * np.cos(yaw), h])
            poses = wire_poses(c, yaw, a.width, a.wire_height, a.loops, np.radians(roll0_d), step,
                               mode=mode, pitch=np.radians(pitch_d))
            best, tried = None, 0
            for s0 in seeds:
                q0, ok = ik_point(kin, poses[0][0], poses[0][1], s0.copy(), iters=80, lo=lo, hi=hi)
                if not ok:
                    continue
                tried += 1
                sc, info = evaluate(kin, poses, q0, lo, hi, a.table_z, max_jump)
                if sc is None:
                    reasons[info.split(" at")[0]] += 1
                elif best is None or sc > best[0]:
                    best = (sc, info)
                if tried >= n_seeds:
                    break
            if tried == 0:
                reasons["start pose unreachable"] += 1
            if best is not None:
                out.append((best[0], cand, best[1]))
            if n % 200 == 0:
                print(f"  [{label}] {n}/{len(cands)} checked, {len(out)} feasible ({time.time()-t0:.0f} s)")
        out.sort(key=lambda x: -x[0])
        return out

    cands = []
    for mode in a.mode:
        rolls = (0, 90, 180, 270) if mode == "roll" else (45, 135, 225, 315)
        cands += [(y, d, h, mode, pt, r0) for y in a.yaw for d in a.dist for h in a.height
                  for pt in a.pitch for r0 in rolls]
    print(f"Searching {len(cands)} placements (ring offset {a.offset*100:.0f} cm, "
          f"wire {a.width*100:.0f} x {a.wire_height*100:.0f} cm, {a.loops} loops). Coarse pass ...")
    coarse = search(cands, 0.025, 2, "coarse", 1.0)
    print(f"Coarse pass: {len(coarse)} feasible.   Failure reasons: {dict(reasons)}")
    results = search([cand for _, cand, _ in coarse[:20]], a.step, 3, "fine", 0.5) if coarse else []
    print(f"Done in {time.time()-t0:.0f} s")
    node.destroy_node()
    rclpy.shutdown()

    if not results:
        print("\nNo feasible placement found. Send Claude this whole output (failure reasons above).")
        return
    print(f"\n{len(results)} feasible placements. Best 10 (higher score = more margin):")
    print(" rank  mode   pitch  dir  dist  height  roll0  margin  sigma_min  joints at start (deg)")
    for i, (sc, cand, info) in enumerate(results[:10]):
        yaw_d, dist, h, mode, pitch_d, roll0_d = cand
        print(f" {i+1:4d}  {mode:5s}  {pitch_d:4.0f}  {yaw_d:4.0f}  {dist:4.2f}  {h:5.2f}   {roll0_d:4.0f}  "
              f"{np.degrees(info['margin']):5.0f}deg  {info['smin']:.3f}     {np.round(np.degrees(info['q0'])).astype(int).tolist()}")
    sc, cand, info = results[0]
    yaw_d, dist, h, mode, pitch_d, roll0_d = cand
    np.save(data_dir("wire_start_q.npy"), info["q0"])
    np.save(data_dir("wire_path_q.npy"), info["qs"])
    yaw = np.radians(yaw_d)
    print(f"\nBEST ({mode} orientation, handle tilted {pitch_d:.0f} deg down):")
    print(f"  WIRE CENTRE {dist*100:.0f} cm from the robot base axis, direction {yaw_d:.0f} deg "
          f"(0 = along +y away from the robot, 90 = to your right, -90 = to your left),")
    print(f"  {h*100:.0f} cm above the robot's mounting surface, wire plane FACING the robot.")
    print(f"  = x {dist*np.sin(yaw)*100:+.0f} cm, y {dist*np.cos(yaw)*100:+.0f} cm from the base centre.")
    print(f"Start joint angles -> data/wire_start_q.npy  ({np.round(np.degrees(info['q0']), 1).tolist()} deg)")


if __name__ == "__main__":
    main()
