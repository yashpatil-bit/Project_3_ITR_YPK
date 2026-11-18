#!/usr/bin/env python3
"""KEYBOARD TEACH mode (used because ./free_drive does not publish /joint_states to ROS).

Needs ./heal_server + the step-5 velocity_control launch (starter README / INSTRUCTIONS B3).

Every key TAP moves a small exact STEP (the robot drives to a target pose with DLS-IK).
Holding a key keeps stepping; the target is never allowed to run more than 2 steps ahead
of the robot, so the arm stops within ~2 steps after you let go.

   Position (base_link axes)      w/s : +x/-x    a/d : +y/-y    r/f : +z/-z
   Rotation about base axes       i/k : x        j/l : y        u/o : z
   ROLL about the gripper axis    n/m : (this is the ring roll in the U-bends)
   TURN about the GRIPPER AXIS    [ / ] : ring turns in place (rotation about the gripper axis, ring centre fixed)
   TURN JOINT 6 ONLY              { / } : only with the probe OFF the wire (swings the handle)
   GO HOME                        g g   : press g twice within 3 s -> all joints slowly to 0 deg
                                          (probe must be OFF the wire / gripper open! any key cancels)
   1/2 : smaller/bigger steps     (0.5 / 1 / 2 / 5 mm  and  1 / 2 / 5 / 10 deg)
   c   : start/stop continuous RECORDING (20 Hz)  -> use this to teach the whole wire
   --auto-roll : the ring turns BY ITSELF to follow your direction of motion (recommended)
   SPACE : add a single waypoint
   h   : hold (cancel pending steps, stop where you are)
   q   : save + quit      x : quit WITHOUT saving

Saved as data/demos/<name>.npz. Then:
    python3 scripts/process_demo.py data/demos/<name>.npz
"""
import argparse
import select
import sys
import termios
import time
import tty

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node

from wire_trace.config import data_dir, load_config
from wire_trace.controller import StallDetector, dls, limit_qdot
from wire_trace.ros_common import RobotIO
from wire_trace.so3 import quat_error, quat_exp, quat_mul, quat_normalize, quat_to_rotmat
from wire_trace.trajectory import Demo

def lowest_point(kin, q):
    """Lowest z among the arm's frames (ignoring the base-most ones) and the tool point."""
    zs = [kin.fk(q)[0][2]]
    if hasattr(kin, "frame_positions"):
        zs += list(kin.frame_positions(q)[3:, 2])
    return float(min(zs))


LIN = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1), "r": (2, 1), "f": (2, -1)}
ANG = {"i": (0, 1), "k": (0, -1), "j": (1, 1), "l": (1, -1), "u": (2, 1), "o": (2, -1)}
ROLL = {"n": 1, "m": -1}
STEP_MM = [0.5, 1.0, 2.0, 5.0]
STEP_DEG = [1.0, 2.0, 5.0, 10.0]
TOOL_R = np.eye(3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=time.strftime("teleop_%Y%m%d_%H%M%S"))
    ap.add_argument("--config", default=None)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--auto-roll", action="store_true",
                    help="turn the ring automatically (about the handle axis) so it stays perpendicular to the "
                         "direction you are moving -> you only steer with the movement keys in the corners")
    ap.add_argument("--roll-tol", type=float, default=20.0,
                    help="auto-roll: start turning when the ring is this many degrees off perpendicular")
    a = ap.parse_args()
    cfg = load_config(a.config)
    rclpy.init()
    node = Node("wire_trace_teleop")
    io = RobotIO(node, cfg)
    io.wait_ready(urdf_file=a.urdf)
    dt = 1.0 / float(cfg["robot"]["rate_hz"])
    global TOOL_R
    from wire_trace.urdf_chain import _rpy
    TOOL_R = _rpy(*cfg["robot"].get("tool_rpy", [0.0, 0.0, 0.0]))
    si = 1
    kp_lin, kp_ang = 3.0, 3.0
    v_cap, w_cap = 0.01, 0.05                 # m/s, rad/s (HEAL safety trips on fast wrist motion)
    last = np.zeros(io.kin.n)
    qdot_max = np.full(io.kin.n, 0.15)
    T, Q, P, R = [], [], [], []
    t0 = time.monotonic()
    recording = False
    last_rec = 0.0
    last_print = 0.0
    last_warn = 0.0
    joint_goal = None                         # joint-space goal (joint-6 jog or go-home) [rad]
    stall = StallDetector()
    trail = []                                # recent target positions (auto-roll)
    last_g = -10.0
    old = termios.tcgetattr(sys.stdin)
    save = True
    print(__doc__)
    p_t, q_t = io.kin.fk(io.q())               # target pose = current pose
    print(f"START tool tip x={p_t[0]:+.3f} y={p_t[1]:+.3f} z={p_t[2]:+.3f} m   step {STEP_MM[si]} mm / {STEP_DEG[si]} deg")

    def record(now):
        q = io.q()
        p, qq = io.kin.fk(q)
        T.append(now - t0); Q.append(q); P.append(p); R.append(qq)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if io.stale():
                print("joint_states stale -> stop (robot in error? check T1)")
                break
            q = io.q()
            p, qq = io.kin.fk(q)
            now = time.monotonic()
            while select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                step = STEP_MM[si] / 1000.0
                ang = np.radians(STEP_DEG[si])
                if c == "q":
                    raise StopIteration
                if c == "x":
                    save = False
                    raise StopIteration
                if c == "1":
                    si = max(0, si - 1); print(f"step {STEP_MM[si]} mm / {STEP_DEG[si]} deg")
                elif c == "2":
                    si = min(len(STEP_MM) - 1, si + 1); print(f"step {STEP_MM[si]} mm / {STEP_DEG[si]} deg")
                elif c == "c":
                    recording = not recording
                    print(">>> RECORDING ON (20 Hz)" if recording else f">>> RECORDING OFF ({len(P)} samples so far)")
                elif c == " ":
                    record(now); print(f"  waypoint {len(P)}: {np.round(P[-1], 4)}")
                elif c == "h":
                    p_t, q_t = p.copy(), qq.copy(); joint_goal = None; print("  hold")
                elif c == "g":
                    if now - last_g > 3.0:
                        last_g = now
                        print("  GO HOME? Probe OFF the wire, gripper open, path clear -> press g again within 3 s")
                    else:
                        last_g = -10.0
                        home = np.zeros(io.kin.n)
                        low = min(lowest_point(io.kin, q + u * (home - q)) for u in np.linspace(0, 1, 40))
                        if low < 0.08:
                            print(f"  !! going home directly would bring the arm to {low*100:.1f} cm above the platform - "
                                  "move up first (r), then try again")
                        else:
                            joint_goal = home
                            print(f"  >>> GOING HOME (lowest point on the way {low*100:.0f} cm). Any key cancels.")
                elif c in "{}":
                    base = q.copy() if joint_goal is None else joint_goal.copy()
                    if abs(base[-1] - q[-1]) < 2.0 * ang:
                        new6 = base[-1] + (ang if c == "}" else -ang)
                        if io.kin.lower[-1] + 0.1 < new6 < io.kin.upper[-1] - 0.1:
                            base[-1] = new6
                            joint_goal = base
                            print(f"  joint 6 -> {np.degrees(new6):.0f} deg")
                        else:
                            print("  !! joint 6 limit")
                elif joint_goal is not None and (c in LIN or c in ANG or c in ROLL or c in "[]"):
                    joint_goal = None
                    p_t, q_t = p.copy(), qq.copy()
                    print("  joint move cancelled")
                elif c in LIN:
                    ax, sg = LIN[c]
                    if np.linalg.norm(p_t - p) < 2.0 * step:          # don't run ahead of the robot
                        p_t = p_t.copy(); p_t[ax] += sg * step
                        trail.append(p_t.copy())
                elif c in ANG or c in ROLL or c in "[]":
                    if np.linalg.norm(quat_error(q_t, qq)) < 2.0 * ang:
                        if c in ANG:
                            axv = np.zeros(3); ax, sg = ANG[c]; axv[ax] = sg
                        elif c in ROLL:
                            axv = quat_to_rotmat(q_t)[:, 2] * ROLL[c]     # handle axis (controlled frame z)
                        else:
                            # gripper approach axis (rq_tool_tip z) expressed through the configured tool rotation
                            axv = quat_to_rotmat(q_t) @ TOOL_R.T @ np.array([0.0, 0.0, 1.0])
                            axv = axv * (1.0 if c == "]" else -1.0)
                        q_t = quat_normalize(quat_mul(quat_exp(axv * ang), q_t))

            # ---- auto-roll: keep the ring normal along the recent direction of motion ----------
            if a.auto_roll and len(trail) >= 2:
                while len(trail) > 2 and np.linalg.norm(trail[-1] - trail[0]) > 0.006:
                    trail.pop(0)
                mv = trail[-1] - trail[0]
                Rt = quat_to_rotmat(q_t)
                hax = Rt[:, 2]                                  # handle axis (roll axis)
                mv = mv - hax * float(mv @ hax)                 # motion within the plane of the ring normal
                if np.linalg.norm(mv) > 0.003:
                    n_now = Rt[:, 0]                            # ring normal
                    d = mv / np.linalg.norm(mv)
                    if float(d @ n_now) < 0:
                        d = -d                                  # a ring has no front/back
                    ang_err = float(np.arctan2(np.cross(n_now, d) @ hax, n_now @ d))
                    if abs(ang_err) > np.radians(a.roll_tol) and np.linalg.norm(quat_error(q_t, qq)) < np.radians(6):
                        stepr = float(np.clip(ang_err - np.sign(ang_err) * np.radians(a.roll_tol) * 0.5,
                                              -np.radians(3), np.radians(3)))
                        q_t = quat_normalize(quat_mul(quat_exp(hax * stepr), q_t))
            e_lin = p_t - p
            e_ang = quat_error(q_t, qq)
            v = kp_lin * e_lin
            w = kp_ang * e_ang
            if np.linalg.norm(v) > v_cap:
                v *= v_cap / np.linalg.norm(v)
            if np.linalg.norm(w) > w_cap:
                w *= w_cap / np.linalg.norm(w)
            J = io.kin.jacobian(q)
            if joint_goal is not None:
                # joint-space move (joint-6 jog or go-home); Cartesian target follows the robot
                err = joint_goal - q
                cmd = 1.5 * err
                peak = float(np.max(np.abs(cmd)))
                if peak > 0.06:
                    cmd *= 0.06 / peak
                smin = 1.0
                p_t, q_t = p.copy(), qq.copy()
                if float(np.max(np.abs(err))) < 0.004:
                    if not np.any(joint_goal):
                        print("  >>> HOME reached")
                    joint_goal = None
                    cmd = np.zeros(io.kin.n)
                elif lowest_point(io.kin, q) < 0.06:
                    print("  !! arm close to the platform - joint move stopped")
                    joint_goal = None
                    cmd = np.zeros(io.kin.n)
            else:
                cmd, smin, _ = dls(J, np.concatenate([v, w]), 0.02, 0.1, 0.04)
            cmd = limit_qdot(cmd, last, qdot_max, 0.5 * dt)
            warn = None
            # 1) the robot cannot follow the target (singularity / limits): hold instead of wandering
            if np.linalg.norm(e_lin) > 0.015 or np.linalg.norm(e_ang) > np.radians(25):
                warn = (f"target unreachable from here (pos err {np.linalg.norm(e_lin)*1000:.0f} mm, "
                        f"ori err {np.degrees(np.linalg.norm(e_ang)):.0f} deg) - HOLDING. Try another direction / pose")
            # 2) near a singularity: big wrist swings for small motions
            elif smin < 0.03:
                warn = f"near a singularity (sigma_min {smin:.3f}) - HOLDING. Move back the way you came"
            # 3) joint-limit guard: never move a joint further into its last 0.1 rad
            lo, hi = io.kin.lower + 0.1, io.kin.upper - 0.1
            q_next = q + cmd * 0.3
            bad = ((q_next < lo) & (cmd < 0)) | ((q_next > hi) & (cmd > 0))
            if np.any(bad):
                warn = f"joint(s) {[int(j) + 1 for j in np.where(bad)[0]]} near their limit - HOLDING. Try another direction"
            # 4) BLOCKED: commanded but not moving (probe pressing on the wire) -> stop before HEAL trips
            blocked, bj = stall.update(now, q, last)
            if blocked:
                warn = (f"BLOCKED: joint {bj + 1} is not moving as commanded - the probe is probably pressing on "
                        "the wire. Stopped. Move the other way / re-centre the ring")
                joint_goal = None
                stall.reset()
            if warn is not None:
                cmd = np.zeros_like(cmd)
                p_t, q_t = p.copy(), qq.copy()        # drop the unreachable target
                if now - last_warn > 1.0:
                    last_warn = now
                    print("  !! " + warn)
            last = cmd
            io.publish_qdot(cmd)

            if recording and now - last_rec >= 0.05:
                last_rec = now
                record(now)
            moving = np.linalg.norm(e_lin) > 2e-4 or np.linalg.norm(e_ang) > 0.005 or joint_goal is not None
            if now - last_print > 1.0 and (moving or recording):
                last_print = now
                zt = quat_to_rotmat(qq)[:, 2]
                print(f"  tip x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:+.3f} m  gripper points ({zt[0]:+.2f},{zt[1]:+.2f},{zt[2]:+.2f})  "
                      f"joints(deg)={np.round(np.degrees(q)).astype(int).tolist()}  max|qdot|={np.max(np.abs(cmd)):.3f}"
                      f"  {'REC ' + str(len(P)) if recording else ''}")
            time.sleep(dt)
    except (KeyboardInterrupt, StopIteration):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        io.publish_zero(10)
        log_path = data_dir("runs", f"teleop_{a.name}_cmds.npy")
        io.save_cmd_log(log_path)
        print(f"command log -> {log_path}")
        try:
            p1, _ = io.kin.fk(io.q())
            print(f"END   tool tip x={p1[0]:+.3f} y={p1[1]:+.3f} z={p1[2]:+.3f} m")
        except Exception:
            pass
        if save and len(P) >= 4:
            path = data_dir("demos", a.name + ".npz")
            Demo(t=np.array(T), q=np.array(Q), pos=np.array(P), quat=np.array(R),
                 joint_names=io.kin.joint_names, meta={"teleop": True, "samples": len(P)}).save(path)
            print(f"Saved {len(P)} samples -> {path}")
            print(f"Next: python3 scripts/process_demo.py {path}")
        elif save:
            print(f"Nothing saved ({len(P)} samples; need >= 4). Press c to record next time.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
