#!/usr/bin/env python3
"""FALLBACK teach mode: jog the probe with the keyboard (DLS-IK Cartesian velocity) and
save waypoints. Use this if /joint_states is NOT available while ./free_drive runs, or to
add careful waypoints in the tightest bends. Needs ./heal_server + velocity controller.

Keys (no Enter needed; motion only while a key is held / repeating = dead-man):
   w/s : +x/-x     a/d : +y/-y     r/f : +z/-z      (base_link axes)
   i/k : rotate about x  j/l : about y  u/o : about z
   1/2 : slower/faster jog speed
   SPACE : save waypoint      q : save + quit      x : quit without saving

Waypoints are saved as a Demo (data/demos/<name>.npz); process them with
    python3 scripts/process_demo.py data/demos/<name>.npz   (set processing.raw_interp: spline)
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
from wire_trace.controller import dls, limit_qdot
from wire_trace.ros_common import RobotIO
from wire_trace.trajectory import Demo

LIN = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1), "r": (2, 1), "f": (2, -1)}
ANG = {"i": (0, 1), "k": (0, -1), "j": (1, 1), "l": (1, -1), "u": (2, 1), "o": (2, -1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=time.strftime("teleop_%Y%m%d_%H%M%S"))
    ap.add_argument("--config", default=None)
    ap.add_argument("--urdf", default=None)
    a = ap.parse_args()
    cfg = load_config(a.config)
    rclpy.init()
    node = Node("wire_trace_teleop")
    io = RobotIO(node, cfg)
    io.wait_ready(urdf_file=a.urdf)
    dt = 1.0 / float(cfg["robot"]["rate_hz"])
    speeds = [0.002, 0.005, 0.01, 0.02]   # m/s
    si = 1
    last = np.zeros(io.kin.n)
    qdot_max = np.full(io.kin.n, 0.3)
    T, Q, P, R = [], [], [], []
    t0 = time.monotonic()
    key_t, key = 0.0, None
    old = termios.tcgetattr(sys.stdin)
    save = True
    print(__doc__)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c == "q":
                    break
                if c == "x":
                    save = False
                    break
                if c == "1":
                    si = max(0, si - 1); print(f"jog speed {speeds[si]*100:.1f} cm/s")
                elif c == "2":
                    si = min(len(speeds) - 1, si + 1); print(f"jog speed {speeds[si]*100:.1f} cm/s")
                elif c == " ":
                    q = io.q(); p, qq = io.kin.fk(q)
                    T.append(time.monotonic() - t0); Q.append(q); P.append(p); R.append(qq)
                    print(f"  waypoint {len(P)}: {np.round(p, 4)}")
                else:
                    key, key_t = c, time.monotonic()
            if io.stale():
                print("joint_states stale -> stop"); break
            twist = np.zeros(6)
            if key is not None and time.monotonic() - key_t < 0.15:     # dead-man
                if key in LIN:
                    ax, sg = LIN[key]; twist[ax] = sg * speeds[si]
                elif key in ANG:
                    ax, sg = ANG[key]; twist[3 + ax] = sg * speeds[si] * 5.0
            q = io.q()
            cmd, _, _ = dls(io.kin.jacobian(q), twist, 0.02, 0.1, 0.04)
            cmd = limit_qdot(cmd, last, qdot_max, 1.0 * dt)
            last = cmd
            io.publish_qdot(cmd)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        io.publish_zero(10)
        if save and len(P) >= 4:
            path = data_dir("demos", a.name + ".npz")
            Demo(t=np.array(T), q=np.array(Q), pos=np.array(P), quat=np.array(R),
                 joint_names=io.kin.joint_names, meta={"teleop_waypoints": True}).save(path)
            print(f"Saved {len(P)} waypoints -> {path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
