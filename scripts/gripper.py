#!/usr/bin/env python3
"""Open / close the HEAL gripper (wraps the starter-code GripperController, which calls
the /gripper_controller/command service).

    python3 scripts/gripper.py open
    python3 scripts/gripper.py close --force 60
    python3 scripts/gripper.py set 0.3            # partial close (0 = closed, 1 = open)

Gripping the probe handle: put the handle between the open fingers so the handle points
straight out of the gripper (along the tool axis), mark the grip position on the handle
with tape so every session grips it the same way, then close with moderate force.
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "starter_code", "utils", "gripper_commands"))
from heal_dynamixel_gripper import GripperController  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["open", "close", "set"])
    ap.add_argument("value", nargs="?", type=float, default=None)
    ap.add_argument("--force", type=float, default=None, help="grasp force (driver default 100)")
    ap.add_argument("--ws", default=os.path.expanduser("~/Debojit_WS/addverb_ws"),
                    help="ROS workspace with install/setup.bash")
    a = ap.parse_args()
    g = GripperController(workspace_path=a.ws)
    if a.action == "open":
        ok = g.open_gripper(force=a.force)
    elif a.action == "close":
        ok = g.close_gripper(force=a.force)
    else:
        if a.value is None:
            raise SystemExit("set needs a value in [0, 1]")
        ok = g.set(a.value, force=a.force)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
