#!/usr/bin/env python3
"""Publishes std_msgs/Bool on /wire_contact. replay.py logs every rising edge (and can stop
on it with --stop-on-contact). Pick the source that matches your setup:

  --mode keyboard   Operator presses Enter every time the buzzer sounds (always works).
  --mode serial     Arduino reads the buzz-wire circuit (hardware/buzz_contact/buzz_contact.ino),
                    most reliable + objective. Needs: pip install pyserial
  --mode effort     Detects sudden joint-torque spikes from /joint_states effort
                    (only if HEAL publishes effort; noisy - tune --threshold).

    python3 scripts/contact_monitor.py --mode keyboard
    python3 scripts/contact_monitor.py --mode serial --port /dev/ttyACM0
    python3 scripts/contact_monitor.py --mode effort --threshold 2.0
"""
import argparse
import time

import _path  # noqa: F401
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from wire_trace.config import load_config
from wire_trace.ros_common import KeyReader


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["keyboard", "serial", "effort"], default="keyboard")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--threshold", type=float, default=2.0, help="effort spike threshold [Nm]")
    ap.add_argument("--config", default=None)
    a = ap.parse_args()
    cfg = load_config(a.config)

    rclpy.init()
    node = Node("wire_contact_monitor")
    pub = node.create_publisher(Bool, cfg["contact"]["topic"], 10)
    n_events = 0

    def send(v: bool):
        m = Bool()
        m.data = bool(v)
        pub.publish(m)

    try:
        if a.mode == "keyboard":
            keys = KeyReader()
            print("Press Enter whenever the buzzer sounds (Ctrl-C to quit).")
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                if keys.pop() is not None:
                    n_events += 1
                    print(f"  contact #{n_events} at {time.strftime('%H:%M:%S')}")
                    send(True)
                    time.sleep(0.05)
                    send(False)
        elif a.mode == "serial":
            import serial  # pyserial
            ser = serial.Serial(a.port, a.baud, timeout=0.01)
            print(f"Listening on {a.port}. Arduino sends '1' on contact, '0' on release.")
            state = False
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                line = ser.readline().decode(errors="ignore").strip()
                if line in ("0", "1"):
                    new = line == "1"
                    if new != state:
                        state = new
                        send(state)
                        if state:
                            n_events += 1
                            print(f"  CONTACT #{n_events} at {time.strftime('%H:%M:%S')}")
        else:
            buf = []
            st = {"on": False}

            def js_cb(msg: JointState):
                nonlocal n_events
                if len(msg.effort) == 0:
                    return
                e = np.array(msg.effort, float)
                buf.append(e)
                if len(buf) > 50:
                    buf.pop(0)
                if len(buf) < 20:
                    return
                base = np.median(np.array(buf[:-5]), axis=0)
                spike = float(np.max(np.abs(e - base)))
                on = spike > a.threshold
                if on != st["on"]:
                    st["on"] = on
                    send(on)
                    if on:
                        n_events += 1
                        print(f"  effort spike {spike:.2f} Nm -> contact #{n_events}")

            node.create_subscription(JointState, cfg["robot"]["joint_state_topic"], js_cb, qos_profile_sensor_data)
            print(f"Watching effort spikes > {a.threshold} Nm (tune with a deliberate light touch test).")
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"{n_events} contact event(s).")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
