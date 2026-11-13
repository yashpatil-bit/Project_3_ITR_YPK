#!/usr/bin/env python3
"""Run any script of this repo against a SIMULATED robot, with no ROS installed.

It injects tiny stand-ins for rclpy / ROS messages / KDL into sys.modules:
  * /joint_states       <- simulated 6-DOF arm (wire_trace.sim_arm.SimArm, NOT the HEAL geometry)
  * /velocity_controller/commands -> integrated by the simulated arm (with lag)
  * /robot_description  <- dummy string (kinematics come from SimArm)
Then it runs the requested script exactly as on the lab PC.

Use it to rehearse the full command sequence at home and to smoke-test code changes:

    python3 tools/fake_robot.py --make-demo                      # synthetic demo -> data/demos/sim_demo.npz
    python3 scripts/process_demo.py data/demos/sim_demo.npz --vmax 0.1
    python3 tools/fake_robot.py scripts/go_to_start.py data/processed/sim_demo_v10.npz --yes
    python3 tools/fake_robot.py scripts/replay.py data/processed/sim_demo_v10.npz --speed 0.5 --yes

The simulated arm state is persisted in data/sim/fake_robot_state.npy between calls,
so go_to_start followed by replay behaves like on the real robot.
"""
from __future__ import annotations

import os
import runpy
import sys
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np  # noqa: E402

from wire_trace.sim_arm import SimArm  # noqa: E402

STATE_FILE = os.path.join(ROOT, "data", "sim", "fake_robot_state.npy")


# ---------------------------------------------------------------------------
# simulated world
# ---------------------------------------------------------------------------
class World:
    def __init__(self):
        q0 = np.load(STATE_FILE) if os.path.exists(STATE_FILE) else None
        self.arm = SimArm(q0=q0, lag=0.03, noise=1e-5)
        self.cmd = np.zeros(6)
        self.t_last = time.monotonic()
        self.subs: dict[str, list] = {}

    def step(self):
        now = time.monotonic()
        dt = min(now - self.t_last, 0.05)
        self.t_last = now
        if dt > 0:
            self.arm.apply(self.cmd, dt)

    def save(self):
        os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
        np.save(STATE_FILE, self.arm.q)


WORLD = World()


# ---------------------------------------------------------------------------
# message stand-ins
# ---------------------------------------------------------------------------
class _Msg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Header(_Msg):
    def __init__(self):
        super().__init__(frame_id="", stamp=None)


class JointState(_Msg):
    def __init__(self):
        super().__init__(name=[], position=[], velocity=[], effort=[], header=Header())


class Float64MultiArray(_Msg):
    def __init__(self):
        super().__init__(data=[])


class String(_Msg):
    def __init__(self):
        super().__init__(data="")


class Bool(_Msg):
    def __init__(self):
        super().__init__(data=False)


class _Vec(_Msg):
    def __init__(self):
        super().__init__(x=0.0, y=0.0, z=0.0, w=1.0)


class Pose(_Msg):
    def __init__(self):
        super().__init__(position=_Vec(), orientation=_Vec())


class PoseStamped(_Msg):
    def __init__(self):
        super().__init__(header=Header(), pose=Pose())


class Path(_Msg):
    def __init__(self):
        super().__init__(header=Header(), poses=[])


# ---------------------------------------------------------------------------
# rclpy stand-in
# ---------------------------------------------------------------------------
class _Logger:
    def info(self, m): print("[INFO]", m)
    def warn(self, m): print("[WARN]", m)
    def error(self, m): print("[ERROR]", m)


class _Clock:
    class _T:
        def __init__(self, ns): self.nanoseconds = ns
        def to_msg(self): return None
        def __sub__(self, o): return _Clock._T(self.nanoseconds - o.nanoseconds)

    def now(self):
        return _Clock._T(int(time.monotonic() * 1e9))


class _Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        if self.topic == "/velocity_controller/commands":
            WORLD.cmd = np.array(msg.data, float)
        for cb in WORLD.subs.get(self.topic, []):
            cb(msg)


class Node:
    def __init__(self, name):
        self.name = name
        self._subs = []
        self._timers = []

    def create_subscription(self, typ, topic, cb, qos):
        WORLD.subs.setdefault(topic, []).append(cb)
        self._subs.append((topic, cb))
        if topic.endswith("robot_description"):
            m = String(); m.data = "<robot name='fake'/>"
            cb(m)
        return (topic, cb)

    def destroy_subscription(self, s):
        pass

    def create_publisher(self, typ, topic, qos):
        return _Pub(topic)

    def create_timer(self, period, cb):
        self._timers.append([period, cb, time.monotonic() + period])
        return self._timers[-1]

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return _Clock()

    def count_subscribers(self, topic):
        return 1

    def get_topic_names_and_types(self):
        return [("/joint_states", ["sensor_msgs/msg/JointState"]),
                ("/velocity_controller/commands", ["std_msgs/msg/Float64MultiArray"])]

    def destroy_node(self):
        WORLD.save()


_NODES: list = []
_orig_node_init = Node.__init__


def _node_init(self, name, *a, **k):
    _orig_node_init(self, name)
    _NODES.append(self)


Node.__init__ = _node_init


def _deliver_joint_state():
    WORLD.step()
    js = JointState()
    js.name = list(SimArm.joint_names)
    js.position = list(WORLD.arm.measured_q())
    js.velocity = list(WORLD.arm.qdot)
    js.effort = []
    for cb in WORLD.subs.get("/joint_states", []):
        cb(js)


def spin_once(node=None, timeout_sec=0.0):
    _deliver_joint_state()
    now = time.monotonic()
    for n in _NODES:
        for tm in n._timers:
            if now >= tm[2]:
                tm[2] = now + tm[0]
                tm[1]()
    if timeout_sec:
        time.sleep(min(timeout_sec, 0.002))


def spin(node=None):
    while True:
        spin_once(node, 0.002)


_ok = {"v": True}


def install():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.ok = lambda: _ok["v"]
    rclpy.shutdown = lambda: _ok.update(v=False)
    rclpy.spin_once = spin_once
    rclpy.spin = spin
    node_mod = types.ModuleType("rclpy.node"); node_mod.Node = Node
    qos = types.ModuleType("rclpy.qos")
    for n in ("QoSProfile",):
        setattr(qos, n, lambda **k: None)
    for n in ("QoSDurabilityPolicy", "QoSHistoryPolicy", "QoSReliabilityPolicy"):
        setattr(qos, n, types.SimpleNamespace(TRANSIENT_LOCAL=1, KEEP_LAST=1, RELIABLE=1, BEST_EFFORT=2, VOLATILE=0))
    qos.qos_profile_sensor_data = None
    ex = types.ModuleType("rclpy.executors"); ex.SingleThreadedExecutor = object
    rclpy.node, rclpy.qos, rclpy.executors = node_mod, qos, ex
    sys.modules.update({"rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos, "rclpy.executors": ex})
    for pkg, names in {"sensor_msgs": ["JointState"], "std_msgs": ["Float64MultiArray", "String", "Bool"],
                       "geometry_msgs": ["Pose", "PoseStamped"], "nav_msgs": ["Path"]}.items():
        m = types.ModuleType(pkg); mm = types.ModuleType(pkg + ".msg")
        for n in names:
            setattr(mm, n, globals()[n])
        m.msg = mm
        sys.modules[pkg] = m
        sys.modules[pkg + ".msg"] = mm

    # KDL stand-in: kinematics come from the simulated arm
    kdl_mod = types.ModuleType("wire_trace.kdl_kinematics")

    class KDLKinematics:
        def __init__(self, urdf_xml, base_link, tip_link, tool_xyz=(0, 0, 0), tool_rpy=(0, 0, 0)):
            self.joint_names = list(SimArm.joint_names)
            self.n = 6
            self.lower = WORLD.arm.lower.copy()
            self.upper = WORLD.arm.upper.copy()
            self.vel_limit = np.full(6, np.nan)

        def fk(self, q):
            return WORLD.arm.fk(np.asarray(q, float))

        def jacobian(self, q):
            return WORLD.arm.jacobian(np.asarray(q, float))

    kdl_mod.KDLKinematics = KDLKinematics
    sys.modules["wire_trace.kdl_kinematics"] = kdl_mod


def make_demo():
    from wire_trace.synthetic import human_demo, wire_path
    arm = SimArm()
    demo = human_demo(arm, wire_path())
    os.makedirs(os.path.join(ROOT, "data", "demos"), exist_ok=True)
    path = os.path.join(ROOT, "data", "demos", "sim_demo.npz")
    demo.save(path)
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    np.save(STATE_FILE, SimArm().q)          # park the fake robot at its home pose
    print(f"synthetic demo -> {path}  ({len(demo.t)} samples)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    if sys.argv[1] == "--make-demo":
        make_demo()
        sys.exit(0)
    if sys.argv[1] == "--reset":
        if os.path.exists(STATE_FILE):
            os.remove(STATE_FILE)
        sys.exit(0)
    install()
    script = sys.argv[1]
    sys.argv = sys.argv[1:]
    sys.path.insert(0, os.path.dirname(os.path.abspath(script)))
    try:
        runpy.run_path(script, run_name="__main__")
    finally:
        WORLD.save()
