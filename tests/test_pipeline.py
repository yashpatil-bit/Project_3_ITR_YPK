"""Offline tests (no ROS): python3 -m pytest -q"""
import os
import subprocess
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from wire_trace.controller import ControllerParams, TrackingController, dls  # noqa: E402
from wire_trace.learning import DMP, ILCParams, ilc_update, kabsch, transform_trajectory  # noqa: E402
from wire_trace.runlog import summarize  # noqa: E402
from wire_trace.sim_arm import SimArm  # noqa: E402
from wire_trace.sim_runner import place_at_start, simulate  # noqa: E402
from wire_trace.so3 import quat_error, quat_exp, quat_log, quat_mul, quat_to_rotmat, rotmat_to_quat, slerp  # noqa: E402
from wire_trace.synthetic import human_demo, wire_path  # noqa: E402
from wire_trace.trajectory import ProcessParams, Trajectory, path_deviation, process_demo  # noqa: E402


@pytest.fixture(scope="module")
def wire():
    return wire_path()


@pytest.fixture(scope="module")
def demo(wire):
    return human_demo(SimArm(), wire)


@pytest.fixture(scope="module")
def traj(demo):
    return process_demo(demo, ProcessParams(v_max=0.08, a_max=0.2, a_lat_max=0.2), verbose=False)


# ---------------------------------------------------------------- so3
def test_quat_roundtrip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        r = rng.normal(size=3)
        r *= min(1.0, 3.0 / np.linalg.norm(r))
        q = quat_exp(r)
        assert np.allclose(quat_log(q), r, atol=1e-9)
        assert abs(np.dot(rotmat_to_quat(quat_to_rotmat(q)), q)) == pytest.approx(1.0, abs=1e-9)


def test_quat_error_and_slerp():
    a = quat_exp(np.array([0.1, -0.2, 0.3]))
    b = quat_exp(np.array([0.3, 0.1, -0.1]))
    e = quat_error(b, a)
    assert np.allclose(quat_mul(quat_exp(e), a) * np.sign(b[3]), b * np.sign(b[3]), atol=1e-9)
    assert np.allclose(np.abs(slerp(a, b, 1.0)), np.abs(b), atol=1e-9)


# ---------------------------------------------------------------- processing
def test_processing_keeps_geometry(demo, traj, wire):
    # start/end pinned to the demo
    assert np.linalg.norm(traj.pos[0] - demo.pos[0]) < 1e-3
    assert np.linalg.norm(traj.pos[-1] - demo.pos[-1]) < 1e-3
    # processed path stays close to the real wire (demo tremor ~1-2 mm)
    assert path_deviation(traj.pos[::5], wire).max() < 0.003
    assert traj.meta["max_smoothing_dev_m"] < 0.0015


def test_speed_profile_limits(traj):
    v = np.linalg.norm(traj.vel, axis=1)
    assert v.max() <= 0.08 * 1.05
    assert v[0] < 1e-6 and v[-1] < 1e-6
    a = np.abs(np.diff(v)) / np.diff(traj.t)
    assert a.max() < 0.2 * 1.6
    assert np.all(np.diff(traj.t) > 0)


def test_trajectory_save_load_reverse(tmp_path, traj):
    p = tmp_path / "t.npz"
    traj.save(str(p))
    t2 = Trajectory.load(str(p))
    assert np.allclose(t2.pos, traj.pos) and t2.joint_names == traj.joint_names
    r = traj.reversed()
    assert np.allclose(r.pos[0], traj.pos[-1]) and np.allclose(r.q_ref[0], traj.q_ref[-1])
    assert np.isclose(r.duration, traj.duration)


def test_sparse_waypoints_spline(wire):
    arm = SimArm()
    d = human_demo(arm, wire)
    idx = np.arange(0, len(d.t), 25)
    d.t, d.q, d.pos, d.quat = d.t[idx], d.q[idx], d.pos[idx], d.quat[idx]
    tr = process_demo(d, ProcessParams(raw_interp="spline", smooth_sigma=0.0), verbose=False)
    assert path_deviation(tr.pos[::5], wire).max() < 0.004


# ---------------------------------------------------------------- control
def test_dls_matches_pinv_away_from_singularity():
    arm = SimArm()
    J = arm.jacobian()
    tw = np.array([0.01, -0.02, 0.01, 0.0, 0.05, 0.0])
    qd, smin, lam = dls(J, tw, 1e-4, 0.1, 0.04)
    assert np.allclose(qd, np.linalg.pinv(J) @ tw, atol=1e-5)
    assert smin > 0.04 and lam == pytest.approx(1e-4)


def test_tracking_in_sim(traj, wire):
    arm = SimArm(noise=1e-5, lag=0.03)
    place_at_start(arm, traj)
    log, status, reason = simulate(traj, arm, ControllerParams(speed_scale=1.0, qdot_max=(1.5,) * 6))
    s = summarize(log.arrays(), log.meta, [])
    assert status == "done", reason
    assert s["max_pos_err_mm"] < 2.5
    assert s["duration_s"] < traj.duration * 1.2


def test_refuses_when_not_at_start(traj):
    arm = SimArm()
    log, status, reason = simulate(traj, arm, ControllerParams())
    assert status == "abort" and "start" in reason


def test_abort_on_disturbance(traj):
    arm = SimArm(lag=0.0)
    place_at_start(arm, traj)
    kick = lambda t: np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]) if 1.0 < t < 1.5 else np.zeros(6)  # noqa: E731
    log, status, reason = simulate(traj, arm, ControllerParams(speed_scale=1.0, qdot_max=(1.5,) * 6), disturbance=kick)
    assert status == "abort" and ("off the path" in reason or "orientation" in reason)


def test_joint_limit_abort(traj):
    arm = SimArm()
    place_at_start(arm, traj)
    lo, hi = arm.q - 0.08, arm.q + 0.08            # absurdly tight limits around the start pose
    ctrl = TrackingController(traj, ControllerParams(speed_scale=1.0), lo, hi)
    for _ in range(3000):
        p, q = arm.fk()
        r = ctrl.step(arm.q, p, q, arm.jacobian(), 0.01)
        if r.status != "running":
            break
        arm.apply(r.qdot, 0.01)
    assert r.status == "abort" and "limit" in r.reason


# ---------------------------------------------------------------- stage 2
def test_ilc_reduces_error(demo):
    tr = process_demo(demo, ProcessParams(v_max=0.2, a_max=0.5, a_lat_max=0.5), verbose=False)
    errs = []
    for it in range(4):
        arm = SimArm(noise=1e-5, lag=0.05, seed=it)
        place_at_start(arm, tr)
        log, status, _ = simulate(tr, arm, ControllerParams(speed_scale=1.0, kp_pos=6.0, qdot_max=(1.5,) * 6))
        a = log.arrays()
        errs.append(a["e_pos"].max())
        tr, _ = ilc_update(tr, a["tau"], a["pos"], ILCParams())
    assert errs[-1] < 0.7 * errs[0]


def test_dmp_reproduces_and_generalises(traj):
    dmp = DMP(n_basis=200).fit([(traj.t, traj.pos)])
    t, y = dmp.rollout()
    assert path_deviation(y[::5], traj.pos).max() < 0.002
    g = traj.pos[-1] + np.array([0.0, 0.02, 0.01])
    _, y2 = dmp.rollout(g=g)
    assert np.linalg.norm(y2[-1] - g) < 0.002


def test_kabsch(traj):
    rng = np.random.default_rng(3)
    A = rng.normal(size=(5, 3))
    R = quat_to_rotmat(quat_exp(np.array([0.05, -0.02, 0.1])))
    t = np.array([0.01, -0.03, 0.02])
    R2, t2 = kabsch(A, A @ R.T + t)
    assert np.allclose(R2, R, atol=1e-9) and np.allclose(t2, t, atol=1e-9)
    tr2 = transform_trajectory(traj, R, t)
    assert np.allclose(tr2.pos[10], R @ traj.pos[10] + t)


# ---------------------------------------------------------------- the real scripts, fake robot
@pytest.mark.slow
def test_scripts_with_fake_robot(tmp_path):
    env = dict(os.environ)
    py = sys.executable

    def run(*args, timeout=180):
        r = subprocess.run([py, *args], cwd=ROOT, env=env, capture_output=True, text=True, timeout=timeout)
        assert r.returncode == 0, r.stdout + r.stderr
        return r.stdout

    run("tools/fake_robot.py", "--make-demo")
    out_traj = str(tmp_path / "t.npz")
    run("scripts/process_demo.py", "data/demos/sim_demo.npz", "--vmax", "0.15", "--amax", "0.4",
        "--alat", "0.4", "--out", out_traj, "--no-plot")
    out = run("tools/fake_robot.py", "scripts/replay.py", out_traj, "--yes", "--tag", "pytest_refuse")
    assert "go_to_start" in out
    run("tools/fake_robot.py", "scripts/go_to_start.py", out_traj, "--yes")
    out = run("tools/fake_robot.py", "scripts/replay.py", out_traj, "--speed", "1.0", "--yes", "--tag", "pytest")
    assert "status   : DONE" in out
