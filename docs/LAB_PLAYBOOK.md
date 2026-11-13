# Lab Playbook: the physical part, session by session

The software is done and tested in simulation. The remaining risk is all on the real setup.
Each session has a **goal**, the **steps**, and a **done-when** test. Don't skip ahead: every
session removes one unknown.

Roles in every session: **Operator** (keyboard), **Buddy** (hand on the E-stop, watches the
probe), and, if there are three of you, a **Recorder** (writes notes and films).

---

## Before the first session (at home, about 1 h)
- `pip3 install -r requirements.txt && python3 -m pytest -q`, which should end with `15 passed`.
- Run the fake-robot rehearsal from README §8 so everyone has typed every command once.
- Bring: a ruler or caliper, tape, a laptop with the repo, and a phone for video.
- Prepare the questions for the TA (bottom of this page).

## Session 1: bring-up and the big unknowns (about 1 h)
1. Start the robot (README "Robot bring-up") with `./heal_server` and the velocity controller.
2. `python3 scripts/check_setup.py`
   - KDL chain OK? Joint names match? If `tip_link` is wrong, run `ros2 run tf2_tools view_frames`
     and fix `robot.tip_link`.
   - Is the `effort` field present? (It decides whether `contact_monitor --mode effort` can work.)
3. **Most important test:** stop `heal_server`, start `./free_drive`, and run
   `python3 scripts/check_setup.py --watch` in T2. Move the arm by hand.
   - The pose changes → teaching by hand works. ✅
   - "Timed out waiting for JointState" → `free_drive` does not publish ROS joint states. Ask the
     TA how gravity compensation is meant to be recorded. Fallback: `scripts/teleop_teach.py`
     under `heal_server`.
4. Attach the probe and measure `tool_xyz` (flange to probe tip, along the flange axes). Write it
   in the config and commit it.
5. Velocity sanity test, wire stand **out of reach**: run the starter
   `starter_code/Dual_Arm/sample_ik/heal_dls_ik.py` once, or our
   `scripts/go_to_start.py <any processed traj>` with the arm far from anything. You are checking
   that the velocity controller accepts our commands and that joint order and signs are right.
6. Capture the workspace box: free-drive the probe around the stand with
   `scripts/capture_points.py --out data/stand_box.npy`, then put the printed min/max (± 3 cm) in
   `controller.workspace_min/max`.

**Done when:** check_setup is green, the free-drive question is answered, and `tool_xyz` and
the workspace box are in the config and committed.

## Session 2: teach and first slow replay (about 1.5 h)
1. Practise the motion once without recording. The quality of the demo decides everything.
   Move slowly (2–4 cm/s), steadily, with the probe centred on the wire, and keep the wrist
   orientation as constant as the wire allows.
2. Record 3–5 demos: `record_demo.py --name demo_01` and so on.
3. Run `process_demo.py` on each and **look at the PNG and the printed numbers**:
   - smoothing deviation < 1.5 mm
   - tightest bend radius (the slowest part)
   - duration

   Keep the cleanest demo.
4. `publish_path_rviz.py` and RViz: does the processed path overlap the demo and sit where the
   wire is?
5. `go_to_start.py` (watch the swing), then `replay.py --dry-run`, which must end with DONE.
6. `replay.py --speed 0.1`, with the buddy watching the clearance closely. Then 0.2 and 0.3.
7. After every run, open `data/runs/<run>.png`: the error plot and the y-z path overlay.

**Done when:** one full no-contact run at 10–30 % speed.

> If the probe drifts toward the wire at one spot while the error plot says < 1 mm, the
> *demonstration* itself was close to the wire there. Re-teach that section with more clearance,
> or add a small `--offset` while processing.

## Session 3: speed and repeatability (about 1.5 h)
1. Raise `--speed` step by step to 1.0 (one clean run per step).
2. Then re-process with a faster profile, e.g. `--vmax 0.10 --amax 0.2 --alat 0.2`, then
   `0.15 / 0.3 / 0.3`, and so on. Each new file starts again at `--speed 0.3`.
3. Watch for the following:
   - the reference factor `k` dropping in the bottom-right plot: the robot cannot keep up there,
     so lower `a_lat` or raise `kp_pos`
   - oscillation: lower the gains or raise the damping
   - "lags the reference" aborts: hitting `qdot_max`
4. Choose the fastest setting that gave **3 clean runs in a row**, run the contact monitor in a
   separate terminal, and log them with `--tag final`.
5. `evaluate_runs.py --tag final`, then paste `summary.md` into the README results table and
   commit `data/runs/final_*`.

**Done when:** 3 consecutive clean, logged runs at your best speed (50 + 20 + 10 points).

## Session 4 (bonus): Stage 2 ILC and DMP (about 1.5 h)
1. Take the final trajectory one step faster than what worked cleanly.
2. Run the ILC loop (README §7A) 3–5 times. Plot or compare `max path err` per iteration in
   `evaluate_runs.py`; it should go down.
3. Run the final `_ilcN` file in one go at that higher speed. That is the "learns over trials,
   then does it in one go" demonstration.
4. Optional: DMP from all demos (§7B), and/or move the stand slightly and run `register_stand.py`
   (§7C) to show generalisation. **Ask the TA before moving the stand.**

## Video (≤ 3 min) checklist
1. Teaching in free-drive, with the terminal visible (10–20 s).
2. Processed path in RViz or the PNG (10 s).
3. 10 % replay (short clip), then the full-speed final run with the timer on screen.
4. The 3 repeat runs and the `evaluate_runs` table.
5. Stage 2: error falling over ILC iterations, then the one-shot run.
6. Safety: the buddy on the E-stop visible in the shot.

---

## Troubleshooting

| Problem | Likely cause and fix |
|---|---|
| `Timed out waiting for: URDF` | The controller manager is not up, or the topic name differs. `ros2 topic list \| grep robot_description`, or save the URDF and pass `--urdf file.urdf` |
| `KDL chain has 0 joints` | Wrong `base_link`/`tip_link`. Check `ros2 run tf2_tools view_frames` |
| Robot moves the wrong way or in another joint | `robot.joint_names` order differs from the velocity controller's order. Check `ros2 param get /velocity_controller joints` |
| "tool is X mm from trajectory start" | Run `go_to_start.py` first. With `--reverse`, run `go_to_start.py --end` |
| Abort "off the path" right at the start | `kp` too low or the robot lags. Try `--speed 0.1`; check `/joint_states` rate ≥ 100 Hz |
| Robot keeps moving after a script crashed | The velocity controller holds the last command. **E-stop**, then report it. All our scripts publish zeros on exit, but a hard crash or kill cannot |
| `control loop late` warnings | The PC is overloaded (close RViz or browsers), or run on the robot PC |
| Contact while the error plot is < 1 mm | The demo itself was too close to the wire there. Re-teach, or add an `--offset` |

## Questions for the TA (ask in session 1)
1. Does `./free_drive` publish `/joint_states` (and `/robot_description`) over ROS 2? If not,
   what is the intended way to record during gravity compensation?
2. What probe or loop should we mount, and how (gripper or a 3D-printed adapter)? Is it
   conductive and wired to the buzzer?
3. HEAL joint velocity and acceleration limits for the velocity controller?
4. Does the velocity controller have a command timeout, or does it hold the last command?
5. May we tap the buzz-wire circuit with an Arduino (opto-isolated) for contact logging?
6. How will "contact" be judged: the buzzer, visually, or by force?
