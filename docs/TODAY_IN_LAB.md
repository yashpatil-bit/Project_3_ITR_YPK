# In-lab procedure for our setup (HEAL + 2-finger gripper + buzz-wire box)

What the setup photos show, and why it matters:

| Observation | Consequence |
|---|---|
| Probe = closed teardrop ring on a black handle, **threaded on the wire** | It can't be lifted off. No long joint moves with the probe on the wire. Get back to the start with `replay.py --reverse` |
| Wire = vertical U-bends in one plane | The ring must **roll about 180° in every bend**. Speed in the bends is limited by `w_max` (wrist roll), not by the linear speed |
| Buzzer and red switch on the box | Switch it ON during every run. Contacts are audible, and you log them with `contact_monitor.py --mode keyboard` |
| HEAL has a 2-finger gripper | The gripper holds the probe handle (`scripts/gripper.py`) |
| Stand far from the robot on the left, wire plane parallel to the wall | Long reach and an awkward wrist. Ask the TA whether the box may be moved (step 3) |
| RealSense camera on a pole | Not needed for Stage 1. Use it for filming, or later for Stage 2 |

Roles: **Operator** at the keyboard, **Buddy** with a hand on the E-stop at all times.

---

## 1. Software and robot up (15 min)
```bash
git clone -b claude/sleepy-bohr-jn2txo https://github.com/yashpatil-bit/Project_3_ITR_YPK.git   # or: git pull
cd Project_3_ITR_YPK && pip3 install -r requirements.txt && python3 -m pytest -q
```
Start the robot (README "Robot bring-up") with `./heal_server` and launch the velocity controller.
In a new terminal, source ROS and the workspace, then:
```bash
python3 scripts/check_setup.py
```
Everything should be OK. Note the printed **tool position**. Measure with a ruler where the
gripper fingertips actually are relative to the robot base. If the printed point is the
fingertips, the URDF `end-effector` frame already includes the gripper. If it is the flange, the
gripper length must go into `tool_xyz` (step 4).

## 2. Motion sanity test, probe NOT in the gripper, arm away from the stand (10 min)
```bash
python3 scripts/gripper.py open
python3 scripts/gripper.py close --force 40
python3 scripts/teleop_teach.py --name test      # tap w a few times: probe point moves +x (base frame)? then x to quit
```
This confirms that the gripper service works, that our velocity commands arrive, and that joint
order and directions are right. If the arm moves in an unexpected way, **E-stop**, then check the
velocity controller's joint order (`ros2 param get /velocity_controller joints`) against
`robot.joint_names` in the config.

Also test free-drive: stop `heal_server`, run `./free_drive`, then
`python3 scripts/check_setup.py --watch` and move the arm by hand.
**If the pose changes, teaching by hand works.** If it times out, tell the TA and me; the
fallback is keyboard teaching (`teleop_teach.py`).

## 3. Place the wire stand (ask the TA first)
Best geometry: the wire plane **facing the robot**, wire centre about 45–60 cm from the robot base,
roughly at gripper height. The handle then points from the robot straight into the wire plane,
and each U-bend roll is mostly the last wrist joint. Keep the elbow bent and the wrist bent
(not straight), which keeps the arm away from singularities.

Then **tape the box outline on the table** and tape the box down, so it can't slide and can be
put back in exactly the same place (repeatability).

## 4. Mount the probe (15 min)
1. The ring rests at the START of the wire (left end). Buzzer switch ON.
2. `./free_drive`: bring the open gripper around the handle while a teammate holds the probe
   handle, ring centred around the wire.
   - The handle should point **straight out of the gripper** along the tool axis, and
     **perpendicular to the wire plane**.
   - Grip in the middle of the handle. Wrap tape there first if it's thin, and mark the spot.
3. Stop `./free_drive`, start `./heal_server` (the robot now holds its pose), then:
   ```bash
   python3 scripts/gripper.py close --force 60
   ```
   The handle must not rotate in the fingers. Test it by gently twisting it by hand.
   - **If the gripper only works under `./heal_server`**, you need this step to switch modes.
   - **If it also works under `./free_drive`**, close it directly and skip the switch.
4. Tape the probe's cable loosely along the wrist with slack, so it can't snag when the wrist
   rolls ±180°.
5. Measure with a ruler from the URDF end-effector frame (see step 1) to the **ring centre**.
   Put it in `config/heal_wire_trace.yaml` → `robot.tool_xyz`, usually only z (along the
   gripper axis), e.g. `[0, 0, 0.12]`.

## 5. Teach (20 min). The quality of the demo decides everything
```bash
./free_drive                                   # robot terminal
python3 scripts/record_demo.py --name demo_01  # our terminal
```
- Press Enter, then guide the probe from the start to the end of the wire.
  - Go slowly (2–3 cm/s) and keep the wire in the **middle of the round part** of the ring.
  - In every U-bend, slow down and roll the wrist so the ring stays perpendicular to the wire.
- Press Enter at the end and keep the arm still.
- **If the buzzer sounded, record again**, because replay copies the demo.
- Make 2–3 demos. Guide back to the start by hand between them; you don't need to record that
  part.

## 6. Process (offline, 2 min)
```bash
python3 scripts/process_demo.py data/demos/demo_01.npz
```
Check the printed numbers: smoothing deviation < 1.5 mm, plus the duration. Open the PNG.
The default `w_max: 0.5 rad/s` makes each U-bend take about 6 s. That's slow, but safe for the
first runs.

## 7. First autonomous runs
Switch to `./heal_server` and launch the velocity controller. The robot is now wherever the last
demo ended. Run the contact monitor in its own terminal:
```bash
python3 scripts/contact_monitor.py --mode keyboard        # press Enter whenever the buzzer sounds
```
If the robot is at the END of the demo:
```bash
python3 scripts/replay.py data/processed/demo_01_v5.npz --reverse --dry-run
python3 scripts/replay.py data/processed/demo_01_v5.npz --reverse --speed 0.1
```
That brings it back to the start along the wire. Then the forward run:
```bash
python3 scripts/replay.py data/processed/demo_01_v5.npz --dry-run
python3 scripts/replay.py data/processed/demo_01_v5.npz --speed 0.1
```
If `replay.py` says the probe is a few mm off the start, run
`go_to_start.py <traj>` (or `go_to_start.py <traj> --end` before a reverse run). It only allows
moves under 2 cm.

After each run, look at `data/runs/<run>.png`. Then step the speed up:
- `--speed` 0.2 → 0.3 → 0.5 → 1.0, with one clean run at each step.
- Re-process faster: `--vmax 0.08 --amax 0.2 --alat 0.2 --wmax 0.8`, then `--wmax 1.0`.
  Re-process with `--wmax` raised step by step; each new file starts again at `--speed 0.3`.
- In simulation, roll speeds above about 1 rad/s overloaded the wrist, so go above 1.0 only in
  small steps.

## 8. Final runs
Three consecutive clean runs with `--tag final` (with `--reverse` runs in between to get back to
the start), then:
```bash
python3 scripts/evaluate_runs.py --tag final
git add data && git commit -m "lab runs" && git push
```

## Ask the TA
1. May we move or rotate the wire box closer to the robot (step 3)?
2. Does `./free_drive` publish `/joint_states`? Does the gripper service work in free-drive?
3. HEAL joint speed limits, especially the last wrist joint (it does the ring roll)?
4. How is contact judged: by the buzzer?
