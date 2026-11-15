# INSTRUCTIONS: Wire-Trace Trial on the Addverb HEAL

This is the complete guide to the project: setup, launching the robot, every step in the lab, and
every script with its options. Follow Part C step by step. Part D is the reference for each
command.

**Contents**
- A. What the project does and what's in the repo
- B. One-time setup and launching HEAL
- C. Step-by-step procedure in the lab (with time estimates)
- D. Command reference: every script, every option
- E. Configuration file reference
- F. Files the scripts produce
- G. Safety rules and emergency stop
- H. Troubleshooting
- I. Practising at home without the robot
- J. Timeline
- K. Saving results to GitHub

---

## A. What the project does and what's in the repo

**Goal:** the HEAL arm holds the buzz-wire probe (a closed teardrop ring on a black handle,
threaded on the wire) and moves it along the whole wire without touching it. It has to be fast,
smooth, and repeatable 3 times. Bonus (Stage 2): the robot learns to do it better over trials.

**How it works:**
1. **Teach.** The robot runs in gravity-compensation mode (`./free_drive`). A person moves the
   arm by hand along the wire, and `record_demo.py` records the joint angles and probe pose
   50 times per second.
2. **Process.** `process_demo.py` cleans the recording: removes pauses, smooths hand shake,
   fits a smooth curve, and plans a speed profile that slows down automatically in the U-bends.
3. **Execute.** `replay.py` makes the robot follow that path with DLS-IK (damped least squares
   inverse kinematics) by sending joint velocity commands, with safety checks the whole time.
4. **Evaluate.** `evaluate_runs.py` reports time, contacts, path error and repeatability.
5. **Stage 2.** `stage2/ilc_update.py` learns a correction from each run, so the next run is
   more accurate (Iterative Learning Control).

**What the photos of our setup mean for the procedure:**

| Observation | Consequence |
|---|---|
| Probe = closed ring **threaded on the wire** | It can never be lifted off. The robot moves only *along* the wire. Get back to the start with `replay.py --reverse` |
| Wire = vertical U-bends in one plane | The ring must roll about 180° in every bend. The roll speed (`w_max`) limits speed in the bends |
| Buzzer and red switch on the box | Switch ON during every run. Contacts are audible; log them with `contact_monitor.py` |
| HEAL has a 2-finger gripper | The gripper holds the probe handle (`gripper.py`) |

**Repository layout:**

| Path | Contents |
|---|---|
| `INSTRUCTIONS.md` | This file |
| `README.md` | Project overview: method, control law, results table |
| `docs/TODAY_IN_LAB.md` | Short lab checklist for our exact setup |
| `docs/LAB_PLAYBOOK.md` | Session plan, troubleshooting, questions for the TA |
| `config/heal_wire_trace.yaml` | **All parameters** (frames, topics, probe offset, speeds, gains, safety limits) |
| `scripts/` | Everything you run in the lab |
| `stage2/` | Bonus learning scripts (ILC, DMP, stand registration) |
| `wire_trace/` | The library the scripts use (math, controller, processing). You normally don't edit it |
| `tools/fake_robot.py` | Runs our scripts against a simulated robot, for practice at home |
| `tests/` | Automatic tests (`./run_tests.sh`) |
| `hardware/buzz_contact/` | Optional Arduino sketch for automatic contact detection |
| `starter_code/` | The course starter code (`LAB_COMMANDS.md` = the official HEAL launch commands) |
| `data/demos/` | Recorded demonstrations (`.npz`) |
| `data/processed/` | Processed trajectories (`.npz` plus `.png` preview) |
| `data/runs/` | One log per robot run (`.npz` + `.json` + `.png`) and `summary.md` |

---

## B. One-time setup and launching HEAL

Our code lives in the group folder **`~/ITR_2026/Grp8/final/Project_3_ITR_YPK`** on the lab PC.

### B1. Get the code (lab PC, once)
```bash
mkdir -p ~/ITR_2026/Grp8/final && cd ~/ITR_2026/Grp8/final && git clone -b claude/sleepy-bohr-jn2txo https://github.com/yashpatil-bit/Project_3_ITR_YPK.git
```
```bash
cd ~/ITR_2026/Grp8/final/Project_3_ITR_YPK && pip3 install -r requirements.txt
```
```bash
./run_tests.sh
```
This must end with `17 passed`.

> Use `./run_tests.sh`, not plain `python3 -m pytest`. ROS Humble's `launch_testing` pytest
> plugin crashes with newer pytest versions ("Error while loading plugin pytest11"), and the
> script switches plugin loading off. Don't `pip3 install pytest` on the shared lab PC. If you
> did, undo it with `pip3 uninstall -y pytest` after testing.

If you already cloned it, update it with:
```bash
cd ~/ITR_2026/Grp8/final/Project_3_ITR_YPK && git fetch origin && git checkout claude/sleepy-bohr-jn2txo && git pull
```

### B2. Terminals
We use 4 terminals. These names are used everywhere below:

| Name | Used for |
|---|---|
| **T1 (robot)** | ssh into the robot and run `./heal_server` or `./free_drive` |
| **T2 (velocity)** | The velocity controller launch (only while `./heal_server` runs) |
| **T3 (scripts)** | Our scripts |
| **T4 (contact)** | `contact_monitor.py` during runs |

**Every T3/T4 terminal** must source the workspace first, exactly like step 5 of the starter
README, then change into our repo:
```bash
cd ~/Debojit_WS/addverb_ws/
source install/setup.bash
cd ~/ITR_2026/Grp8/final/Project_3_ITR_YPK
```

### B3. Launching HEAL: only the starter README commands
We launch HEAL **only** with the commands from the course starter README
(`starter_code/LAB_COMMANDS.md`), unchanged. Our scripts never start or reconfigure anything on
the robot.

**T1:**
1. Open a terminal and connect (password `cobot`):
   ```bash
   ssh cobot@192.168.1.25
   ```
2. Go to the robot software folder and start it (password `cobot`):
   ```bash
   cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
   ```
   ```bash
   sudo env HOME=/home/cobot ./cobot run
   ```
3. Type `1` and press ENTER.
4. Run **one** of these:

   | Command | What it does | Use it for |
   |---|---|---|
   | `./heal_server` | Robot accepts commands | replay, go_to_start, teleop, gripper |
   | `./free_drive` | Gravity compensation: move the arm by hand | teaching (record_demo), capture_points |
   | `./base_rigid` | Go home | parking the robot |

**T2 (only while `./heal_server` is running):**

5. Source the workspace:
   ```bash
   cd Debojit_WS/addverb_ws/
   source install/setup.bash
   ```
   Then launch the velocity controller:
   ```bash
   ros2 launch addverb_cobot_control velocity_control.launch.py
   ```

### B4. Switching modes
To switch, for example from `./free_drive` (teaching) to `./heal_server` (replay):
1. Stop T2 with Ctrl-C, if it is running.
2. Stop the program in T1 with Ctrl-C.
3. Start the other program in T1.
4. If that is `./heal_server`, start T2 again (step 5).

> The starter README doesn't say how to switch between these programs. Ask the TA to confirm
> that this is correct.

---

## C. Step-by-step procedure in the lab

Do the steps in order. Each step has a **✅ check**. Don't continue until it passes. Times are
estimates for a smooth run; plan for about 1.5× that.

Roles at all times: an **Operator** at the keyboard and a **Buddy** with a hand on the
E-stop.

### Step 1: Code ready (15 min)
Do B1. ✅ `./run_tests.sh` prints `17 passed`.

### Step 2: Launch HEAL (10 min)
Run B3 steps 1–4 with `./heal_server` in T1, then step 5 in T2.
✅ Both run without errors.

### Step 3: Check the connection (5 min)
In T3 (sourced as in B2):
```bash
python3 scripts/check_setup.py
```
✅ The output ends with `RESULT: ready`.
- Note the printed **tool pos**. Measure with a ruler where the gripper fingertips are relative
  to the robot base, to find out whether the URDF frame `end-effector` is the flange or the
  fingertips (needed in Step 7).
- If it fails, see Part H.

### Step 4: Gripper and motion test, no probe, arm away from the wire stand (15 min)
```bash
python3 scripts/gripper.py open
```
```bash
python3 scripts/gripper.py close --force 40
```
```bash
python3 scripts/teleop_teach.py --name test
```
In teleop, tap `w` 3–4 times; the arm should creep a few mm in +x. Then press `x` to quit
without saving.
✅ The gripper opens and closes, and the arm moves slowly and in a sensible direction. If it
jumps or moves the wrong way, the Buddy presses the **E-stop**. Then compare
`robot.joint_names` in the config with the controller's order:
```bash
ros2 param get /velocity_controller joints
```

### Step 5: Free-drive test (10 min). This decides how you teach
Switch modes (B4) to `./free_drive`, then in T3:
```bash
python3 scripts/check_setup.py --watch
```
Move the arm by hand. ✅ The printed pose changes as you move.
If it says "Timed out waiting for JointState", tell the TA. The fallback is keyboard teaching
with `teleop_teach.py` under `./heal_server` (see D7).

### Step 6: Place the wire box (15 min)
- Ask the TA before moving it.
- Best geometry: wire plane **facing the robot**, wire centre about 45–60 cm from the robot
  base, about the height of the gripper.
- Tape the box outline on the table and tape the box down, so it can't slide and can go back to
  exactly the same place.

### Step 7: Mount the probe (20–30 min)
1. Put the ring at the **start (left end)** of the wire and switch the buzzer ON.
2. In `./free_drive`, bring the open gripper to the handle while a teammate holds the probe
   with the ring centred around the wire.
   - The handle should point **straight out of the gripper** along the tool axis, and
     **perpendicular to the wire plane**.
   - Grip the middle of the handle. Wrap tape there if it's thin, and mark the spot.
3. Close the gripper.
   - If the gripper only responds under `./heal_server`, switch modes (B4) first; the robot
     holds its pose.
   - Then:
     ```bash
     python3 scripts/gripper.py close --force 60
     ```
4. Twist the handle gently by hand. It must not rotate in the fingers.
5. Tape the probe's cable loosely along the wrist, with slack, so it can't snag when the wrist
   rolls ±180°.
6. Measure with a ruler from the `end-effector` frame (see Step 3) to the **centre of the
   ring**. Put it in `config/heal_wire_trace.yaml` as `robot.tool_xyz`, usually only z, e.g.
   `tool_xyz: [0.0, 0.0, 0.12]`.

✅ The probe is held firmly and `tool_xyz` is set.

### Step 8: Teach (30–40 min)
1. Switch modes (B4) to `./free_drive`, with T2 stopped.
2. In T3:
   ```bash
   python3 scripts/record_demo.py --name demo_01
   ```
3. Press **Enter** to start, then guide the probe from the start to the end of the wire.
   - Go slowly, about 2–3 cm/s. The script prints your speed and warns above 10 cm/s.
   - Keep the wire in the **middle of the round part** of the ring.
   - In every U-bend, slow down and roll the wrist so the ring stays perpendicular to the wire.
4. Press **Enter** at the end, then keep the arm still.
5. If the buzzer sounded, record again, because replay copies the demo exactly.
6. Make 2–3 demos (`demo_02`, `demo_03`). Between demos, guide the arm back to the start by
   hand; that part is not recorded.

✅ `data/demos/demo_01.npz` exists, recorded without a buzz.

### Step 9: Process (10 min)
```bash
python3 scripts/process_demo.py data/demos/demo_01.npz
```
✅ The printed smoothing deviation is below 1.5 mm. Open the `.png` it prints and check that the
path looks like the wire.

- Output: `data/processed/demo_01_v5.npz` (the `v5` means 5 cm/s top speed).
- Optional: view it in RViz (D9).

### Step 10: First robot runs at 10 % speed (45 min)
1. Switch modes (B4) to `./heal_server` and start T2.
2. In T4:
   ```bash
   python3 scripts/contact_monitor.py --mode keyboard
   ```
   Press Enter whenever the buzzer sounds.
3. In T3. The robot is at the **end** of the wire after teaching, so first bring it back along
   the wire:
   ```bash
   python3 scripts/replay.py data/processed/demo_01_v5.npz --reverse --dry-run
   ```
   ```bash
   python3 scripts/replay.py data/processed/demo_01_v5.npz --reverse --speed 0.1
   ```
4. Then the forward run:
   ```bash
   python3 scripts/replay.py data/processed/demo_01_v5.npz --dry-run
   ```
   ```bash
   python3 scripts/replay.py data/processed/demo_01_v5.npz --speed 0.1
   ```

✅ One full run with no buzz.
- If `replay.py` says the probe is a few mm from the start, run `go_to_start.py` (D5) and try
  again.
- After every run, look at `data/runs/<run>.png`.

### Step 11: Speed up (1.5–2 h)
1. Same file, faster: run `--speed` 0.2 → 0.3 → 0.5 → 1.0. Do one clean run at each level, with
   a `--reverse` run to come back between forward runs.
2. Then a faster file:
   ```bash
   python3 scripts/process_demo.py data/demos/demo_01.npz --vmax 0.08 --amax 0.2 --alat 0.2 --wmax 0.8
   ```
   This writes `data/processed/demo_01_v8.npz`. Start it again at `--speed 0.3`, then go up.
3. Next, try `--wmax 1.0`, then `--vmax 0.10`, and so on. Raise one value at a time.
4. If a run aborts or touches the wire, go back to the last good setting (see Part H).

✅ You know the fastest file and speed that run cleanly.

### Step 12: Final 3 runs. Stage 1 complete (30 min)
1. With the buzzer ON, T4 running, and the video recording:
   ```bash
   python3 scripts/replay.py data/processed/<best>.npz --speed <best> --tag final
   ```
   ```bash
   python3 scripts/replay.py data/processed/<best>.npz --speed 0.5 --reverse --tag back
   ```
   Do the forward and reverse pair 3 times.
2. Evaluate:
   ```bash
   python3 scripts/evaluate_runs.py --tag final
   ```
3. Save to GitHub (Part K).

✅ 3 consecutive successful `final` runs.

### Step 13: Stage 2 bonus, ILC (1.5–2 h)
1. Take the file **one step faster** than your best clean one. Run it at a speed where it runs
   but isn't perfect:
   ```bash
   python3 scripts/replay.py data/processed/demo_01_v10.npz --speed 1.0 --tag ilc0
   ```
2. Learn from that run. Use the run file it printed (`data/runs/ilc0_<date>_<time>.npz`):
   ```bash
   python3 stage2/ilc_update.py data/processed/demo_01_v10.npz data/runs/ilc0_<date>_<time>.npz
   ```
   This creates `data/processed/demo_01_v10_ilc1.npz`.
3. Go back with `--reverse`, then run the new file:
   ```bash
   python3 scripts/replay.py data/processed/demo_01_v10_ilc1.npz --speed 1.0 --tag ilc1
   ```
4. Repeat 3–5 times (`ilc1 → ilc2 → ...`). The "max err" should drop each time.
5. Run the final `_ilcN` file once more "in one go" for the video.

### Step 14: Video and README (1.5 h, can be done outside the lab)
1. Make a video, 3 minutes max: teaching, a slow replay, the final 3 runs with a timer, the
   Stage 2 result, and the Buddy at the E-stop.
2. Paste `data/runs/summary.md` into the Results section of `README.md` and add the video link.
3. Push (Part K).

---

## D. Command reference

All commands run in T3 from `~/ITR_2026/Grp8/final/Project_3_ITR_YPK`, after sourcing (B2). Every script accepts
`-h` for help. Options shared by most scripts:
- `--config FILE`: use another config file (default `config/heal_wire_trace.yaml`).
- `--urdf FILE`: read the robot model from a file if `/robot_description` isn't published.

### D1. `scripts/check_setup.py`: diagnostics, moves nothing
- **Robot mode:** `./heal_server` or `./free_drive`.
- **Checks:** the topics, the URDF, the KDL chain `base_link → end-effector`, joint names and
  limits, the `/joint_states` rate, whether the velocity controller is running, and the current
  tool pose and singular values.

| Option | Meaning |
|---|---|
| `--watch` | Keep printing the tool pose every 0.5 s (use in free-drive) |
```bash
python3 scripts/check_setup.py
```
```bash
python3 scripts/check_setup.py --watch
```

### D2. `scripts/gripper.py`: open or close the gripper
- **Robot mode:** whatever the gripper service needs (test it in Step 4).
- **How:** wraps the starter `GripperController`, which calls the `/gripper_controller/command`
  service.

| Argument / option | Meaning |
|---|---|
| `open` / `close` / `set VALUE` | `set` = partial, 0 = closed, 1 = open |
| `--force F` | Grasp force (driver default 100) |
| `--ws PATH` | Workspace with `install/setup.bash` (default `~/Debojit_WS/addverb_ws`) |
```bash
python3 scripts/gripper.py open
```
```bash
python3 scripts/gripper.py close --force 60
```
```bash
python3 scripts/gripper.py set 0.3
```

### D3. `scripts/record_demo.py`: teach by hand
- **Robot mode:** `./free_drive`. Publishes nothing; it only listens.
- **Keys:** Enter = start recording, Enter = stop and save, `q` + Enter = quit without saving.
  Ctrl-C saves what was recorded.
- **Output:** `data/demos/<name>.npz` (time, joint angles, probe position and orientation) and
  `<name>_effort.npy`.

| Option | Meaning |
|---|---|
| `--name NAME` | File name (default `demo_<date>_<time>`) |
| `--rate HZ` | Sampling rate (default 50 from config) |
```bash
python3 scripts/record_demo.py --name demo_01
```

### D4. `scripts/process_demo.py`: smooth and plan the trajectory (offline)
- **Robot mode:** none. It also works on a laptop.
- **Output:** `data/processed/<demo>_v<cm/s>.npz` plus a `.png` preview (3D path, speed profile,
  hand speed, joint angles).
- **Printed checks:** path length, smoothing deviation (must be < 1.5 mm), tightest bend radius,
  peak speed, duration.

| Option | Meaning | Default (config) |
|---|---|---|
| `--vmax V` | Top linear speed [m/s] | 0.05 |
| `--amax A` | Linear acceleration [m/s²] | 0.10 |
| `--alat A` | Sideways acceleration in bends [m/s²]. Lower = slower in bends | 0.10 |
| `--wmax W` | Probe roll speed [rad/s]. The main limit in U-bends | 0.5 |
| `--smooth S` | Smoothing width [m]. Raise if the demo was shaky | 0.003 |
| `--offset X Y Z` | Shift the whole path [m], e.g. `0 0 0.002` | 0 0 0 |
| `--trim L` | Cut L metres from both ends | 0 |
| `--out FILE` | Output file name | auto |
| `--no-plot` | Skip the PNG | |
```bash
python3 scripts/process_demo.py data/demos/demo_01.npz
```
```bash
python3 scripts/process_demo.py data/demos/demo_01.npz --vmax 0.08 --amax 0.2 --alat 0.2 --wmax 0.8
```

### D5. `scripts/go_to_start.py`: small correction to the start (or end) pose
- **Robot mode:** `./heal_server` and T2.
- **How:** a quintic (smooth) joint move to the recorded joint angles, then a fine Cartesian
  alignment to under 0.5 mm.
- **Limit:** it **refuses moves longer than 2 cm**, because the ring is threaded on the wire.
  To travel along the wire, use `replay.py` (`--reverse` to go back).

| Option | Meaning |
|---|---|
| `--end` | Go to the END of the trajectory (before a `--reverse` run) |
| `--max-dist D` | Allowed distance [m] (default 0.02). Only raise it with the probe off the wire |
| `--max-joint-speed W` | Peak joint speed [rad/s] (default 0.25) |
| `--yes` | Skip the confirmation |
```bash
python3 scripts/go_to_start.py data/processed/demo_01_v5.npz
```
```bash
python3 scripts/go_to_start.py data/processed/demo_01_v5.npz --end
```

### D6. `scripts/replay.py`: execute the trajectory (main script)
- **Robot mode:** `./heal_server` and T2.
- **Before starting:**
  - Checks that the probe is within 5 mm and 6° of the start.
  - Asks "Enter = GO", then counts down 3-2-1.
- **During the run:** DLS-IK velocity control at 100 Hz.
  - It slows the reference down by itself if the probe drifts or the wrist lags.
  - It aborts on any of these: more than 6 mm off the path, more than 3 cm behind, orientation
    error over 11°, leaving the workspace box, a joint near its limit, stale joint states, or
    Ctrl-C.
- **At the end:** ramps to zero and sends zero commands.
- **Output:** a summary in the terminal, plus `data/runs/<tag>_<date>_<time>.npz/.json/.png`.

| Option | Meaning |
|---|---|
| `--speed S` | Speed scale: 0.1 = 10 % (default 0.1). **Always start at 0.1** |
| `--reverse` | Run end → start (go back along the wire) |
| `--dry-run` | Simulate on the real robot model. **The robot does not move** |
| `--stop-on-contact` | Abort when `contact_monitor` reports a contact |
| `--tag NAME` | Name prefix of the run log (e.g. `final`, `ilc0`) |
| `--yes` | Skip the confirmation |
```bash
python3 scripts/replay.py data/processed/demo_01_v5.npz --dry-run
```
```bash
python3 scripts/replay.py data/processed/demo_01_v5.npz --speed 0.1
```
```bash
python3 scripts/replay.py data/processed/demo_01_v5.npz --reverse --speed 0.3
```
```bash
python3 scripts/replay.py data/processed/demo_01_v8.npz --speed 1.0 --tag final
```

### D7. `scripts/teleop_teach.py`: keyboard jog (sanity test, or fallback teaching)
- **Robot mode:** `./heal_server` and T2.
- **Keys** (no Enter needed; the arm moves only while a key is held):

| Keys | Action |
|---|---|
| `w`/`s`, `a`/`d`, `r`/`f` | ±x, ±y, ±z |
| `i`/`k`, `j`/`l`, `u`/`o` | Rotate about x, y, z |
| `1` / `2` | Slower / faster |
| SPACE | Save waypoint |
| `q` | Save and quit |
| `x` | Quit without saving |

- **Output:** `data/demos/<name>.npz` with the waypoints. To process sparse waypoints, set
  `processing.raw_interp: spline` in the config first.

```bash
python3 scripts/teleop_teach.py --name teleop_01
```

### D8. `scripts/contact_monitor.py`: log contacts (run in T4)
Publishes `/wire_contact`; `replay.py` records every contact with its time.

| Option | Meaning |
|---|---|
| `--mode keyboard` | Press Enter when the buzzer sounds (default, always works) |
| `--mode serial --port /dev/ttyACM0` | Arduino on the wire circuit (`hardware/buzz_contact/`, needs `pip3 install pyserial`, **ask the TA first**) |
| `--mode effort --threshold 2.0` | Joint-torque spikes (only if HEAL publishes effort) |
```bash
python3 scripts/contact_monitor.py --mode keyboard
```

### D9. `scripts/publish_path_rviz.py`: look at paths in RViz
Publishes `/wire_trace/planned`, `/wire_trace/demo` and `/wire_trace/actual` as `nav_msgs/Path`.

| Option | Meaning |
|---|---|
| `--demo FILE` | Also show the raw demo |
| `--run FILE` | Also show the measured path of a run (`data/runs/*.npz`) |
```bash
python3 scripts/publish_path_rviz.py data/processed/demo_01_v5.npz --demo data/demos/demo_01.npz
```
Then in another terminal run `rviz2`. Set **Fixed Frame** to `base_link`, and add the Path
displays for the three topics.

### D10. `scripts/capture_points.py`: measure points with the probe
- **Robot mode:** `./free_drive`.
- **Keys:** Enter = save the current probe position (averaged over 20 samples), `q` + Enter =
  finish.
- **Use it for:** the workspace box (`controller.workspace_min/max` in the config), new
  start/goal points, and reference points (Stage 2).
```bash
python3 scripts/capture_points.py --out data/stand_box.npy
```

### D11. `scripts/evaluate_runs.py`: results table (offline)
- **Reports for each run:** speed, status, time, contacts, max and mean path error, jerk, and
  success.
- **Overall:** success rate, best and mean time, and repeatability (spread of the 3 paths).
- **Output:** `data/runs/summary.md`.

| Option | Meaning |
|---|---|
| `--tag NAME` | Only runs whose tag starts with NAME |
| `--include-dry` | Also include dry runs |
| `FILES...` | Specific `.json` files |
```bash
python3 scripts/evaluate_runs.py --tag final
```

### D12. `stage2/ilc_update.py`: learn from a run (Stage 2)
- **Formula:** c ← lowpass(c + gain · error(t + lead)), with the correction capped at 8 mm.
- **Output:** `<traj>_ilcN.npz`.
- **Input:** use forward runs only (not reverse or dry). Aborted runs only update the part of
  the path they covered.

| Option | Meaning | Default |
|---|---|---|
| `--gain` | Learning rate | 0.7 |
| `--lead` | Samples of look-ahead (compensates robot lag) | 5 |
| `--q-sigma` | Smoothing of the correction | 4 |
| `--max-offset` | Maximum correction [m] | 0.008 |
```bash
python3 stage2/ilc_update.py data/processed/demo_01_v10.npz data/runs/ilc0_<date>_<time>.npz
```

### D13. `stage2/learn_dmp.py`: learn one movement from several demos (Stage 2)
- **What it does:** fits a Dynamic Movement Primitive to all the demos, which averages out hand
  shake. With `--start-goal` it generates the path for a new start and end point.
- **Output:** `data/processed/dmp_<n>demos.npz`. Run it like any trajectory.

| Option | Meaning |
|---|---|
| `--start-goal FILE` | `.npy` with 2 points (new start, new end) from `capture_points.py` |
| `--vmax V` | Top speed of the generated trajectory |
| `--n-basis N` | Number of basis functions (default 200) |
```bash
python3 stage2/learn_dmp.py data/demos/demo_01.npz data/demos/demo_02.npz data/demos/demo_03.npz
```

### D14. `stage2/register_stand.py`: adapt to a moved wire stand (Stage 2)
1. Capture at least 3 points on the stand before moving it (`capture_points.py --out
   data/ref_A.npy`).
2. Move the stand, then capture the same points in the same order (`--out data/ref_B.npy`).
3. Run:
   ```bash
   python3 stage2/register_stand.py data/processed/<traj>.npz data/ref_A.npy data/ref_B.npy
   ```
**Ask the TA before moving the stand.**

### D15a. `scripts/report.py`: one block of text for remote help
Prints the code version, key config values, your demos and processed files, and the latest run
summaries. Copy the whole output into the chat.
```bash
python3 scripts/report.py
```

### D15. Practice tools (no robot): `tools/fake_robot.py` and `scripts/simulate_replay.py`
See Part I.

### D16. Starter code we use
- `starter_code/utils/gripper_commands/heal_dynamixel_gripper.py`, used by `gripper.py`.
- `starter_code/utils/quintic_planner.py` (copied to `wire_trace/quintic.py`), used by
  `go_to_start.py`.
- `starter_code/LAB_COMMANDS.md`: the official launch commands (B3).

---

## E. Configuration file reference (`config/heal_wire_trace.yaml`)

Edit it with any text editor. Values are in metres, radians and seconds.

| Key | What it is | When to change |
|---|---|---|
| `robot.tip_link` | URDF frame of the end-effector (`end-effector`) | If check_setup reports 0 joints |
| `robot.joint_names` | Joint order of the velocity controller | If the robot moves the wrong joint |
| `robot.tool_xyz` | **Ring centre relative to `tip_link`** | Step 7 (measure with a ruler) |
| `robot.rate_hz` | Control rate (100 Hz) | Leave it |
| `recording.rate_hz` | Teaching sample rate (50 Hz) | Leave it |
| `processing.v_max` / `a_max` / `a_lat_max` | Speed and accelerations of the planned path | Speed-up (Step 11), or use the command-line options |
| `processing.w_max` / `alpha_max` | Ring roll speed and acceleration in the U-bends | Speed-up, carefully (up to about 1.0) |
| `processing.smooth_sigma` | Smoothing width | If the demo was shaky |
| `processing.raw_interp` | `linear` (hand demos) or `spline` (teleop waypoints) | Only for teleop teaching |
| `controller.kp_pos` / `kp_ori` | Feedback gains (3, 2) | Raise a little if the path error grows in bends; lower if it oscillates |
| `controller.damping` / `damping_max` | DLS damping | Raise if jumpy near a stretched-out arm |
| `controller.qdot_max` | Joint speed limits [rad/s] | Ask the TA for HEAL's limits |
| `controller.speed_scale` | Default `--speed` (0.1) | Leave it |
| `controller.err_abort` | Abort if more than 6 mm off the path | Leave it |
| `controller.workspace_min/max` | Safety box for the probe | Set from `capture_points.py` plus a few cm |

---

## F. Files the scripts produce

| File | Made by | Contents |
|---|---|---|
| `data/demos/<name>.npz` | record_demo / teleop_teach | Time, joint angles, probe position and orientation |
| `data/processed/<name>_v<N>.npz` | process_demo | Planned trajectory at 100 Hz (position, orientation, velocity, joints) |
| `data/processed/<name>_v<N>.png` | process_demo | Preview plots |
| `data/processed/<name>_ilc<K>.npz` | ilc_update | Trajectory with the learned correction |
| `data/runs/<tag>_<date>_<time>.npz` | replay | Full log of one run |
| `data/runs/<tag>_<date>_<time>.json` | replay | Summary: status, time, errors, contacts |
| `data/runs/<tag>_<date>_<time>.png` | replay | Error plot, joint commands, desired vs actual path |
| `data/runs/summary.md` | evaluate_runs | Results table for the README |

---

## G. Safety rules and emergency stop

- Two people at all times. The **Buddy** keeps a hand on the **E-stop** and never types.
- Keep the robot in sight. Nobody inside its reach while it moves.
- Every new or changed trajectory gets `--dry-run` first, then `--speed 0.1`, then a gradual
  increase with one clean run per level.
- Buzzer switch ON during runs.
- Never change the config or code between "Enter = GO" and the end of a run.
- The velocity controller **keeps executing the last command** if a script dies. Our scripts
  always send zeros at the end, but a hard crash can't. If the robot keeps moving: **E-stop**.

**Emergency procedure:**
1. Anything unexpected (wrong direction, contact, strange noise, shaking) → the Buddy presses
   the **E-stop immediately**.
2. The Operator presses Ctrl-C in T3 (and T2).
3. If needed, free the probe by hand in `./free_drive`, then check the wire for damage.
4. Find the cause in `data/runs/<last run>.png/.json` before trying again. Restart from
   `--dry-run` and 10 % speed.

---

## H. Troubleshooting

| Problem | Fix |
|---|---|
| `Error while loading plugin "pytest11"` (launch_testing) | Use `./run_tests.sh` instead of `python3 -m pytest` |
| `ModuleNotFoundError` | `pip3 install -r requirements.txt`; make sure you are inside `~/ITR_2026/Grp8/final/Project_3_ITR_YPK` |
| `Timed out waiting for: URDF` | Velocity controller (T2) not running, or the topic name differs: check `ros2 topic list`. Or save the URDF and use `--urdf file.urdf` |
| `Timed out waiting for: JointState` | Robot program not running, or the terminal wasn't sourced (B2) |
| `KDL chain has 0 joints` | Wrong `robot.tip_link`; check `ros2 run tf2_tools view_frames` |
| Robot moves the wrong joint or direction | `robot.joint_names` order ≠ `ros2 param get /velocity_controller joints` |
| `tool is X mm from trajectory start` | Small (< 2 cm): `go_to_start.py`. At the end: `replay.py --reverse` |
| `go_to_start` refuses (> 2 cm) | Correct: the ring is on the wire. Use `replay.py --reverse`, or free-drive closer |
| Abort "probe X mm off the path" | Lower `--speed`; re-process with lower `--alat`/`--wmax`; or slightly raise `kp_pos` |
| Abort "orientation error too large" | Lower `--wmax` when re-processing; check that the probe isn't slipping in the gripper |
| Abort "lags the reference" | Speed too high for `qdot_max`: lower `--speed` |
| Robot shakes or oscillates | Lower `kp_pos`/`kp_ori`, raise `damping` |
| Smoothing deviation warning | Re-teach more steadily, or lower `--smooth` |
| Buzz although the error plot is < 1 mm | The demo itself was close to the wire there. Re-teach that part, or use `--offset` |
| `control loop late` warnings | Close RViz and browsers on the PC |

---

## I. Practising at home without the robot

No ROS is needed. `tools/fake_robot.py` runs the same scripts against a simulated arm. It has
generic kinematics, not HEAL's.
```bash
./run_tests.sh
```
```bash
python3 scripts/simulate_replay.py --vmax 0.15 --ilc 3
```
```bash
python3 tools/fake_robot.py --make-demo
```
```bash
python3 scripts/process_demo.py data/demos/sim_demo.npz --vmax 0.1
```
```bash
python3 tools/fake_robot.py scripts/go_to_start.py data/processed/sim_demo_v10.npz --yes --max-dist 0.5
```
```bash
python3 tools/fake_robot.py scripts/replay.py data/processed/sim_demo_v10.npz --speed 0.5 --yes
```
```bash
python3 scripts/evaluate_runs.py
```
Plots go to `data/sim/`, `data/processed/` and `data/runs/`. Delete the `sim_*` files before
committing real lab results.

---

## J. Timeline

| Phase | Steps | If smooth | Realistic |
|---|---|---|---|
| Setup and checks | 1–5 | 1 h | 1.5–2 h |
| Probe mount and teaching | 6–9 | 1.5 h | 2–2.5 h |
| First runs and speed-up | 10–11 | 2 h | 3–4 h |
| Final runs (Stage 1 done) | 12 | 0.5 h | 0.5–1 h |
| Stage 2 (ILC) | 13 | 1.5 h | 2–3 h |
| Video and README | 14 | 1.5 h | 1.5 h |
| **Total** | | **about 8 h** | **about 10–13 h** |

**Plan:**
- **Day 1:** Steps 1–10 (goal: one clean run at 10–30 % speed).
- **Day 2:** Steps 11–13.
- **Day 3 (morning):** buffer, then Step 14 and submit.

**Biggest risks:** Step 5 (free-drive may not publish joint states: +1–2 h for keyboard
teaching) and Step 11 (speed tuning).

---

## K0. Working with remote help (Claude can't access the lab PC)
1. **Getting code fixes:** Claude pushes them to GitHub. On the lab PC, run in T3:
   ```bash
   cd ~/ITR_2026/Grp8/final/Project_3_ITR_YPK && git pull
   ```
   Then re-run the command that failed.
2. **Reporting a problem:** send Claude:
   - the **full terminal output** (copy the text, not a photo, when possible)
   - the output of `python3 scripts/report.py`
   - the run plot `data/runs/<run>.png` or the processing preview `.png` (screenshot)
   - a photo or video of the robot if the motion looks wrong
3. **Never edit the code on the lab PC.** Config values Claude asks you to change are the only
   exception. Then `git pull` keeps working. If `git pull` complains about local changes,
   run `git stash`, then `git pull`, then `git stash pop`.

## K. Saving results to GitHub

After a lab session, commit the real demos, processed files, run logs and config:
```bash
cd ~/ITR_2026/Grp8/final/Project_3_ITR_YPK && git add config data && git commit -m "lab session: demos, runs, config"
```
```bash
git push
```
If the push asks for credentials on the lab PC, copy the `data/` folder to a USB stick or your
laptop and push from there.
