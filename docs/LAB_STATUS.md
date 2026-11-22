# Lab status: findings so far and where we are

> **Historical notes from the first lab session.** The crosswise grip described here was later
> found to be the cause of the corner failures. Current status, failure analysis and plan: see
> `README.md` (§3–§6) and `docs/HEAL_Wire_Trace_Lab_Guide.pdf`.

## What we learned on the real robot

| Finding | Consequence / what we changed |
|---|---|
| `./heal_server` + velocity launch work; `/joint_states` at about 100 Hz; joints `joint1..6` | Our scripts connect fine (`check_setup.py` → `RESULT: ready`) |
| The URDF includes the Robotiq gripper; frame `rq_tool_tip` is between the fingertips; its z axis points out of the gripper | `robot.tip_link: rq_tool_tip` |
| **`./free_drive` publishes nothing to ROS** (no `/joint_states`) | Hand-guided recording isn't possible as-is, so we **teach with the keyboard** (`teleop_teach.py`). Asked the TA how gravity-comp recording is meant to work |
| **HEAL safety stop "Large Velocity for Joint"** during a 0.4 rad/s wrist roll | Every command is capped at `robot.qdot_hard_limit = 0.25 rad/s`; teleop at most 0.15 rad/s; replay slows the reference instead of cutting corners |
| HEAL joint limits: joint2 −43…117°, **joint3 only ±57°**, others ±180° | Holding the probe *straight out* of the gripper is nearly impossible near the wire (4° margin) |
| `find_pose.py` with the real kinematics | **Hold the handle ACROSS the fingertips ("cross_y" grip)**, ring tilted about 45° (fixed, no roll needed), handle horizontal. **Wire centre 45 cm from the base axis, 22 cm high** (current box height), wire plane facing the robot. 35° joint margin. Start pose in `data/wire_start_q.npy` (on the lab PC) |
| Config for the cross grip | `tool_xyz: [0, 0.08, 0]`, `tool_rpy: [-1.5708, 0, 0]`, so all scripts control the **ring centre** |
| `move_joints.py --file data/wire_start_q.npy` with the **box still in the way** | The arm stopped next to the box → HEAL error |
| **After a HEAL error, `/joint_states` reports every joint as exactly 0** (fake "home") | Scripts now refuse to move on all-zero readings. Recovery: full restart with `./base_rigid` (go home); if T2 says `Failed to power on robot`, check the E-stop, then ask the TA to power-cycle |

## Where we are (next steps, in order)
1. Robot back home and healthy: `./base_rigid`, then `./heal_server`, then the T2 launch shows `activate successful`. Then `git pull` and `check_setup.py` → `RESULT: ready`.
2. **Box far away**, then `python3 scripts/move_joints.py --file data/wire_start_q.npy` (empty gripper, open).
   (If `data/wire_start_q.npy` is missing, re-run: `python3 scripts/find_pose.py --offset 0.08 --width 0.30 --wire-height 0.15 --loops 4 --grip cross_y --mode fixed --pitch 0 --height 0.22 --dist 0.45 0.50`.)
3. Bring the box to the gripper: ring on the wire start, handle across the open fingers with the ring 8 cm out and about 45° diagonal, then `gripper.py close --force 60`. Tape the box down.
4. `teleop_teach.py --name test`: `r`/`f` move the ring up and down around the wire.
5. Teach: `teleop_teach.py --name demo_01`, `c` (record), step along the whole wire, `c`, `q`.
6. `process_demo.py` → `replay.py --reverse --dry-run` / `--speed 0.1` → forward runs → speed up → 3 × `--tag final` → `evaluate_runs.py`.
7. Stage 2: ILC loop (INSTRUCTIONS Step 13).
8. Video, README results, pull request to `main`.

## Open questions for the TA
1. How to record during gravity compensation (`./free_drive` publishes no joint states)?
2. Joint speed threshold of "Large Velocity for Joint"?
3. Proper recovery after a robot error (ErrorRecoveryService)?
4. OK to move the wire box (45 cm in front of the robot)?
5. Is the camera expected anywhere?
6. **Deadline:** the brief says November 16. Is there an extension?
