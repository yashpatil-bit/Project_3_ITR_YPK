# Why every keyboard teaching attempt failed at the corners, and how to avoid it

Analysis of all lab attempts, using HEAL's real kinematics (`config/heal_chain.json`, from the
robot's URDF), the measured setup (wire 30 × 15 cm, 4 up-down loops, wire centre about 22 cm above
HEAL's base, ring about 3 cm with a 3–4 mm wire, ring centre about 8 cm beyond the fingertips) and
the logs of every stop.

## 1. What actually stopped the robot

| # | When | Message | Real cause (from the logs) |
|---|---|---|---|
| 1 | Jog test, `n` roll, **no probe** | `Large Velocity for Joint` | Wrist joints commanded up to 0.4 rad/s for a tilt the wrist can barely make (see 2a) |
| 2 | `move_joints` to the start pose | arm touched the box | The path check knew the platform, **not the box**. The pose was at the bottom of the wire |
| 3 | `move_joints` again | E-stop | Same: the box was already in place |
| 4 | Teaching `demo_01` | `Large Velocity for Joint` | `[`/`]` turned joint 6 **with the probe on the wire**. The handle swung, the ring jammed on the wire, the joint was **commanded but did not move** (log: constant command, constant joints), then slipped |
| 5 | Teaching `demo_02` | `Large Velocity for Joint` | Identical to 4: `joint 6 -> 5 deg`, joints frozen at `[2, 5, -17, -41, 9, 4]` for 5 s, then the error |

The command logs prove that our commands were never fast (≤ 0.12 rad/s, smooth). **Stops 4 and 5
were contact**, and HEAL reports a jammed-then-slipping joint as "Large Velocity".

## 2. The three underlying reasons

### a) HEAL's wrist can only tilt ±30°
Joint 5's axis is **30° from joint 4's** (forearm) axis, not 90° as on most arms. Simulated over
20,000 random poses, **the gripper axis is always 60°–90° from the forearm**:
- the gripper can **never point along the forearm** (e.g. straight forward with a straight arm)
- tilting it toward or away from the forearm direction needs **about 2× joint speed** and saturates at ±30°

Every pose we tried that "tilted the gripper forward/sideways" (the drift test, the axial grip
with the wire facing the robot: 4° margin) fought this.

### b) The ring must roll at every corner, unavoidably
Seen from the robot, the wire direction sweeps the **full 180°** over each U (up → across →
down). For **any** fixed ring orientation, the wire is edge-on to the ring at every second
corner (simulated: 88° misalignment). The ring must roll about 90° per top/bottom of each U,
about an axis perpendicular to the wire plane.

### c) With the handle held across the fingers, "rolling the ring" means swinging the whole arm
In the crosswise grip the handle is perpendicular to the gripper axis. Rolling the ring about the
handle means rotating the gripper about a line through the ring, about 30 cm away from the wrist,
so the wrist must travel about ±20 cm sideways while every joint moves. It's slow, near the limits
of (a), and the intuitive key for "turn" (`[`/`]`, joint 6) does the **wrong** rotation: it swings
the handle like a clock hand. At 8 cm, **3–4° of joint 6 moves the ring 5 mm**, which is the
whole clearance. That is exactly stops 4 and 5.

On top of that, keyboard teaching asked a person to translate and roll **at the same time** through
7 corners. Any slip ends in contact.

## 3. What fixes it

1. **Choose a layout where the ring roll is ONE joint: joint 6.** That needs the handle
   **straight out of the gripper** (coaxial with joint 6) and the gripper **perpendicular to the
   forearm** (reason a). Joint 6 then turns the ring **in place**: no swinging, no other joint moves.
   See section 4 for the layout and the simulation.
2. **Let the software roll, not the person:** `teleop_teach.py --auto-roll` turns the ring to
   follow the direction you move. You only steer with the movement keys.
3. **Stop before HEAL does:** the stall detector (teleop, replay, move_joints) stops within about
   0.6 s if a joint is commanded but doesn't move.
4. **Plan moves with the box in mind:** approach from above (`move_joints --approach`), start at
   the **top** of the wire, and keep the box away until the arm is in place.

## 4. Simulation of the layouts (real HEAL kinematics, whole wire traced, ring kept within 40° of perpendicular)

| Layout | Feasible | Joint margin | sigma_min | Joints moving along the wire |
|---|---|---|---|---|
| Crosswise grip, wire facing the robot (taught today) | yes | 39° | 0.061 | **all six**: the arm swings around the ring at every corner |
| Straight-out grip, wire facing the robot | barely | 4° | 0.044 | — |
| Straight-out grip, wire running toward the robot, centred | barely | 7–8° | 0.064 | — |
| **SIDE LAYOUT: straight-out grip, gripper pointing sideways, wire beside the arm** | **93/288 variants** | **18–25°** | **0.11–0.19** | **joints 2, 3 and 6 only.** Joint 1 = 0°, joint 4 = ∓90°, joint 5 = 0° stay constant |

Why the side layout works: the arm reaches straight forward (joint 1 = 0) and the gripper points 90°
sideways. That is **always perpendicular to the forearm**, so reason (a) never bites. The handle is
coaxial with joint 6, so **rolling the ring is joint 6 alone**: the ring turns in place and nothing
swings into the wire (reason c is gone). Moving along the wire is just shoulder and elbow
(joints 2, 3) in a vertical plane.

## 5. The side layout in the lab

```
        (top view)            wire stand, wire running AWAY from the robot
                                   |<- 38 cm ->|
     robot base  O ---- arm reaches straight ahead (+y) ----
                              gripper ==== handle ==== ring  on the wire
                                     (points sideways)
```
- Wire **plane vertical and parallel to the arm's reach direction**, about **38 cm to the side** of
  the line the arm reaches along (measured from that line to the wire).
- The end of the wire nearest the robot is about **30 cm** in front of the base axis; the wire
  centre is about **45 cm**. The box stays at its current height (wire centre about 22 cm).
- Board orientation: the edge **closest to the wire faces the robot's reach line**, so the gripper
  isn't above the board.
- Grip: handle **straight out of the fingers**, gripped near its end; ring centre about 8 cm beyond the
  fingertips (`robot.tool_xyz: [0, 0, 0.08]`; measure and correct).
- Start pose, ring at the top of the leg nearest the robot. The path from home stays ≥ 23 cm above the platform:
  - stand on the **RIGHT** (+x): `move_joints.py --deg 0.3 32.5 -32.4 -89.5 -0.2 -36.2 --approach 0.10`
  - stand on the **LEFT** (−x): `move_joints.py --deg -0.6 32.5 -32.4 89.9 -0.5 14.0 --approach 0.10` (sigma 0.19, margin 25°)
- Teaching: `teleop_teach.py --name demo_side --auto-roll`.
  - **`a`/`d` = along the wire, `r`/`f` = up/down.** Never `w`/`s` (toward the wire).
  - The ring rolls by itself (joint 6). `n`/`m` or `[`/`]` correct it by hand.

## 6. Verification: full replay simulated on HEAL's model (speed caps, stall detector, 30 ms robot lag)

| Stand | Margin | sigma_min | Replay at speed 1.0 | Max path error | Peak joint speed (cap) |
|---|---|---|---|---|---|
| **RIGHT side** (recommended) | **18°** | 0.108 | completed, 112 s, no stall | 0.62 mm | j2 0.18 (0.25), j3 0.16 (0.25), j6 0.135 (0.15) |
| Left side, centre 45 cm | 8° | 0.087 | completed, 116 s | 0.60 mm | similar |
| Left side, centre 40 cm | 15° | 0.125 | — | — | — |

The other group's layout (wire facing the robot, handle straight out, gripper horizontal) was
re-checked with 250 IK seeds per variant: only 2 of 25 variants are feasible (65 cm, 22 cm high).
That's the edge of HEAL's workspace, because the gripper cannot point along the forearm (2a).
The side layout uses the **same grip**, with the stand beside the arm instead.
