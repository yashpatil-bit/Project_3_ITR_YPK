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

## 4. Simulation of the layouts (real HEAL kinematics)
(see the table filled in below)
