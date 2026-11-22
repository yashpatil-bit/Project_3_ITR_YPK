#!/usr/bin/env python3
"""Builds docs/HEAL_Wire_Trace_Lab_Guide.pdf (python3 tools/make_lab_guide_pdf.py)."""
import os

from reportlab.graphics.shapes import Circle, Drawing, Line, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Paragraph, Preformatted, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "docs", "HEAL_Wire_Trace_Lab_Guide.pdf")
REPO = "~/ITR_2026/Grp8/final/Project_3_ITR_YPK"

ss = getSampleStyleSheet()
H1 = ParagraphStyle("H1", parent=ss["Heading1"], fontSize=15, spaceBefore=10, spaceAfter=6,
                    textColor=colors.HexColor("#1F3A5F"))
H2 = ParagraphStyle("H2", parent=ss["Heading2"], fontSize=12, spaceBefore=8, spaceAfter=4,
                    textColor=colors.HexColor("#1F3A5F"))
B = ParagraphStyle("B", parent=ss["BodyText"], fontSize=9.5, leading=13)
SMALL = ParagraphStyle("S", parent=B, fontSize=8.5, leading=11)
NOTE = ParagraphStyle("N", parent=B, backColor=colors.HexColor("#FFF4D6"), borderPadding=5,
                      borderColor=colors.HexColor("#E0B34A"), borderWidth=0.6, spaceBefore=4, spaceAfter=6)
WARN = ParagraphStyle("W", parent=NOTE, backColor=colors.HexColor("#FDE2E2"), borderColor=colors.HexColor("#D9534F"))
CODE = ParagraphStyle("C", fontName="Courier", fontSize=8.3, leading=10.4, backColor=colors.HexColor("#F2F4F7"),
                      borderPadding=5, borderColor=colors.HexColor("#C9CED6"), borderWidth=0.5,
                      spaceBefore=3, spaceAfter=7, leftIndent=4, rightIndent=4)
TITLE = ParagraphStyle("T", parent=ss["Title"], fontSize=20, textColor=colors.HexColor("#1F3A5F"))
SUB = ParagraphStyle("Sub", parent=B, alignment=TA_CENTER, fontSize=10.5, textColor=colors.HexColor("#555555"))

story = []


def h1(t): story.append(Paragraph(t, H1))
def h2(t): story.append(Paragraph(t, H2))
def p(t, st=B): story.append(Paragraph(t, st))
def code(t): story.append(Preformatted(t.strip("\n"), CODE))
def note(t): story.append(Paragraph(t, NOTE))
def warn(t): story.append(Paragraph(t, WARN))
def sp(h=4): story.append(Spacer(1, h))


def table(rows, widths, header=True, font=8.5):
    data = [[Paragraph(str(c), ParagraphStyle("tc", parent=B, fontSize=font, leading=font + 2.5)) for c in r]
            for r in rows]
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    st = [("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#B8BEC8")),
          ("VALIGN", (0, 0), (-1, -1), "TOP"),
          ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4)]
    if header:
        st += [("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1F3A5F")),
               ("TEXTCOLOR", (0, 0), (-1, 0), colors.white)]
        for i in range(len(data[0])):
            data[0][i] = Paragraph(f"<font color='white'><b>{rows[0][i]}</b></font>",
                                   ParagraphStyle("th", parent=B, fontSize=font))
    t.setStyle(TableStyle(st))
    story.append(t)
    sp(6)


def layout_drawing():
    d = Drawing(170 * mm, 62 * mm)
    s = mm
    d.add(String(2 * s, 58 * s, "TOP VIEW (stand behind the robot, looking where the arm reaches at home)",
                 fontSize=8, fillColor=colors.HexColor("#333333")))
    # robot base
    d.add(Circle(25 * s, 12 * s, 6 * s, fillColor=colors.HexColor("#333333"), strokeColor=None))
    d.add(String(14 * s, 3 * s, "HEAL base", fontSize=8))
    # reach line
    d.add(Line(25 * s, 18 * s, 25 * s, 55 * s, strokeColor=colors.HexColor("#1F3A5F"), strokeDashArray=[3, 2]))
    d.add(String(10 * s, 52 * s, "arm reaches", fontSize=7.5))
    d.add(String(10 * s, 49 * s, "straight ahead", fontSize=7.5))
    # forearm + gripper + handle
    d.add(Line(25 * s, 18 * s, 25 * s, 34 * s, strokeColor=colors.black, strokeWidth=4))
    d.add(Rect(23 * s, 33 * s, 22 * s, 4 * s, fillColor=colors.HexColor("#555555"), strokeColor=None))
    d.add(String(27 * s, 38.5 * s, "gripper points RIGHT", fontSize=7.5))
    d.add(Line(45 * s, 35 * s, 58 * s, 35 * s, strokeColor=colors.HexColor("#8A5A00"), strokeWidth=2))
    d.add(String(46 * s, 30.5 * s, "handle", fontSize=7))
    d.add(Circle(60 * s, 35 * s, 2 * s, fillColor=None, strokeColor=colors.HexColor("#C0392B"), strokeWidth=1.5))
    # wire stand
    d.add(Rect(55 * s, 22 * s, 10 * s, 34 * s, fillColor=colors.HexColor("#F3D9A4"), strokeColor=colors.HexColor("#8A5A00")))
    d.add(Line(60 * s, 25 * s, 60 * s, 53 * s, strokeColor=colors.HexColor("#7F8C8D"), strokeWidth=2))
    d.add(String(67 * s, 50 * s, "wire stand: wire runs AWAY from the robot", fontSize=7.5))
    d.add(String(67 * s, 46.5 * s, "(wire plane vertical, parallel to the reach line)", fontSize=7.5))
    d.add(String(67 * s, 38 * s, "ring on the wire, at the end nearest the robot", fontSize=7.5))
    # dimensions
    d.add(Line(25 * s, 20 * s, 60 * s, 20 * s, strokeColor=colors.HexColor("#C0392B")))
    d.add(String(33 * s, 16.5 * s, "38 cm (reach line -> wire)", fontSize=7.5, fillColor=colors.HexColor("#C0392B")))
    d.add(String(67 * s, 26 * s, "near end of wire ~30 cm in front of the base", fontSize=7.5, fillColor=colors.HexColor("#C0392B")))
    d.add(String(67 * s, 22.5 * s, "wire centre ~45 cm in front, ~22 cm high (box as now)", fontSize=7.5, fillColor=colors.HexColor("#C0392B")))
    return d


# ============================================================================ cover
story.append(Spacer(1, 18))
story.append(Paragraph("HEAL Wire-Trace Trial", TITLE))
story.append(Paragraph("Complete lab guide: starting HEAL, every terminal, every command", SUB))
story.append(Paragraph("ME 639 ITR, Project Task 3, Group 8 &nbsp;|&nbsp; repo branch <font face='Courier'>claude/sleepy-bohr-jn2txo</font>", SUB))
sp(14)
note("<b>Current plan (from the simulations with HEAL's real kinematics):</b> hold the probe handle "
     "<b>straight out of the gripper</b> (like the other group), the gripper points <b>sideways (RIGHT)</b>, and the "
     "wire stand stands <b>beside the arm</b> with the wire running away from the robot. Then the ring roll in "
     "every corner is <b>joint 6 alone</b> (the ring turns in place) and only joints 2, 3 and 6 move. Simulated full replay: "
     "112 s, 0.6 mm path error, 18 degrees joint margin, no singularity, all joints under the speed caps.")
table([
    ["Terminal", "Where", "What runs in it"],
    ["T1 (robot)", "ssh into the robot (cobot@192.168.1.25)", "./heal_server, or ./free_drive, or ./base_rigid (go home)"],
    ["T2 (velocity)", "lab PC", "ros2 launch addverb_cobot_control velocity_control.launch.py (only with ./heal_server)"],
    ["T3 (scripts)", "lab PC", "all our scripts: check, move, gripper, teach, process, replay, evaluate"],
    ["T4 (contact)", "lab PC", "contact_monitor.py during replays (press Enter when the buzzer sounds)"],
], [26 * mm, 52 * mm, 96 * mm])
p("<b>Roles:</b> Operator at the keyboard. <b>Buddy keeps a hand on the E-stop the whole time</b> and never types. "
  "Film everything from the tripod (it becomes the video).")

# ============================================================================ 1 setup
h1("1. One-time setup on the lab PC (T3)")
code(f"""
cd ~/ITR_2026/Grp8/final
git clone -b claude/sleepy-bohr-jn2txo https://github.com/yashpatil-bit/Project_3_ITR_YPK.git
cd {REPO}
pip3 install -r requirements.txt
./run_tests.sh                # must end with: 18 passed
""")
p("Already cloned? Then just update it (do this <b>every time</b> before a session):")
code(f"""
cd {REPO} && git pull
""")
warn("Do not <font face='Courier'>pip3 install pytest</font> on the shared lab PC (it breaks ROS launch_testing). "
     "Always run tests with <font face='Courier'>./run_tests.sh</font>.")

# ============================================================================ 2 start HEAL
h1("2. Starting HEAL (only the starter README commands)")
h2("T1: robot")
code("""
ssh cobot@192.168.1.25                      # password: cobot
cd /home/cobot/addverb_images/gripper_fix_2026-09-08/
sudo env HOME=/home/cobot ./cobot run       # password: cobot
                                            # -> type 1 and press ENTER (container 1)
./base_rigid                                # go HOME first; when the arm is still -> Ctrl-C
./heal_server                               # wait for: "Start writing velocities to
                                            #            control in velocity mode"
""")
p("After <font face='Courier'>./heal_server</font> starts, <b>do not type anything else in T1.</b>")
h2("T2: velocity controller (new terminal, only after heal_server is ready)")
code("""
cd ~/Debojit_WS/addverb_ws/
source install/setup.bash
ros2 launch addverb_cobot_control velocity_control.launch.py
                                            # wait for: "velocity_controller: activate successful"
""")
p("The warnings <i>Could not contact service ... Attempt 1 of 3</i> are normal. "
  "If it says <b>Failed to power on robot</b>, see section 6 (recovery).", SMALL)
h2("T3: our scripts (new terminal)")
code(f"""
cd ~/Debojit_WS/addverb_ws/ && source install/setup.bash && cd {REPO}
git pull
python3 scripts/check_setup.py              # must end with: RESULT: ready
""")
p("The scipy/NumPy version warning and the <i>Unknown tag \"surface\"</i> lines are harmless.", SMALL)
h2("T4: contact monitor (only during replays)")
code(f"""
cd ~/Debojit_WS/addverb_ws/ && source install/setup.bash && cd {REPO}
python3 scripts/contact_monitor.py --mode keyboard     # press Enter whenever the buzzer sounds
""")

# ============================================================================ 3 T3 procedure
h1("3. Everything to do in T3, in order")
h2("Step 1: check (robot at home, heal_server + T2 running)")
code("""
git pull
python3 scripts/check_setup.py              # RESULT: ready ; q values near 0 but NOT exactly 0
""")
h2("Step 2: move the EMPTY gripper to the start pose (wire box AWAY from the arm)")
code("""
python3 scripts/gripper.py open
python3 scripts/move_joints.py --deg 0.3 32.5 -32.4 -89.5 -0.2 -36.2 --approach 0.10
""")
p("It prints the plan (lowest point on the way about 23 cm) and waits for Enter. The arm moves slowly to 10 cm above "
  "the start, then descends straight down. The gripper ends up <b>pointing horizontally to the RIGHT</b>. "
  "Hand on the E-stop. (Stand on the LEFT instead? Use "
  "<font face='Courier'>--deg -0.6 32.5 -32.4 89.9 -0.5 14.0</font>, but RIGHT has more margin.)")
h2("Step 3: place the wire stand (robot does not move)")
story.append(layout_drawing())
sp(4)
p("Wire <b>plane vertical, parallel to the arm's reach direction</b>, about <b>38 cm to the right</b> of the reach line. "
  "Near end of the wire about 30 cm in front of the base, so that the <b>top of the leg nearest the robot is ~8 cm beyond "
  "the open fingertips</b>. Box at its current height. The board edge closest to the wire faces the robot. "
  "<b>Tape the box down.</b> Buzzer switch ON.")
h2("Step 4: grip the probe (like the other group)")
p("Teammate threads the ring at the top of the leg nearest the robot and lays the handle <b>straight out of the open "
  "fingers</b> (coaxial with the gripper), gripped near the end of the handle. Fine-adjust with the jog tool if needed "
  "(1 mm steps, section 4), then:")
code("""
python3 scripts/gripper.py close --force 60
""")
p("Check: the handle does not twist, the ring does not touch the wire (buzzer silent), the cable hangs free. "
  "<b>Measure fingertips -> ring centre</b>; if it is not about 8 cm, change <font face='Courier'>robot.tool_xyz</font> in "
  "<font face='Courier'>config/heal_wire_trace.yaml</font> to <font face='Courier'>[0.0, 0.0, &lt;distance in m&gt;]</font>.")
h2("Step 5: teach (record the whole wire with the keyboard)")
code("""
python3 scripts/teleop_teach.py --name demo_side --auto-roll
""")
table([
    ["Do", "Keys"],
    ["Step size 1 mm (corners) / 2 mm (straight legs)", "1 (smaller)  /  2 (bigger)"],
    ["Start recording", "c   (prints RECORDING ON)"],
    ["Along the wire", "a / d"],
    ["Up / down", "r / f"],
    ["Ring roll in the corners", "automatic (--auto-roll); correct by hand with n / m or [ / ]  (all = joint 6 now)"],
    ["Stop recording, save", "c   then   q"],
    ["NEVER while the ring is on the wire", "w / s (into the wire),  { / } ,  g g (go home)"],
], [72 * mm, 102 * mm])
p("Go slowly and keep the wire in the middle of the ring. <b>If it prints <font face='Courier'>!! BLOCKED</font></b> the ring "
  "is pressing on the wire: step back the way you came, re-centre, continue. If the buzzer sounds: step back, re-centre. "
  "At the far end: <font face='Courier'>c</font>, then <font face='Courier'>q</font> -> "
  "<font face='Courier'>Saved ... -> data/demos/demo_side.npz</font>.")
h2("Step 6: process")
code("""
python3 scripts/process_demo.py data/demos/demo_side.npz
#   -> data/processed/demo_side_v5.npz  (+ .png preview);  smoothing deviation must be < 1.5 mm
""")
h2("Step 7: first replays (start T4 contact monitor first)")
p("After teaching, the robot is at the <b>far end</b> of the wire. Go back along the wire first:")
code("""
python3 scripts/replay.py data/processed/demo_side_v5.npz --reverse --dry-run
python3 scripts/replay.py data/processed/demo_side_v5.npz --reverse --speed 0.3
python3 scripts/replay.py data/processed/demo_side_v5.npz --dry-run
python3 scripts/replay.py data/processed/demo_side_v5.npz --speed 0.3
python3 scripts/replay.py data/processed/demo_side_v5.npz --reverse --speed 0.3
python3 scripts/replay.py data/processed/demo_side_v5.npz --speed 1.0
""")
p("Each run asks <i>Enter = GO</i>, counts 3-2-1, and prints a summary + saves "
  "<font face='Courier'>data/runs/&lt;tag&gt;_&lt;time&gt;.png</font>. If it says the probe is a few mm off the start: "
  "<font face='Courier'>python3 scripts/go_to_start.py data/processed/demo_side_v5.npz</font> (add "
  "<font face='Courier'>--end</font> before a reverse run).")
h2("Step 8: final runs (3 clean in a row, filmed) and evaluation")
code("""
python3 scripts/replay.py data/processed/demo_side_v5.npz --speed 1.0 --tag final
python3 scripts/replay.py data/processed/demo_side_v5.npz --speed 0.5 --reverse --tag back
#   ... repeat the pair 3 times ...
python3 scripts/evaluate_runs.py --tag final    # -> data/runs/summary.md (send to Claude)
""")
h2("Step 9 (bonus, if time): Stage 2 learning (ILC)")
code("""
python3 scripts/replay.py data/processed/demo_side_v5.npz --speed 1.0 --tag ilc0
python3 stage2/ilc_update.py data/processed/demo_side_v5.npz data/runs/ilc0_<date>_<time>.npz
python3 scripts/replay.py data/processed/demo_side_v5.npz --reverse --speed 0.5
python3 scripts/replay.py data/processed/demo_side_v5_ilc1.npz --speed 1.0 --tag ilc1
#   repeat 2-3 times; the error should drop each time
""")

# ============================================================================ 4 keys
h1("4. Jog tool (teleop_teach.py) key reference")
table([
    ["Keys", "Action"],
    ["w/s, a/d, r/f", "Move the ring point +-x, +-y, +-z (base axes; x = right, y = reach direction, z = up)"],
    ["n / m", "Roll the ring about the handle (= joint 6 in the side layout)"],
    ["[ / ]", "Turn the ring in place about the gripper axis (same as n/m in the side layout)"],
    ["i/k, j/l, u/o", "Rotate about base x / y / z (rarely needed)"],
    ["{ / }", "Turn joint 6 only. Probe OFF the wire only"],
    ["1 / 2", "Step size 0.5 / 1 / 2 / 5 mm  and  1 / 2 / 5 / 10 deg"],
    ["c", "Recording on / off (20 Hz)"],
    ["SPACE", "Single waypoint"],
    ["h", "Hold: cancel pending steps"],
    ["g g", "Go home (press twice within 3 s). Probe OFF the wire, gripper open"],
    ["q / x", "Save and quit / quit without saving"],
    ["--auto-roll", "Start option: the ring turns by itself to follow your direction of motion"],
], [32 * mm, 142 * mm])
p("Safety built into the jog tool: it <b>holds</b> instead of moving when a target is unreachable, near a singularity or "
  "a joint limit, and <b>stops (BLOCKED)</b> within ~0.6 s if a joint is commanded but does not move. All commands are "
  "capped per joint (0.25 / 0.25 / 0.25 / 0.10 / 0.10 / 0.15 rad/s).")

h1("5. Other useful commands (T3)")
code("""
python3 scripts/save_pose.py --name grip_pose        # save current joints + coordinates
python3 scripts/move_joints.py --file data/poses/grip_pose.npy --approach 0.10
python3 scripts/move_joints.py --home                # all joints to 0 (probe OFF the wire)
python3 scripts/gripper.py open                      # or: close --force 60
python3 scripts/check_setup.py --watch               # live tool position
python3 scripts/report.py                            # one text block to send to Claude
python3 scripts/publish_path_rviz.py data/processed/demo_side_v5.npz    # RViz
""")

# ============================================================================ 6 recovery
h1("6. Recovery after an E-stop or a robot error")
p("Symptoms: T1 shows <i>Estop pressed</i> or <i>Safety error</i>, T2 shows <i>Robot gone into error</i>, scripts say "
  "<i>joint_states stale</i> or <i>all joints read exactly 0.0</i> (after an error HEAL reports fake zeros).")
code("""
# 0. E-stop released; if the probe is held on the wire: open the gripper or free it by hand
#    (E-stop pressed while touching the robot). Box away from the arm if needed.
# T2:  Ctrl-C
# T1:  Ctrl-C
#      exit                                   (leave the container)
#      sudo env HOME=/home/cobot ./cobot run  -> 1
#      ./base_rigid      -> wait until home -> Ctrl-C
#      ./heal_server     -> wait for "Start writing velocities..."
# T2:  ros2 launch addverb_cobot_control velocity_control.launch.py
# T3:  python3 scripts/check_setup.py         -> RESULT: ready
""")
warn("If T2 still prints <b>Failed to power on robot ... Robot fails to recover from error</b>: the robot needs a "
     "power cycle. <b>Call the TA.</b>")
p("<b>Switching modes</b> (e.g. to free_drive to move the arm by hand): T2 Ctrl-C, T1 Ctrl-C, T1 "
  "<font face='Courier'>./free_drive</font> (hold the arm). Free-drive publishes no joint data and T2 does not work "
  "with it; switch back with T1 Ctrl-C, <font face='Courier'>./heal_server</font>, T2 launch.")

# ============================================================================ 7 safety
h1("7. Safety rules")
for t in [
    "Two people: Operator + Buddy with a hand on the E-stop. Robot always in sight, nobody in its reach.",
    "Every new trajectory: --dry-run first, then --speed 0.3, then faster.",
    "Never move the robot with the jog keys that go INTO the wire (w/s in the side layout) while the ring is threaded.",
    "Never use g g or { } while the probe is on the wire.",
    "move_joints: keep the wire box away until the arm has reached the pose (its path check does not know the box).",
    "Anything unexpected (wrong direction, buzz you cannot fix, strange noise): E-stop first, then section 6.",
]:
    p("&bull; " + t)

h1("8. Troubleshooting")
table([
    ["Message / symptom", "What to do"],
    ["!! BLOCKED: joint N is not moving", "Ring pressing on the wire. Step back the opposite way, re-centre, continue"],
    ["!! target unreachable / near a singularity / joint limit", "Tool is holding. Move back the way you came"],
    ["T1: Safety error : Large Velocity for Joint", "Usually contact (ring jammed then slipped). Recovery (section 6)"],
    ["tool is X mm from trajectory start", "go_to_start.py (small corrections < 2 cm) or replay --reverse"],
    ["probe X mm off the path (replay abort)", "Lower --speed; re-teach that section more centred"],
    ["robot blocked: joint N not moving (replay)", "Contact. E-stop not needed: it already stopped. Check the ring"],
    ["Error while loading plugin pytest11", "Use ./run_tests.sh"],
    ["git pull complains about local changes", "git stash ; git pull ; git stash pop"],
], [70 * mm, 104 * mm])

h1("9. What every script does")
table([
    ["Script", "Purpose"],
    ["scripts/check_setup.py", "Diagnostics: topics, robot model, joint limits, current pose. Moves nothing"],
    ["scripts/move_joints.py", "Safe joint-space move (path checked, approach from above, stall stop)"],
    ["scripts/gripper.py", "Open / close the gripper (starter GripperController)"],
    ["scripts/teleop_teach.py", "Keyboard jog + recording (teaching); --auto-roll"],
    ["scripts/record_demo.py", "Recording while moving by hand (needs joint data; not available in free_drive here)"],
    ["scripts/process_demo.py", "Smooth, spline, speed profile -> data/processed/*.npz + .png"],
    ["scripts/replay.py", "DLS-IK velocity replay with safety, logging, --dry-run, --reverse, --tag"],
    ["scripts/go_to_start.py", "Small correction to the start (or --end) of a trajectory (< 2 cm)"],
    ["scripts/contact_monitor.py", "Logs buzzer contacts (keyboard / serial / effort)"],
    ["scripts/evaluate_runs.py", "Results table: time, contacts, errors, repeatability -> summary.md"],
    ["scripts/save_pose.py", "Save current joints + coordinates, with singularity check"],
    ["scripts/find_pose.py", "Placement search with HEAL's kinematics (also offline: --chain)"],
    ["scripts/report.py", "One text block with code version, config, demos, last runs"],
    ["stage2/ilc_update.py", "Stage 2: learn a correction from a run (ILC)"],
    ["stage2/learn_dmp.py, register_stand.py", "Stage 2: DMP from demos; adapt to a moved stand"],
], [58 * mm, 116 * mm])
p("Full details: <font face='Courier'>INSTRUCTIONS.md</font>, <font face='Courier'>docs/ROOT_CAUSE_ANALYSIS.md</font>, "
  "<font face='Courier'>README.md</font> in the repo.", SMALL)


def footer(c, d):
    c.saveState()
    c.setFont("Helvetica", 7.5)
    c.setFillColor(colors.HexColor("#777777"))
    c.drawString(15 * mm, 9 * mm, "HEAL Wire-Trace Trial - lab guide (Group 8)")
    c.drawRightString(195 * mm, 9 * mm, f"page {d.page}")
    c.restoreState()


doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=15 * mm, rightMargin=15 * mm, topMargin=14 * mm,
                        bottomMargin=16 * mm, title="HEAL Wire-Trace Trial - Lab Guide", author="Group 8")
doc.build(story, onFirstPage=footer, onLaterPages=footer)
print("wrote", OUT)
