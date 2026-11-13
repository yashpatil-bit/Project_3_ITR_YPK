Here are the commands to run HEAL robot:

1. Open a terminal and run:

ssh cobot@192.168.1.25

password: cobot

2. Then run:
cd /home/cobot/addverb_images/gripper_fix_2026-09-08/

now run:

 sudo env HOME=/home/cobot ./cobot run

password: cobot

3. type 1 and press ENTER

4. Now run: 

./heal_server 

or

./base_rigid # to go home 

or 

./free_drive #to manually drive the robot

5. OPEN A FRESH TERMINAL (only when ./heal_server is running) and run:

cd Debojit_WS/addverb_ws/
source install/setup.bash

run the following command to launch the velocity control node:

ros2 launch addverb_cobot_control velocity_control.launch.py
