BEETLEBOT / LYRA 
ROS 2 Jazzy | Ubuntu 24.04 | Raspberry Pi robot | source guides consolidated

FILES
  beetlebot_motion.py       measured shapes + 3 m obstacle avoidance
  beetlebot_scan_tools.py   /scan CSV + static occupancy grid PGM
  beetlebot_vision.py       camera red/green and black arrows
  README_FIRST.txt         this quick-start

1. CONNECT (on PC):
   ssh veerobot@192.168.0.BOT_NO
   Replace BOT_NO with the assigned number. Your robot may differ from 125.

2. AFTER LOGIN IN *EVERY* SSH TERMINAL (replace 125):
   source /opt/ros/jazzy/setup.bash
   source ~/lyra_ws/install/setup.bash
   export ROS_DOMAIN_ID=125

3. T1 - START ROBOT, leave process running:
   ros2 launch lyra_bringup robot.launch.py
   Use --show-args to confirm valid launch options; do not start duplicates.

4. T2 - SENSOR, MOTOR STATUS and SAFETY:
   ros2 topic hz /scan
   ros2 topic echo /odometry/filtered --once
   ros2 topic echo /lyra/armed --once
   ros2 topic info /cmd_vel_nav
   ros2 node info /cmd_vel_gate
   ros2 run tf2_ros tf2_echo base_link lidar_link
   IMPORTANT: hold obstacle IN FRONT while stationary to verify the scan front.
   Most supplied guides report LiDAR rotated ~180 degrees on their setup.

5. ON YOUR PC, transfer files (from the extracted kit folder):
   scp beetlebot_*.py veerobot@192.168.0.BOT_NO:~/
   (or create/paste each file using nano on the robot.)

6. ON ROBOT (T2), check syntax:
   python3 -m py_compile ~/beetlebot_motion.py ~/beetlebot_scan_tools.py ~/beetlebot_vision.py

7. DRY RUN (no nonzero Twist published; no motor movement):
   python3 ~/beetlebot_motion.py square --side 0.4
   python3 ~/beetlebot_motion.py avoid --distance 3 --distance-mode path
   python3 ~/beetlebot_vision.py traffic
   python3 ~/beetlebot_vision.py arrow

8. MOVE ONLY WITH SUPERVISED CLEAR FLOOR, CONFIRMED FRONT, ARM AND STOP READY:
   ros2 service call /lyra/arm std_srvs/srv/Trigger '{}'
   ros2 topic echo /lyra/armed --once
   python3 ~/beetlebot_motion.py square --side 0.4 --motion
   python3 ~/beetlebot_motion.py avoid --distance 3 --distance-mode path --motion
   To request 3 metres NET advance along starting heading instead, replace
     --distance-mode path with --distance-mode forward

   Examples:
     python3 ~/beetlebot_motion.py rectangle --length 0.6 --width 0.3 --motion
     python3 ~/beetlebot_motion.py triangle --side 0.4 --motion
     python3 ~/beetlebot_motion.py circle --motion
     python3 ~/beetlebot_motion.py forward --distance 1.0 --motion
     python3 ~/beetlebot_motion.py right --angle 90 --motion
     python3 ~/beetlebot_motion.py zigzag --side 0.3 --motion

9. LiDAR logging and grid (robot STATIONARY for grid):
   python3 ~/beetlebot_scan_tools.py log --seconds 10
   python3 ~/beetlebot_scan_tools.py grid --seconds 15
   ls -lh ~/beetlebot_lab5

10. CAMERA:
   python3 -c 'import cv2, numpy; print("OpenCV OK")'
   ros2 topic hz /camera/image_raw/compressed
   python3 ~/beetlebot_vision.py traffic
   python3 ~/beetlebot_vision.py arrow
   # Only after safe dry run, scan verification and arming add --motion.

EMERGENCY STOP:
   CTRL+C in the running motor program.
   lyra-stop (if available)
   ros2 topic pub -r 20 -t 40 /cmd_vel_nav geometry_msgs/msg/Twist '{}'
   ros2 service call /lyra/disarm std_srvs/srv/Trigger '{}'
   Physical power cutoff if software/network fails.

IMPORTANT:
 - Scripts are newly integrated examples; syntax-checked, NOT tested on your assigned robot.
 - Most earlier documents report /scan and ROS graph working, not guaranteed motor action.
 - lidar_yaw default 180 comes from documented earlier robot, NOT a universal hardware fact.
 - --no-scan-safety bypasses a safety precondition. Never use except where a supervisor
   explicitly authorizes an isolated elevated-wheel demonstration.
 - Do not publish motor commands directly to /cmd_vel to bypass /cmd_vel_gate.
 - Dry run cannot complete movement distance without physical odometry advancement.
 - Wheel/odometry distances and yaw are estimates; floor calibration matters.
