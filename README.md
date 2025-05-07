# drone_racing_group_1

Group 1 common project. Alexander Gävert, Eero Kuusisto, Elias Khabbal and Santtu Toivomäki

The working code can be found in **src/tello_ros/cameramask/cameramask**

**How to run:**
  1. Open 5 terminals
  2. Do source /opt/ros/galactic/setup.bash AND source install/setup.bash on every terminal
  3. On terminal 1 do ros2 launch tello_driver teleop_launch.py
  4. On terminal 2 first do cd src/tello_ros/cameramask/cameramask/ and then do python3 cameramask.py
  5. On terminal 3 do ros2 service call /tello_action tello_msgs/TelloAction "{cmd: 'takeoff'}"
  6. On terminal 4 do ros2 run cameramask fly_through_gate
  7. On terminal 5 first do cd src/tello_ros/cameramask/cameramask/ and then do python3 fly_through_gate.py
