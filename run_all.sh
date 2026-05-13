#!/bin/bash
source /opt/ros/humble/setup.bash

# Terminal 1: Gazebo
gnome-terminal --title="Gazebo" -- bash -c "
  source /opt/ros/humble/setup.bash
  ign gazebo -v 4 ~/montecarlo-localization/worlds/warehouse.sdf
  bash"

sleep 3

# Terminal 2: Bridge
gnome-terminal --title="Bridge" -- bash -c "
  source /opt/ros/humble/setup.bash
  ros2 run ros_gz_bridge parameter_bridge \
    /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
    /model/puzzlebot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
    /model/puzzlebot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
    /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
  bash"

sleep 2

# Terminal 3: TF estático
gnome-terminal --title="TF" -- bash -c "
  source /opt/ros/humble/setup.bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map world
  bash"

sleep 2

# Terminal 4: MCL
gnome-terminal --title="MCL" -- bash -c "
  source /opt/ros/humble/setup.bash
  python3 ~/montecarlo-localization/src/mcl.py
  bash"

sleep 2

# Terminal 5: Teleop
gnome-terminal --title="Teleop" -- bash -c "
  source /opt/ros/humble/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args --remap cmd_vel:=/model/puzzlebot/cmd_vel
  bash"

# Terminal 6: RViz
gnome-terminal --title="RViz" -- bash -c "
  source /opt/ros/humble/setup.bash
  rviz2
  bash"

echo "Todo lanzado. Presiona PLAY en Gazebo."
