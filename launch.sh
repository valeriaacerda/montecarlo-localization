#!/bin/bash
# launch.sh — levanta todo para MCL en el almacén

source /opt/ros/humble/setup.bash

# Terminal 1: Gazebo
ign gazebo -v 4 $(dirname "$0")/worlds/warehouse.sdf &
sleep 5

# Terminal 2: Bridge ROS <-> Gazebo
ros2 run ros_gz_bridge parameter_bridge \
  /scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan \
  /odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry \
  /model/puzzlebot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist \
  /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock &

sleep 3
echo "Todo listo. Tópicos disponibles:"
ros2 topic list
