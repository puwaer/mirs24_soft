#!/bin/bash

sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1
source ~/ros2_ws/install/setup.bash

gnome-terminal --tab -- bash -c 'ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1 -v6'
gnome-terminal --tab -- bash -c 'ros2 run odometry_publisher odometry_publisher'
gnome-terminal --tab -- bash -c 'ros2 launch slam_toolbox online_async_launch.py'
gnome-terminal --tab -- bash -c 'ros2 run tf2_ros static_transform_publisher 0 0 0.35 1.57 0 0 base_link laser'
gnome-terminal --tab -- bash -c 'ros2 launch sllidar_ros2 sllidar_s1_launch.py'
