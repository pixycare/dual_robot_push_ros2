#!/bin/bash

# Topic definition for ROS2-Gazebo bridge
# Syntax: /TOPIC@ROS_MSG_TYPE@GZ_MSG_TYPE

ros2 run ros_gz_bridge parameter_bridge \
/J0/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
/J0/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
/J0/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
/J0/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
/J1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
/J1/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
/J1/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
/J1/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry \
/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
