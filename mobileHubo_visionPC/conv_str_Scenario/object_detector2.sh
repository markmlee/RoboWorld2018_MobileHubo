#!bin/bash

export ROS_IP=127.0.0.1 \
source catkin_ws/devel/setup.bash \
rosrun object_detector ros_realsense_3d.py
