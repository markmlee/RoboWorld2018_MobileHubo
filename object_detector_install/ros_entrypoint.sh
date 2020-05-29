#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# Always make catkin_workspace on startup
cd /home/hubo/catkin_ws
catkin_make
source /.bashrc
source /home/hubo/catkin_ws/devel/setup.bash
cd /home/hubo
exec "$@"