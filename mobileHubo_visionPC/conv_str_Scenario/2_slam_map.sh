#!bin/bash

cd /home/rainbow/Desktop/hubo_slam_ws
. source
cd /home/rainbow/Desktop/hubo_slam_ws/src/microstrain_3DM_GX5
. setup_device.sh
cd /home/rainbow/Desktop/hubo_slam_ws/src/hubo_slam/launch

roslaunch demo_hubo_2d.launch
