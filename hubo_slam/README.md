# hubo_mapping
Configuration files for hubo + cartographer

## Usage:
* `roslaunch hubo_slam demo_hubo_3d.launch` (launches IMU + Velo drivers + SLAM + viz)
* `roslaunch hubo_slam demo_hubo_3d_localization.launch` (WIP, will launch IMU + Velo drivers + Pure Localization + viz)
* `roslaunch hubo_slam offline_hubo_3d.launch` (This launches SLAM to process data in a rosbag as quickly as possible)
