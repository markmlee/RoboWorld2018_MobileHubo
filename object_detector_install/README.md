# hubo_object_detector

## Installation
1. Install [Docker CE](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
2. Install [Nvidia-Docker](https://github.com/NVIDIA/nvidia-docker)  
   Note: Make sure that you have the latest nvidia-driver installed on your system (at time of writing version 390.77). Check [here](http://www.nvidia.com/object/unix.html)
3. Pull the Docker image from phibenz/hubo:latest 
   `sudo docker pull phibenz/hubo:latest` (Note that this is currently a private repo)  
   PW: hubo123
   **TODO**: push docker image to hubos docker hub  
   Alternatively you can build the docker image by yourself `sudo docker build -t hubo:my_build .`
4. Clone this repo into your $HOME folder `cd ~/; git clone https://Kaonashi2@bitbucket.org/Kaonashi2/hubo_object_detector.git`  

## Usage 2D detector
1. Run the docker `./run_hubo_docker.sh`  
   This script mounts the object detector this repo `./object_detector` to `/home/hubo/catkin_ws/src/object_detector`  Consequently all changes in the package are permanent  
   On docker start the catkin workspace is made (`catkin_make`)
2. Source the setup.bash `source /home/hubo/catkin_ws/devel/setup.bash`  
3. Run the object detection module for realsense OR zed with one of:  
   `rosrun object_detector ros_realsense_2d.py`  
   `rosrun object_detector ros_zed_2d.py`  
4. To run zed and realsense object detector in parallel enter the same docker from a new terminal
   1. Enter this docker with `sudo docker exec -it hubo_docker bash`
   2. Source setup.bashes and /.bashrc
      `source /.bashrc`
      `source /opt/ros/kinetic/setup.bash`
      `source /home/hubo/catkin_ws/devel/setup.bash`
   3. Run the other object detector
      `rosrun object_detector ros_zed_2d.py`
    
## Usage 3D detector
Same procedure as 2D detector, but instead run  
`rosrun object_detector ros_realsense_3d.py`  
`rosrun object_detector ros_zed_3d.py`  

## Troubleshooting 
If something does not work try the following  
  *  Source ros setup file `source /opt/ros/kinetic/setup.bash`  
  *  Source .bashrc `source /.bashrc`  
  *  Source catkin setup file `source /home/hubo/catkin_ws/devel/setup.bash`  
  *  Set ROS_MASTER_URI `export ROS_MASTER_URI=http://vision_pc:11311`  
  *  Set ROS_IP `export ROS_IP=10.12.3.5`  
  *  Check that you are mounting the correct locations in the `run_hubo_docker.sh` file  
  *  Check that you are running the correct docker file in `run_hubo_docker.sh`  
  
If you want to use the local network change:
   `export ROS_MASTER_URI=http://localhost:11311`  
   `export ROS_IP=127.0.0.1`  
