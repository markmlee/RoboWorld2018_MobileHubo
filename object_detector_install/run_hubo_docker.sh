#!/bin/bash
xhost +si:localuser:root
sudo docker run \
		--name=hubo_docker \
		--runtime=nvidia \
		-ti --rm \
		--net=host \
		--env="DISPLAY" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix" \
		--volume="${HOME}/.Xauthority:/home/hubo/.Xauthority:rw" \
		--volume="${HOME}/hubo_object_detector/object_detector:/home/hubo/catkin_ws/src/object_detector/" \
		phibenz/hubo:latest
xhost -local:root # resetting permissions