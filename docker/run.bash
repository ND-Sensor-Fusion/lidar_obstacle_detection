#!/bin/bash

xhost +local:root
docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name=lidar_obstacle_detection \
    --privileged \
    --net="host" \
    --volume="/dev/shm:/dev/shm" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(realpath ../config):/root/colcon_ws/install/lidar_obstacle_detection/share/lidar_obstacle_detection/config" \
    pang/lidar_obstacle_detection:1 \
    bash
    
