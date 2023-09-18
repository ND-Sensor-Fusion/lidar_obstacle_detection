#!/bin/bash

xhost +local:root

docker exec -it \
    lidar_obstacle_detection \
    bash
