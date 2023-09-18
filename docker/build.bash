#!/bin/bash

if [ -z "$1" ]
then
  echo "No argument supplied. Please provide BRANCH as an argument."
  exit 1
fi

BRANCH=$1

if [[ $3 == "--no-cache" ]]; then
  echo "Building docker image without cache..."
  docker build --no-cache --build-arg DUMMY=`date +%s` -t pang/lidar_obstacle_detection:1 . --build-arg BRANCH=$BRANCH
else
    echo "Building docker image with cache..."
    docker build --build-arg DUMMY=`date +%s` -t pang/lidar_obstacle_detection:1 . --build-arg BRANCH=$BRANCH
fi
