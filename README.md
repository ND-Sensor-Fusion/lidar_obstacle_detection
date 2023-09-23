# NanoDegree Sensor Fusion - Lidar Obstacle Detection - First Project

## Github Actions

![build-check](https://github.com/ND-Sensor-Fusion/lidar_obstacle_detection/actions/workflows/build-check.yml/badge.svg)
![clang-format-check](https://github.com/ND-Sensor-Fusion/lidar_obstacle_detection/actions/workflows/clang-format-check.yml/badge.svg)
![yapf-format-check](https://github.com/ND-Sensor-Fusion/lidar_obstacle_detection/actions/workflows/yapf-format-check.yml/badge.svg)

## Results

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />
<img src="media/lidar_object_detection_demo.gif" width="700" height="400" />

## starter Code

[Starter Code](https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git)

## Overview

- Implemented : Ransac Plane Segmentation
- Implemented : a KDTree 3d Search templated algorithm
- Implemented : Euclidian Clustering templated algorithm
- Implemented : Stream Real Point Cloud Data
- Implemented : Familiaze with templates and pcl library
- Implemented (not asked) : Restructure all the code as a ROS2 humble package and parameterize it
- Implemented (not asked) : Docker Support ( In order to be graded also since i used ros2 depedency )
- Implemented (not asked) : RViz viewer class to easily visualize the pointclouds, clusters and bounding boxes (with a similar interface to the provided pcl viewer)
- Implemented (not asked) : Github action for validating build of the package and c++-clang / python-yapf formatting
- TODO : Minimum Bounding Boxes with yaw only rotation
- TODO : Object Correspondance between consecutive objects

## Issue with pcl viewer

In this version of pcl debian installation ( 1.12 ) and vtk 9.0. There is a problem with the viewer->spinOnce crashing the viewer.

Using viewer->spin() in another thread fixed it for single frames but was causing crushes in streaming.

So i used the RViz as the main viewer

## How to build Docker

```bash
#=== clone the repo from my organization of this Nanodegree
git clone https://github.com/orgs/ND-Sensor-Fusion/repositories -b main
#=== cd into the docker folder
cd lidar_obstacle_detection/docker

#=== Build the Docker Image
# ! If necessary give permissions a.k.a -> chmod +x build.bash
# It takes one argument which is the desired branch
./build.bash main

#=== Run the Image into a container
./run.bash
# Or with docker compose
docker compose up
# to close
docker compose down
#=== Want another terminal?
./exec.bash
```

## How to use

```bash
#=== Run in the Docker Terminal
#=== Main Launch File
ros2 launch lidar_obstacle_detection environment.launch.py use_rviz:=true env_config:=config/environment.yaml
#=== quizes
# ransac2d quiz
ros2 launch lidar_obstacle_detection ransac.launch.py
# cluster quiz
ros2 launch lidar_obstacle_detection cluster.launch.py
```

## Yaml Configuration

See the configuration file config folder named environment.yaml.

**A docker volume for this configuration is provided**

```yaml
/**:
  lidar_obstacle_detection:
    ros__parameters:
      renderScene: false
      simpleHighway: false # true for static example
      publish_input_cloud: true
      data_folder: "data_2"
      downroi:
        filterRes: 0.2
        minx: -1.0
        miny: -7.0
        minz: -3.0 # with -5 we have a cluster below the road?
        mini: 1.0
        maxx: 30.0
        maxy: 7.0
        maxz: 5.0
        maxi: 1.0
      ransac:
        maxIterations: 50
        distanceThreshold: 0.25
      clustering:
        distanceTol: 0.3 #1.0 #for kdtree
        minPoints: 10
        maxPoints: 1000
```

## RViz

RViz will open with a specific configuration ready , toggle the displays to see different point clouds.
