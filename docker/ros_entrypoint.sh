#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash

exec "$@"