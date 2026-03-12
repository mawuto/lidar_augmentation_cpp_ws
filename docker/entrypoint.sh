#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Execute whatever command is passed to docker run
exec "$@"
