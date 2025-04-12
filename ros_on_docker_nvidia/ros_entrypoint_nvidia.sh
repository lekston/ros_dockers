#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
exec "$@" 