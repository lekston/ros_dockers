#!/bin/bash
set -e

# Setup ROS2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/usr/share/gazebo/setup.bash
exec "$@" 