#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/noetic/setup.bash"

# Source Livox SDK workspace if it exists
if [ -f "/livox_sdk/catkin_ws/devel/setup.bash" ]; then
    source "/livox_sdk/catkin_ws/devel/setup.bash"
fi

# Source catkin workspace if it exists
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

# Execute the passed command
exec "$@" 