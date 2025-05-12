#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/noetic/setup.bash"
if [ -f "/catkin_make_ws/devel/setup.bash" ]; then
    source "/catkin_make_ws/devel/setup.bash"
fi

if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

exec "$@" 