#!/bin/bash
set -e

# Source the ROS setup bash script
source "/opt/ros/noetic/setup.bash"

# Source catkin workspace if it exists and has been built
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

# Execute the command passed to docker run
exec "$@" 