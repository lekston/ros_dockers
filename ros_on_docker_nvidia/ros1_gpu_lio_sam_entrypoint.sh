#!/bin/bash
set -e

# Create runtime directory for XDG
mkdir -p /tmp/runtime-root
chmod 700 /tmp/runtime-root

# Start D-Bus session if not already running (for GUI apps)
if [ -z "$DBUS_SESSION_BUS_ADDRESS" ]; then
    # Try to start a D-Bus session
    eval $(dbus-launch --sh-syntax --exit-with-session)
    export DBUS_SESSION_BUS_ADDRESS
fi

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