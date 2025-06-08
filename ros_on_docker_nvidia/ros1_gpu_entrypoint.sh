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

# Source the ROS setup bash script
source "/opt/ros/noetic/setup.bash"

# Source catkin workspace if it exists and has been built
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    source "/catkin_ws/devel/setup.bash"
fi

# Execute the command passed to docker run
exec "$@" 