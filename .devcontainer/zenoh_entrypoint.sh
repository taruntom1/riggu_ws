#!/bin/bash
set -e

# Source the ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Optionally, source any workspace overlay (if you have one)
# source /path/to/workspace/install/setup.bash

# Start rmw_zenohd in the background if not already running
if ! pgrep -x "rmw_zenohd" > /dev/null; then
    ros2 run rmw_zenoh_cpp rmw_zenohd >/dev/null 2>&1 &
fi

# Execute the command passed to the container
exec "$@"
