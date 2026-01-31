#!/bin/bash
# Source the ROS 2 workspace
# Usage: source scripts/source_ws.sh

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ws"

# Source ROS 2 base
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ROS 2 not found!"
    return 1
fi

# Source workspace overlay
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "Workspace sourced successfully!"
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "Workspace: $WS_DIR"
else
    echo "Workspace not built yet!"
    echo "Run: ./scripts/build_ws.sh"
    return 1
fi