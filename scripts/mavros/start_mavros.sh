#!/bin/bash
# Launch MAVROS node with PX4 connection
# Usage: ./start_mavros.sh [connection_url]
# Example: ./start_mavros.sh /dev/ttyACM0:57600
# Example: ./start_mavros.sh udp://:14540@localhost:14557 (for SITL)

set -e

# Default connection (USB serial to Pixhawk)
CONN_URL="${1:-/dev/ttyACM0:57600}"

echo "==============================================="
echo "Starting MAVROS with connection: $CONN_URL"
echo "==============================================="

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source /root/ros2_ws/install/setup.bash
fi

# Launch MAVROS node
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="$CONN_URL" \
    -p gcs_url:=udp://@0.0.0.0 \
    -p target_system_id:=1 \
    -p target_component_id:=1 \
    -p fcu_protocol:=v2.0

echo "MAVROS node stopped"
