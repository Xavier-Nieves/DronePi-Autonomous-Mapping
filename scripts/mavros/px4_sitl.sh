#!/bin/bash
# Start PX4 SITL (Software In The Loop) simulator
# Useful for testing MAVROS without physical hardware
# Note: Requires PX4-Autopilot repository cloned separately

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    echo "Clone it with:"
    echo "  git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot"
    echo "  cd ~/PX4-Autopilot"
    echo "  make px4_sitl_default gazebo"
    exit 1
fi

echo "==============================================="
echo "Starting PX4 SITL Simulator"
echo "==============================================="

cd "$PX4_DIR"
make px4_sitl_default gazebo

# PX4 SITL will listen on UDP port 14540
# Connect MAVROS with: ./start_mavros.sh udp://:14540@localhost:14557
