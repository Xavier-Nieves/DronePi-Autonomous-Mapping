#!/bin/bash
set -e

sleep 5
source /opt/ros/jazzy/setup.bash
source /home/dronepi/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash

exec python3 /home/dronepi/unitree_lidar_project/rpi_server/rpi_health_node.py
