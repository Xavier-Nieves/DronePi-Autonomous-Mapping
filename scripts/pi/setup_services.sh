#!/bin/bash
# Setup systemd services for auto-start on boot
# Run this after build_workspace.sh

set -e

echo "========================================="
echo "Setting up Systemd Services"
echo "========================================="
echo ""

# Get project root and username
PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
USERNAME=$(whoami)

echo "Project root: $PROJECT_ROOT"
echo "Username: $USERNAME"
echo ""

# Create LiDAR SLAM service
echo "[1/3] Creating LiDAR SLAM service..."
sudo tee /etc/systemd/system/lidar-slam.service > /dev/null <<EOF
[Unit]
Description=LiDAR SLAM Service
After=network.target

[Service]
Type=simple
User=$USERNAME
Environment="ROS_DOMAIN_ID=0"
WorkingDirectory=$PROJECT_ROOT
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source $PROJECT_ROOT/ws_livox/install/setup.bash && source $PROJECT_ROOT/ws/install/setup.bash && ros2 launch point_lio mapping.launch.py"
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Create MAVROS service
echo "[2/3] Creating MAVROS service..."
sudo tee /etc/systemd/system/mavros.service > /dev/null <<EOF
[Unit]
Description=MAVROS Service
After=network.target

[Service]
Type=simple
User=$USERNAME
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyACM0:57600 -p gcs_url:=udp://@0.0.0.0"
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd
echo "[3/3] Reloading systemd..."
sudo systemctl daemon-reload

echo ""
echo "========================================="
echo "Services Created!"
echo "========================================="
echo ""
echo "Available commands:"
echo ""
echo "Enable auto-start on boot:"
echo "  sudo systemctl enable lidar-slam.service"
echo "  sudo systemctl enable mavros.service"
echo ""
echo "Start services now:"
echo "  sudo systemctl start lidar-slam.service"
echo "  sudo systemctl start mavros.service"
echo ""
echo "Check status:"
echo "  sudo systemctl status lidar-slam.service"
echo "  sudo systemctl status mavros.service"
echo ""
echo "View logs:"
echo "  sudo journalctl -u lidar-slam.service -f"
echo "  sudo journalctl -u mavros.service -f"
echo ""
echo "Stop services:"
echo "  sudo systemctl stop lidar-slam.service"
echo "  sudo systemctl stop mavros.service"
echo ""
echo "Disable auto-start:"
echo "  sudo systemctl disable lidar-slam.service"
echo "  sudo systemctl disable mavros.service"
echo ""
