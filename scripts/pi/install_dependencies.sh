#!/bin/bash
# Install all dependencies for drone mapping system on Raspberry Pi
# Run this script on a fresh Ubuntu 22.04 installation

set -e

echo "========================================="
echo "Drone Mapping System - Pi Installation"
echo "========================================="
echo ""

# Check if running on Ubuntu 22.04
if [ ! -f /etc/lsb-release ]; then
    echo "ERROR: This script requires Ubuntu"
    exit 1
fi

source /etc/lsb-release
if [ "$DISTRIB_RELEASE" != "22.04" ]; then
    echo "WARNING: This script is designed for Ubuntu 22.04. You have $DISTRIB_RELEASE"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "[1/8] Updating system..."
sudo apt update
sudo apt upgrade -y

# Install essential tools
echo "[2/8] Installing essential tools..."
sudo apt install -y \
    git curl wget \
    build-essential cmake \
    python3-pip \
    software-properties-common

# Install ROS2 Humble
echo "[3/8] Installing ROS2 Humble..."
sudo add-apt-repository universe -y
sudo apt update

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-ros-dev-tools \
    python3-colcon-common-extensions

# Install LiDAR dependencies
echo "[4/8] Installing LiDAR dependencies..."
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs \
    libpcl-dev \
    libeigen3-dev

# Install MAVROS
echo "[5/8] Installing MAVROS..."
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-geographic-msgs

# Install GeographicLib datasets
echo "[6/8] Installing GeographicLib datasets..."
wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh

# Configure user permissions
echo "[7/8] Configuring permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Create udev rules
sudo tee /etc/udev/rules.d/99-drone-devices.rules > /dev/null <<EOF
# Unitree L1 LiDAR
KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"

# Pixhawk/PX4
KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# Setup ROS2 environment
echo "[8/8] Setting up ROS2 environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo ""
echo "========================================="
echo "Installation Complete!"
echo "========================================="
echo ""
echo "IMPORTANT: Log out and log back in for group changes to take effect."
echo ""
echo "Next steps:"
echo "1. Log out: logout"
echo "2. Log back in"
echo "3. Clone your project:"
echo "   git clone https://github.com/Xavier-Nieves/unitree_lidar_project.git"
echo "4. Build workspace:"
echo "   cd unitree_lidar_project"
echo "   ./scripts/pi/build_workspace.sh"
echo ""
