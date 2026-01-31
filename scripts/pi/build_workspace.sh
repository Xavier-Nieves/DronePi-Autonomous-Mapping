#!/bin/bash
# Build ROS2 workspace on Raspberry Pi
# Run this after install_dependencies.sh

set -e

echo "========================================="
echo "Building ROS2 Workspace"
echo "========================================="
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

# Get project root
PROJECT_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "Project root: $PROJECT_ROOT"
echo ""

# Install Livox SDK2
echo "[1/4] Building Livox SDK2..."
if [ ! -d "Livox-SDK2" ]; then
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
fi

cd Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
cd "$PROJECT_ROOT"

# Install Livox ROS driver
echo "[2/4] Building Livox ROS2 driver..."
if [ ! -d "ws_livox/src/livox_ros_driver2" ]; then
    mkdir -p ws_livox/src
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git \
        ws_livox/src/livox_ros_driver2
fi

cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
cd "$PROJECT_ROOT"

# Clone Unitree LiDAR SDK
echo "[3/4] Setting up Unitree LiDAR SDK..."
if [ ! -d "ws/src/unilidar_sdk" ]; then
    cd ws/src
    git clone https://github.com/unitreerobotics/unilidar_sdk.git
    cd unilidar_sdk/unitree_lidar_sdk
    mkdir -p build && cd build
    cmake .. && make -j$(nproc)
    cd "$PROJECT_ROOT"
fi

# Clone Point-LIO
echo "[4/4] Setting up Point-LIO..."
if [ ! -d "ws/src/point_lio_ros2" ]; then
    cd ws/src
    git clone https://github.com/dfloreaa/point_lio_ros2.git
    cd "$PROJECT_ROOT"
fi

# Build main workspace
echo "Building main workspace..."
cd ws
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/ws_livox/install/setup.bash"
colcon build --symlink-install \
    --packages-select unitree_lidar_ros2 point_lio

cd "$PROJECT_ROOT"

# Setup bashrc
echo "Configuring environment..."
BASHRC_LINES=(
    "source $PROJECT_ROOT/ws_livox/install/setup.bash"
    "source $PROJECT_ROOT/ws/install/setup.bash"
)

for line in "${BASHRC_LINES[@]}"; do
    if ! grep -qF "$line" ~/.bashrc; then
        echo "$line" >> ~/.bashrc
    fi
done

echo ""
echo "========================================="
echo "Build Complete!"
echo "========================================="
echo ""
echo "Workspace has been built successfully."
echo ""
echo "To use the workspace, run:"
echo "  source ~/.bashrc"
echo ""
echo "Or source manually:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $PROJECT_ROOT/ws_livox/install/setup.bash"
echo "  source $PROJECT_ROOT/ws/install/setup.bash"
echo ""
echo "Next steps:"
echo "1. Test LiDAR: ros2 launch unitree_lidar_ros2 lidar.launch.py"
echo "2. Test SLAM: ros2 launch point_lio mapping.launch.py"
echo "3. Setup services: ./scripts/pi/setup_services.sh"
echo ""
