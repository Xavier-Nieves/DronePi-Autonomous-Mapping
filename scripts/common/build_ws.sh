#!/bin/bash
# Build ROS 2 workspace

set -e

echo "=================================="
echo "Building ROS 2 Workspace"
echo "=================================="

# Navigate to workspace
cd "$(dirname "$0")/../ws"

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS 2 Humble"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ Sourced ROS 2 Jazzy"
else
    echo "❌ ROS 2 not found!"
    exit 1
fi

# Clean build (optional)
if [ "$1" == "--clean" ]; then
    echo "🧹 Cleaning build artifacts..."
    rm -rf build/ install/ log/
fi

# Build with colcon
echo "🔨 Building packages..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)

# Check build status
if [ $? -eq 0 ]; then
    echo ""
    echo "=================================="
    echo "✅ Build successful!"
    echo "=================================="
    echo ""
    echo "To use the workspace, run:"
    echo "  source ws/install/setup.bash"
    echo "Or use the helper script:"
    echo "  source scripts/source_ws.sh"
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
