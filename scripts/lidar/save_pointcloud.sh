#!/bin/bash

# Save Point Cloud Script
# This script helps you save point clouds from the Docker container

echo "Point Cloud Save Helper"
echo "======================="
echo ""

# Create data directory if it doesn't exist
mkdir -p ~/unitree_lidar_project/data

# Find running or recently stopped containers
echo "Looking for LiDAR containers..."
CONTAINER=$(docker ps -a --filter "name=unitree" --format "{{.Names}}" | head -n 1)

if [ -z "$CONTAINER" ]; then
    echo "No unitree container found!"
    echo "Make sure the container is still running or hasn't been removed."
    exit 1
fi

echo "✓ Found container: $CONTAINER"
echo ""

# Check if PCD file exists
if docker exec $CONTAINER test -f /root/ros2_ws/PCD/scans.pcd; then
    echo "✓ Point cloud file found!"
    
    # Copy the file
    echo "Copying scans.pcd to ~/unitree_lidar_project/data/ ..."
    docker cp $CONTAINER:/root/ros2_ws/PCD/scans.pcd ~/unitree_lidar_project/data/
    
    if [ $? -eq 0 ]; then
        echo "✓ Successfully saved!"
        echo ""
        echo "File location: ~/unitree_lidar_project/data/scans.pcd"
        echo "Windows path: \\\\wsl\$\\Ubuntu\\home\\$(whoami)\\unitree_lidar_project\\data\\scans.pcd"
        
        # Check file size
        SIZE=$(ls -lh ~/unitree_lidar_project/data/scans.pcd | awk '{print $5}')
        echo "File size: $SIZE"
    else
        echo "Copy failed!"
        exit 1
    fi
else
    echo "No point cloud file found in container!"
    echo "Make sure you've run Point-LIO and created a scan first."
    exit 1
fi

echo ""
echo "Next steps:"
echo "1. Open CloudCompare on Windows"
echo "2. Load the file from the Windows path shown above"
echo "3. Start processing your 3D map!"
