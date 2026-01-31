# LiDAR Container Setup Guide

This guide covers setup and usage of the LiDAR container for Unitree L1 LiDAR and Point-LIO SLAM.

## Overview

The LiDAR container provides:
- **Unitree L1 4D LiDAR driver** (via unilidar_sdk)
- **Livox SDK2 and ROS driver** (dependency for Point-LIO)
- **Point-LIO SLAM** (LiDAR-inertial odometry)
- **Point cloud processing tools** (bag conversion, mesh generation)

## Hardware Requirements

- **Unitree L1 LiDAR**
  - Interface: USB or Ethernet
  - USB device typically appears as `/dev/ttyUSB0`
- **Computer**: Ubuntu 22.04 with Docker installed
- **Memory**: 4GB RAM minimum, 8GB recommended
- **Storage**: 10GB free space for Docker image

## Quick Start

### 1. Build the Container

```bash
./docker-run.sh lidar build
```

This builds the LiDAR container with all dependencies (~10 minutes first time).

### 2. Start the Container

```bash
./docker-run.sh lidar start
```

Starts the container in the background with:
- USB device access (`/dev/ttyUSB0`)
- Shared workspace (`ws/`)
- X11 forwarding for RViz visualization

### 3. Access the Container

```bash
./docker-run.sh lidar shell
```

Opens a bash shell inside the running container.

## Running the LiDAR Driver

Inside the container:

```bash
# Source workspace
source /root/ws_livox/install/setup.bash
source /root/ros2_ws/install/setup.bash

# Launch Unitree L1 LiDAR driver
ros2 launch unitree_lidar_ros2 lidar.launch.py
```

Verify topics:
```bash
# In another terminal/shell
ros2 topic list
ros2 topic hz /livox/lidar  # Check publishing rate
```

## Running Point-LIO SLAM

```bash
# Source workspace
source /root/ws_livox/install/setup.bash
source /root/ros2_ws/install/setup.bash

# Launch Point-LIO
ros2 launch point_lio mapping.launch.py
```

Visualize in RViz:
```bash
rviz2
```

Add displays:
- PointCloud2: `/cloud_registered` (mapped point cloud)
- Path: `/path` (odometry trajectory)
- TF frames

## Recording Data

Use the recording script:

```bash
# Inside container
/root/scripts/common/record_bag.sh
```

Or manually:
```bash
ros2 bag record -a  # Record all topics
ros2 bag record /livox/lidar /cloud_registered /path  # Record specific topics
```

## Processing Point Clouds

Convert ROS2 bag to PCD format:

```bash
python3 /root/ros2_ws/src/tools/bag_tools/bag_to_pcd.py /root/data/my_scan.db3
```

Generate mesh from point cloud:

```bash
python3 /root/ros2_ws/src/tools/cloudcompare_utils/auto_mesh_generator.py
```

## Troubleshooting

### LiDAR Not Detected

Check USB connection:
```bash
ls -l /dev/ttyUSB*
```

If no device found:
- Check physical USB connection
- Try different USB port
- Check `docker-compose.yml` devices mapping

Fix permissions:
```bash
sudo chmod 666 /dev/ttyUSB0
```

### No Point Cloud Published

Check driver status:
```bash
ros2 node list  # Verify lidar node is running
ros2 topic hz /livox/lidar  # Should show ~10-20 Hz
```

Check Livox driver logs:
```bash
./docker-run.sh lidar logs
```

### RViz Won't Start

Verify X11 forwarding:
```bash
echo $DISPLAY  # Should show :0 or similar
```

On host machine:
```bash
xhost +local:docker
```

### Build Errors

Clean rebuild:
```bash
./docker-run.sh lidar clean
./docker-run.sh lidar build
```

## Container Management

**Stop container:**
```bash
./docker-run.sh lidar stop
```

**View logs:**
```bash
./docker-run.sh lidar logs
```

**Remove container and image:**
```bash
./docker-run.sh lidar clean
```

## Configuration Files

- **Dockerfile**: [docker/lidar/Dockerfile](../docker/lidar/Dockerfile)
- **Docker Compose**: [docker/docker-compose.yml](../docker/docker-compose.yml)
- **Launch files**: `/root/ros2_ws/src/` (inside container)

## Advanced Usage

### Custom LiDAR Configuration

Edit launch file inside container:
```bash
vim /root/ros2_ws/src/unilidar_sdk/unitree_lidar_ros2/launch/lidar.launch.py
```

### Building Custom Packages

Add your package to `ws/src/` on host:
```bash
ws/src/my_custom_package/
```

Build inside container:
```bash
cd /root/ros2_ws
colcon build --packages-select my_custom_package
source install/setup.bash
```

### Multi-Container Integration

Run LiDAR and MAVROS together:
```bash
./docker-run.sh all start
```

ROS2 DDS enables automatic topic discovery across containers via `network_mode: host`.

## Next Steps

- [MAVROS Setup](mavros_setup.md) - Connect to flight controller
- [Architecture](architecture.md) - System design overview
- [Raspberry Pi Deployment](pi_deployment.md) - Deploy to Pi
