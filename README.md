# Unitree L1 4D LiDAR Docker Setup

This Docker setup provides a containerized environment for working with the Unitree L1 4D LiDAR sensor, following the [Point-LIO ROS2](https://github.com/dfloreaa/point_lio_ros2) implementation which has been tested and verified to work with ROS2 Humble on Ubuntu 22.04.

## Available Docker Images

Two versions are provided:
- **ROS Noetic** (Ubuntu 20.04) - `Dockerfile.ros1` - For ROS1 users
- **ROS2 Humble** (Ubuntu 22.04) - `Dockerfile.ros2` - **RECOMMENDED** - Following Point-LIO setup

## Why ROS2 Humble?

While Unitree's official SDK documentation mentions ROS2 Foxy, we're following the [Point-LIO ROS2](https://github.com/dfloreaa/point_lio_ros2) implementation which has been successfully tested on:
- Ubuntu 22.04
- ROS2 Humble
- With full support for Unitree L1 LiDAR

This approach provides:
- ✅ Modern ROS2 LTS version (Humble)
- ✅ Advanced SLAM capabilities with Point-LIO
- ✅ Verified community implementation
- ✅ Better long-term support

## Prerequisites

1. Docker and Docker Compose installed
2. Unitree L1 LiDAR connected via USB
3. X11 server access for GUI (RViz visualization)

## Finding Your LiDAR Device

First, identify your LiDAR's serial port:

```bash
# List all USB devices
ls -l /dev/ttyUSB*

# Or check dmesg after plugging in the LiDAR
dmesg | grep tty
```

Update the `docker-compose.yml` file with the correct device path (default is `/dev/ttyUSB0`).

## Quick Start

### 1. Build the Docker Image

Using Docker Compose (recommended):
```bash
# Build ROS2 Humble (Point-LIO setup)
docker-compose build unitree-l1-ros2

# Or build ROS Noetic
docker-compose build unitree-l1-ros1
```

Using Docker directly:
```bash
# ROS2 Humble
docker build -f Dockerfile.ros2 -t unitree-l1-lidar:ros2-humble .

# ROS Noetic
docker build -f Dockerfile.ros1 -t unitree-l1-lidar:ros1 .
```

### 2. Enable X11 Forwarding

```bash
xhost +local:docker
```

### 3. Run the Container

```bash
# ROS2 Humble (recommended)
docker-compose run --rm unitree-l1-ros2

# ROS Noetic
docker-compose run --rm unitree-l1-ros1
```

## Using the LiDAR

### ROS2 Humble Setup

Once inside the container, you have multiple options:

#### Option 1: Run Basic LiDAR Driver Only

```bash
# Terminal 1: Launch the LiDAR driver
source /root/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py

# Terminal 2: Visualize in RViz2
docker exec -it unitree_l1_ros2 bash
source /root/ros2_ws/install/setup.bash
rviz2
```

In RViz2:
- Set Fixed Frame to `unilidar_lidar`
- Add PointCloud2 topic: `/unilidar/cloud`
- Add IMU topic: `/unilidar/imu`

#### Option 2: Run Point-LIO SLAM

```bash
# Terminal 1: Launch Point-LIO
source /root/ros2_ws/install/setup.bash
ros2 launch point_lio mapping_unilidar_l1.launch.py

# Terminal 2: Launch the LiDAR driver (in another terminal)
docker exec -it unitree_l1_ros2 bash
source /root/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

**Important**: Keep the LiDAR stationary for the first few seconds to allow IMU initialization!

### ROS Noetic Setup

```bash
# Source the workspace
source /root/catkin_ws/src/unilidar_sdk/unitree_lidar_ros/devel/setup.bash

# Launch the LiDAR node
roslaunch unitree_lidar_ros run.launch

# In a new terminal for RViz
docker exec -it unitree_l1_ros1 bash
source /root/catkin_ws/src/unilidar_sdk/unitree_lidar_ros/devel/setup.bash
rviz
```

## Available ROS Topics

The LiDAR SDK publishes the following topics:

### ROS2
- **Point Cloud**: `/unilidar/cloud` (sensor_msgs/msg/PointCloud2)
- **IMU Data**: `/unilidar/imu` (sensor_msgs/msg/Imu)

### ROS1
- **Point Cloud**: `/unilidar/cloud` (sensor_msgs/PointCloud2)
- **IMU Data**: `/unilidar/imu` (sensor_msgs/Imu)

## Point-LIO Features

When using Point-LIO SLAM, you get:

- ✅ **Robust odometry** under aggressive motions
- ✅ **High-bandwidth** LiDAR-inertial fusion
- ✅ **Real-time mapping** with point cloud accumulation
- ✅ **PCD file export** for post-processing
- ✅ **IMU saturation handling** for dynamic scenarios

### Saving Point Cloud Maps

Point-LIO automatically saves accumulated point clouds to:
```
/root/ros2_ws/PCD/scans.pcd
```

View the saved map:
```bash
pcl_viewer /root/ros2_ws/PCD/scans.pcd
```

Tips for pcl_viewer:
- Press `1`: Random colors
- Press `2`: Color by X values
- Press `3`: Color by Y values
- Press `4`: Color by Z values
- Press `5`: Color by intensity

## Configuration

### LiDAR Configuration

Edit the launch parameters in:
- ROS2: `/root/ros2_ws/src/unilidar_sdk/unitree_lidar_ros2/launch/launch.py`
- ROS1: `/root/catkin_ws/src/unilidar_sdk/unitree_lidar_ros/launch/run.launch`

Key parameters:
- `port`: Serial port (default: `/dev/ttyUSB0`)
- `cloud_topic`: Point cloud topic name
- `imu_topic`: IMU topic name

### Point-LIO Configuration

Edit SLAM parameters in:
```
/root/ros2_ws/src/point_lio_ros2/config/unilidar_l1.yaml
```

Important parameters:
- `lid_topic`: LiDAR topic name
- `imu_topic`: IMU topic name
- `pcd_save_enable`: Enable/disable PCD saving
- `extrinsic_T`: Translation between LiDAR and IMU
- `extrinsic_R`: Rotation matrix between LiDAR and IMU

## Troubleshooting

### Permission Denied on Serial Port

```bash
# On the host system
sudo chmod 666 /dev/ttyUSB0

# Or add your user to dialout group (requires logout/login)
sudo usermod -a -G dialout $USER
```

### Display Issues with RViz

```bash
# On host
xhost +local:docker

# Inside container, verify
echo $DISPLAY
```

### Device Not Found

- Verify LiDAR is connected: `ls -l /dev/ttyUSB*`
- Check device path in `docker-compose.yml`
- Ensure container has `privileged: true`

### Point-LIO Not Starting

- **Keep LiDAR stationary** during the first 3-5 seconds for IMU initialization
- Verify topics are publishing: `ros2 topic list`
- Check topic data: `ros2 topic echo /unilidar/imu`

### Build Failures

If you encounter build issues:

```bash
# Clean and rebuild
cd /root/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

## LiDAR Specifications

- **Model**: Unitree L1 4D LiDAR
- **Field of View**: 360° horizontal × 90° vertical
- **Range**: 0.05m to 30m (90% reflectivity)
- **Point Rate**: 21,600 points/second
- **IMU**: Built-in 6-axis (3-axis accelerometer + 3-axis gyroscope)
- **Interface**: USB serial (typically /dev/ttyUSB0)
- **Communication**: Serial or Ethernet (default: Serial)

## Advanced Usage

### Recording Data

```bash
# Record LiDAR and IMU data
ros2 bag record /unilidar/cloud /unilidar/imu

# Play back later
ros2 bag play your_bag_file.db3
```

### Custom Development

The SDK and Point-LIO code is located at:
- Unilidar SDK: `/root/ros2_ws/src/unilidar_sdk/`
- Point-LIO: `/root/ros2_ws/src/point_lio_ros2/`

After modifications, rebuild:
```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Using Multiple Terminals

Open additional shells in the running container:
```bash
docker exec -it unitree_l1_ros2 bash
```

## Data Storage

Point cloud data and maps are saved to the `./data` directory mounted at `/root/data` inside the container.

## Cleaning Up

Stop and remove containers:
```bash
docker-compose down
```

Remove images:
```bash
docker rmi unitree-l1-lidar:ros1
docker rmi unitree-l1-lidar:ros2-humble
```

Revoke X11 access:
```bash
xhost -local:docker
```

## Resources

- [Unitree L1 Official Page](https://www.unitree.com/LiDAR/)
- [Unilidar SDK GitHub](https://github.com/unitreerobotics/unilidar_sdk)
- [Point-LIO ROS2 GitHub](https://github.com/dfloreaa/point_lio_ros2)
- [Point-LIO Original Paper](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459)

## What's Included

This Docker setup includes:

1. **ROS2 Humble** (or ROS Noetic for ROS1)
2. **Unitree Unilidar SDK** - For L1 LiDAR driver
3. **Livox ROS Driver 2** - Required dependency
4. **Point-LIO ROS2** - Advanced SLAM algorithm
5. **All dependencies** - PCL, Eigen, visualization tools

## Contributing

Found an issue or have an improvement? Feel free to:
- Open an issue
- Submit a pull request
- Share your experience

## License

This setup follows the licenses of the included packages:
- Point-LIO: GPL-2.0
- Unilidar SDK: Check repository
- ROS packages: Various open-source licenses
