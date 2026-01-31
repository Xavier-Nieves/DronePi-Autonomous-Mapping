# Raspberry Pi Deployment Guide

This guide covers deploying the drone mapping system to a Raspberry Pi for flight operations.

## Why Native Installation on Pi?

For production deployment on Raspberry Pi, we recommend **native ROS2 installation** instead of Docker:

**Reasons:**
- **Resource efficiency**: Docker adds 100-200MB overhead per container
- **Better performance**: Direct hardware access without virtualization layer
- **Lower latency**: Critical for real-time SLAM and flight control
- **Simpler management**: No container orchestration on resource-constrained device

**Development vs Production:**
- **Development (PC)**: Use Docker containers for isolation and flexibility
- **Production (Pi)**: Use native ROS2 for performance and efficiency

## Hardware Requirements

### Recommended Setup

- **Raspberry Pi 5** (8GB RAM)
  - Or Raspberry Pi 4 (8GB RAM minimum)
- **Storage**: 128GB microSD card (Class 10/U3) or NVMe SSD
- **Power**: 5V 5A USB-C power supply (official Pi power supply recommended)
- **Cooling**: Active cooling (fan + heatsink) - REQUIRED for continuous operation

### Connected Hardware

- Unitree L1 LiDAR (USB)
- Pixhawk/PX4 flight controller (USB/UART)
- Optional: Hailo-8 AI accelerator
- Optional: Camera module

## Installation Steps

### 1. Prepare Raspberry Pi

#### Flash OS

Use Raspberry Pi Imager to flash **Ubuntu 22.04 LTS Server (64-bit)**:

1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Choose OS: Ubuntu Server 22.04 LTS (64-bit)
3. Configure:
   - Set hostname: `drone-pi`
   - Enable SSH
   - Set username/password
   - Configure WiFi (optional)
4. Flash to SD card

#### First Boot

```bash
# SSH into Pi
ssh ubuntu@drone-pi.local  # or use IP address

# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y git curl wget build-essential cmake
```

### 2. Install ROS2 Humble

```bash
# Add ROS2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-ros-dev-tools

# Install colcon
sudo apt install -y python3-colcon-common-extensions

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Dependencies

#### LiDAR Dependencies

```bash
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-visualization-msgs \
    libpcl-dev \
    libeigen3-dev
```

#### MAVROS Dependencies

```bash
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-geographic-msgs

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
rm install_geographiclib_datasets.sh
```

### 4. Clone and Build Workspace

```bash
# Clone your project
cd ~
git clone https://github.com/Xavier-Nieves/unitree_lidar_project.git
cd unitree_lidar_project

# Install Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j4
sudo make install
cd ../..

# Install Livox ROS driver
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
cd ws_livox/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
cd ../../..

# Build main workspace
cd ws
source /opt/ros/humble/setup.bash
source ~/unitree_lidar_project/ws_livox/install/setup.bash
colcon build --symlink-install
cd ..

# Source workspace
echo "source ~/unitree_lidar_project/ws_livox/install/setup.bash" >> ~/.bashrc
echo "source ~/unitree_lidar_project/ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Configure Permissions

```bash
# USB devices
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Create udev rules
sudo tee /etc/udev/rules.d/99-drone-devices.rules > /dev/null <<EOF
# Unitree L1 LiDAR
KERNEL=="ttyUSB*", ATTRS{idVendor}=="xxxx", MODE="0666", GROUP="dialout"

# Pixhawk/PX4
KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Log out and back in for group changes to take effect
```

### 6. Create Systemd Services

Create auto-start services for your nodes.

#### LiDAR SLAM Service

```bash
sudo tee /etc/systemd/system/lidar-slam.service > /dev/null <<EOF
[Unit]
Description=LiDAR SLAM Service
After=network.target

[Service]
Type=simple
User=ubuntu
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/unitree_lidar_project/ws_livox/install/setup.bash && source /home/ubuntu/unitree_lidar_project/ws/install/setup.bash && ros2 launch point_lio mapping.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
```

#### MAVROS Service

```bash
sudo tee /etc/systemd/system/mavros.service > /dev/null <<EOF
[Unit]
Description=MAVROS Service
After=network.target

[Service]
Type=simple
User=ubuntu
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyACM0:57600"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
```

#### Enable Services

```bash
sudo systemctl daemon-reload
sudo systemctl enable lidar-slam.service
sudo systemctl enable mavros.service

# Start services
sudo systemctl start lidar-slam.service
sudo systemctl start mavros.service

# Check status
sudo systemctl status lidar-slam.service
sudo systemctl status mavros.service
```

## Performance Optimization

### CPU Governor

Set CPU to performance mode:

```bash
sudo apt install -y linux-tools-generic
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
```

### Swap Configuration

Increase swap for memory-intensive operations:

```bash
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

### Disable Unnecessary Services

```bash
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon
```

## Monitoring

### View Logs

```bash
# LiDAR SLAM logs
sudo journalctl -u lidar-slam.service -f

# MAVROS logs
sudo journalctl -u mavros.service -f

# System logs
dmesg | tail
```

### Check Resource Usage

```bash
# CPU and memory
htop

# Disk usage
df -h

# USB devices
lsusb
```

### ROS2 Topics

```bash
# List all topics
ros2 topic list

# Monitor topic rate
ros2 topic hz /cloud_registered
ros2 topic hz /mavros/state

# Echo topic data
ros2 topic echo /mavros/battery
```

## Troubleshooting

### Service Won't Start

```bash
# Check logs
sudo journalctl -u lidar-slam.service -n 50

# Test manually
source /opt/ros/humble/setup.bash
source ~/unitree_lidar_project/ws/install/setup.bash
ros2 launch point_lio mapping.launch.py
```

### USB Devices Not Detected

```bash
# List USB devices
lsusb

# Check serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check permissions
groups  # Should include 'dialout'
```

### Performance Issues

- Verify active cooling is working (fan running)
- Check CPU temperature: `vcgencmd measure_temp`
- Monitor CPU usage: `htop`
- Consider reducing SLAM parameters for lower CPU usage

### Out of Memory

- Reduce Point-LIO map size parameters
- Increase swap space
- Use Raspberry Pi 5 with 8GB RAM

## Flight Testing Checklist

1. **Pre-flight:**
   - Verify all services running: `sudo systemctl status lidar-slam mavros`
   - Check topics publishing: `ros2 topic hz /cloud_registered /mavros/state`
   - Verify GPS lock: `ros2 topic echo /mavros/global_position/global`
   - Check battery: `ros2 topic echo /mavros/battery`

2. **During flight:**
   - Monitor via laptop connected to same WiFi
   - Log data: `ros2 bag record -a`
   - Watch for warnings in logs

3. **Post-flight:**
   - Download bag files
   - Process point clouds
   - Review logs for errors

## Remote Access

### SSH over WiFi

```bash
ssh ubuntu@drone-pi.local
```

### VNC (for GUI)

```bash
# Install VNC server
sudo apt install -y tightvncserver

# Start VNC
vncserver :1

# Connect from laptop with VNC client to drone-pi:5901
```

### ROS2 Multi-Machine

Set up laptop to receive ROS2 topics from Pi:

**On Pi:**
```bash
export ROS_DOMAIN_ID=0
```

**On Laptop:**
```bash
export ROS_DOMAIN_ID=0
ros2 topic list  # Should see Pi topics
```

## Backup and Updates

### Backup Configuration

```bash
# Backup workspace
tar -czf ~/drone-ws-backup.tar.gz ~/unitree_lidar_project/ws

# Backup system config
sudo tar -czf ~/system-config-backup.tar.gz /etc/systemd/system/*.service
```

### Update Software

```bash
cd ~/unitree_lidar_project
git pull
cd ws
colcon build
sudo systemctl restart lidar-slam.service mavros.service
```

## Next Steps

- [Flight Procedure](flight_procedure.md) - Pre-flight checklist and operations
- [Hardware Setup](hardware_setup.md) - Physical assembly
- [LiDAR Setup](lidar_setup.md) - Development environment
