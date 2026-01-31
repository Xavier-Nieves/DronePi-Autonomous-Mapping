# MAVROS Container Setup Guide

This guide covers setup and usage of the MAVROS container for PX4/ArduPilot flight controller communication.

## Overview

The MAVROS container provides:
- **MAVROS** - MAVLink to ROS2 bridge
- **Geographic datasets** - For GPS/global positioning
- **Serial device access** - Direct connection to Pixhawk/PX4

MAVROS enables:
- Flight controller telemetry (position, attitude, battery, etc.)
- Sending commands (takeoff, land, waypoints)
- Offboard control mode for autonomous flight
- Integration with LiDAR SLAM via shared ROS2 topics

## Hardware Requirements

- **Flight Controller**
  - PX4 or ArduPilot firmware
  - USB connection (appears as `/dev/ttyACM0`)
  - Or telemetry radio link
- **Computer**: Ubuntu 22.04 with Docker installed

## Quick Start

### 1. Build the Container

```bash
./docker-run.sh mavros build
```

This builds the MAVROS container with all dependencies (~5 minutes first time).

### 2. Connect Flight Controller

Connect Pixhawk/PX4 via USB. Verify device:

```bash
ls -l /dev/ttyACM*
# Should show: /dev/ttyACM0
```

Update `docker/docker-compose.yml` if device is different:
```yaml
devices:
  - /dev/ttyACM0:/dev/ttyACM0  # Change if needed
```

### 3. Start the Container

```bash
./docker-run.sh mavros start
```

### 4. Access the Container

```bash
./docker-run.sh mavros shell
```

## Running MAVROS

Inside the container:

### Basic Connection (USB Serial)

```bash
/root/scripts/mavros/start_mavros.sh /dev/ttyACM0:57600
```

Or manually:
```bash
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=/dev/ttyACM0:57600 \
    -p gcs_url:=udp://@0.0.0.0
```

### Test Connection

In another shell:

```bash
# Check nodes
ros2 node list
# Should show: /mavros

# Check topics
ros2 topic list | grep mavros
# Should show many topics: /mavros/state, /mavros/battery, etc.

# Monitor connection state
ros2 topic echo /mavros/state
# connected: True (if successful)
```

## SITL Testing (Without Hardware)

For testing without physical hardware, use PX4 SITL simulator:

### 1. Clone PX4 (on host machine)

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl_default gazebo
```

### 2. Start PX4 SITL

```bash
# On host or inside container
/root/scripts/mavros/px4_sitl.sh
```

### 3. Connect MAVROS to SITL

```bash
/root/scripts/mavros/start_mavros.sh udp://:14540@localhost:14557
```

## Common MAVROS Topics

### Telemetry (Subscribe)

```bash
/mavros/state                 # Connection state, armed status, mode
/mavros/battery               # Battery voltage and percentage
/mavros/global_position/global  # GPS position (lat, lon, alt)
/mavros/local_position/pose   # Local position in NED frame
/mavros/imu/data              # IMU measurements
/mavros/altitude              # Altitude data
```

### Commands (Publish)

```bash
/mavros/setpoint_position/local    # Position setpoints (offboard mode)
/mavros/setpoint_velocity/cmd_vel  # Velocity commands
/mavros/cmd/arming                  # Arm/disarm
/mavros/set_mode                    # Change flight mode
```

## Example: Monitoring Telemetry

```bash
# View current state
ros2 topic echo /mavros/state

# View GPS position
ros2 topic echo /mavros/global_position/global

# View battery
ros2 topic echo /mavros/battery

# View local position
ros2 topic echo /mavros/local_position/pose
```

## Example: Basic Commands

### Arm the Drone

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### Change Flight Mode

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

## Integration with LiDAR

Run both containers together:

```bash
# Start both
./docker-run.sh all start

# In lidar container
./docker-run.sh lidar shell
ros2 launch point_lio mapping.launch.py

# In mavros container (different terminal)
./docker-run.sh mavros shell
/root/scripts/mavros/start_mavros.sh
```

Topics from both containers are visible across the system:
```bash
ros2 topic list
# Shows both /cloud_registered (from LiDAR) and /mavros/* topics
```

## Troubleshooting

### Flight Controller Not Detected

Check device:
```bash
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*
```

Check permissions:
```bash
sudo chmod 666 /dev/ttyACM0
```

Check USB cable (use data cable, not charge-only cable).

### MAVROS Not Connecting

Check connection URL:
```bash
# For USB serial
-p fcu_url:=/dev/ttyACM0:57600  # PX4 default baud

# For telemetry radio
-p fcu_url:=/dev/ttyUSB0:57600

# For UDP (SITL)
-p fcu_url:=udp://:14540@localhost:14557
```

Check PX4 parameters:
- `MAV_0_CONFIG` - Serial port configuration
- `SER_TEL1_BAUD` - Baud rate

### Geographic Library Dataset Missing

If you see errors about GeographicLib:

```bash
# Inside container
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

### Topics Not Visible

Check ROS_DOMAIN_ID:
```bash
echo $ROS_DOMAIN_ID  # Should be 0 in both containers
```

Verify network mode in `docker-compose.yml`:
```yaml
network_mode: host  # Required for ROS2 DDS discovery
```

## Configuration Files

- **Dockerfile**: [docker/mavros/Dockerfile](../docker/mavros/Dockerfile)
- **Docker Compose**: [docker/docker-compose.yml](../docker/docker-compose.yml)
- **Start script**: [scripts/mavros/start_mavros.sh](../scripts/mavros/start_mavros.sh)

## Flight Modes

Common PX4 modes:
- `MANUAL` - Manual control
- `STABILIZED` - Stabilized flight
- `ALTITUDE` - Altitude hold
- `POSITION` - Position hold (GPS)
- `OFFBOARD` - Offboard control (for autonomous flight)
- `AUTO.TAKEOFF` - Automatic takeoff
- `AUTO.LAND` - Automatic landing

Set mode:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

## Safety Notes

- Always test with props off first
- Use a kill switch when testing autonomous flight
- Monitor battery voltage via `/mavros/battery`
- Implement failsafe in your code
- Start with SITL before flying real hardware

## Next Steps

- [LiDAR Setup](lidar_setup.md) - Configure LiDAR SLAM
- [Flight Procedure](flight_procedure.md) - Pre-flight checklist
- [Raspberry Pi Deployment](pi_deployment.md) - Deploy to Pi
- [PX4 Documentation](https://docs.px4.io/) - PX4 user guide
- [MAVROS Documentation](https://github.com/mavlink/mavros) - MAVROS wiki
