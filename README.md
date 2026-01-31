# Drone Mapping ROS 2 Project

A comprehensive ROS 2 workspace for autonomous drone mapping using LiDAR SLAM, AI-powered perception, and real-time point cloud processing.

## Features

- **Point-LIO SLAM**: High-performance LiDAR-inertial odometry for real-time mapping
- **Unitree L1 LiDAR**: 4D LiDAR integration with 360° coverage
- **Hailo AI Inference**: Hardware-accelerated object detection
- **Semantic Fusion**: Combining LiDAR and camera data for semantic mapping
- **Mission Control**: Autonomous waypoint navigation and flight planning
- **Health Monitoring**: Real-time system diagnostics and failsafe management

---

## Project Structure

```
drone-mapping-ros2/
│
├── README.md                    # This file
├── .gitignore                   # Git ignore patterns
├── .clang-format                # C++ code formatting
├── .editorconfig                # Editor configuration
├── CODEOWNERS                   # Code ownership & review assignments
│
├── docs/                        # Documentation
│   ├── architecture.md          # System architecture overview
│   ├── lidar_setup.md           # LiDAR container guide
│   ├── mavros_setup.md          # MAVROS container guide
│   ├── pi_deployment.md         # Raspberry Pi deployment
│   ├── ros_graph.md             # ROS 2 nodes and topics
│   ├── hardware_setup.md        # Hardware assembly guide
│   └── flight_procedure.md      # Flight operations manual
│
├── docker/                      # Containerization
│   ├── lidar/
│   │   └── Dockerfile           # LiDAR container (Unitree L1, Point-LIO)
│   ├── mavros/
│   │   └── Dockerfile           # MAVROS container (PX4/ArduPilot)
│   └── docker-compose.yml       # Multi-container orchestration
│
├── scripts/                     # Helper scripts
│   ├── lidar/                   # LiDAR-specific scripts
│   ├── mavros/                  # MAVROS-specific scripts
│   ├── common/                  # Shared scripts
│   │   ├── build_ws.sh          # Build ROS 2 workspace
│   │   ├── source_ws.sh         # Source workspace overlay
│   │   └── record_bag.sh        # Record flight data
│   └── pi/                      # Raspberry Pi deployment scripts
│
├── ws/                          # ROS 2 workspace
│   ├── src/
│   │   ├── drivers/             # Hardware drivers
│   │   │   ├── lidar_driver/
│   │   │   ├── imu_driver/
│   │   │   ├── camera_driver/
│   │   │   └── mavlink_bridge/
│   │   │
│   │   ├── perception/          # SLAM & AI
│   │   │   ├── point_lio_slam/      # ✅ YOUR COMPLETED WORK
│   │   │   ├── hailo_inference/     # AI object detection
│   │   │   └── semantic_fusion/     # LiDAR + Camera fusion
│   │   │
│   │   ├── control/             # Flight control
│   │   │   ├── mission_control/
│   │   │   ├── health_monitor/
│   │   │   └── failsafe_manager/
│   │   │
│   │   ├── system/              # Configuration
│   │   │   ├── launch/          # Launch files
│   │   │   ├── tf_setup/        # TF transforms
│   │   │   ├── params/          # Parameter files
│   │   │   └── logging/         # Logging config
│   │   │
│   │   └── tools/               # Utilities
│   │       ├── bag_tools/       # Bag processing
│   │       ├── cloudcompare_utils/  # Point cloud tools
│   │       └── visualization/   # Custom visualizations
│   │
│   └── colcon.meta              # Build configuration
│
└── .github/
    └── workflows/
        └── build.yml            # CI/CD pipeline
```

---

## Quick Start

### Prerequisites

- **OS**: Ubuntu 22.04 (recommended)
- **ROS 2**: Humble Hawksbill
- **Hardware**:
  - Unitree L1 LiDAR
  - Compatible drone platform (PX4/ArduPilot)
  - Optional: Hailo-8 AI accelerator

### 1. Clone and Setup

```bash
git clone https://github.com/Xavier-Nieves/unitree_lidar_project.git
cd unitree_lidar_project
```

### 2. Build Workspace

```bash
# Make scripts executable
chmod +x scripts/*.sh

# Build all packages
./scripts/build_ws.sh

# Source workspace
source ./scripts/source_ws.sh
```

### 3. Launch System

```bash
# Launch full system (when ready)
ros2 launch system/launch full_system.launch.py

# Or launch individual components
ros2 launch drivers/lidar_driver lidar.launch.py
ros2 launch perception/point_lio_slam mapping.launch.py
```

---

## Current Status

### Completed
- ✅ Point-LIO SLAM integration
- ✅ Unitree L1 LiDAR driver
- ✅ Point cloud processing tools
- ✅ Mesh generation pipeline
- ✅ Docker containerization

### In Progress
- 🔄 Camera driver integration
- 🔄 Hailo AI inference setup
- 🔄 Mission control system
- 🔄 MAVLink bridge

### Planned
- ⏳ Semantic fusion
- ⏳ Health monitoring
- ⏳ Failsafe management
- ⏳ Full flight testing

---

## Team Structure

This project uses [CODEOWNERS](CODEOWNERS) for code review assignments:

- **Perception & SLAM**: @Xavier-Nieves
- **AI & Camera**: @member2
- **Mission Control**: @member3
- **System & Drivers**: @member4

---

## Usage Examples

### Record Flight Data

```bash
# Start recording all topics
./scripts/record_bag.sh

# Stop with Ctrl+C
```

### Process Point Cloud to Mesh

```bash
# Convert bag file to point cloud
python3 ws/src/tools/bag_tools/bag_to_pcd.py data/my_flight.db3

# Generate mesh from point cloud
python3 ws/src/tools/cloudcompare_utils/auto_mesh_generator.py
```

### Monitor System Health

```bash
# Check all nodes
ros2 node list

# Monitor specific topics
ros2 topic hz /cloud_registered
ros2 topic echo /health/status
```

---

## Docker Support

The project uses separate containers for different components, enabling independent development:

### Component Containers

- **LiDAR Container**: Unitree L1 LiDAR, Point-LIO SLAM, point cloud tools
- **MAVROS Container**: PX4/ArduPilot flight controller communication

### Quick Docker Commands

```bash
# Build containers
./docker-run.sh all build          # Build both containers
./docker-run.sh lidar build        # Build only LiDAR container
./docker-run.sh mavros build       # Build only MAVROS container

# Start containers
./docker-run.sh all start          # Start both containers
./docker-run.sh lidar start        # Start only LiDAR container
./docker-run.sh mavros start       # Start only MAVROS container

# Access containers
./docker-run.sh lidar shell        # Shell into LiDAR container
./docker-run.sh mavros shell       # Shell into MAVROS container

# Stop containers
./docker-run.sh all stop           # Stop all containers
```

### Container Communication

Containers use `network_mode: host` enabling ROS2 DDS topic discovery across containers. Topics from both are visible system-wide:

```bash
ros2 topic list  # Shows topics from both containers
```

See detailed guides:
- [LiDAR Setup Guide](docs/lidar_setup.md)
- [MAVROS Setup Guide](docs/mavros_setup.md)
- [QUICKSTART.md](QUICKSTART.md) for step-by-step instructions

---

## Documentation

Comprehensive documentation is available in the [docs/](docs/) directory:

- **[Architecture](docs/architecture.md)**: System design and component overview
- **[ROS Graph](docs/ros_graph.md)**: Nodes, topics, and message flow
- **[Hardware Setup](docs/hardware_setup.md)**: Assembly and wiring guide
- **[Flight Procedure](docs/flight_procedure.md)**: Pre-flight, operations, and post-processing

---

## Hardware Specifications

### Unitree L1 LiDAR
- **Field of View**: 360° × 90°
- **Range**: 0.05m to 30m
- **Point Rate**: 21,600 pts/sec
- **IMU**: Built-in 6-axis
- **Interface**: USB/Ethernet

### Recommended Computing
- **Platform**: Jetson Orin Nano / Raspberry Pi 5
- **RAM**: 8GB minimum
- **Storage**: 128GB SSD
- **Optional**: Hailo-8 AI accelerator (26 TOPS)

---

## Development

### Code Style

- **C++**: Follow `.clang-format` (Google style, 100 char line)
- **Python**: PEP 8 (4 spaces, enforced by `.editorconfig`)
- **CMake**: 2 space indentation
- **YAML**: 2 space indentation

### Building Individual Packages

```bash
cd ws
source /opt/ros/humble/setup.bash

# Build specific package
colcon build --packages-select point_lio_slam

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Running Tests

```bash
cd ws
source install/setup.bash

# Run all tests
colcon test

# View results
colcon test-result --verbose
```

---

## Troubleshooting

### LiDAR Not Detected

```bash
# Check USB connection
ls -l /dev/ttyUSB*

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
```

### Build Failures

```bash
# Clean rebuild
cd ws
rm -rf build/ install/ log/
./scripts/build_ws.sh --clean
```

### TF Transform Issues

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

---

## Contributing

1. Create a feature branch: `git checkout -b feature/my-feature`
2. Make changes following code style guidelines
3. Test thoroughly
4. Submit pull request (auto-assigned via CODEOWNERS)

---

## Resources

### Official Documentation
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Unitree L1 LiDAR](https://www.unitree.com/LiDAR/)
- [Point-LIO Paper](https://onlinelibrary.wiley.com/doi/epdf/10.1002/aisy.202200459)

### Related Projects
- [Point-LIO ROS 2](https://github.com/dfloreaa/point_lio_ros2)
- [Unilidar SDK](https://github.com/unitreerobotics/unilidar_sdk)
- [PX4 Autopilot](https://px4.io/)

---

## License

This project follows the licenses of included packages:
- Point-LIO: GPL-2.0
- Unitree SDK: See repository
- Custom code: MIT (unless otherwise specified)

---

## Support

- **Issues**: [GitHub Issues](https://github.com/Xavier-Nieves/unitree_lidar_project/issues)
- **Discussions**: [GitHub Discussions](https://github.com/Xavier-Nieves/unitree_lidar_project/discussions)
- **Email**: Contact repository owner

---

**Status**: 🚧 Active Development | **ROS 2**: Humble | **Platform**: Ubuntu 22.04