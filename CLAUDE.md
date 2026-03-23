# Project Context for Claude

## Project Overview
This is a **drone mapping system** combining:
- **Unitree L1 4D LiDAR** with Point-LIO SLAM for odometry
- **MAVROS/PX4** for flight control and communication
- **ROS2 Humble** as the middleware
- Target deployment: **Raspberry Pi 5** (native, no Docker)

## Repository Structure
```
unitree_lidar_project/
├── docker/
│   ├── lidar/Dockerfile      # LiDAR + Point-LIO container
│   ├── mavros/Dockerfile     # MAVROS container
│   └── docker-compose.yml    # Multi-container orchestration
├── scripts/
│   ├── lidar/                # LiDAR-specific scripts
│   ├── mavros/               # MAVROS and simulation scripts
│   │   ├── setup_px4_sitl.sh      # PX4 SITL installation
│   │   ├── start_simulation.sh    # Launch Gazebo hexacopter
│   │   ├── start_mavros_sitl.sh   # Connect MAVROS to SITL
│   │   └── test_connection.sh     # Verify MAVROS topics
│   ├── common/               # Shared utilities
│   └── pi/                   # Raspberry Pi deployment
├── ws/                       # ROS2 workspace
│   └── src/tools/drone_control/simple_flight.py  # Flight demo script
├── docs/
│   ├── simulation_quickstart.md
│   ├── lidar_setup.md
│   ├── mavros_setup.md
│   └── pi_deployment.md
└── docker-run.sh             # Container management wrapper
```

## Key Technical Details

### Ubuntu Version
- **Ubuntu 22.04** required for Point-LIO and LiDAR compatibility
- PX4 v1.14.0 with Gazebo Classic (not new Gazebo/Ignition)

### Docker Architecture
- Two containers (lidar + mavros) with `network_mode: host`
- Shared ROS2 workspace via volume mount
- DDS communication works across containers on same host network

### PX4 SITL Setup
PX4-Autopilot installed at `~/PX4-Autopilot` (outside project folder)
- Branch: v1.14.0 (stable, Ubuntu 22.04 compatible)
- Simulator: Gazebo Classic
- Vehicle: Hexacopter (typhoon_h480)

## Current State / Pending Tasks

### IMMEDIATE: Complete PX4 SITL Installation
The setup script was updated with `PIP_BREAK_SYSTEM_PACKAGES=1` to fix PEP 668 error.

**To complete installation:**
```bash
# Option 1: Clean start (recommended)
rm -rf ~/PX4-Autopilot
./scripts/mavros/setup_px4_sitl.sh

# Option 2: Continue from current state
cd ~/PX4-Autopilot
export PIP_BREAK_SYSTEM_PACKAGES=1
bash ./Tools/setup/ubuntu.sh --no-nuttx
make px4_sitl gazebo-classic
```

### NEXT: Test Simulation (3 terminals)
```bash
# Terminal 1: Start Gazebo with hexacopter
./scripts/mavros/start_simulation.sh hexacopter

# Terminal 2: Start MAVROS (connects to SITL)
./scripts/mavros/start_mavros_sitl.sh

# Terminal 3: Test and fly
./scripts/mavros/test_connection.sh
cd ws/src/tools/drone_control
python3 simple_flight.py
```

### FUTURE TASKS
1. Verify MAVROS connection to simulated hexacopter
2. Test simple_flight.py automation script
3. Integration testing of LiDAR + MAVROS containers together
4. Raspberry Pi native deployment setup

## Common Commands

```bash
# Container management
./docker-run.sh lidar build    # Build LiDAR container
./docker-run.sh mavros build   # Build MAVROS container
./docker-run.sh all start      # Start both containers
./docker-run.sh lidar shell    # Shell into LiDAR container

# Simulation
cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_typhoon_h480
```

## Error Fixes Applied
1. **libignition-rendering3 not found**: Let PX4's ubuntu.sh handle dependencies
2. **PEP 668 externally-managed-environment**: Added `export PIP_BREAK_SYSTEM_PACKAGES=1`

## Git Branches
- `main` - Current working branch
- `backup-before-restructure` - Pre-restructure backup
- Tag: `pre-restructure` - Snapshot before reorganization
