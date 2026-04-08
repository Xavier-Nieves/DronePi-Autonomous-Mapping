# DronePi — Autonomous LiDAR Mapping Drone

**Institution:** University of Puerto Rico, Mayaguez (UPRM)
**Platform:** Raspberry Pi 5 · Ubuntu 24.04 · ROS 2 Jazzy
**Repository:** [Xavier-Nieves/unitree_lidar_project](https://github.com/Xavier-Nieves/unitree_lidar_project)


---

## Overview

DronePi is a capstone engineering project that builds a fully autonomous drone capable of flying a GPS survey mission over a target structure, capturing LiDAR geometry and pose simultaneously, and producing a textured 3D mesh viewable in a browser — with zero manual steps required between landing and mesh display.

The core mapping pipeline runs entirely on a Raspberry Pi 5 companion computer and remains usable with no cloud dependency. All local processing, serving, and visualization happen on the drone itself or on a laptop connected to its local Wi-Fi hotspot. An optional Foxglove cloud layer now exists for post-flight recording upload and replay, but it is additive and does not gate the local deliverable path.

---

## System Architecture

The full pipeline now flows through eight layers, from raw hardware input to browser-rendered 3D output, with an optional Foxglove cloud sync layer after local publish.

```
Hardware Layer
  Pixhawk 6X (flight controller)
  Unitree 4D L1 (LiDAR sensor)
  M9N GPS (secondary augmentation / geotagging)
  IMX477 camera (installed, capture path active)
  Hailo-8 HAT+ (optical flow / ground class path written)
        |
        v
ROS 2 + MAVROS Bridge
  MAVROS              /mavros/state + mission + tuning topics
  Point-LIO SLAM      /cloud_registered + /aft_mapped_to_init
  SLAM bridge         /mavros/vision_pose/pose + /mavros/odometry/out
  collision_monitor   obstacle zones + downward range estimate
        |
        v
Flight Stack (Pi 5, Ubuntu 24.04)
  drone_watchdog.py   stack supervisor + postflight trigger + LED/buzzer IPC
  main.py             MODE 1 / MODE 2 / MODE 3 orchestrator
  camera_capture.py   persistent IMX477 capture with waypoint trigger
  ros2 bag record     LiDAR + SLAM + MAVROS + camera/Hailo topics -> .mcap
        |
        v
Post-Processing Pipeline
  run_postflight.py       finds latest bag, triggers pipeline
  postprocess_mesh.py     7-stage orchestrator
  flight_logger.py        flight_history.log entry
        |
        v
mesh_tools + texture_tools
  BagReader          .mcap -> point array
  MLSSmoother        noise removal
  GroundClassifier   SMRF terrain split
  DTMBuilder         Delaunay 2.5D ground mesh
  DSMBuilder         Ball Pivoting surface mesh
  MeshMerger         merges DTM + DSM
  TextureProjection  camera replay -> per-vertex color
  Publisher          mesh_final.ply / textured_mesh.ply + JSON outputs
        |
        v
Outputs + HTTP Serving
  PLY outputs        combined_cloud.ply, mesh_final.ply, textured_mesh.ply
  JSON manifests     metadata.json per session, latest.json for browser poll
  serve.py           HTTP port 8080, /api/flights + static PLY
        |
        v
3D Viewers
  meshview.html      Pi browser viewer, live + mesh tabs
  local_test.html    offline laptop viewer, drag-and-drop flight folder
        |
        v
Optional Foxglove Cloud Sync
  foxglove_bridge    live ROS 2 websocket viewer (:8765)
  foxglove-agent     post-flight MCAP upload / cloud replay (verification pending)
```

All services start automatically on boot via systemd: hotspot, mesh-server, foxglove bridge, MAVROS, and drone-watchdog — no user interaction required after power-on.

---

## Hardware

| Component | Model | Role |
|---|---|---|
| Flight computer | Raspberry Pi 5 (16 GB, 3.0 GHz OC) | Companion computer — runs full stack |
| Flight controller | Pixhawk 6X (PX4 firmware) | Stabilization, OFFBOARD control |
| LiDAR | Unitree 4D L1 | 360-degree LiDAR-inertial SLAM |
| GPS | M9N | Global position for survey missions |
| Camera | IMX477 (RPi HQ) | Installed; waypoint-triggered capture and texturing pipeline input |
| AI accelerator | Hailo-8 HAT+ (26 TOPS) | Inference hardware validated; FlowBridge integration path written |
| Storage | SanDisk Extreme 1 TB SSD (USB 3.0) | Rosbag and mesh output storage |

---

## Software Stack

| Package | Version | Role |
|---|---|---|
| Ubuntu | 24.04 (arm64) | Operating system |
| ROS 2 | Jazzy | Middleware |
| MAVROS | ros-jazzy-mavros | MAVLink bridge |
| Point-LIO | built from source | LiDAR-inertial SLAM |
| foxglove_bridge | 3.2.3 | ROS 2 WebSocket for Foxglove Studio |
| Foxglove Agent | 1.4.7 | Optional post-flight MCAP upload to Foxglove cloud (verification pending) |
| Open3D | 0.19.0 (conda-forge) | MLS smoothing, BPA mesh |
| PDAL | 3.5.3 (conda-forge) | SMRF ground classification |
| pymeshlab | ARM64 wheel | Mesh processing fallback |
| trimesh | ARM64 wheel | Mesh I/O and merge |
| rosbags | ARM64 wheel | MCAP bag reading without full ROS install |

Python environment is managed via a `dronepi` Conda environment (Miniforge) on the Pi.

---

## Repository Structure

```
unitree_lidar_project/
├── main.py                            Top-level flight mode orchestrator
├── dronepi-main.service               systemd unit for main.py
├── preflight_check.sh                 17-section hardware + software gate
├── flight/
│   ├── drone_watchdog.py              Flight stack supervisor (state machine)
│   ├── drone-watchdog.service
│   ├── _slam_bridge.py                Point-LIO -> MAVROS EKF2 vision fusion
│   ├── collision_monitor.py           3-zone LaserScan + AGL estimator
│   ├── gap_detector.py                LiDAR coverage gap detector
│   └── watchdog_core/                 Modular watchdog subpackage
│       ├── buzzer.py                  Pixhawk audio feedback tunes
│       ├── mavros_reader.py           ROS 2 MAVROS interface
│       ├── flight_stack.py            Subprocess manager (Point-LIO, bridge, bag)
│       ├── postflight.py              Monitored post-flight trigger + log pipe
│       └── led_controller.py          Canonical LED state definitions
├── tests/
│   ├── test_offboard_flight.py        OFFBOARD hover test
│   ├── test_offboard_gps.py           GPS waypoint mission test
│   ├── test_manual_scan.py            Manual RC flight + LiDAR scan
│   ├── test_slam_bridge_flight.py     SLAM bridge validation flight
│   ├── collision_zone_test.py         Zone characterization + PLY export
│   ├── test_mesh_algorithms.py        Poisson vs BPA vs Delaunay comparison
│   ├── test_texture_projection.py     Texture replay / projection validation
│   └── test_hover_camera.py           Hover-square camera + LiDAR sync integration test
├── utils/
│   ├── postprocess_mesh.py            7-stage mesh pipeline orchestrator
│   ├── run_postflight.py              Post-flight trigger script
│   ├── flight_logger.py               Persistent flight history log
│   ├── bench_test.py                  MAVROS pipeline bench test (8 checks)
│   ├── foxglove_events.py             Foxglove post-flight Events client
│   ├── foxglove_cloud_rotation.py     Cloud recording retention policy
│   ├── postflight_ipc.py              Pipeline stage IPC for LED/buzzer feedback
│   ├── mesh_tools/
│       ├── bag_reader.py
│       ├── mls_smoother.py
│       ├── ground_classifier.py
│       ├── dtm_builder.py
│       ├── dsm_builder.py
│       ├── mesh_merger.py
│       └── publisher.py
│   └── texture_tools/
│       ├── camera_model.py
│       ├── pose_interpolator.py
│       ├── texture_projector.py
│       └── texture_stage.py
├── config/
│   ├── px4_config.yaml
│   ├── px4_pluginlists.yaml
│   └── camera_calibration.yaml
├── rpi_server/
│   ├── serve.py                       HTTP server, /api/flights endpoint
│   ├── meshview.html                  Pi browser 3D viewer
│   └── local_test.html                Offline laptop viewer
├── RPI5/
│   └── ros2_ws/                       Native ROS 2 workspace
│       └── src/
│           ├── point_lio_ros2/
│           ├── unitree_lidar_ros2/    (symlinked at correct colcon depth)
│           └── unilidar_sdk/
├── docker/
│   ├── lidar/Dockerfile
│   ├── mavros/Dockerfile
│   └── docker-compose.yml
└── logs/
    └── flight_history.log
```

---

## Flight Stack Architecture

### Orchestration and Lock Protocol

Three services cooperate via a lock file at `/tmp/dronepi_mission.lock`:

```
network.target
  |-- mavros.service
  |-- drone-watchdog.service
        |-- dronepi-main.service    (inactive until armed)
```

`main.py` implements a state machine: IDLE -> DEBOUNCE (10s) -> MODE 1 / MODE 2 / MODE 3.

| Lock mode | Written by | Stack ownership |
|---|---|---|
| absent | — | RC toggle operation (CH6) |
| bench_scan | Test scripts | Watchdog yields — test script owns stack |
| manual_scan | main.py MODE 2 | Watchdog owns stack, monitors disarm |
| autonomous | main.py MODE 3 | main.py owns stack, watchdog yields |

### Watchdog Modular Architecture

The watchdog was refactored from a 516-line monolithic script into a `watchdog_core/` subpackage. Each module has a single responsibility and can be tested in isolation. Post-flight output is now fully piped into `journalctl` via `PostflightMonitor`, replacing the original fire-and-forget subprocess.

### Collision Avoidance

`collision_monitor.py` publishes a three-zone cylindrical LaserScan to `/mavros/obstacle/send` and an AGL height estimate to the downward rangefinder topic. Zone radii are derived from physical drone measurements:

| Zone | Radius | Purpose |
|---|---|---|
| Ignore | 0.70 m | Drone legs + drift margin |
| Obstacle (hard stop) | 2.00 m | Matches CP_DIST in PX4 |
| Caution | 3.50 m | Early warning ring |

### EKF2 Sensor Fusion

Point-LIO SLAM is fused into PX4 EKF2 via `_slam_bridge.py`, which converts `/aft_mapped_to_init` from ENU to the format MAVROS expects and publishes to `/mavros/vision_pose/pose` and `/mavros/odometry/out`.

Key QGC parameters: `EKF2_EV_CTRL=15` (all vision bits), `EKF2_HGT_REF=3` (SLAM as primary height), `COM_ARM_WO_GPS=1` (arm without GPS lock).

---

## Post-Processing Pipeline

After every disarm, `run_postflight.py` automatically finds the latest rosbag and runs the current seven-stage mesh pipeline:

```
BagReader -> MLSSmoother -> GroundClassifier -> DTMBuilder -> DSMBuilder -> MeshMerger -> TextureProjection -> Publisher
.mcap->pts    denoise         terrain split      ground mesh    surface mesh   merge           per-vertex RGB      PLY+JSON
```

All stages are independent classes following the project's modular architecture principle. The orchestrator (`postprocess_mesh.py`) supports CLI flags for tuning resolution, skipping stages, and selecting algorithms.

Results are served immediately at `http://10.42.0.1:8080/meshview.html`. When Stage 6 succeeds, `textured_mesh.ply` is published alongside `mesh_final.ply`; if texture projection is skipped or fails, the grey mesh path remains valid and the pipeline still completes.

---

## Deployment

### Native (Production — Raspberry Pi)

```bash
# Source ROS environment (required in every terminal)
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash

# Activate Python environment
conda activate dronepi

# Check all services are running
systemctl status mavros drone-watchdog drone-mesh-server foxglove-bridge

# Run preflight check before any flight
bash ~/unitree_lidar_project/preflight_check.sh

# Check full pipeline topics
ros2 topic list | grep -E "unilidar|cloud_registered|mavros"
ros2 topic hz /cloud_registered

# View results
# Browser: http://10.42.0.1:8080/meshview.html
```

### Docker (Development)

Docker containers are provided for local development on x86 machines. They use `network_mode: host` so ROS 2 DDS discovery works across both containers.

```bash
# Build containers
./docker-run.sh all build

# Start both containers
./docker-run.sh all start

# Access a container shell
./docker-run.sh lidar shell
./docker-run.sh mavros shell
```

The `lidar` container packages the Unitree L1 driver and Point-LIO. The `mavros` container packages MAVROS and PX4 communication. On the Pi, both run natively in the same ROS 2 workspace.

---

## Third-Party Libraries and References

| Library | Repository | Role |
|---|---|---|
| Point-LIO | [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) | LiDAR-inertial odometry SLAM |
| point_lio_ros2 | [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) | ROS 2 wrapper for Point-LIO |
| Unilidar SDK | [unitreerobotics/unilidar_sdk](https://github.com/unitreerobotics/unilidar_sdk) | Unitree L1 hardware driver |
| MAVROS | [mavlink/mavros](https://github.com/mavlink/mavros) | MAVLink to ROS 2 bridge |
| PX4 Autopilot | [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | Flight controller firmware |
| foxglove_bridge | [foxglove/ros-foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge) | ROS 2 WebSocket for Foxglove Studio |
| Open3D | [isl-org/Open3D](https://github.com/isl-org/Open3D) | MLS smoothing, Ball Pivoting mesh |
| PDAL | [PDAL/PDAL](https://github.com/PDAL/PDAL) | SMRF ground classification |
| trimesh | [mikedh/trimesh](https://github.com/mikedh/trimesh) | Mesh I/O and boolean merge |
| rosbags | [rpng/rosbags](https://gitlab.com/robustrobotics/rosbags) | MCAP bag reading without full ROS install |
| Three.js | [mrdoob/three.js](https://github.com/mrdoob/three.js) | WebGL 3D viewer in browser |

---

## Architecture Principles

All pipeline stages in this project follow a consistent modular design:

- Every processing stage is an independent class with a single responsibility.
- A dedicated orchestrator script (such as `postprocess_mesh.py` or `drone_watchdog.py`) calls each class in sequence via a `main` entry point.
- Hardware-optional modules (Hailo-8 inference, LED controller) implement an `.is_available()` check so the pipeline degrades gracefully when hardware is absent.
- `preflight_check.sh` is the single arming gate — all new health checks are added as new sections to this script, not scattered across startup code.
- Systemd services declare explicit `Requires` and `After` dependencies so no service starts against an absent resource.

---

## Known Limitations

- OFFBOARD flight has been demonstrated once but is not yet consistent due to GPS satellite geometry constraints at the UPRM campus. An open field with at least 10 satellites and HDOP below 1.2 is required.
- The texture mapping pipeline is written but untested against real imagery.
- The Hailo-8 AI HAT is present and powered but has no pipeline integration in the current release. I
- The external RGB LED controller (`led_controller.py`) is fully designed and documented 
---

## License

- Point-LIO: GPL-2.0
- Unitree SDK: see upstream repository
- Custom code (flight stack, mesh pipeline, viewers): MIT
