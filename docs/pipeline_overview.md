# Pipeline Overview and Data Flow

**Document:** 2 of 6
**Repo path:** `docs/pipeline_overview.md`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [System Layers](#1-system-layers)
2. [Data Types Between Layers](#2-data-types-between-layers)
3. [Layer 0 — Hardware](#3-layer-0--hardware)
4. [Layer 1 — ROS 2 Bridge](#4-layer-1--ros-2-bridge)
5. [Layer 2 — Flight Stack](#5-layer-2--flight-stack)
6. [Layer 3 — Post-Processing Trigger](#6-layer-3--post-processing-trigger)
7. [Layer 4 — mesh_tools Pipeline](#7-layer-4--mesh_tools-pipeline)
8. [Layer 5 — Outputs and Serving](#8-layer-5--outputs-and-serving)
9. [Layer 6 — Viewers](#9-layer-6--viewers)
10. [Systemd Boot Sequence](#10-systemd-boot-sequence)
11. [Lock File State Machine](#11-lock-file-state-machine)

---

## 1. System Layers

The DronePi pipeline is organized into seven layers. Each layer has a single
responsibility and communicates with adjacent layers through well-defined
interfaces — ROS 2 topics, files on disk, or HTTP endpoints.

```
Layer 0   Hardware
Layer 1   ROS 2 bridge (MAVROS, Point-LIO, SLAM bridge, collision monitor)
Layer 2   Flight stack (main.py, drone_watchdog.py, bag record, preflight)
Layer 3   Post-processing trigger (run_postflight.py, postprocess_mesh.py)
Layer 4   mesh_tools modules (BagReader → MLSSmoother → ... → Publisher)
Layer 5   Outputs and serving (PLY files, JSON manifests, serve.py, Foxglove)
Layer 6   Viewers (meshview.html, local_test.html, Foxglove Studio)
```

---

## 2. Data Types Between Layers

This table shows what data crosses each layer boundary and in what format.

| From | To | Data | Format |
|---|---|---|---|
| Pixhawk 6X | MAVROS | MAVLink telemetry, arming state, mode | Serial USB, 57600 baud |
| Unitree L1 | unitree_lidar_ros2 | Raw 0xFDD1 packet stream | Serial USB, 2,000,000 baud |
| unitree_lidar_ros2 | Point-LIO | Unprocessed point cloud | ROS 2 topic `/unilidar/cloud` @ 9.8 Hz |
| Point-LIO | SLAM bridge | Pose estimate (ENU frame) | ROS 2 topic `/aft_mapped_to_init` |
| Point-LIO | bag recorder | Registered point cloud | ROS 2 topic `/cloud_registered` @ 9.7 Hz |
| SLAM bridge | MAVROS / EKF2 | Vision pose (NED frame) | ROS 2 topic `/mavros/vision_pose/pose` @ ~90 Hz |
| SLAM bridge | MAVROS / EKF2 | 3D velocity | ROS 2 topic `/mavros/odometry/out` |
| Collision monitor | PX4 | Obstacle distances (3 zones) | ROS 2 topic `/mavros/obstacle/send` (LaserScan) |
| Collision monitor | PX4 | AGL height estimate | ROS 2 topic `/mavros/distance_sensor/lidar_down_sub` |
| drone_watchdog | bag recorder | Start/stop trigger | Lock file `/tmp/dronepi_mission.lock` |
| bag recorder | SSD | Raw scan session | MCAP file at `/mnt/ssd/rosbags/scan_YYYYMMDD_HHMMSS/` |
| run_postflight | postprocess_mesh | Bag directory path | CLI argument |
| BagReader | MLSSmoother | Raw point array | NumPy Nx3 float32 in memory |
| MLSSmoother | GroundClassifier | Denoised point array | NumPy Nx3 float32 in memory |
| GroundClassifier | DTMBuilder | Ground point array | NumPy Nx3 float32 in memory |
| GroundClassifier | DSMBuilder | Non-ground point array | NumPy Nx3 float32 in memory |
| DTMBuilder | Publisher | Terrain mesh | trimesh object in memory |
| DSMBuilder | Publisher | Surface mesh | Open3D TriangleMesh in memory |
| Publisher | SSD | Final mesh outputs | PLY binary files at `/mnt/ssd/maps/` |
| Publisher | SSD | Session metadata | `metadata.json` and `latest.json` |
| serve.py | browser | PLY files + metadata | HTTP on port 8080 |
| serve.py | browser | Flight index | JSON via `/api/flights` endpoint |
| Foxglove bridge | Foxglove Studio | All ROS 2 topics | WebSocket on port 8765 |

---

## 3. Layer 0 — Hardware

| Component | Output | Notes |
|---|---|---|
| Pixhawk 6X | MAVLink stream | Flight controller — stabilization, arming, mode management |
| Unitree 4D L1 | 0xFDD1 serial packets | 360-degree LiDAR-inertial sensor — 9.8 Hz raw cloud |
| M9N GPS | NMEA via FC | Routed through Pixhawk — requires outdoor sky view |
| IMX477 camera | CSI image frames | Pending reinstall — texture pipeline not yet active |

All hardware connects to the Raspberry Pi 5. The Pixhawk communicates over
`/dev/ttyACM0`. The LiDAR communicates over `/dev/ttyUSB0` at 2,000,000 baud,
hardcoded in the Unitree SDK.

---

## 4. Layer 1 — ROS 2 Bridge

Four nodes run in this layer. All are started by systemd services on boot.

### MAVROS

Bridges MAVLink (Pixhawk) to ROS 2. Publishes vehicle state, telemetry, and
EKF2 estimates. Receives arming commands and setpoints from the flight stack.

Key topics consumed by the flight stack:
- `/mavros/state` — connected, armed, flight mode
- `/mavros/local_position/pose` — EKF2 local position estimate
- `/mavros/imu/data` — IMU data from FCU

Key topics published to the FCU:
- `/mavros/setpoint_position/local` — OFFBOARD position setpoints
- `/mavros/cmd/arming` — arm and disarm commands
- `/mavros/set_mode` — flight mode changes

### Point-LIO SLAM

Subscribes to `/unilidar/cloud` and `/unilidar/imu`. Runs LiDAR-inertial
odometry. Publishes:
- `/cloud_registered` — point cloud in the map frame at 9.7 Hz
- `/aft_mapped_to_init` — pose estimate used by the SLAM bridge

Configuration file: `unilidar_l1.yaml`

### SLAM bridge (`_slam_bridge.py`)

Converts Point-LIO pose output from ENU to the format MAVROS requires and
publishes it to the EKF2 vision fusion topic. Also publishes odometry for
EKF2 velocity fusion (bit 2 of `EKF2_EV_CTRL`).

- Input: `/aft_mapped_to_init` (BEST_EFFORT QoS)
- Output: `/mavros/vision_pose/pose` at ~90 Hz
- Output: `/mavros/odometry/out`

### Collision Monitor (`collision_monitor.py`)

Processes `/cloud_registered` and produces two outputs:
- A three-zone cylindrical LaserScan on `/mavros/obstacle/send` — feeds
  PX4 collision prevention (`CP_DIST = 2.0 m`)
- An AGL height estimate on the downward rangefinder topic — feeds
  `EKF2_RNG_CTRL`

Zone radii are defined by physical drone measurements:

| Zone | Radius | Purpose |
|---|---|---|
| Ignore | 0.70 m | Drone legs plus drift margin |
| Obstacle | 2.00 m | Hard stop — matches CP_DIST |
| Caution | 3.50 m | Early warning ring |

---

## 5. Layer 2 — Flight Stack

Three components cooperate through a lock file at `/tmp/dronepi_mission.lock`.

### main.py

Top-level state machine. States: IDLE → DEBOUNCE (10s) → MODE 1 / MODE 2 / MODE 3.
Manages the mission lock. Writes `manual_scan` lock for real flight and
`autonomous` lock for waypoint missions.

### drone_watchdog.py

Supervises the flight stack. On arming with OFFBOARD mode, starts three
subprocesses: Point-LIO, SLAM bridge, and ros2 bag recorder. Monitors all
three for unexpected exits. Stops them cleanly on disarm and triggers
post-flight processing via `run_postflight.py`.

The watchdog yields control when it reads `autonomous` or `bench_scan` from
the lock file. It takes ownership only when the lock reads `manual_scan` or
is absent.

### ros2 bag record

Records six topics to an MCAP file during every flight session:
- `/cloud_registered`
- `/unilidar/cloud`
- `/aft_mapped_to_init`
- `/unilidar/imu`
- `/mavros/state`
- `/mavros/local_position/pose`

Output: `/mnt/ssd/rosbags/scan_YYYYMMDD_HHMMSS/`

### preflight_check.sh

17-section gate that must pass before any flight. Checks services, FCU
connection, LiDAR topics, SSD health, and software environment. All new
health checks are added here as new sections.

---

## 6. Layer 3 — Post-Processing Trigger

### run_postflight.py

Triggered automatically by `drone_watchdog.py` after every disarm. Waits for
the bag recorder to close the MCAP file cleanly, then calls `postprocess_mesh.py`
with the latest bag directory as argument.

All output from the mesh pipeline is piped into the watchdog's `journalctl`
stream via `PostflightMonitor`, prefixed `[POSTFLIGHT]`.

### postprocess_mesh.py

Six-stage orchestrator. Each stage calls its corresponding `mesh_tools/` class.
Supports CLI flags for algorithm selection, resolution tuning, and stage skipping.

| Flag | Effect |
|---|---|
| `--no-mls` | Skip MLS smoothing |
| `--no-mesh` | Skip mesh generation, output cloud only |
| `--use-poisson` | Use Poisson reconstruction instead of BPA |
| `--grid-res N` | Set DTM grid resolution in metres |
| `--bpa-radius N` | Set fixed BPA radius |
| `--auto` | Triggered by watchdog — uses latest bag |

### flight_logger.py

Writes one line to `logs/flight_history.log` per session with timestamp,
point count, duration, and mesh status.

---

## 7. Layer 4 — mesh_tools Pipeline

Each module is an independent class. The orchestrator calls them in sequence.
All intermediate data passes in memory as NumPy arrays or mesh objects.
No module writes to disk — only `Publisher` writes final outputs.

```
BagReader
  Input:  MCAP bag directory path
  Output: NumPy Nx3 float32 point array
  Method: rosbags MCAP parser, no full ROS install required

MLSSmoother
  Input:  Nx3 float32
  Output: Nx3 float32 (denoised)
  Method: Moving Least Squares via Open3D / SOR fallback

GroundClassifier
  Input:  Nx3 float32
  Output: ground Nx3, non-ground Nx3
  Method: SMRF via PDAL / Z-percentile fallback

DTMBuilder
  Input:  ground Nx3
  Output: trimesh terrain mesh
  Method: Delaunay 2.5D triangulation, grid binning, long-edge gap filter

DSMBuilder
  Input:  non-ground Nx3
  Output: Open3D TriangleMesh
  Method: Ball Pivoting Algorithm, auto radius from point density

MeshMerger
  Input:  DTM (trimesh) + DSM (Open3D)
  Output: combined trimesh
  Method: concatenate vertices and faces, handle missing DTM or DSM gracefully

Publisher
  Input:  combined mesh, point cloud, session metadata
  Output: PLY files + JSON manifests written to /mnt/ssd/maps/
```

---

## 8. Layer 5 — Outputs and Serving

### File outputs (per session)

Written to `/mnt/ssd/maps/scan_YYYYMMDD_HHMMSS/`:

| File | Contents |
|---|---|
| `combined_cloud.ply` | Full cleaned point cloud |
| `mesh_final.ply` | Combined DTM + DSM mesh |
| `mesh_dtm.ply` | Terrain mesh only |
| `mesh_dsm.ply` | Surface mesh only |
| `metadata.json` | Processing metadata for this session |

Written to `/mnt/ssd/maps/`:

| File | Contents |
|---|---|
| `latest.json` | Points to the most recent session — polled by browser |

### serve.py

CORS-enabled HTTP server on port 8080. Bound to `0.0.0.0` — accessible on
both the hotspot interface (`10.42.0.1`) and the lab router interface
(`192.168.2.2`).

Endpoints:
- `GET /api/flights` — returns JSON array of all sessions with mesh URLs
- `GET /rosbags/<session>/<file>` — serves PLY files
- `GET /meshview.html` — serves the browser viewer
- `GET /latest.json` — polled by the viewer for auto-load

### Foxglove bridge

WebSocket server on port 8765. Exposes all active ROS 2 topics to Foxglove
Studio on the ground station laptop for live telemetry visualization.

---

## 9. Layer 6 — Viewers

### meshview.html

Pi-hosted browser viewer. Loads at `http://10.42.0.1:8080/meshview.html`.
Polls `latest.json` every 10 seconds and auto-loads any new mesh after landing.
Three.js and PLYLoader are served locally from the SSD — no internet required.

Features: PLY cloud and mesh rendering, height color ramp, orbit/pan/zoom,
3D view cube, measurement tool, screenshot export, flight database sidebar.

### local_test.html

Offline laptop viewer. No server required. Drag a flight folder onto the page
to load all PLY files. Confirmed working in Edge with trackpad support.

### Foxglove Studio

Ground station laptop application. Connects to `ws://10.42.0.1:8765`.
Displays live telemetry, EKF2 innovations, IMU data, and point cloud in 3D.
Used for flight monitoring and post-flight SLAM bridge validation.

---

## 10. Systemd Boot Sequence

All services start automatically on power-on. Approximate timing from boot:

```
~15s   dronepi-hotspot       Wi-Fi AP on wlan0 — SSID dronepi-ap
~16s   drone-mesh-server     HTTP server on port 8080 (waits for /mnt/ssd)
~18s   foxglove-bridge       WebSocket on port 8765
~20s   rpi-health            Publishes /rpi/health every 2s (pending deploy)
~25s   mavros                FCU bridge via /dev/ttyPixhawk
~26s   drone-watchdog        Polls /mavros/state, manages flight stack
```

Dependency chain:

```
network.target
  |-- mavros.service
  |-- drone-watchdog.service
        |-- dronepi-main.service    (inactive until first arm)

mnt-ssd.mount
dronepi-hotspot.service
  |-- drone-mesh-server.service
        RequiresMountsFor=/mnt/ssd
        ExecStartPre=/bin/mountpoint -q /mnt/ssd
```

---

## 11. Lock File State Machine

The lock file at `/tmp/dronepi_mission.lock` coordinates ownership of the
flight stack between `main.py`, `drone_watchdog.py`, and test scripts.

| Lock value | Written by | Watchdog behavior | main.py behavior |
|---|---|---|---|
| absent | — | RC toggle on CH6 starts and stops stack | Monitors, does not intervene |
| `manual_scan` | main.py MODE 2 | Owns the stack, monitors disarm | Delegates to watchdog |
| `autonomous` | main.py MODE 3 | Yields — does not touch stack | Owns the stack |
| `bench_scan` | Test scripts | Yields — test script owns stack | Inactive |

The lock file is cleaned up on service stop via `ExecStopPost` in
`dronepi-main.service`. A stale lock from a crash can be removed manually:

```bash
rm /tmp/dronepi_mission.lock
sudo systemctl restart drone-watchdog
```
