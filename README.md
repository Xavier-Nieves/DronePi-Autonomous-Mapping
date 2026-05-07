# DronePi — Autonomous LiDAR Mapping Hexacopter

**Institution:** University of Puerto Rico, Mayaguez (UPRM)
**Capstone demonstration:** May 2026
**Repository:** [Xavier-Nieves/unitree_lidar_project](https://github.com/Xavier-Nieves/unitree_lidar_project)
**Platform:** Raspberry Pi 5 · Ubuntu 24.04 LTS · ROS 2 Jazzy · PX4

---

## What This Is

DronePi is a fully autonomous hexacopter that surveys a target structure, captures
LiDAR geometry and drone pose simultaneously during flight, and produces both a
textured 3D mesh and a geographically registered orthomosaic map — viewable in a
browser with zero manual steps between landing and display.

The entire pipeline runs on the drone's companion computer. No cloud dependency is
required. The operator connects a laptop to the drone's Wi-Fi hotspot and opens a
browser.

**Demonstration goal:** Power on → arm → fly GPS survey → land → textured mesh
and ortho map auto-load in browser within seconds of disarm.

---

## How It Works — Top-Level Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          DRONEPI SYSTEM OVERVIEW                             │
│                                                                              │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────────┐   │
│  │  Pixhawk 6X  │    │ Unitree L1   │    │ IMX477 Camera  + Hailo-8L   │   │
│  │  Flight Ctrl │    │  LiDAR 360°  │    │ imagery + AI inference       │   │
│  └──────┬───────┘    └──────┬───────┘    └──────────────┬───────────────┘   │
│         │ MAVLink           │ USB Serial                │ CSI / PCIe        │
│         ▼                   ▼                           ▼                   │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                    Raspberry Pi 5 (16 GB, Ubuntu 24.04)              │   │
│  │                                                                      │   │
│  │  MAVROS ──► PX4 EKF2 ◄── Point-LIO SLAM ◄── Unitree L1             │   │
│  │                 ▲               ▲                                   │   │
│  │           SLAM Bridge    Collision Monitor                           │   │
│  │                                                                      │   │
│  │  ┌──────────────────────────────────────────────────────────────┐   │   │
│  │  │         Flight Stack  (main.py  +  drone_watchdog.py)        │   │   │
│  │  │   rosbag ──► Point-LIO ──► SLAM Bridge ──► OFFBOARD mission  │   │   │
│  │  └────────────────────────────────┬─────────────────────────────┘   │   │
│  │                                   │ on disarm (parallel)             │   │
│  │         ┌─────────────────────────┴──────────────────────────┐      │   │
│  │         ▼                                                     ▼      │   │
│  │  ┌─────────────────────────────┐   ┌────────────────────────────┐   │   │
│  │  │   Mesh Pipeline  (7 stages) │   │  Ortho Pipeline  (5 stages)│   │   │
│  │  │  .mcap → cloud → smooth →   │   │  frames → filter → stitch  │   │   │
│  │  │  classify → DTM → DSM →     │   │  → tile pyramid → publish  │   │   │
│  │  │  texture → mesh_final.ply   │   │  → ortho_metadata.json     │   │   │
│  │  └──────────────┬──────────────┘   └───────────┬────────────────┘   │   │
│  │                 └─────────────┬────────────────┘                    │   │
│  │                               ▼                                     │   │
│  │                    serve.py  (HTTP :8080)                           │   │
│  └───────────────────────────────┬──────────────────────────────────────┘   │
│                                  │ Wi-Fi hotspot (dronepi-ap)               │
│              ┌───────────────────▼───────────────────────┐                  │
│              │           meshview.html                    │                  │
│              │  3D tab  ─  textured_mesh.ply  (Three.js)  │                  │
│              │  Ortho tab ─  tile map  (Leaflet XYZ)       │                  │
│              │  Live tab  ─  camera stream + telemetry     │                  │
│              │  http://10.42.0.1:8080/meshview.html        │                  │
│              └───────────────────────────────────────────┘                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware

| Component | Part | Role |
|---|---|---|
| Frame | Tarot 810 hexacopter | Airframe — 6 motors |
| Companion computer | Raspberry Pi 5 (16 GB, 3.0 GHz OC) | Runs entire software stack |
| Flight controller | Pixhawk 6X (PX4 firmware) | Stabilization, OFFBOARD control, arming |
| LiDAR | Unitree L1 (4D, 360°) | 9.8 Hz point cloud via USB serial |
| GPS | M9N (internal to Pixhawk) | Secondary position augmentation |
| Camera | IMX477 ArduCam HQ | Texture imagery and orthomosaic frames |
| AI accelerator | Hailo-8L HAT+ (26 TOPS) | In-flight optical flow + ground classification |
| Storage | SanDisk Extreme 1 TB USB 3.0 SSD | Rosbags, flight frames, mesh + ortho outputs |

---

## Software Stack

| Layer | Technology | Version |
|---|---|---|
| Operating system | Ubuntu (arm64) | 24.04 LTS |
| Robotics middleware | ROS 2 | Jazzy |
| Flight controller bridge | MAVROS | ros-jazzy-mavros |
| SLAM | Point-LIO (point_lio_ros2) | built from source |
| Flight firmware | PX4 Autopilot | v1.14 |
| AI inference runtime | HailoRT | 4.23.0 |
| 3D processing | Open3D | 0.19.0 (conda-forge) |
| Ground classification | PDAL | 3.5.3 (conda-forge) |
| Image stitching | OpenCV | cv2.Stitcher (conda-forge) |
| Tile generation | gdal2tiles / Pillow | system GDAL + fallback |
| Bag reading | rosbags | ARM64 wheel |
| Python environment | Miniforge Conda (`dronepi` env) | — |
| Telemetry viewer | foxglove_bridge | 3.2.3 |
| Cloud recording (optional) | Foxglove Agent | 1.4.7 |

---

## Data Flow — Start to Finish

### Phase 1 — Boot (automatic, ~30 seconds)

```
Power on
  │
  ├─ systemd: dronepi-hotspot    → Wi-Fi AP "dronepi-ap" on 10.42.0.1
  ├─ systemd: foxglove-bridge    → WebSocket :8765 (live ROS 2 telemetry)
  ├─ systemd: drone-mesh-server  → HTTP server :8080
  ├─ systemd: rpi-health         → /rpi/health topic every 2s
  └─ systemd: mavros             → MAVLink bridge to Pixhawk 6X
        │
        └─ systemd: drone-watchdog  → polls /mavros/state,
                                      manages flight stack lifecycle
```

All services start without user interaction. The operator connects a laptop to
`dronepi-ap` and opens QGroundControl and Foxglove Studio.

---

### Phase 2 — Pre-Flight Gate

```
Operator SSH: bash preflight_check.sh
  │
  ├─ Section  1: Power + thermal   (vcgencmd get_throttled → 0x0)
  ├─ Section  2: SSD mount + SMART status
  ├─ Section  3: Pixhawk serial + MAVROS process
  ├─ Section  4: Hailo-8 PCIe + hailo_inference_env
  ├─ Section  5: Unitree LiDAR device node
  ├─ Section  6: ROS 2 workspace build
  ├─ Section  7: MAVROS topics publishing
  ├─ Section  8: GPS fix quality   (HDOP, satellites, fix type)
  ├─ Section  9–17: services, envs, output dirs, calibration files
  │
  └─ RESULT: N PASS  0 FAIL  →  "ALL SYSTEMS GO"
             Any FAIL blocks arming.
```

The preflight script is the single arming gate. No health checks exist in any
other file.

---

### Phase 3 — Flight and Data Capture

```
Arm (RC transmitter, left stick down-right)
  │
  ├─ main.py detects arm via /mavros/state
  │
  ├─ SafeFlightMixin.start_bag_recorder()       ← FIRST subprocess always
  │     ros2 bag record → /mnt/ssd/rosbags/scan_YYYYMMDD_HHMMSS/
  │     Topics: /cloud_registered, /aft_mapped_to_init,
  │             /mavros/state, /mavros/local_position/pose,
  │             /mavros/imu/data, /mavros/setpoint_position/local,
  │             /arducam/image_raw, /hailo/optical_flow,
  │             /hailo/ground_class, /rpi/health
  │
  ├─ camera_capture.py (sole Picamera2 owner)
  │     → /arducam/image_raw  (15 Hz, rgb8)
  │     → /mnt/ssd/rosbags/<session>/flight_frames/  (JPEG + JSON sidecar)
  │           sidecar fields: ros_timestamp, gps (lat/lon/alt), enu (x/y/z)
  │
  ├─ drone_watchdog starts SLAM subprocesses:
  │     Point-LIO       /unilidar/cloud → /cloud_registered + /aft_mapped_to_init
  │     SLAM bridge      /aft_mapped_to_init → /mavros/odometry/out → EKF2
  │     Hailo node       hailo_inference_env: SCDepthV3 optical flow
  │                        + YOLOv8m ground class
  │                        → /hailo/optical_flow + /hailo/ground_class
  │
  ├─ main.py: OFFBOARD mode, 20 Hz setpoint loop
  │     Heartbeat watchdog: > 2s stale → AUTO.LAND   ← non-negotiable
  │     RC CH7 kill switch: PWM > 1700 → immediate teardown
  │
  └─ GPS survey waypoints:
        climb → waypoint 1 → waypoint 2 → ... → AUTO.RTL → land → disarm
```

Point-LIO fuses LiDAR and IMU to produce a pose-registered map in real time.
The SLAM bridge feeds this pose into PX4 EKF2 as the primary position source
(`EKF2_GPS_CTRL = 0` — GPS is secondary). `camera_capture.py` holds the only
Picamera2 instance; `hailo_flight_node.py` subscribes to its topic rather than
opening the camera directly, eliminating hardware conflicts.

---

### Phase 4 — Post-Processing (automatic on disarm, two pipelines in parallel)

```
Disarm detected
  │
  ├─ drone_watchdog stops subprocesses in order:
  │     bag recorder → Hailo node → SLAM bridge → Point-LIO
  │
  └─ run_postflight.py
        │
        ├──────────────────────────────────────────────────────────────────┐
        │  MESH PIPELINE  (postprocess_mesh.py)                            │
        │                                                                  │
        │  Stage 1: BagReader                                              │
        │    .mcap → numpy point array  (source: /cloud_registered)        │
        │                                                                  │
        │  Stage 2: MLSSmoother                                            │
        │    Moving Least Squares noise reduction (Open3D)                 │
        │    Removes LiDAR noise while preserving structural edges         │
        │                                                                  │
        │  Stage 3: GroundClassifier                                       │
        │    PDAL SMRF — separates ground from structure                   │
        │    Output: ground_cloud  +  nonground_cloud                      │
        │                                                                  │
        │  Stage 4: DTMBuilder                                             │
        │    Delaunay 2.5D triangulation of ground points                  │
        │    Output: terrain mesh (digital terrain model)                  │
        │                                                                  │
        │  Stage 5: DSMBuilder                                             │
        │    Ball Pivoting Algorithm on nonground points                   │
        │    Output: surface mesh (structures, vegetation)                 │
        │                                                                  │
        │  Stage 6: TextureProjectionStage  [skipped if camera absent]     │
        │    Loads intrinsics + extrinsics from camera_calibration.yaml    │
        │    Pass 1: PoseInterpolator loads all SLAM poses from bag        │
        │    Pass 2: for each /arducam/image_raw frame in bag:             │
        │      - SLERP-interpolate SLAM pose at frame timestamp            │
        │      - Compute camera world pose: T_cam = T_drone × T_cam_body   │
        │      - Project each mesh vertex to pixel:                        │
        │          p = K · [R|t] · P_world                                 │
        │      - Z-buffer occlusion culling via Open3D raycasting          │
        │      - Accumulate colour into per-vertex weighted buffer         │
        │    Output: per-vertex RGB → textured_mesh.ply                    │
        │                                                                  │
        │  Stage 7: Publisher                                              │
        │    Writes to /mnt/ssd/maps/<session>/                            │
        │      combined_cloud.ply                                          │
        │      mesh_final.ply                                              │
        │      textured_mesh.ply   (when Stage 6 succeeds)                 │
        │      metadata.json                                               │
        │    Writes latest.json → browser auto-loads mesh tab              │
        └──────────────────────────────────────────────────────────────────┘
        │
        └─────────────────────────────────────────────────────────────────┐
           ORTHO PIPELINE  (postprocess_ortho.py)  [non-fatal]            │
           Runs after mesh pipeline. Failure never affects mesh exit code.│
                                                                          │
           Stage 1: FrameIngestor                                         │
             Reads flight_frames/<session>/*.jpg + sidecar JSON           │
             Parses ros_timestamp and GPS from each sidecar               │
             Calls PoseInterpolator (shared with texture pipeline)        │
             to match SLAM pose to each frame                             │
             Output: list[FrameRecord]                                    │
                                                                          │
           Stage 2: QualityFilter                                         │
             Rejects blurry frames via Laplacian variance threshold       │
             Default threshold: 50.0 (configurable via --blur-threshold)  │
             Output: filtered list[FrameRecord]                           │
                                                                          │
           Stage 3: MosaicBuilder  (fast path, default)                   │
             cv2.Stitcher_create() — feature-based panoramic stitching    │
             GPS affine applied post-stitch via world file                │
             Falls back to grid collage if Stitcher fails                 │
             Output: stitched numpy canvas + geo bounds dict              │
             -- OR --                                                     │
           Stage 3: Orthorectifier  (--full flag, planned)                │
             Ray-casts each frame pixel through LiDAR DTM                 │
             Co-registration with mesh_final.ply is native                │
             Requires mesh pipeline to have completed first               │
                                                                          │
           Stage 4: TileCutter                                            │
             gdal2tiles → XYZ PNG tile pyramid (zoom 14–19)               │
             Pillow fallback when gdal2tiles unavailable                  │
             Output: ortho/tiles/{z}/{x}/{y}.png                          │
                                                                          │
           Stage 5: OrthoPublisher                                        │
             Writes ortho_metadata.json consumed by meshview.html:        │
               session_id, mode, tile_url, bounds (WGS-84),               │
               zoom_min, zoom_max, frame_count, gps_count,                │
               collage_fallback, processing_s                             │
             Writes mosaic.jpg for file-selector preview                  │
             Browser Ortho tab auto-loads on next poll                    │
           └──────────────────────────────────────────────────────────────┘

        └─ flight_logger.py → logs/flight_history.log (one line per session)
```

Each stage in both pipelines is an independent importable class. Any stage can
fail or be skipped without stopping the pipeline — the last successful stage's
output is always published.

---

### Phase 5 — Browser Visualization (automatic)

```
latest.json + ortho_metadata.json written
  │
  └─ meshview.html polls both files every 10 seconds
        │
        ├─ 3D Tab
        │    Loads textured_mesh.ply (preferred) or mesh_final.ply
        │    Three.js WebGL renderer
        │    Features: orbit/pan/zoom, height color ramp,
        │              3D measurement tool, flight database sidebar,
        │              debug file toggle (intermediate PLY files)
        │
        ├─ Ortho Tab
        │    Leaflet slippy-map with XYZ tile layer
        │    Auto-centers on GPS bounds from ortho_metadata.json
        │    Zoom 14–19 (≈0.3 m/px at zoom 19)
        │    Mosaic JPEG also available in file selector for download
        │
        └─ Live Tab
             Camera MJPEG stream via /stream (proxied through serve.py)
             Foxglove Studio: ws://10.42.0.1:8765 (live ROS 2 topics)
```

The viewer requires no internet connection. Three.js, PLYLoader, and Leaflet
are served from the SSD alongside the mesh and tile data.

---

## Architecture — Key Design Decisions

### Modular pipeline with a single orchestrator

Every processing stage in every pipeline is an independent importable class
with typed inputs and outputs. A single orchestrator script calls each class in
sequence. No production module has a `main()`. Test scripts are standalone with
their own `main()` and are never imported by production code. Any stage can be
tested in isolation, replaced, or skipped without touching anything else.

### Two isolated Python environments

```
dronepi (Conda, Miniforge)
  └─ All flight stack, SLAM, mesh pipeline, ortho pipeline, ROS 2 nodes
  └─ Activated by default in .bashrc
  └─ Entry point: python3 main.py

hailo_inference_env (venv)
  └─ HailoRT 4.23.0 + hailo_platform only
  └─ Never activated manually during flight
  └─ Launched via: ~/hailo_inference_env/bin/python3 hailo/hailo_flight_node.py
  └─ Communicates with flight stack via ROS 2 topics only
```

### SLAM is the sole EKF2 position source

`EKF2_GPS_CTRL = 0`. The SLAM bridge is the only position source fed to the
flight controller. GPS failure is always non-fatal. GPS coordinates from the
M9N are used only for orthomosaic geographic registration.

### Hailo augmentation is additive

Hailo optical flow (SCDepthV3) and ground classification (YOLOv8m) augment —
but do not replace — Point-LIO SLAM. Hailo failures degrade gracefully and
never block flight or SLAM operations.

### Texture and ortho pipelines share pose infrastructure

`PoseInterpolator` (in `texture_tools/`) is shared by both the texture
projection stage (Stage 6 of the mesh pipeline) and the ortho pipeline's
`FrameIngestor`. Both use SLERP interpolation of the SLAM trajectory from the
bag to match sensor data to exact drone poses.

### Camera ownership is centralised

`camera_capture.py` holds the single persistent `Picamera2` instance for the
entire flight. `hailo_flight_node.py` subscribes to `/arducam/image_raw` rather
than opening the camera directly. This eliminates the hardware conflict that
occurred when two processes competed for `/dev/media0`.

### External services never gate local outputs

Foxglove cloud sync and the Events API are wrapped in try/except. The ortho
pipeline failure never changes the mesh pipeline exit code. Every output
produced locally is always published regardless of network state or secondary
pipeline status.

### Flight safety invariants (non-negotiable)

Every flight script enforces all four of the following. Any script violating
these must not be merged:

1. Must inherit `SafeFlightMixin` before any arming call.
2. Must launch the rosbag recorder as the first subprocess before any other init.
3. Every setpoint publisher must have a heartbeat watchdog with a maximum
   2-second timeout; stale heartbeat triggers immediate AUTO.LAND.
4. RC Channel 7 monitoring must be present and capable of interrupting
   autonomous operation at any point.

---

## Repository Structure

```
unitree_lidar_project/
│
├── main.py                              Top-level orchestrator (sole entry point)
├── preflight_check.sh                   Arming gate — 17 sections, 63 checks
├── logs/
│   └── flight_history.log               Persistent log (one line per session)
│
├── flight/
│   ├── drone_watchdog.py                Flight stack supervisor
│   ├── safe_flight_mixin.py             Safety base class — all scripts inherit this
│   ├── _slam_bridge.py                  Point-LIO → EKF2 pose bridge
│   ├── _flow_bridge.py                  Hailo optical flow → MAVROS velocity bridge
│   ├── collision_monitor.py             3-zone obstacle LaserScan + AGL estimator
│   ├── gap_detector.py                  LiDAR coverage gap detector
│   ├── gps_reader.py                    GPS secondary layer (MAVROS-backed)
│   ├── camera_capture.py                IMX477 capture node (sole Picamera2 owner)
│   ├── flight_controller.py             Thread-safe MAVROS primitives (test use only)
│   ├── led_service.py                   LED state derivation from system snapshot
│   └── watchdog_core/
│       ├── buzzer.py                    Audio feedback (Pixhawk buzzer via MAVROS)
│       ├── mavros_reader.py             ROS 2 MAVROS subscriber interface
│       ├── flight_stack.py              Subprocess lifecycle manager
│       └── postflight.py               Monitored post-flight trigger
│
├── hailo/
│   ├── hailo_device.py                  VDevice lifecycle wrapper
│   ├── hailo_optical_flow.py            SCDepthV3 depth-difference velocity estimator
│   ├── hailo_ground_class.py            YOLOv8m nadir surface classifier
│   └── hailo_flight_node.py             ROS 2 node (runs in hailo_inference_env)
│
├── core/
│   ├── camera_model.py                  Intrinsics + extrinsics, pinhole projection
│   └── texture_projector.py             Per-vertex colour accumulator with Z-buffer
│
├── tests/                               Standalone — all have main(), never imported
│   ├── test_offboard_flight.py
│   ├── test_offboard_gps.py
│   ├── test_manual_scan.py
│   ├── test_slam_bridge_flight.py
│   ├── collision_zone_test.py
│   └── test_mesh_algorithms.py
│
├── utils/
│   ├── postprocess_mesh.py              7-stage mesh pipeline orchestrator
│   ├── postprocess_ortho.py             5-stage ortho pipeline orchestrator
│   ├── run_postflight.py                Post-flight trigger (mesh + ortho)
│   ├── flight_logger.py                 Flight history log writer
│   ├── bench_test.py                    MAVROS 8-point bench test
│   ├── foxglove_events.py               Foxglove Events API client (optional)
│   ├── foxglove_cloud_rotation.py       Cloud recording retention policy
│   │
│   ├── mesh_tools/                      Mesh pipeline stages
│   │   ├── bag_reader.py                MCAP → NumPy point array
│   │   ├── mls_smoother.py              Open3D MLS noise reduction
│   │   ├── ground_classifier.py         PDAL SMRF terrain split
│   │   ├── dtm_builder.py               Delaunay 2.5D terrain mesh
│   │   ├── dsm_builder.py               Ball Pivoting surface mesh
│   │   ├── mesh_merger.py               DTM + DSM merge
│   │   └── publisher.py                 PLY + JSON output writer
│   │
│   ├── texture_tools/                   Texture pipeline — shared by mesh + ortho
│   │   ├── camera_model.py              Intrinsics, extrinsics, pinhole projection
│   │   ├── pose_interpolator.py         SLERP SLAM pose interpolation from bag
│   │   ├── texture_projector.py         Per-vertex colour accumulator, Z-buffer culling
│   │   └── texture_stage.py             Stage 6 interface for postprocess_mesh.py
│   │
│   └── ortho_tools/                     Ortho pipeline stages
│       ├── frame_ingestor.py            flight_frames/ → list[FrameRecord] + SLAM poses
│       ├── quality_filter.py            Laplacian blur rejection
│       ├── mosaic_builder.py            cv2.Stitcher + GPS affine (fast path)
│       ├── tile_cutter.py               gdal2tiles XYZ pyramid (Pillow fallback)
│       └── publisher.py                 ortho_metadata.json writer
│
├── config/
│   ├── px4_config.yaml
│   ├── px4_pluginlists.yaml
│   ├── camera_calibration.yaml          IMX477 intrinsics (RMS 1.99) + extrinsics
│   └── GPS+SLAM_HEXCOPTER.params        Full PX4 parameter export for QGroundControl
│
├── rpi_server/
│   ├── serve.py                         HTTP server, /api/flights, /stream, /rosbags/
│   ├── meshview.html                    Browser viewer — 3D / Ortho / Live tabs
│   └── local_test.html                  Offline laptop viewer (drag-and-drop)
│
├── RPI5/ros2_ws/src/
│   ├── point_lio_ros2/
│   ├── unitree_lidar_ros2/              Symlink at correct colcon discovery depth
│   └── unilidar_sdk/
│
├── setup_scripts/
│   ├── setup.sh                         Master setup — run once as root
│   └── setup_*.sh                       Per-service systemd installers
│
└── docs/
    ├── architecture.md
    ├── hardware_setup.md
    ├── pipeline_overview.md
    ├── ekf_slam_fusion.md
    ├── post_processing.md
    ├── flight_procedure.md
    ├── calibration.md
    ├── test_procedure.md
    └── texture_pipeline.md
```

---

## Deployment

### Fresh Pi Setup (run once as root)

```bash
git clone https://github.com/Xavier-Nieves/unitree_lidar_project.git \
  ~/unitree_lidar_project
cd ~/unitree_lidar_project/setup_scripts
sudo bash setup.sh
```

The master script installs all system packages, clones the ROS 2 workspace,
creates the `dronepi` Conda environment, installs all Python dependencies
(including opencv-python, piexif, gdal-bin for the ortho pipeline), writes udev
rules, builds the colcon workspace, and installs all six systemd services.

After the script completes:

```bash
# 1. Log out and back in (dialout + hailo group membership)
# 2. Verify SSD
lsblk && df -h /mnt/ssd
# 3. Verify udev symlink
ls -l /dev/ttyPixhawk
# 4. Reboot
sudo reboot
```

After reboot:

```bash
# Check all services
systemctl status mavros drone-watchdog drone-mesh-server foxglove-bridge

# Run bench test (no flight required)
conda activate dronepi
python3 utils/bench_test.py

# Connect laptop to dronepi-ap, then open:
# http://10.42.0.1:8080/meshview.html
```

---

### Running a Mission

```bash
conda activate dronepi
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
sudo bash setup_scripts/preflight_check.sh   # 0 FAIL required
python3 main.py
```

---

### Reprocessing Pipelines (no flight required)

```bash
conda activate dronepi

# Reprocess mesh from any existing bag
python3 utils/postprocess_mesh.py --bag /mnt/ssd/rosbags/<session>/

# Reprocess ortho from any existing session
python3 utils/postprocess_ortho.py --session /mnt/ssd/rosbags/<session>/

# Run both automatically (same as post-flight trigger)
python3 utils/run_postflight.py --bag /mnt/ssd/rosbags/<session>/
```

**Mesh pipeline flags:** `--no-texture`, `--no-mls`, `--no-mesh`,
`--use-poisson`, `--grid-res N`, `--bpa-radius N`, `--fast`, `--debug`, `--auto`

**Ortho pipeline flags:** `--blur-threshold N`, `--fast`, `--full` (planned),
`--frames-dir PATH`, `--no-ortho`, `--fast-ortho`, `--zoom-min N`, `--zoom-max N`

---

## Network Reference

| Interface | Address | Purpose |
|---|---|---|
| Wi-Fi hotspot (field) | `10.42.0.1` | Primary field access — no router required |
| Lab router | `192.168.2.2` | Lab development |
| Mesh viewer | `http://10.42.0.1:8080/meshview.html` | Browser 3D + Ortho + Live viewer |
| Flight API | `http://10.42.0.1:8080/api/flights` | Flight index JSON |
| Camera stream | `http://10.42.0.1:8080/stream` | Live MJPEG proxy |
| Foxglove WebSocket | `ws://10.42.0.1:8765` | Live ROS 2 telemetry |
| SSH | `ssh dronepi@10.42.0.1` | Remote terminal |

---

## Key PX4 Parameters

Set in QGroundControl before first flight. Full parameter file at
`config/GPS+SLAM_HEXCOPTER.params`.

| Parameter | Value | Purpose |
|---|---|---|
| `EKF2_EV_CTRL` | `15` | Enable all external vision inputs |
| `EKF2_GPS_CTRL` | `0` | Disable GPS as EKF2 position source (SLAM is sole source) |
| `EKF2_HGT_REF` | `3` | SLAM as primary height reference |
| `EKF2_EV_DELAY` | `25` | ms — SLAM bridge + Hailo inference latency |
| `COM_ARM_WO_GPS` | `1` | Allow arming without GPS lock |
| `CP_DIST` | `2.0` | Collision prevention hard stop (metres) |
| `TRIG_MODE` | `4` | PWM camera trigger for IMX477 shutter sync |

---

## Systemd Services

| Service | Purpose | Port |
|---|---|---|
| `mavros` | FCU bridge (MAVLink ↔ ROS 2) | — |
| `drone-watchdog` | Flight stack supervisor | — |
| `dronepi-hotspot` | Wi-Fi AP (SSID: dronepi-ap, pw: set at setup time) | — |
| `drone-mesh-server` | HTTP server + camera stream proxy | 8080 |
| `foxglove-bridge` | ROS 2 WebSocket bridge | 8765 |
| `rpi-health` | System health topic publisher | — |

```bash
journalctl -u drone-watchdog -f
journalctl -u drone-mesh-server -f
```

---

## Known Open Items

| Item | Status |
|---|---|
| OFFBOARD flight consistency | Inconsistent; under investigation — GPS geometry at UPRM campus |
| Texture pipeline real-bag validation | Architecture complete; awaiting IMX477 remount |
| Ortho full path (Orthorectifier) | Planned module — fast path (cv2.Stitcher) is operational |
| Mid-flight live view dropout | Options scoped (directional antenna, LTE, SiK radio) — not implemented |
| `EKF2_RNG_POS_X/Y/Z` in QGroundControl | Measured, not yet entered |
| 9-phase test gate completion | Phases 1–7 verified; phases 8–9 (full mission) pending |

---

## Third-Party Libraries and References

| Library | Repository | Role |
|---|---|---|
| Point-LIO | [hku-mars/Point-LIO](https://github.com/hku-mars/Point-LIO) | LiDAR-inertial odometry SLAM |
| point_lio_ros2 | [dfloreaa/point_lio_ros2](https://github.com/dfloreaa/point_lio_ros2) | ROS 2 wrapper for Point-LIO |
| Unitree SDK | [unitreerobotics/unilidar_sdk](https://github.com/unitreerobotics/unilidar_sdk) | Unitree L1 hardware driver |
| MAVROS | [mavlink/mavros](https://github.com/mavlink/mavros) | MAVLink to ROS 2 bridge |
| PX4 Autopilot | [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | Flight controller firmware |
| foxglove_bridge | [foxglove/ros-foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge) | ROS 2 WebSocket for Foxglove Studio |
| Open3D | [isl-org/Open3D](https://github.com/isl-org/Open3D) | MLS smoothing, Ball Pivoting mesh, raycasting for occlusion |
| PDAL | [PDAL/PDAL](https://github.com/PDAL/PDAL) | SMRF ground classification |
| OpenCV | [opencv/opencv](https://github.com/opencv/opencv) | cv2.Stitcher panoramic stitching, blur detection |
| GDAL / gdal2tiles | [OSGeo/gdal](https://github.com/OSGeo/gdal) | XYZ tile pyramid generation |
| trimesh | [mikedh/trimesh](https://github.com/mikedh/trimesh) | Mesh I/O and merge |
| rosbags | [rpng/rosbags](https://gitlab.com/robustrobotics/rosbags) | MCAP bag reading without full ROS install |
| Three.js | [mrdoob/three.js](https://github.com/mrdoob/three.js) | WebGL 3D mesh viewer |
| Leaflet | [Leaflet/Leaflet](https://github.com/Leaflet/Leaflet) | Slippy-map tile viewer (Ortho tab) |

---

## License

| Component | License |
|---|---|
| Point-LIO | GPL-2.0 |
| Unitree SDK | See upstream repository |
| Custom flight stack, mesh pipeline, texture pipeline, ortho pipeline, viewers | MIT |
