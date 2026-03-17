# Tarot 810 Autonomous Texture-Mapping Drone
## Project Status Report
**Date:** March 16, 2026 | **Target Completion:** May 2026 | **Institution:** UPRM

---

## 1. Project Overview

The goal is a fully autonomous drone capable of flying a survey mission over a structure, capturing LiDAR geometry and camera imagery, and producing a textured 3D mesh viewable in a browser on a laptop in the field — with zero manual steps between landing and mesh display.

**Core pipeline:** QGroundControl waypoint plan → OFFBOARD autonomous flight → Point-LIO SLAM → post-flight Poisson reconstruction → camera texture projection → browser-based Three.js viewer.

---

## 2. Hardware — Assembly Status

| Component | Status | Notes |
|---|---|---|
| Tarot 810 hexacopter frame | ✅ Complete | Non-foldable legs, rigid sensor plate |
| Pixhawk 6X (PX4 firmware) | ✅ Complete | MAVROS over USB `/dev/ttyACM0` |
| Raspberry Pi 5 (16GB, 3.0GHz OC) | ✅ Complete | Ubuntu 24.04, ROS 2 Jazzy |
| Unitree 4D L1 LiDAR | ✅ Mounted | `/dev/ttyUSB0`, co-mounted with camera |
| Hailo-8 AI HAT | ✅ Mounted | PCIe Gen 3, `hailo_inference_env` venv |
| Fanxiang NVMe SSD | ✅ Mounted | USB-connected, `/mnt/ssd`, ext4 `nofail` |
| TP-Link AC600 Wi-Fi dongle | ✅ Working | RTL8821AU driver installed |
| Holybro RGB/USB module | ✅ Connected | Pixhawk I2C, system health indication |
| RPi HQ Camera (IMX477) | 🔴 Dismounted | Awaiting Arducam CSI-to-HDMI kit arrival |
| Tamron 4–12mm C-mount lens | ✅ Ready | C-CS adapter removed for infinity focus |
| Arducam CSI-to-HDMI kit | ⏳ On order | ~1m extension for vibration resistance |

---

## 3. Software Stack — Installation Status

| Package | Status | Notes |
|---|---|---|
| Ubuntu 24.04 | ✅ | Pi 5, arm64 |
| ROS 2 Jazzy | ✅ | Native install, not Docker |
| MAVROS + mavros_msgs | ✅ | `ros-jazzy-mavros` |
| Point-LIO SLAM | ✅ | Built from source, `unilidar_l1.yaml` config |
| foxglove_bridge | ✅ | `ros-jazzy-foxglove-bridge` v3.2.3 |
| pymeshlab | ✅ | ARM64 wheel, Python 3.12 system |
| trimesh | ✅ | ARM64 wheel, Python 3.12 system |
| rosbags | ✅ | ARM64 wheel, Python 3.12 system |
| Open3D | ❌ | No ARM64 wheel — replaced by pymeshlab |
| hailo_platform 4.17.0 | ✅ | `hailo_inference_env` venv |
| rpicam-apps / libcamera | ✅ | Matched versions, `dtoverlay=imx477` |
| QGroundControl | ✅ | Ground station laptop |
| Foxglove Studio | ✅ | Ground station laptop |

---

## 4. Calibration Status

| Calibration | Status | Notes |
|---|---|---|
| Camera intrinsic (IMX477) | ✅ Complete | 10×7 checkerboard, RMS ~1.99, accepted |
| Extrinsic LiDAR↔Camera | ✅ Measured | Translation `[-0.070, 0.000, 0.030]`, rotation `[0,0,0]` |
| Extrinsic in code | ⚠️ Unverified | Audit `test_texture_live.py` — confirm not identity matrix |
| PX4 accelerometer | ✅ Complete | Via QGroundControl |
| PX4 magnetometer | ✅ Complete | Via QGroundControl |
| PX4 RC calibration | ✅ Complete | Via QGroundControl |
| ESC calibration | ✅ Complete | Via QGroundControl |

---

## 5. Boot Services — Auto-Start on Power-On

All three services start automatically on boot with no user interaction required.

| Service | Port | Status | Notes |
|---|---|---|---|
| `dronepi-hotspot` | — | ✅ Active | SSID `dronepi-ap`, IP `10.42.0.1` |
| `drone-mesh-server` | 8080 | ✅ Active | Serves `/mnt/ssd/maps/`, PLY + viewer |
| `foxglove-bridge` | 8765 | ✅ Active | WebSocket ROS 2 → Foxglove Studio |
| `mavros + point-lio` | — | ⏳ Pending | Systemd service not yet created |

**Laptop workflow after Pi boots:**
1. Connect to Wi-Fi `dronepi-ap` / `dronepi123`
2. Open browser → `http://10.42.0.1:8080/meshview.html` (mesh viewer)
3. Open Foxglove Studio → `ws://10.42.0.1:8765` (live ROS 2 monitoring)
4. SSH if needed → `ssh dronepi@10.42.0.1`

---

## 6. Testing Completed

### Step 1 — LiDAR Pipeline ✅
- L1 enumerates on `/dev/ttyUSB0`
- Point-LIO launches cleanly, IMU initializes to 100%
- `/cloud_registered` publishes at ~6.7 Hz
- `/aft_mapped_to_init` publishes at ~250 Hz (canonical pose topic)
- RViz and Foxglove both confirmed receiving point cloud live
- **Key finding:** `/Odometry` is never published by this Point-LIO build — `/aft_mapped_to_init` is the correct pose topic

### Step 2 — Mesh Reconstruction ✅
- Rosbag recorded: `lidar_test_03` — 155s, 334MB, all four topics
- `test_mesh_from_bag.py` pipeline passes end-to-end
- Adaptive voxel sizing: 0.20m, 7,522 points after downsample
- Z-buffer occlusion: 1,811 visible, 24 occluded
- Poisson reconstruction: 25,168 vertices, 50,456 faces, **watertight**
- Runtime: 2.1 seconds on Pi 5

### Step 3 — Ground Software Stack ✅
- MAVROS + Point-LIO running simultaneously confirmed
- `/cloud_registered` mild degradation under load: 6.7 Hz → 4.8 Hz (acceptable)
- RTT timesync warnings 14–145ms over USB (monitor in flight)

### Step 4A — PX4 Parameter Verification ✅
- Motor order and direction confirmed in QGroundControl
- RC mode switches mapped: Stabilized, Position, Kill switch
- Pre-arm checks configured for USB operation
- Props-off motor test passed

### Step 4B — Manual Flight ✅
- Stabilized mode hover: stable
- Position mode hover: stable
- Altitude mode: stable
- GPS lock confirmed outdoors, preflight warnings cleared

### Step 4C — OFFBOARD Dry Run ✅
- EKF initializes without GPS home indoors
- Home Z captured correctly (`-5.566m` — ENU frame, expected)
- Setpoints streaming at 20 Hz confirmed
- `test_offboard_flight.py` dry run: PASSED

### Step 4C — OFFBOARD Live Flight ⚠️ Partially Tested
- First attempt: aggressive behavior due to EKF not initialized before arming
- Second attempt: completed but altitude readings inverted (home Z not captured)
- **Root causes identified and fixed:**
  - EKF initialization wait added (2s stability check)
  - Home Z captured at EKF stable time, all commands relative to baseline
  - Yaw quaternion now read from current pose (prevents snap rotation on mode switch)
- **Status:** Ready for re-test outdoors with GPS lock

### Step 5 — Mesh Viewer ✅
- `meshview.html` Three.js viewer deployed to `/mnt/ssd/maps/`
- PLY parser rewritten to handle `double` XYZ + `uchar` RGBA + `double quality` (pymeshlab format)
- Auto-polls `latest.json` every 10s, loads mesh automatically
- 25,168 vertex mesh rendering at 36 fps on laptop GPU
- Orbit / pan / zoom controls working
- **Color rendering pending** — camera offline, mesh renders white until texture projection

### Step 6 — Foxglove Integration ✅
- foxglove_bridge v3.2.3 installed and confirmed working
- Point cloud, pose trajectory visible in Foxglove 3D panel
- Auto-starts on boot as systemd service

---

## 7. Scripts Inventory

| Script | Location | Purpose | Status |
|---|---|---|---|
| `test_slam_live.py` | `tests/` | Launch LiDAR + Point-LIO + RViz | ✅ Working (timeout display bug — cosmetic) |
| `test_mesh_from_bag.py` | `tests/` | Mesh reconstruction from rosbag | ✅ Working |
| `test_offboard_flight.py` | `tests/` | OFFBOARD hover test (local ENU) | ✅ Ready, pending re-test |
| `test_offboard_gps.py` | `tests/` | OFFBOARD GPS waypoint mission | ✅ Ready, pending first test |
| `test_Mavros_hover.py` | `tests/` | pymavlink direct arm/status check | ✅ Working |
| `test_texture_live.py` | `tests/` | Full in-flight texture pipeline | ⚠️ Camera offline |
| `serve.py` | `/mnt/ssd/maps/` | CORS HTTP server for viewer | ✅ Running as service |
| `meshview.html` | `/mnt/ssd/maps/` | Three.js mesh viewer | ✅ Deployed |
| `postprocess_mesh.py` | `tests/` | Post-flight mesh + texture + trigger | ⏳ Not yet created |

---

## 8. Pending — Blocked on Camera Hardware

All items below require the Arducam CSI-to-HDMI kit to arrive and the IMX477 to be reinstalled.

- [ ] Camera CSI signal verification (`rpicam-still` test capture)
- [ ] Extrinsic calibration code audit — confirm `[-0.070, 0.000, 0.030]` is in `test_texture_live.py`
- [ ] `cv2.undistort` distortion correction with calibration coefficients
- [ ] `rpicam-still` / V4L2 fallback capture path
- [ ] IMX477 4056×3040 full resolution config
- [ ] Bilinear interpolation in texture projection
- [ ] Full texture projection validation — camera color onto LiDAR mesh
- [ ] `postprocess_mesh.py` — post-flight script that runs mesh + texture and writes `latest.json`
- [ ] Colored mesh end-to-end test in `meshview.html`

---

## 9. Pending — Not Yet Started

- [ ] **MAVROS + Point-LIO systemd service** — auto-start full ROS 2 stack on boot
- [ ] **OFFBOARD live flight re-test** — patched script with EKF init + yaw fix
- [ ] **GPS waypoint OFFBOARD test** — `test_offboard_gps.py` first live run
- [ ] **`test_slam_live.py` timeout fix** — cosmetic bug, low priority
- [ ] **QGroundControl survey mission integration** — pull waypoint plan into ROS 2
- [ ] **Camera trigger via PX4 `TRIG_MODE=4`** — event-driven capture on `/mavros/cam_imu_sync`
- [ ] **Rosbag record during flight** — confirm post-flight pipeline has data
- [ ] **Full autonomous survey flight** — complete end-to-end mission
- [ ] **Capstone demo preparation** — rehearsal flight, documentation

---

## 10. Immediate Next Steps (Priority Order)

1. **Battery charged → OFFBOARD re-test outdoors** — confirm patched script flies cleanly
2. **MAVROS + Point-LIO systemd service** — 30 min, completes the boot automation
3. **CSI converters arrive → camera reinstall** — unblocks texture pipeline
4. **Extrinsic calibration audit** in `test_texture_live.py`
5. **`postprocess_mesh.py`** — post-flight processing script
6. **GPS waypoint OFFBOARD test** — `test_offboard_gps.py`
7. **Full autonomous survey flight**
8. **Capstone demo**

---

## 11. Key Technical Decisions (Reference)

| Decision | Rationale |
|---|---|
| Native ROS 2 Jazzy over Docker | Reliability, no DDS bridging overhead |
| MAVROS over USB vs XRCE-DDS | XRCE-DDS over Ethernet proved unreliable in practice |
| `/aft_mapped_to_init` not `/Odometry` | `/Odometry` never published by this Point-LIO build |
| pymeshlab over Open3D | No Open3D ARM64 wheel for Python 3.12 |
| Post-flight mesh processing | Preserve in-flight CPU budget for SLAM + MAVROS |
| Physical extrinsic measurement | Practical for capstone scope |
| GPS global setpoints for survey | Avoids local ENU frame conversion complexity |
| EKF home Z capture before arming | Prevents aggressive takeoff from frame origin mismatch |
| Yaw quaternion from current pose | Prevents snap rotation on OFFBOARD mode switch |

---

*Generated March 16, 2026 — DronePi Capstone Project, UPRM*
