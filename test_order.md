# DronePi Test Order — UPRM Capstone
## Autonomous Texture-Mapping Drone

**Purpose:** Systematic test sequence from bench validation to full autonomous survey flight.  
**Rule:** Never proceed to the next phase until the current phase passes cleanly.  
**Log every session:** `bash preflight_check.sh` at the start of every test day.

---

## PHASE 0 — System Health Check (Every Session)

Run this before anything else. Checks hardware, services, network, and scripts.

```bash
cd ~/unitree_lidar_project/unitree_drone_mapper
sudo bash preflight_check.sh
```

Check the log:
```bash
ls ~/unitree_lidar_project/logs/
cat ~/unitree_lidar_project/logs/log_1.txt
```

**Pass criteria:** Zero FAIL. WARN items reviewed and accepted.

---

## PHASE 1 — Service Verification (Bench, No Props)

### 1.1 Confirm all services running
```bash
sudo systemctl status mavros drone-watchdog dronepi-main drone-mesh-server foxglove-bridge
```

### 1.2 Confirm watchdog is polling
```bash
sudo journalctl -u drone-watchdog -f
# Expected: [WAITING] armed=False mode=ALTCTL  (every 0.5s)
# Ctrl+C to exit
```

### 1.3 Confirm main.py is running
```bash
sudo journalctl -u dronepi-main -f
# Expected: [IDLE] armed=False mode=ALTCTL lidar=no/yes
# Ctrl+C to exit
```

### 1.4 Confirm MAVROS FCU connection
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /mavros/state --once
# Expected: connected: true  armed: false  mode: ALTCTL
```

### 1.5 Confirm mesh server and browser viewer
```bash
curl http://10.42.0.1:8080/api/flights
# Open browser: http://10.42.0.1:8080/meshview.html
```

### 1.6 Confirm Foxglove connection
```
Open Foxglove Studio → Connect → ws://10.42.0.1:8765
Expected: topics list appears within 5s
```

**Pass criteria:** All services active, FCU connected, browser loads.

---

## PHASE 2 — LiDAR and SLAM Validation (Bench, No Props)

### 2.1 Start Point-LIO manually
```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
ros2 launch ~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/combined_lidar_mapping.launch.py rviz:=false port:=/dev/ttyUSB0
```

### 2.2 Verify topics publishing (new terminal)
```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
ros2 topic hz /unilidar/cloud --window 5
ros2 topic hz /aft_mapped_to_init --window 5
ros2 topic hz /cloud_registered --window 5
# Expected: /unilidar/cloud ~10Hz, /aft_mapped_to_init ~250Hz, /cloud_registered ~10Hz
```

### 2.3 Verify Z axis correction in Foxglove
```
Foxglove → 3D panel → topic: /cloud_registered
Lift LiDAR upward → cloud should move UP in viewer
Plot panel → /aft_mapped_to_init.pose.pose.position.z
Lift LiDAR → value should INCREASE
```

### 2.4 Run test_slam_live.py
```bash
cd ~/unitree_lidar_project/unitree_drone_mapper/tests
python3 test_slam_live.py --no-rviz
# Walk around with LiDAR for 60s
# Expected: Topic status shows all three topics at expected rates
# Ctrl+C to stop
```

### 2.5 Test SLAM bridge
```bash
# Terminal 1: Point-LIO already running
# Terminal 2:
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/flight/_slam_bridge.py
# Expected: "SLAM bridge ready: /aft_mapped_to_init -> /mavros/vision_pose/pose"
# Then every 250 frames: "Bridge: 250 frames  ENU=(x, y, z)"

# Terminal 3 -- verify topic publishing:
ros2 topic hz /mavros/vision_pose/pose --window 5
# Expected: ~250Hz
```

**Pass criteria:** All topics publishing at expected rates. Z increases on lift.

---

## PHASE 3 — Postflight Pipeline Validation (Bench)

### 3.1 Process existing bag
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/utils/run_postflight.py \
  --bag /mnt/ssd/rosbags/scan_20260315_191900
# Expected: cloud + mesh PLY generated, browser auto-loads within 10s
```

### 3.2 Verify browser loads result
```
http://10.42.0.1:8080/meshview.html
Toggle Cloud / Mesh buttons
Check flight database sidebar shows the session
```

### 3.3 Process latest bag
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/utils/run_postflight.py
# Should find latest bag automatically
```

**Pass criteria:** PLY files generated, viewer loads, cloud and mesh toggle works.

---

## PHASE 4 — Watchdog Arm Detection Test (Bench, No Props)

### 4.1 Watch watchdog respond to arm/mode changes
```bash
sudo journalctl -u drone-watchdog -f
# On RC transmitter: arm the drone (no props installed)
# Expected: [WAITING] armed=True mode=ALTCTL
# Switch to OFFBOARD on transmitter (if setpoints streaming)
# Expected: [ACTIVATING] Session #1 armed=True mode=OFFBOARD
# Disarm
# Expected: [DEACTIVATING] then postflight triggered
```

### 4.2 Test manual scan mode detection in main.py
```bash
sudo journalctl -u dronepi-main -f
# Arm without switching to OFFBOARD
# Wait 10s
# Expected: [MANUAL_SCAN] committed
# Disarm
# Expected: back to IDLE
```

### 4.3 Verify lock file behavior
```bash
# While armed in manual scan mode:
cat /tmp/dronepi_mission.lock
# Expected: {"mode": "manual_scan", "started_at": "..."}
# After disarm:
ls /tmp/dronepi_mission.lock
# Expected: file gone (cleared by watchdog after postflight)
```

**Pass criteria:** State machine transitions correct, lock file written and cleared.

---

## PHASE 5 — OFFBOARD Flight Tests (Outdoor, Props On, GPS Lock Required)

> ⚠️ **SAFETY:** RC transmitter in hand. Kill switch ready. Area clear. GPS lock confirmed (≥10 sats, HDOP < 2.0) before any test.

### 5.1 Preflight check outdoors
```bash
sudo bash preflight_check.sh
# Must show: GPS home position received, FCU connected
# Zero FAIL
```

### 5.2 GPS hover dry run (no arming)
```bash
source /opt/ros/jazzy/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_offboard_gps.py --dry-run
# Expected: GPS lock confirmed, home position captured
# NO MOTORS
```

### 5.3 Simple OFFBOARD hover test (3m AGL, 15s hold)
```bash
source /opt/ros/jazzy/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_offboard_flight.py \
  --alt 3.0 --hold 15
# Script arms and switches to OFFBOARD automatically
# Drone climbs to 3m, holds, lands
# RC override available at any time (move sticks)
```

### 5.4 GPS waypoint hover test
```bash
source /opt/ros/jazzy/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_offboard_gps.py \
  --alt 5.0 --hold 10
# Hover above home at 5m for 10s then land
```

**Pass criteria:** Drone takes off, holds position, lands cleanly.

---

## PHASE 6 — SLAM Bridge Flight Test (Outdoor, Props On)

### 6.1 SLAM bridge out-and-back dry run
```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_slam_bridge_flight.py \
  --dry-run --no-bridge --no-bag
# Expected: EKF stable, home position captured, waypoints previewed
# NO MOTORS
```

### 6.2 Out-and-back without SLAM bridge (GPS only baseline)
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_slam_bridge_flight.py \
  --distance 5 --alt 3.0 --hold 5 --no-bridge --no-bag
# 5m forward, hold 5s, return, land
# GPS only -- establishes baseline position hold accuracy
```

### 6.3 Out-and-back with SLAM bridge active
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_slam_bridge_flight.py \
  --distance 5 --alt 3.0 --hold 5 --no-bag
# Same flight with SLAM bridge feeding /mavros/vision_pose/pose
# Compare position hold accuracy vs 6.2
# Watch Foxglove: EKF innovations panel should show vision residuals
```

### 6.4 Out-and-back with full recording
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_slam_bridge_flight.py \
  --distance 8 --alt 4.0 --hold 10
# Full recording -- bag saved, postflight runs automatically
# Browser auto-loads mesh within 10s of landing
```

**Pass criteria:** Drone follows path accurately, SLAM bridge active, mesh appears in browser.

---

## PHASE 7 — Full Flight Mission Dry Run (Outdoor, No Arming)

### 7.1 Upload survey mission in QGC
```
QGC → Plan → Survey
Draw bounding box over test area (~30m x 30m)
Set altitude: 15m AGL
Set overlap: 75%
Click Upload to Vehicle
```

### 7.2 Mission executor dry run
```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/flight/mission_executor.py --dry-run
# Expected: waypoints received and converted to local ENU
# Prints waypoint list with ENU coordinates and footprint size
# NO MOTORS
```

### 7.3 Full flight mission dry run
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/flight_mission.py --dry-run
# Runs all 6 pre-flight checks:
#   [1] FCU connected
#   [2] GPS lock + home
#   [3] EKF stable
#   [4] SSD writable
#   [5] Launch file exists
#   [6] Mission waypoints present
# Prints mission waypoints in ENU
# NO MOTORS
```

**Pass criteria:** All pre-flight checks pass, waypoints displayed correctly.

---

## PHASE 8 — Full Autonomous Survey Mission (Outdoor, Props On)

> ⚠️ **SAFETY:** Only proceed after ALL previous phases pass. Full area clear. Second person as safety observer recommended.

### 8.1 Final preflight check
```bash
sudo bash preflight_check.sh
# Zero FAIL required
```

### 8.2 Verify mission uploaded in QGC
```
QGC → Plan → confirm waypoints visible
QGC → Fly → confirm home position set
```

### 8.3 Execute full autonomous mission
```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/flight_mission.py \
  --trigger-mode stop --fov-deg 70.0
# Sequence:
#   Pre-flight checks (all 6 must pass)
#   watchdog stopped
#   Point-LIO + SLAM bridge + bag started
#   Arm gate: arm on transmitter + switch to OFFBOARD
#   Climb → execute survey waypoints → gap fill → camera trigger stubs
#   AUTO.RTL → land → disarm
#   watchdog restored
#   postflight runs → mesh in browser within 10s
```

### 8.4 Verify mesh in browser
```
http://10.42.0.1:8080/meshview.html
Toggle cloud / mesh
Check flight database sidebar
Download PLY if needed
```

**Pass criteria:** Mission completes, RTL and landing clean, mesh appears in browser.

---

## PHASE 9 — Post-Camera Integration (After IMX477 Reinstall)

> Run after camera hardware is reinstalled and calibrated.

### 9.1 Camera calibration verification
```bash
source /opt/ros/jazzy/setup.bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/tests/test_camera_preview.py
# Verify image stream active, calibration YAML matches actual FOV
```

### 9.2 Full mission with camera trigger
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/flight_mission.py \
  --trigger-mode stop
# Camera trigger stubs replaced by real capture
# Verify images saved alongside bag
```

### 9.3 Textured mesh generation
```bash
python3 ~/unitree_lidar_project/unitree_drone_mapper/utils/run_postflight.py
# Verify textured PLY loads in browser with real color
```

---

## Quick Reference — All Test Commands

| Test | Command |
|---|---|
| System check | `sudo bash preflight_check.sh` |
| Watchdog logs | `sudo journalctl -u drone-watchdog -f` |
| Main.py logs | `sudo journalctl -u dronepi-main -f` |
| MAVROS state | `ros2 topic echo /mavros/state --once` |
| SLAM live test | `python3 tests/test_slam_live.py --no-rviz` |
| Process bag | `python3 utils/run_postflight.py` |
| OFFBOARD hover | `python3 tests/test_offboard_flight.py --alt 3.0` |
| GPS hover | `python3 tests/test_offboard_gps.py --alt 5.0 --dry-run` |
| Out-and-back dry | `python3 tests/test_slam_bridge_flight.py --dry-run --no-bridge --no-bag` |
| Out-and-back live | `python3 tests/test_slam_bridge_flight.py --distance 5 --alt 3.0 --no-bag` |
| Mission dry run | `python3 flight_mission.py --dry-run` |
| Full mission | `python3 flight_mission.py` |
| Viewer | `http://10.42.0.1:8080/meshview.html` |
| Hotspot on | `hotspot on` |
| Lock file check | `cat /tmp/dronepi_mission.lock` |
| Clear lock | `rm /tmp/dronepi_mission.lock` |
| View logs | `ls ~/unitree_lidar_project/logs/` |
