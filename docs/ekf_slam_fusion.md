# EKF2 and SLAM Sensor Fusion Reference

**Document:** 3 of 6
**Repo path:** `docs/ekf_slam_fusion.md`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [Overview — What We Are Fusing and Why](#1-overview--what-we-are-fusing-and-why)
2. [Sensors in the Fusion Stack](#2-sensors-in-the-fusion-stack)
3. [What EKF2 Is](#3-what-ekf2-is)
4. [EKF2 State Vector](#4-ekf2-state-vector)
5. [Sensor Fusion Architecture](#5-sensor-fusion-architecture)
6. [Point-LIO SLAM](#6-point-lio-slam)
7. [SLAM Bridge — Coordinate Frame Conversion](#7-slam-bridge--coordinate-frame-conversion)
8. [EKF2 Parameter Reference](#8-ekf2-parameter-reference)
9. [Collision Prevention Parameters](#9-collision-prevention-parameters)
10. [Topic Reference](#10-topic-reference)
11. [Tuning After First Flight](#11-tuning-after-first-flight)
12. [Failure Modes and Diagnostics](#12-failure-modes-and-diagnostics)

---

## 1. Overview — What We Are Fusing and Why

A drone in autonomous flight needs to know six things at all times: where it is
(X, Y, Z), how fast it is moving (velocity in three axes), and how it is oriented
(roll, pitch, yaw). No single sensor provides all of this accurately enough on its
own.

DronePi fuses five data sources into a single consistent state estimate using the
PX4 Extended Kalman Filter (EKF2):

| Source | What it contributes | Weakness |
|---|---|---|
| LiDAR-inertial SLAM (Point-LIO) | Absolute 3D position and yaw in a local map frame | Drifts over long sessions, no global reference |
| M9N GPS | Global horizontal position (lat/lon) | Poor indoors, needs open sky, ~2m accuracy |
| IMU (in Pixhawk 6X) | High-rate angular velocity and acceleration | Integrates drift rapidly without corrections |
| Barometer (in Pixhawk 6X) | Atmospheric pressure altitude | Affected by wind and temperature, noisy |
| LiDAR AGL (from collision_monitor.py) | Height above ground (downward ranging) | Only valid for flat terrain directly below |

EKF2 weighs all five sources against each other every IMU cycle (~250 Hz) and
produces a fused estimate that is more accurate and more robust than any single
sensor alone. If GPS is unavailable indoors, SLAM fills the position role. If SLAM
drifts during a long flight, GPS corrects it. If both fail, the barometer and IMU
keep the drone stable long enough to land.

---

## 2. Sensors in the Fusion Stack

### IMU — Pixhawk 6X internal

The IMU is the backbone of EKF2. It runs at ~250 Hz and provides the prediction
step of the filter. Every other sensor provides correction steps at lower rates.

- Accelerometer: 3-axis linear acceleration (m/s²)
- Gyroscope: 3-axis angular rate (rad/s)

EKF2 integrates the IMU continuously. Without corrections, position error grows
as the square of time. All other sensors exist to bound this drift.

### GPS — M9N

The M9N connects internally through the Pixhawk. It provides horizontal position
(latitude and longitude) and velocity. In this build it is configured as a
secondary horizontal position source, subordinate to SLAM.

Arming requirement: `COM_ARM_WO_GPS = 1` is set, so the drone can arm without GPS
lock. GPS is used when available but is not required.

Outdoor requirement: GPS needs unobstructed sky view. UPRM campus tests have shown
4 satellites and HDOP 2.5 indoors or near buildings — insufficient for reliable
fusion. An open field away from structures is required for GPS to contribute
meaningfully. Target: at least 10 satellites, HDOP below 1.2, VDOP below 1.5.

### Barometer — Pixhawk 6X internal

Used as a last-resort height fallback only. `EKF2_BARO_CTRL = 1` keeps it active
in the fusion but at low weight when SLAM is providing height. It becomes the
primary height source only if both SLAM and GPS are unavailable.

### SLAM (Point-LIO via SLAM bridge)

The primary position, velocity, and height source in this build. SLAM provides
a pose estimate in a local ENU map frame at ~90 Hz after conversion by the SLAM
bridge. PX4 EKF2 receives this as a vision pose measurement.

SLAM does not provide a global position. Its reference frame is the drone's
position at the moment Point-LIO was initialized. Any drift in SLAM accumulates
until a GPS correction is applied.

### LiDAR AGL (collision_monitor.py)

`collision_monitor.py` extracts the downward-facing point cloud slice from
`/cloud_registered` and computes the median distance to the ground below the
drone. This is published as a rangefinder measurement and fused by EKF2 as a
conditional height source (`EKF2_RNG_CTRL = 1` — conditional mode uses the
rangefinder only when it is consistent with the primary height source).

---

## 3. What EKF2 Is

EKF2 is an Extended Kalman Filter implementation inside PX4 firmware. It runs
on the Pixhawk flight controller, not on the Raspberry Pi.

A Kalman filter works in two steps repeated every cycle:

**Predict:** Using the IMU measurement and a motion model, project the current
state estimate forward in time. Uncertainty grows during this step because the
IMU integrates noise.

**Update:** When a sensor measurement arrives (SLAM pose, GPS fix, barometer
reading), compare it to what the filter predicted. Weight the correction by the
relative trust in the sensor versus the current prediction. High-trust sensors
pull the estimate strongly toward their measurement. Low-trust sensors contribute
weakly.

EKF2 is an *extended* Kalman filter because the drone's motion model is nonlinear
(rotation in 3D space cannot be described with a simple linear equation). EKF2
linearizes the model at each step to apply the standard Kalman update.

The result is a continuously updated estimate of position, velocity, orientation,
and sensor biases — the best available answer to "where is the drone right now"
given all available sensor data.

---

## 4. EKF2 State Vector

EKF2 tracks 24 states internally:

| States | Description |
|---|---|
| Position (3) | X, Y, Z in local NED frame (metres) |
| Velocity (3) | Vx, Vy, Vz in local NED frame (m/s) |
| Quaternion (4) | Orientation as unit quaternion (roll, pitch, yaw) |
| Gyro bias (3) | Estimated bias on each gyroscope axis (rad/s) |
| Accelerometer bias (3) | Estimated bias on each accelerometer axis (m/s²) |
| Earth magnetic field (3) | Estimated local magnetic field vector |
| Wind velocity (2) | Estimated horizontal wind (m/s) — if enabled |
| Terrain height (1) | Estimated ground height relative to home |

Biases are important: EKF2 learns the systematic error in the IMU and subtracts
it, greatly improving position hold accuracy over time.

---

## 5. Sensor Fusion Architecture

The following describes how data flows from each sensor through to EKF2.

```
Unitree L1 LiDAR
    |
    | /unilidar/cloud (9.8 Hz)
    v
Point-LIO SLAM (running on Raspberry Pi 5)
    |
    | /aft_mapped_to_init (pose, ENU frame, ~250 Hz)
    v
_slam_bridge.py (running on Raspberry Pi 5)
    |
    | /mavros/vision_pose/pose (PoseStamped, RELIABLE QoS, ~90 Hz)
    | /mavros/odometry/out    (Odometry, velocity bits, ~90 Hz)
    v
MAVROS (running on Raspberry Pi 5)
    |
    | MAVLink VISION_POSITION_ESTIMATE message
    | MAVLink ODOMETRY message
    v
Pixhawk 6X — EKF2
    ^
    | MAVLink GPS_RAW_INT
M9N GPS (via Pixhawk internal port)
    ^
    | Internal ADC
Barometer (Pixhawk internal)
    ^
    | /mavros/distance_sensor/lidar_down_sub
collision_monitor.py → Pixhawk AGL rangefinder
```

EKF2 fuses all inputs and publishes its estimate back to MAVROS:
- `/mavros/local_position/pose` — fused local position
- `/mavros/local_position/velocity_local` — fused velocity
- `/mavros/imu/data` — corrected IMU with biases removed

The flight stack reads these topics to issue OFFBOARD setpoints.

---

## 6. Point-LIO SLAM

Point-LIO is a LiDAR-inertial odometry algorithm developed at HKU-MARS lab.
It fuses LiDAR point clouds with IMU data to estimate the drone's 6-DOF pose
in real time without requiring loop closures or prior maps.

### How it works

Point-LIO maintains a local 3D map of the environment built from accumulated
point clouds. For each new scan, it registers the incoming points against the
existing map to find the transformation (position + rotation) that best aligns
them. The IMU provides high-rate motion predictions between scans. The result
is a smooth, high-rate pose estimate even when the LiDAR update rate is limited.

### Key topics

| Topic | Rate | Description |
|---|---|---|
| `/unilidar/cloud` | 9.8 Hz | Raw point cloud from Unitree L1 driver |
| `/unilidar/imu` | ~200 Hz | Raw IMU from Unitree L1 |
| `/cloud_registered` | 9.7 Hz | Point cloud transformed into the map frame |
| `/aft_mapped_to_init` | ~250 Hz | Pose estimate (position + quaternion) in ENU |

### Configuration

Configuration file: `RPI5/ros2_ws/src/point_lio_ros2/config/unilidar_l1.yaml`

Key parameters:
- `filter_size_map` — voxel size for the map (smaller = more detail, more memory)
- `cube_side_length` — size of the local map cube around the drone
- `time_sync_en` — time synchronization between LiDAR and IMU

### Known limitation

Point-LIO has no loop closure. On scans longer than approximately 2 minutes,
SLAM drift accumulates. The drift manifests as a slow translation of the pose
estimate from the true position. GPS fusion partially corrects this. For the
capstone demo, scan sessions are kept short enough that drift is not a blocker.

---

## 7. SLAM Bridge — Coordinate Frame Conversion

PX4 uses a NED (North-East-Down) coordinate frame internally. Point-LIO outputs
pose in ENU (East-North-Up). MAVROS handles the ENU to NED conversion internally
when the pose is published to the correct topic — the SLAM bridge does not need
to perform this conversion manually.

### What the SLAM bridge does

```python
# _slam_bridge.py — simplified logic

# Subscribe to Point-LIO pose (ENU, BEST_EFFORT QoS)
/aft_mapped_to_init  →  geometry_msgs/PoseStamped

# Publish vision pose to MAVROS (ENU pass-through — MAVROS converts internally)
/mavros/vision_pose/pose  →  geometry_msgs/PoseStamped  (RELIABLE QoS)

# Also publish odometry for EKF2 velocity fusion (bit 2 of EKF2_EV_CTRL)
/mavros/odometry/out  →  nav_msgs/Odometry
```

The SLAM bridge subscribes with `BEST_EFFORT` QoS to match the Point-LIO
publisher profile. The vision pose topic is published with `RELIABLE` QoS as
required by MAVROS. A QoS mismatch on either topic causes silent connection
failure — the topic appears in `ros2 topic list` but no messages flow through.

### Timestamp handling

The bridge passes the Point-LIO header timestamp directly through to MAVROS.
EKF2 uses this timestamp to compute the measurement delay for fusion weighting.
The `EKF2_EV_DELAY` parameter must be set to the actual measured latency
between LiDAR scan capture and MAVROS receipt. Default value is 175 ms.
This should be measured empirically after the first SLAM flight and adjusted.

---

## 8. EKF2 Parameter Reference

All parameters are set in QGroundControl under Parameters. Save presets to
`slam_gps.params` and `gps_only.params` via Parameters → Tools → Save to file.

### Vision / SLAM fusion control

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_EV_CTRL` | 15 | Bitmask enabling all four vision bits: horizontal position (bit 0), vertical position (bit 1), 3D velocity (bit 2), yaw (bit 3). Value 15 = all bits set (0b1111). |
| `EKF2_HGT_REF` | 3 | Primary height source. 0 = baro, 1 = GPS, 2 = rangefinder, 3 = vision (SLAM). Setting 3 makes SLAM the height reference. |
| `EKF2_EV_DELAY` | 30 ms | Estimated delay between the SLAM measurement timestamp and when it arrives at EKF2. Compensates for Pi processing latency. Measure empirically after first flight. |
| `EKF2_EV_POS_NOISE` | default | Measurement noise for vision position. Increase if SLAM is noisy; decrease if it is smooth and trusted. |
| `EKF2_EV_ANG_NOISE` | default | Measurement noise for vision yaw. |

### GPS fusion control

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_GPS_CTRL` | 1 | GPS provides horizontal position only. Bit 0 only. SLAM provides height (bit 1 not set). |
| `EKF2_GPS_DELAY` | default | GPS measurement delay. Usually accurate for M9N via Pixhawk. |
| `COM_ARM_WO_GPS` | 1 | Allow arming without GPS fix. Required for indoor and SLAM-only flight. |

### Magnetometer

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_MAG_TYPE` | 1 | Automatic mode. EKF2 uses SLAM yaw as the primary yaw source when SLAM is active, falls back to magnetometer when SLAM is not available. |

### Rangefinder (LiDAR AGL)

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_RNG_CTRL` | 1 | Conditional rangefinder fusion. The AGL estimate from `collision_monitor.py` is fused only when it is consistent with the primary height source. Prevents bad reads during banking turns. |
| `EKF2_RNG_POS_X` | TBD | X offset of the LiDAR from center of gravity (metres). Must be measured and set. |
| `EKF2_RNG_POS_Y` | TBD | Y offset of the LiDAR from center of gravity (metres). |
| `EKF2_RNG_POS_Z` | TBD | Z offset of the LiDAR from center of gravity (metres). |

### Barometer

| Parameter | Value | Meaning |
|---|---|---|
| `EKF2_BARO_CTRL` | 1 | Barometer kept active as last-resort fallback. It contributes weakly when SLAM height is healthy. |

---

## 9. Collision Prevention Parameters

These parameters are set in QGroundControl and work alongside `collision_monitor.py`.

| Parameter | Value | Meaning |
|---|---|---|
| `CP_DIST` | 2.0 m | Hard stop distance. The flight controller will resist any setpoint that would bring the drone within 2.0 m of a detected obstacle. Must match `OBSTACLE_RADIUS` in `collision_monitor.py`. |
| `CP_DELAY` | 0.2 s | Estimated processing delay for obstacle distance data. Used by PX4 to compute a braking margin. |
| `MPC_POS_MODE` | 4 | Acceleration-based position control mode. Required for collision prevention to function — mode 4 is the only mode where CP modifies setpoints. |

---

## 10. Topic Reference

Complete list of topics relevant to the fusion stack.

| Topic | Direction | Type | Rate | Description |
|---|---|---|---|---|
| `/unilidar/cloud` | LiDAR driver → Point-LIO | PointCloud2 | 9.8 Hz | Raw LiDAR scan |
| `/unilidar/imu` | LiDAR driver → Point-LIO | Imu | ~200 Hz | IMU from L1 sensor |
| `/cloud_registered` | Point-LIO → bag, collision | PointCloud2 | 9.7 Hz | Map-frame point cloud |
| `/aft_mapped_to_init` | Point-LIO → SLAM bridge | PoseStamped | ~250 Hz | SLAM pose in ENU |
| `/mavros/vision_pose/pose` | SLAM bridge → MAVROS | PoseStamped | ~90 Hz | Vision pose for EKF2 |
| `/mavros/odometry/out` | SLAM bridge → MAVROS | Odometry | ~90 Hz | Vision velocity for EKF2 |
| `/mavros/obstacle/send` | collision_monitor → MAVROS | LaserScan | ~9 Hz | 3-zone obstacle distances |
| `/mavros/distance_sensor/lidar_down_sub` | collision_monitor → MAVROS | Range | ~9 Hz | AGL height estimate |
| `/mavros/state` | MAVROS → flight stack | State | 1 Hz | FCU connection, arm, mode |
| `/mavros/local_position/pose` | MAVROS → flight stack | PoseStamped | ~30 Hz | EKF2 fused position |
| `/mavros/imu/data` | MAVROS → flight stack | Imu | ~100 Hz | Bias-corrected IMU |
| `/mavros/setpoint_position/local` | flight stack → MAVROS | PoseStamped | 20 Hz | OFFBOARD position setpoints |
| `/mavros/cmd/arming` | flight stack → MAVROS | service | on demand | Arm and disarm |
| `/mavros/set_mode` | flight stack → MAVROS | service | on demand | Flight mode changes |

---

## 11. Tuning After First Flight

These parameters require empirical measurement after the first flight with
SLAM running. They cannot be accurately determined in advance.

### EKF2_EV_DELAY

Measures the actual latency between the LiDAR scan timestamp and the MAVROS
receipt time. To measure:

1. Record a flight with `/aft_mapped_to_init` and `/mavros/local_position/pose`
   both logged in the rosbag.
2. In Foxglove Studio, plot both topics on the same time axis.
3. Find a sharp motion event (a quick yaw or position step).
4. Measure the time offset between the same event appearing in both topics.
5. Set `EKF2_EV_DELAY` to this measured value in milliseconds.

Current setting: 30 ms (estimated). Default was 175 ms.

### EKF2_RNG_POS_X/Y/Z

Measure the physical offset of the Unitree L1 LiDAR from the drone's center of
gravity using a ruler. The center of gravity is approximately at the geometric
center of the frame.

Current measured values:
- X: -0.070 m (LiDAR is 70mm forward of CG)
- Y: 0.000 m (centered)
- Z: 0.030 m (LiDAR is 30mm above CG)

These values must be entered in QGroundControl before the LiDAR AGL height
fusion will be accurate.

### MPC_THR_HOVER

The throttle percentage at which the drone hovers with no vertical velocity.
This cannot be known in advance — it depends on battery charge, all-up weight,
and motor wear. Measure after first stable hover:

1. Achieve a stable hover in Position mode.
2. Read the actual throttle percentage from QGroundControl telemetry.
3. Set `MPC_THR_HOVER` to this value.

Accurate hover throttle improves altitude hold performance and reduces the
chance of unintended climbs or descents on mode transitions.

---

## 12. Failure Modes and Diagnostics

### SLAM pose stops arriving

Symptom: `/mavros/vision_pose/pose` goes silent. EKF2 falls back to GPS and
barometer. Drone may drift in Position mode.

Diagnosis:
```bash
ros2 topic hz /aft_mapped_to_init
# If this also stops, Point-LIO has crashed — check journalctl
ros2 topic hz /mavros/vision_pose/pose
# If aft_mapped publishes but this does not, SLAM bridge has crashed
sudo journalctl -u drone-watchdog -f
# watchdog logs all subprocess exits with exit code
```

Recovery: Watchdog detects the crash and logs it. If running in bench_scan mode,
restart manually. In production flight, land immediately.

### EKF2 not fusing vision pose

Symptom: Drone arms and flies but `/mavros/local_position/pose` does not change
with drone movement in the expected way. QGroundControl EKF2 Innovations panel
shows large vision residuals.

Diagnosis: Verify in QGroundControl MAVLink Inspector that
`VISION_POSITION_ESTIMATE` messages are arriving. If not, MAVROS is not
receiving the vision pose topic — check QoS mismatch on `/mavros/vision_pose/pose`
(must be RELIABLE).

Also verify `EKF2_EV_CTRL = 15` is saved and active. Parameters require a
reboot to take effect in some PX4 versions.

### GPS not fusing

Symptom: Drone arms (COM_ARM_WO_GPS = 1 allows this) but GPS innovations are
large in Foxglove. SLAM drifts without GPS correction.

Diagnosis:
```bash
ros2 topic echo /mavros/state --once
# Check: armed, mode

ros2 topic echo /mavros/global_position/global --once
# Should show lat/lon — if empty, GPS has no fix
```

Outdoor requirement: minimum 10 satellites, HDOP below 1.2. Current UPRM
campus geometry gives 4 satellites indoors. Use an open field with 360-degree
sky view for GPS-dependent flights.

### EKF2_EV_DELAY mismatch

Symptom: Position hold oscillates in short-period cycles even in calm air.
Drone appears to overshoot corrections. EKF2 Innovations in Foxglove show
alternating positive and negative residuals at a regular frequency.

Diagnosis: The delay compensation is wrong — EKF2 is applying corrections
at the wrong time. Measure actual delay as described in section 11 and update
the parameter.

### AGL rangefinder inconsistent

Symptom: Altitude jumps or oscillates near the ground. QGroundControl shows
sudden height changes that do not correspond to actual drone motion.

Diagnosis: The LiDAR AGL estimate may be returning values from non-ground
objects (walls, vegetation) below the drone. `EKF2_RNG_CTRL = 1` (conditional
mode) should reject these, but may need increased rejection threshold.

Also verify `EKF2_RNG_POS_X/Y/Z` are set to the measured LiDAR offset. If
these are zero and the LiDAR is not at the CG, the height estimate will be
incorrect during banked turns.
