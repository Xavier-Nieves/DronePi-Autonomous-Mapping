# System Architecture

**Document:** 6 of 6
**Repo path:** `docs/architecture.md`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Architecture Diagram

The diagram below shows the full DronePi system architecture across seven
layers, from raw hardware inputs to browser-rendered 3D mesh output.

The source SVG file is at `docs/dronepi_architecture.svg`. Embed it in any
document or render it in a browser directly.

![DronePi System Architecture](dronepi_architecture.svg)

---

## Layer Descriptions

### Hardware Layer

Four physical sensors feed the system. The Pixhawk 6X is the flight controller
and handles all stabilization, arming, and mode management. The Unitree 4D L1
LiDAR provides 360-degree point cloud data at 9.8 Hz over USB serial at
2,000,000 baud. The M9N GPS provides global position via the Pixhawk internal
port. The IMX477 camera is pending reinstall on the sensor plate.

### ROS 2 + MAVROS Bridge

Four ROS 2 nodes translate hardware data into topics used by the flight stack.
MAVROS bridges MAVLink from the Pixhawk to ROS 2 topics. Point-LIO SLAM
processes the LiDAR stream and produces a registered point cloud and pose
estimate. The SLAM bridge converts the Point-LIO pose from ENU to the format
EKF2 expects and publishes it at ~90 Hz. The collision monitor processes the
point cloud to produce a three-zone obstacle distance LaserScan and an AGL
height estimate, both fed back to PX4 via MAVROS.

### Flight Stack

Three components coordinate via a lock file. `main.py` is the top-level state
machine managing mission mode and lifecycle. `drone_watchdog.py` supervises the
three scan session subprocesses — Point-LIO, SLAM bridge, and bag recorder —
starting them on arm with OFFBOARD and stopping them cleanly on disarm.
`preflight_check.sh` is the arming gate covering 17 sections and 63 checks.

### Post-Processing Pipeline

Triggered automatically on disarm. `run_postflight.py` finds the latest bag
and calls `postprocess_mesh.py`, a six-stage orchestrator. `flight_logger.py`
records a one-line entry to the persistent flight history log.

### mesh_tools — Modular Pipeline Modules

Six independent classes each handle one transformation step:
BagReader (MCAP to NumPy), MLSSmoother (noise reduction), GroundClassifier
(terrain split), DTMBuilder (Delaunay 2.5D terrain mesh), DSMBuilder
(Ball Pivoting surface mesh), Publisher (PLY files and JSON manifests).

### Outputs and HTTP Serving

`Publisher` writes PLY files and JSON manifests to `/mnt/ssd/maps/`.
`serve.py` serves them over HTTP on port 8080 with a `/api/flights` endpoint.
The Foxglove bridge exposes all ROS 2 topics as a WebSocket on port 8765.

### 3D Viewers

`meshview.html` is the Pi-hosted browser viewer that polls `latest.json` every
10 seconds and auto-loads new meshes. `local_test.html` is the offline laptop
viewer with drag-and-drop flight folder support. Foxglove Studio connects to
the WebSocket bridge for live telemetry during flight.

---

## Systemd Service Dependency Graph

```
network.target
  |
  +-- mavros.service
  |     FCU bridge, always running
  |
  +-- drone-watchdog.service
  |     Flight stack supervisor
  |     Requires: mavros.service
  |
  +-- dronepi-main.service
        Top-level orchestrator
        Requires: mavros.service + drone-watchdog.service
        Inactive until first arm

mnt-ssd.mount
  |
  +-- dronepi-hotspot.service
  |     Wi-Fi AP — oneshot, exited state is normal
  |
  +-- drone-mesh-server.service
        HTTP server, port 8080
        Requires: mnt-ssd.mount + dronepi-hotspot.service
        ExecStartPre: /bin/mountpoint -q /mnt/ssd

foxglove-bridge.service
  WebSocket, port 8765
  No hard dependencies
```

---

## Key Design Principles

Every component in this system was built to one consistent set of rules.
These principles are documented here so future contributors understand why
the code is structured the way it is.

### Modular classes with a single orchestrator

Every processing stage is an independent class with one public method and
typed inputs and outputs. A dedicated orchestrator script calls the classes
in sequence. No class imports another class from the same pipeline — they
communicate only through the data types defined in the interface.

This means any stage can be tested in isolation, replaced with a better
algorithm, or skipped entirely without touching anything else.

### Hardware-optional modules use runtime detection

Modules that depend on optional hardware (Hailo-8 AI accelerator, external
RGB LED, camera) implement an `is_available()` classmethod that checks for
the hardware or dependency at runtime. If the hardware is absent, the module
logs a warning and the pipeline continues with a fallback or skips the stage.

The pipeline never fails hard on missing optional hardware.

### preflight_check.sh is the single arming gate

All health checks live in `preflight_check.sh` as numbered sections. No
arming-related checks are scattered in startup scripts or service files.
When a new health check is needed (SSD smart status, voltage throttle check,
camera calibration presence), it is added as a new section to this script,
not patched into service start conditions.

### Systemd services declare explicit dependencies

Every service that needs a resource (the SSD mount, another service, the
network) declares it explicitly with `Requires=` and `After=`. No service
relies on timing or boot order assumptions. `ExecStartPre` guards are used
for runtime checks that systemd dependency declarations cannot express
(such as verifying a mount point is actually a live mountpoint, not just
that the mount unit succeeded).

### Lock file coordination instead of IPC

`main.py` and `drone_watchdog.py` share state through a JSON lock file at
`/tmp/dronepi_mission.lock`. This is intentional. A lock file is readable
by any process, survives service restarts, and can be inspected manually
with `cat`. It does not require both processes to be alive simultaneously
to communicate intent.

---

## Network Reference

| Interface | Address | Purpose |
|---|---|---|
| Hotspot (field) | `10.42.0.1` | Primary field access — no router needed |
| Lab router | `192.168.2.2` | Lab development access |
| Mesh viewer | `http://10.42.0.1:8080/meshview.html` | 3D mesh browser |
| API | `http://10.42.0.1:8080/api/flights` | Flight index JSON |
| Foxglove | `ws://10.42.0.1:8765` | Live ROS 2 telemetry |
| SSH | `ssh dronepi@10.42.0.1` | Remote terminal |

---

## Repository Quick Reference

```
unitree_lidar_project/
|
|-- main.py                        Top-level orchestrator
|-- preflight_check.sh             Arming gate — 17 sections, 63 checks
|-- logs/flight_history.log        Persistent flight log
|
|-- flight/
|   |-- drone_watchdog.py          Flight stack supervisor
|   |-- _slam_bridge.py            Point-LIO → MAVROS EKF2 bridge
|   |-- collision_monitor.py       3-zone obstacle + AGL estimator
|   |-- gap_detector.py            LiDAR coverage gap detector
|   +-- watchdog_core/             Modular watchdog subpackage
|       |-- buzzer.py              Audio feedback tunes
|       |-- mavros_reader.py       ROS 2 MAVROS interface
|       |-- flight_stack.py        Subprocess manager
|       |-- postflight.py          Monitored post-flight trigger
|       +-- led_controller.py      RGB LED state machine (pending wiring)
|
|-- tests/
|   |-- test_offboard_flight.py    OFFBOARD hover test
|   |-- test_offboard_gps.py       GPS waypoint mission
|   |-- test_manual_scan.py        Manual RC + LiDAR scan
|   |-- test_slam_bridge_flight.py SLAM bridge validation
|   |-- collision_zone_test.py     Zone characterization
|   +-- test_mesh_algorithms.py    Algorithm comparison
|
|-- utils/
|   |-- postprocess_mesh.py        6-stage orchestrator
|   |-- run_postflight.py          Post-flight trigger
|   |-- flight_logger.py           Flight history log
|   |-- bench_test.py              MAVROS 8-point bench test
|   +-- mesh_tools/
|       |-- bag_reader.py
|       |-- mls_smoother.py
|       |-- ground_classifier.py
|       |-- dtm_builder.py
|       |-- dsm_builder.py
|       |-- mesh_merger.py
|       +-- publisher.py
|
|-- config/
|   |-- px4_config.yaml
|   |-- px4_pluginlists.yaml
|   +-- camera_calibration.yaml
|
|-- rpi_server/
|   |-- serve.py
|   |-- meshview.html
|   +-- local_test.html
|
|-- RPI5/ros2_ws/src/
|   |-- point_lio_ros2/
|   |-- unitree_lidar_ros2/        (symlink at correct colcon depth)
|   +-- unilidar_sdk/
|
|-- docker/
|   |-- lidar/Dockerfile
|   |-- mavros/Dockerfile
|   +-- docker-compose.yml
|
+-- docs/
    |-- architecture.md            This file
    |-- hardware_setup.md          Hardware setup and requirements
    |-- pipeline_overview.md       Data flow and layer reference
    |-- ekf_slam_fusion.md         EKF2 and sensor fusion reference
    |-- post_processing.md         Mesh pipeline reference
    +-- flight_procedure.md        Flight operations manual
```
