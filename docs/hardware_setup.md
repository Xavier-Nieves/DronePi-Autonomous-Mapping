# Hardware Setup and Minimal Requirements

**Document:** 1 of 6
**Repo path:** `docs/hardware_setup.md`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [Minimum Hardware Required](#1-minimum-hardware-required)
2. [Full Build — Component List](#2-full-build--component-list)
3. [Power System](#3-power-system)
4. [Wiring and Connections](#4-wiring-and-connections)
5. [Physical Assembly Notes](#5-physical-assembly-notes)
6. [Operating System and Initial Software](#6-operating-system-and-initial-software)
7. [Device Identifiers and udev Rules](#7-device-identifiers-and-udev-rules)
8. [Hardware Verification Checklist](#8-hardware-verification-checklist)
9. [Known Hardware Issues and Resolutions](#9-known-hardware-issues-and-resolutions)

---

## 1. Minimum Hardware Required

This is the smallest set of components needed to run the LiDAR mapping pipeline
and produce a mesh. GPS, camera, and AI accelerator are not required at this tier.

| Component | Minimum Spec | Notes |
|---|---|---|
| Flight computer | Raspberry Pi 5, 8 GB RAM | 16 GB recommended for full stack under load |
| Flight controller | Pixhawk 6X with PX4 firmware | Any MAVLink-compatible FC works with MAVROS |
| LiDAR | Unitree 4D L1 | Required — Point-LIO is tuned for this sensor |
| Storage | 256 GB USB 3.0 SSD | Must support USB Mass Storage mode — see SSD notes |
| Power supply | 5V / 5A regulated (25W) | Absolute minimum — see power budget in section 3 |
| Wi-Fi | Any USB Wi-Fi adapter (hostapd-compatible) | TP-Link AC600 RTL8821AU confirmed working |

To run autonomous OFFBOARD flight, add:

| Component | Minimum Spec | Notes |
|---|---|---|
| GPS module | M9N or equivalent | Requires >= 10 satellites and HDOP < 1.2 to arm |
| RC transmitter and receiver | Any MAVLink-compatible set | Required for mode switching and safety override |
| Drone frame | Tarot 810 hexacopter or equivalent | Must support Pixhawk 6X mounting |
| ESCs and motors | Matched to frame and battery | PX4 ESC calibration required before first flight |

---

## 2. Full Build — Component List

This is the complete hardware configuration used in the DronePi capstone build.

| Component | Model | Connection | Status |
|---|---|---|---|
| Flight computer | Raspberry Pi 5, 16 GB, 3.0 GHz OC | — | Active |
| Flight controller | Pixhawk 6X, PX4 firmware | USB `/dev/ttyACM0` at 57600 baud | Active |
| LiDAR | Unitree 4D L1, 360 degrees | USB `/dev/ttyUSB0` at 2,000,000 baud | Active |
| GPS | M9N | Internal to Pixhawk | Active — requires outdoor sky view |
| Camera | Raspberry Pi HQ IMX477 | CSI via Arducam CSI-to-HDMI extension | Pending reinstall |
| Lens | Tamron 4–12mm C-mount | C-CS adapter removed, infinity focus set | Ready |
| AI accelerator | Hailo-8 HAT+, 26 TOPS | PCIe Gen 3 | Present — not yet integrated |
| Storage | SanDisk Extreme 1 TB SSD | USB 3.0 at `/mnt/ssd` | Active — USB 2.0 mode enforced |
| Wi-Fi | TP-Link AC600, RTL8821AU driver | USB, `wlan0` | Active |
| Frame | Tarot 810 hexacopter | — | Active |
| RGB LED indicator | 3-colour PCB (Red, Yellow, Green) | Raspberry Pi GPIO (BCM) | Pending wiring |

---

## 3. Power System

### 3.1 Power Budget

All companion computer electronics are powered from a regulated 5V rail derived
from the main battery via a dedicated buck converter.

| Consumer | Typical (W) | Peak (W) |
|---|---|---|
| Raspberry Pi 5 | 7 | 9 |
| Hailo-8 AI HAT+ | 2.5 | 3 |
| Unitree 4D L1 LiDAR | 6 | 8 |
| SanDisk SSD (USB burst) | 2 | 4.5 |
| IMX477 camera + Wi-Fi hotspot | 1.5 | 2 |
| **Total** | **19** | **24.5** |

A 5V / 10A synchronous buck converter (50W rated) is required to provide adequate
headroom. The current undersized supply leaves approximately 0.5W at peak load,
which has been the confirmed root cause of past SSD instability.

### 3.2 Output Capacitor Plan

Two capacitor stages are required between the buck converter output and the Pi.

| Component | Value | Placement | Purpose |
|---|---|---|---|
| Aluminium electrolytic | 4700 uF / 16V | At buck output terminals | Bulk energy reservoir for slow transients |
| Ceramic X7R | 2x 100 uF / 10V | Within 20mm of RPi 5 USB-C input | Fast spike suppression for microsecond-scale bursts |

The 300mm cable between the buck output and the Pi introduces approximately
150–300 nH of inductance. The ceramic capacitors must be placed at the Pi end
of this cable. Placing them at the buck output end defeats their purpose entirely.

Upgrade the electrolytic to 16V-rated parts on the next order. The current 10V
parts are functional on a 5V rail but have higher ESR and shorter service life.

### 3.3 SSD Stability Fix

The SanDisk Extreme SSD triggers USB Attached SCSI (UAS) protocol by default,
which causes disconnect events under power instability. Apply the following quirk
to force USB Mass Storage mode, which is more tolerant of voltage fluctuation:

Add to `/boot/firmware/cmdline.txt` (single line, no newline):

```
usb-storage.quirks=0781:55dd:u
```

Reboot and confirm with:

```bash
sudo dmesg | grep -i "usb\|sda" | head -30
# Should show: "usb-storage: device found at ..." with no UAS driver binding
```

Verify filesystem integrity after any power event:

```bash
sudo fsck -y /dev/sda1
# Expected: clean, N/61022208 files
```

---

## 4. Wiring and Connections

### 4.1 Serial Connections

| Device | Port | Baud Rate | udev Symlink |
|---|---|---|---|
| Pixhawk 6X | `/dev/ttyACM0` | 57600 | `/dev/ttyPixhawk` |
| Unitree L1 LiDAR | `/dev/ttyUSB0` | 2,000,000 | `/dev/ttyUSB0` |

The baud rate for the Unitree L1 is hardcoded at 2,000,000 inside the SDK.
It is not a configurable parameter.

### 4.2 Pixhawk 6X to Raspberry Pi

Connect Pixhawk 6X TELEM2 port to Pi USB via FTDI or direct USB cable.
MAVROS communicates over this link at 57600 baud.

Confirm connection after boot:

```bash
ros2 topic echo /mavros/state --once
# Expected: connected: True
```

### 4.3 LiDAR to Raspberry Pi

Connect Unitree L1 USB cable directly to a Pi USB 3.0 port. The CP210x USB-to-UART
driver loads automatically on Ubuntu 24.04.

Confirm after connection:

```bash
sudo dmesg | grep cp210x
# Expected: cp210x converter now attached to ttyUSB0

sudo stty -F /dev/ttyUSB0 raw
sudo timeout 3 cat /dev/ttyUSB0 | xxd | head -5
# Expected: packets starting with fd d1 (0xFDD1 sync header)
```

### 4.4 SSD Mount

The SSD mounts automatically at boot via `/etc/fstab`. Verify with:

```bash
lsblk
# Expected: sda1 mounted at /mnt/ssd

df -h /mnt/ssd
# Expected: ~862 GB free on a 1TB drive
```

### 4.5 RGB LED (Pending Wiring)

The RGB LED PCB connects directly to Raspberry Pi GPIO pins using BCM numbering.
Pin assignments are to be confirmed and recorded in `watchdog_core/led_controller.py`
before enabling. Use 220–330 ohm inline resistors on each LED signal pin.
Confirm common anode vs common cathode on the PCB before connecting.

Suggested GPIO pins (BCM): Red = 17, Yellow = 27, Green = 22.

---

## 5. Physical Assembly Notes

### 5.1 Sensor Plate

The Tarot 810 frame uses a rigid sensor plate below the main body for mounting
companion computer hardware. All electronics are secured to this plate.

Cable management is critical for flight stability. Loose cables near propellers
or vibration-sensitive connectors will cause intermittent failures in flight.

### 5.2 LiDAR Placement

The Unitree L1 LiDAR must have an unobstructed 360-degree horizontal field of view.
Mount it at the highest point on the sensor plate, clear of landing legs and props.

The physical offset of the LiDAR from the drone center of gravity must be measured
and recorded. These values are required for:

- `EKF2_RNG_POS_X`, `EKF2_RNG_POS_Y`, `EKF2_RNG_POS_Z` in QGroundControl
- Extrinsic calibration between LiDAR and camera frames

Current measured offset from center of gravity:

```
Translation: [-0.070, 0.000, 0.030] meters (X, Y, Z)
Rotation: zero (LiDAR aligned with drone frame)
```

### 5.3 Camera Placement

The IMX477 camera is mounted on the sensor plate below the LiDAR. A 1-meter
Arducam CSI-to-HDMI extension kit is used to route the CSI signal from the
camera to the Pi while isolating the camera PCB from vibration.

The camera must be remounted after the extension kit arrives. After remount:

1. Run `libcamera-hello --list-cameras` to confirm detection
2. Run `rpicam-still -o test.jpg` to confirm capture
3. Verify `dmesg` shows no `failed to read chip id 477, error -5`

### 5.4 Hailo-8 HAT

The Hailo-8 AI HAT connects via PCIe Gen 3 on the Pi 5 HAT+ connector. It is
present and powered but has no active pipeline integration in the current build.
It draws 2.5W typical and is safe to leave installed.

---

## 6. Operating System and Initial Software

### 6.1 Operating System

- Ubuntu 24.04 LTS (arm64)
- ROS 2 Jazzy (native install, not Docker)
- Hostname: `dronepi-desktop`
- Static IP on hotspot interface: `10.42.0.1`

### 6.2 Python Environment

All pipeline Python dependencies are managed in a Conda environment named `dronepi`,
created via Miniforge. Activate before running any pipeline script:

```bash
conda activate dronepi
```

Key packages in the `dronepi` environment:

```bash
open3d==0.19.0       # conda-forge — MLS smoothing, Ball Pivoting mesh
pdal==3.5.3          # conda-forge — SMRF ground classification
pymeshlab            # ARM64 wheel
trimesh              # ARM64 wheel
rosbags              # ARM64 wheel — MCAP bag reading
scipy
numpy
pyyaml
psutil
empy==3.3.4          # pinned — ROS 2 Jazzy requires legacy em module API
catkin_pkg
colcon-common-extensions
colcon-ros
```

Note: `empy` must be pinned to `3.3.4`. Version 4.x removed the `em` module API
that ROS 2 Jazzy build tools require. Installing a newer version will break colcon.

### 6.3 ROS 2 Workspace

The ROS 2 workspace is at:

```
~/unitree_lidar_project/RPI5/ros2_ws/
```

Source in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
```

To rebuild a specific package:

```bash
cd ~/unitree_lidar_project/RPI5/ros2_ws
colcon build --symlink-install \
  --packages-select unitree_lidar_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 7. Device Identifiers and udev Rules

A udev rule creates a persistent symlink for the Pixhawk to prevent device path
changes across reboots:

```
/etc/udev/rules.d/99-pixhawk.rules
```

Contents:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="XXXX", SYMLINK+="ttyPixhawk"
```

Replace vendor and product IDs with those from `udevadm info /dev/ttyACM0`.

After installing the rule:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/ttyPixhawk
# Expected: symlink to /dev/ttyACM0
```

---

## 8. Hardware Verification Checklist

Run `preflight_check.sh` before every flight session. It covers 17 sections and
63 individual checks. Below is the manual verification sequence for initial setup.

```
[ ] Pi boots to Ubuntu 24.04, SSH accessible at 10.42.0.1
[ ] sudo dmesg shows cp210x attached to ttyUSB0 (LiDAR)
[ ] sudo dmesg shows ttyACM0 present (Pixhawk)
[ ] /dev/ttyPixhawk symlink exists
[ ] /mnt/ssd mounted, df shows expected free space
[ ] fsck /dev/sda1 returns clean
[ ] vcgencmd get_throttled returns 0x0
[ ] conda activate dronepi succeeds
[ ] ros2 topic echo /mavros/state --once returns connected: True
[ ] ros2 topic hz /cloud_registered shows ~9.7 Hz
[ ] ros2 topic hz /unilidar/cloud shows ~9.8 Hz
[ ] All systemd services active (mavros, drone-watchdog, drone-mesh-server,
    foxglove-bridge, dronepi-hotspot)
[ ] http://10.42.0.1:8080/meshview.html loads in browser
[ ] bench_test.py returns 7/8 PASS (GPS test skipped indoors)
```

---

## 9. Known Hardware Issues and Resolutions

### 9.1 LiDAR Package Never Built — Colcon Discovery Failure

**Symptom:** `[WARNING] Serial port timeout!` immediately on node start.
Point-LIO receives no data. `/unilidar/cloud` never publishes.

**Root cause:** The `unitree_lidar_ros2` `package.xml` was nested two levels
deep inside the SDK directory structure. Colcon requires `package.xml` at direct
child depth of `src/`. The package was never discovered, never compiled, and the
binary never existed in `install/`. The serial timeout was a red herring.

**Resolution:** Create a symlink at the correct discovery depth:

```bash
ln -s \
  ~/unitree_lidar_project/RPI5/ros2_ws/src/unilidar_sdk/unitree_lidar_ros2/src/unitree_lidar_ros2 \
  ~/unitree_lidar_project/RPI5/ros2_ws/src/unitree_lidar_ros2
```

Rebuild:

```bash
cd ~/unitree_lidar_project/RPI5/ros2_ws
colcon build --symlink-install --packages-select unitree_lidar_ros2
source install/setup.bash
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node
```

### 9.2 Conda Environment Poisoning Colcon Builds

**Symptom:** `ament_cmake` fails during CMake configuration. Error references
missing `catkin_pkg` or `empy` module.

**Root cause:** The `dronepi` Conda environment was active during `colcon build`.
CMake resolved to the Conda Python, which lacked the ROS 2 build dependencies.

**Resolution:** Install ROS 2 build dependencies into the Conda environment:

```bash
conda activate dronepi
pip install catkin_pkg empy==3.3.4 lark colcon-common-extensions colcon-ros \
  rosdep osrf-pycommon
```

`empy` must be pinned to `3.3.4`. Do not upgrade it.

### 9.3 SSD EXT4 Corruption

**Symptom:** UAS disconnect events in `dmesg`. EXT4 errors on `/dev/sda1`.
Rosbag files may be truncated or unreadable.

**Root cause:** USB Attached SCSI protocol combined with insufficient PSU
headroom causes disconnect events during SSD burst writes. Each disconnect
can leave open file handles, corrupting the filesystem.

**Resolution:**
1. Apply UAS quirk fix (see section 3.3)
2. Run `sudo fsck -y /dev/sda1` before trusting any existing flight data
3. Install 5V/10A buck converter to eliminate root cause

### 9.4 Camera I/O Error on Boot

**Symptom:** `dmesg` shows `failed to read chip id 477, error -5`.
`libcamera-hello --list-cameras` returns no cameras.

**Root cause:** Physical connection issue at the CSI connector or extension
cable. No deeper hardware fault confirmed.

**Resolution:** Physical reconnect of the CSI cable. The Arducam CSI-to-HDMI
extension kit must be fully seated at both ends. After reconnect, verify with:

```bash
libcamera-hello --list-cameras
rpicam-still -o /tmp/test.jpg && echo "Capture OK"
```

### 9.5 mmc0 / mmc1 Voltage Warnings at Boot

**Symptom:** Boot log shows `mmc0: cannot verify signal voltage switch` or
similar warnings.

**Root cause:** Likely cosmetic on Ubuntu 24.04 with Pi 5. These warnings
originate from the SD card or eMMC voltage negotiation sequence and do not
indicate a problem with the 5V companion computer rail.

**Confirmation:** Run under full postflight processing load and verify:

```bash
vcgencmd get_throttled
# Must return: throttled=0x0
# Any non-zero value indicates real undervoltage — investigate PSU immediately
```
