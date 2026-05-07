# Flight Procedure User Manual

**Document:** 5 of 6
**Repo path:** `docs/flight_procedure.md`
**Last updated:** March 2026
**Project:** DronePi — Autonomous LiDAR Mapping Drone — UPRM Capstone

---

## Table of Contents

1. [Before You Fly — Requirements](#1-before-you-fly--requirements)
2. [Pre-Flight Checklist](#2-pre-flight-checklist)
3. [Pixhawk 6X LED Reference](#3-pixhawk-6x-led-reference)
4. [DronePi RGB LED Reference](#4-dronepi-rgb-led-reference)
5. [Buzzer Audio Cues](#5-buzzer-audio-cues)
6. [RC Channel Assignments](#6-rc-channel-assignments)
7. [Flight Modes Reference](#7-flight-modes-reference)
8. [Standard Flight Procedure](#8-standard-flight-procedure)
9. [Post-Flight Procedure](#9-post-flight-procedure)
10. [Emergency Procedures](#10-emergency-procedures)

---

## 1. Before You Fly — Requirements

Never proceed to the flight checklist unless all of the following are satisfied.

### Site requirements

- Open outdoor field with 360-degree unobstructed sky view
- No structures within 50 metres of the flight area
- GPS: at least 10 satellites visible, HDOP below 1.2, VDOP below 1.5
  (confirm in QGroundControl before arming)
- Wind below 5 m/s
- No rain or moisture

### System requirements

- All five boot services confirmed running
- SSD mounted and healthy — `vcgencmd get_throttled` returns `0x0`
- `preflight_check.sh` completed with 0 FAIL
- Battery fully charged, voltage confirmed in QGroundControl
- QGroundControl connected on ground station laptop
- Foxglove Studio connected on ground station laptop

### Legal and safety

- Maintain visual line of sight at all times
- Brief all personnel on the flight area before arming
- Designate a safety observer separate from the pilot

---

## 2. Pre-Flight Checklist

Run `preflight_check.sh` first. This covers 17 automated sections. The manual
steps below supplement the script and must be completed in order.

### Hardware checks

```
[ ] Frame — inspect for cracks, loose screws, prop damage
[ ] Props — all four/six props tight, correct rotation direction
[ ] Motors — spin freely by hand, no grinding or resistance
[ ] Battery — fully charged, secured, balance connector seated
[ ] LiDAR — USB cable secure, no cable near props
[ ] Pixhawk — USB connected to Pi, indicator LED active
[ ] SSD — USB cable secure
[ ] RC transmitter — battery charged, trims centered
```

### Software checks (run on Pi via SSH)

```bash
# Confirm all services running
systemctl status mavros drone-watchdog drone-mesh-server foxglove-bridge dronepi-hotspot

# Confirm LiDAR publishing
ros2 topic hz /cloud_registered
# Expected: ~9.7 Hz

# Confirm MAVROS connected
ros2 topic echo /mavros/state --once
# Expected: connected: True

# Confirm SSD healthy
df -h /mnt/ssd
vcgencmd get_throttled
# Expected: throttled=0x0

# Run full preflight check
bash ~/unitree_lidar_project/preflight_check.sh
# Expected: N PASS, 0 FAIL
```

### QGroundControl checks

```
[ ] Vehicle appears on map at correct GPS location
[ ] Satellite count >= 10, HDOP < 1.2 (shown in top bar)
[ ] Battery voltage nominal (check with fresh charge — note threshold)
[ ] Flight mode shows STABILIZE or POSITION (not MANUAL)
[ ] No active failsafes or warnings in the status bar
[ ] EKF2 status: green (no red warnings in vehicle health)
```

---

## 3. Pixhawk 6X LED Reference

The Pixhawk 6X has an RGB LED that reflects PX4 flight controller state. These
meanings are set by PX4 firmware and cannot be changed by the companion computer.

### Solid colors

| LED color | Meaning | Action required |
|---|---|---|
| Solid white | Initializing — sensors calibrating | Wait, do not arm |
| Solid yellow | Calibration required or sensor warning | Check QGroundControl warnings |
| Solid red | Error — will not arm | Check QGroundControl error messages |
| Solid blue | Disarmed, POSITION mode, GPS lock | Ready to arm |
| Solid green | Armed and flying | Normal flight state |

### Blinking patterns

| Pattern | Meaning | Action required |
|---|---|---|
| Slow blue blink | Disarmed, waiting for GPS | Wait for satellite count |
| Fast blue blink | Disarmed, GPS acquired | Ready to arm |
| Slow green blink | Armed, Position mode, flying | Normal |
| Fast green blink | Armed, Mission mode | Normal |
| Yellow/orange blink | Low battery warning | Begin return to land |
| Red blink | Critical error or failsafe active | Land immediately |
| Red/blue alternating | Firmware update mode | Do not fly |
| Purple blink | OFFBOARD mode active | Normal — autonomous flight |

### During OFFBOARD autonomous flight

A purple blinking LED indicates PX4 is accepting OFFBOARD setpoints from the
companion computer. This is the expected state during autonomous scan missions.
If the LED transitions away from purple mid-flight, OFFBOARD mode was lost —
the drone will hold position or enter a failsafe depending on `COM_OBL_ACT`.

---

## 4. DronePi RGB LED Reference

The DronePi external RGB LED (3-colour PCB wired to Raspberry Pi GPIO) reflects
companion computer software state, independent of PX4 flight modes. This LED
is pending physical GPIO wiring — see `watchdog_core/led_controller.py`.

Unlike the Pixhawk LED, this LED shows what the Pi software is doing, not what
the flight controller is doing. Both LEDs should be checked together.

| LED state | Color | Pattern | Meaning |
|---|---|---|---|
| BOOTING | Red | Slow blink (1s) | Watchdog started, MAVROS not yet connected |
| READY | Green | Solid | MAVROS connected, no active session |
| STACK ACTIVE | Green | Slow blink (1s) | Flight stack running — scan session in progress |
| POSTPROCESSING | Yellow | Fast blink (0.3s) | Mesh pipeline running after landing |
| POSTPROCESSING DONE | Yellow | Solid (5s then clears) | Mesh complete — viewer will auto-load |
| CRITICAL ERROR | Red + Yellow + Green | All solid | Fault detected — do not fly until resolved |

### CRITICAL ERROR triggers

The LED latches to CRITICAL ERROR (all three solid) on any of the following.
It does not clear automatically. Restart the watchdog service after resolving
the underlying issue.

- SSD unmounted or rosbag directory inaccessible
- Point-LIO process crashed mid-scan
- Bag recorder exited unexpectedly during a session
- SLAM bridge exited unexpectedly
- Post-flight script exited with non-zero return code
- MAVROS FCU connection lost after initial connect
- Lock file unreadable or corrupt

```bash
# Check the cause before restarting
sudo journalctl -u drone-watchdog -n 50

# Restart after resolving the issue
sudo systemctl restart drone-watchdog
```

---

## 5. Buzzer Audio Cues

The Pixhawk 6X buzzer provides audio feedback for flight stack events. Tones
are published by `watchdog_core/buzzer.py` via the MAVROS `/mavros/play_tune`
topic using QBASIC Format 1 tune strings.

A tone sounds when the described action occurs. No user action is needed
unless the tone indicates an error.

| Sound | Description | Trigger |
|---|---|---|
| Rising scale (C-E-G-C) | Flight stack starting | RC toggle pressed to start scan, or lock mode activates |
| Falling scale (C-G-E-C) | Flight stack stopping | Drone disarmed, scan session ended |
| Single short beep | Acknowledged | RC button press confirmed, or FCU connection established |
| Staccato triple beep | Error | Post-flight pipeline failed with non-zero exit code |
| Slow mid pulse (every 5s) | Processing | Post-flight mesh pipeline running |
| Two-note up (C-E) | Done | Post-flight pipeline completed successfully |

If no tones are heard during flight stack events, verify the buzzer is connected
to the Pixhawk buzzer port and that the MAVROS `play_tune` plugin is active in
`px4_pluginlists.yaml`.

---

## 6. RC Channel Assignments

RC channel assignments are set in QGroundControl under Radio Setup.
The table below documents the current assignment. Update this table when
assignments change.

| Channel | Control | Notes |
|---|---|---|
| CH1 | Roll | Standard — right stick horizontal |
| CH2 | Pitch | Standard — right stick vertical |
| CH3 | Throttle | Standard — left stick vertical |
| CH4 | Yaw | Standard — left stick horizontal |
| CH5 | Flight mode switch | 3-position: STABILIZE / ALTITUDE / POSITION |
| CH6 | Scan toggle | 2-position switch — starts and stops scan session via watchdog |
| CH7–CH8 | Not assigned | Reserve for future use |

Note: CH6 is the primary operator interface for scan control. A rising edge
on CH6 (switch flipped to active position) triggers the watchdog to start the
flight stack (Point-LIO + SLAM bridge + bag recorder). A falling edge stops it
and triggers post-flight processing.

Full channel map including switch positions should be documented here once
RC transmitter mapping is finalized.

---

## 7. Flight Modes Reference

| Mode | PX4 name | Description | When to use |
|---|---|---|---|
| STABILIZE | MANUAL / STABILIZE | Attitude stabilized. Throttle is direct. No position hold. | Fallback only — requires pilot skill |
| ALTITUDE | ALTCTL | Altitude hold. No horizontal position hold. | Transition mode |
| POSITION | POSCTL | Full position hold using GPS + EKF2. Easiest to fly. | Default for all flights |
| OFFBOARD | OFFBOARD | Companion computer sends position setpoints. Drone follows autonomously. | Autonomous scan missions |
| AUTO.LAND | AUTO.LAND | Controlled descent and landing at current position. | Emergency landing |
| AUTO.RTL | AUTO.RTL | Return to launch point and land. | Battery failsafe or manual abort |

### OFFBOARD mode requirements

OFFBOARD mode requires:
1. MAVROS sending setpoints at 20 Hz before switching to OFFBOARD
2. EKF2 local position to be healthy (SLAM or GPS providing position)
3. The drone to be armed

If setpoints stop for more than approximately 500 ms, PX4 exits OFFBOARD mode
and enters the failsafe behavior defined by `COM_OBL_ACT`.

---

## 8. Standard Flight Procedure

### Power-on to ready (approximately 90 seconds)

```
1. Place drone on level ground at the launch point
2. Power on RC transmitter
3. Connect battery
4. Wait ~30s for Pi to boot and all services to start
5. Connect to dronepi-ap Wi-Fi on laptop (password: set during hotspot setup — see setup_scripts/setup_hotspot_service.sh)
6. Open QGroundControl — confirm vehicle appears and FCU connected
7. Open Foxglove Studio — connect to ws://10.42.0.1:8765
8. Wait for DronePi RGB LED to show solid green (READY)
9. Confirm Pixhawk LED shows fast blue blink (GPS acquired) or solid blue
10. Run preflight_check.sh via SSH — confirm 0 FAIL
```

### Arming

```
1. Confirm satellite count >= 10 and HDOP < 1.2 in QGroundControl
2. Set flight mode to POSITION on RC transmitter (CH5)
3. Arm via RC: hold left stick down-right for 2 seconds
4. Pixhawk LED turns solid green
5. DronePi RGB LED continues solid green (READY) — stack not yet active
```

### Starting a scan session

```
1. Confirm armed and hovering stably
2. Flip CH6 scan toggle to active position
3. Buzzer plays rising scale (stack starting)
4. DronePi RGB LED changes to slow green blink (STACK ACTIVE)
5. Confirm via SSH: ros2 topic hz /cloud_registered should remain ~9.7 Hz
6. Begin flight path over target structure
```

### OFFBOARD autonomous flight

```
1. Complete arming procedure above
2. Via SSH on laptop, run:
   python3 tests/test_offboard_flight.py
3. Script waits for EKF2 stability, then arms and switches to OFFBOARD
4. Pixhawk LED shows purple blink (OFFBOARD active)
5. Monitor flight in QGroundControl
6. Script executes: climb → hold → AUTO.LAND → disarm
7. Do not switch modes manually during autonomous sequence unless aborting
```

### Landing

```
1. For manual flight: descend slowly, set throttle to land
2. For OFFBOARD: script handles AUTO.LAND automatically
3. Pixhawk LED changes from green to rapid yellow/orange as it lands
4. On disarm: Pixhawk LED returns to fast blue blink
5. Buzzer plays falling scale (stack stopping)
6. DronePi RGB LED changes to fast yellow blink (post-processing running)
7. Wait for LED to show solid yellow (DONE) — mesh is ready
8. Open http://10.42.0.1:8080/meshview.html to view result
```

---

## 9. Post-Flight Procedure

```
1. Confirm DronePi LED shows solid yellow (POSTPROCESSING DONE)
   or has returned to solid green (READY) after 5s hold
2. Verify mesh loaded in browser: http://10.42.0.1:8080/meshview.html
3. Check flight_history.log for session entry:
   cat ~/unitree_lidar_project/logs/flight_history.log | tail -5
4. Disconnect battery
5. Power off RC transmitter
6. Check SSD health:
   vcgencmd get_throttled   # Must return 0x0
   df -h /mnt/ssd           # Confirm space available
7. Back up rosbag if session is important:
   rsync -av /mnt/ssd/rosbags/scan_YYYYMMDD/ user@laptop:/backup/
```

---

## 10. Emergency Procedures

### LiDAR stops publishing mid-flight

Symptom: DronePi RGB LED transitions to CRITICAL ERROR (all solid).
Buzzer plays staccato triple beep. `/cloud_registered` goes silent.

Immediate action:
1. Switch to POSITION mode if not already
2. Land manually and safely
3. Do not attempt to restart the scan session in flight

After landing:
```bash
sudo journalctl -u drone-watchdog -n 50
# Look for: [FlightStack] Point-LIO exited with code N
ros2 topic hz /unilidar/cloud
# If this is also silent, check USB connection to LiDAR
sudo dmesg | tail -20
# Look for USB disconnect events
```

The MCAP bag file up to the crash point is still valid and can be processed
manually: `python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_YYYYMMDD/`

### MAVROS loses FCU connection mid-flight

Symptom: QGroundControl shows vehicle disconnected. Pixhawk LED behavior
becomes erratic. OFFBOARD mode may exit.

Immediate action:
1. Take manual RC control immediately — switch to STABILIZE if needed
2. Land as quickly as safely possible
3. Do not attempt to re-establish OFFBOARD control

After landing:
```bash
sudo journalctl -u mavros -n 50
# Look for: FCU connection lost
ls -la /dev/ttyPixhawk /dev/ttyACM0
# Confirm udev symlink still exists
sudo systemctl restart mavros
```

Root cause is usually a loose USB cable or power fluctuation. Secure all
cables before next flight.

### SSD unmounts during recording

Symptom: DronePi RGB LED transitions to CRITICAL ERROR. Buzzer plays error tone.
`drone-mesh-server` may also stop responding.

Immediate action:
1. Land immediately — the bag recorder has stopped and data is at risk
2. Do not power cycle until safely on the ground

After landing:
```bash
sudo dmesg | tail -30
# Look for: EXT4-fs error or UAS disconnect
lsblk
# Confirm sda is still present
sudo mount /mnt/ssd
sudo systemctl restart drone-mesh-server
sudo fsck -y /dev/sda1
# Run fsck before trusting any data on the drive
```

Root cause: power instability from undersized PSU. This is the primary reason
the 5V/10A buck converter upgrade is required before extended field operations.

### Point-LIO crashes mid-scan

Symptom: `/cloud_registered` stops publishing. SLAM bridge goes silent.
EKF2 loses vision fusion and falls back to GPS/barometer.

Immediate action:
1. Drone will continue flying on GPS + barometer in POSITION mode
2. If flying OFFBOARD: setpoints continue but without SLAM correction —
   position accuracy will degrade. Land within 60 seconds.
3. Switch to POSITION mode if not already, land manually

After landing:
```bash
sudo journalctl -u drone-watchdog -n 50
# Look for: [FlightStack] Point-LIO exited
# Check exit code — code 139 = segfault, code 1 = assertion failure
```

The most common cause is a LiDAR USB disconnect mid-scan (see LiDAR stops
publishing above). If the USB connection is solid, check available RAM:
`free -h` — Point-LIO can be killed by the OOM killer on very long scans.

### drone-watchdog service dies

Symptom: All flight stack management stops. Scan session does not trigger
post-flight processing. DronePi RGB LED may go dark or freeze in last state.

Immediate action:
1. Drone continues flying on flight controller — no immediate flight risk
2. Scan session is still recording if bag recorder process survived
3. Land normally

After landing:
```bash
sudo systemctl status drone-watchdog
# Check exit code and last log lines
sudo journalctl -u drone-watchdog -n 100

# If bag is still open, close it manually
ros2 bag record --stop   # or SIGINT the process

# Trigger post-flight manually
conda activate dronepi
python3 ~/unitree_lidar_project/unitree_drone_mapper/utils/run_postflight.py --auto

# Restart watchdog for next session
sudo systemctl restart drone-watchdog
```
