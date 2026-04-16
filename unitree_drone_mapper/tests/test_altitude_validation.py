#!/usr/bin/env python3
"""
test_altitude_validation.py — Multi-altitude stationary hover + camera validation test.

Self-contained: manages its own services, Point-LIO, ArducamNode, and
rosbag recorder. One command does everything.

Flight profile
--------------
For each altitude level (default 3m, 6m, 9m, 12m, 15m):

  1. Climb to target altitude
  2. Start a fresh rosbag for this level
  3. Hold for --hold-per-level seconds (default 60s)
  4. Capture --pics-per-level frames (default 5), evenly spaced
  5. Evaluate each frame: focus + LiDAR sync
  6. Stop the rosbag
  7. Climb to next level (or descend and land after the last)

Each level gets its own bag so you can run postprocess_mesh.py on any
single level independently.

What it validates
-----------------
  Focus        — Laplacian variance > SHARPNESS_THRESHOLD (80)
                 Shows whether the IMX477 is sharp at each altitude.
                 At higher altitudes the scene is further away —
                 a fixed-focus lens should stay sharp to infinity,
                 but vibration / motion blur may increase with altitude.

  LiDAR sync   — Camera timestamp gap to LiDAR pose < 45ms
                 Verifies ArducamNode is stamping with ROS clock at all
                 altitudes, not wall clock.

  Coverage FOV — Computed per level: footprint width/height at altitude
                 using the calibrated focal length. Shows how much ground
                 each frame covers at each level.

Output
------
  tests/hover_camera_output/
    alt_03m/
      frame_01.jpg ... frame_05.jpg
    alt_06m/
      frame_01.jpg ... frame_05.jpg
    ...
    alt_15m/
      frame_01.jpg ... frame_05.jpg
    report.txt      ← per-level + per-frame pass/fail + FOV table
    bags/
      hover_alt_03m_<ts>/    ← rosbag2 directory
      hover_alt_06m_<ts>/
      ...

Usage
-----
  # Dry run — starts services, checks sensors, stops before arming
  python3 tests/test_altitude_validation.py --dry-run

  # Default: 3m → 15m in 3m steps, 60s per level, 5 pics per level
  python3 tests/test_altitude_validation.py

  # Custom altitudes and timing
  python3 tests/test_altitude_validation.py --altitudes 3 6 9 12 15 --hold 60 --pics 5

  # Quick test — 2 levels, 20s hold, 3 pics
  python3 tests/test_altitude_validation.py --altitudes 3 6 --hold 20 --pics 3

  # Collision avoidance remains active by default (stops only watchdog)
  python3 tests/test_altitude_validation.py

  # Skip service management if already stopped manually
  python3 tests/test_altitude_validation.py --no-service-stop

Service management
------------------
  Default: stops only drone-watchdog and leaves dronepi-main running so
  collision_monitor.py continues publishing obstacle distances and AGL
  height at all altitudes.

  The test now reuses an already-running Point-LIO instance when
  /aft_mapped_to_init is already active. It only launches its own local
  Point-LIO if no running SLAM publisher is detected. The bench_scan
  mission lock still prevents main.py and drone-watchdog from taking over.

Safety
------
  AUTO.LAND + disarm on any exception or Ctrl+C.
  Current rosbag is stopped cleanly before emergency land.
  Services always restored in finally block.
  RC override kills OFFBOARD immediately.

Prerequisites
-------------
  MAVROS must already be running (mavros.service).
  Run from the unitree_drone_mapper directory or tests/ subdirectory.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time
import threading
from pathlib import Path
from typing import Optional

# ── Project paths ─────────────────────────────────────────────────────────────
_TESTS_DIR    = Path(__file__).resolve().parent
_MAPPER_DIR   = _TESTS_DIR.parent
_ROS_SETUP    = "/opt/ros/jazzy/setup.bash"
_WS_SETUP     = Path.home() / "unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
_ARDUCAM_NODE = _MAPPER_DIR / "flight" / "arducam_node.py"
OUTPUT_DIR    = _TESTS_DIR / "hover_camera_output"
BAG_DIR       = OUTPUT_DIR / "bags"

# ── Mission lock file ─────────────────────────────────────────────────────────
# bench_scan tells both main.py and drone-watchdog to yield entirely.
# Written before arming so main.py does not enter autonomous mode when it
# sees armed=True + OFFBOARD — it would otherwise launch its own Point-LIO
# and bag recorder, conflicting with this script's processes.
MISSION_LOCK  = Path("/tmp/dronepi_mission.lock")

# ── Camera intrinsics for FOV calculation ─────────────────────────────────────
# Read from camera_calibration.yaml at runtime if available, else use these
_CALIB_PATH  = _MAPPER_DIR / "config" / "camera_calibration.yaml"
_DEFAULT_FX  = 4805.613
_DEFAULT_FY  = 4773.894
_DEFAULT_W   = 4056
_DEFAULT_H   = 3040

# ── Thresholds ────────────────────────────────────────────────────────────────
SHARPNESS_THRESHOLD = 80.0    # Laplacian variance — below = blurry
SYNC_TOLERANCE_S    = 0.045   # Max camera-to-LiDAR timestamp gap (45ms)

# ── Collision monitor topics (published by dronepi-main/collision_monitor.py) ─
TOPIC_COLLISION_ZONE = "/dronepi/collision_zone"       # CLEAR|CAUTION|OBSTACLE
TOPIC_AGL_RANGE      = "/mavros/distance_sensor/lidar_down"  # Range msg, AGL metres
EXPECTED_ZONES       = {"CLEAR", "CAUTION", "OBSTACLE"}
# At survey altitudes (3–15m) over open terrain the expected zone is CLEAR.
# AGL from the downward cone should match commanded altitude within this margin.
AGL_TOLERANCE_M      = 1.5    # metres — acceptable AGL vs commanded altitude delta

# ── Collision zone characterisation constants (match collision_zone_test.py) ──
IGNORE_RADIUS   = 0.70   # m — drone body + legs
OBSTACLE_RADIUS = 2.00   # m — prop tip + braking distance
CAUTION_RADIUS  = 3.50   # m — outer warning ring
COLOUR_IGNORE   = (128, 128, 128)   # grey
COLOUR_OBSTACLE = (220,  50,  50)   # red
COLOUR_CAUTION  = (255, 200,   0)   # yellow
COLOUR_CLEAR    = ( 50, 200,  50)   # green
# Messages to accumulate per level (~5s at 10 Hz = ~250k-500k points)
ZONE_COLLECT_MSGS = 50

# ── Default flight parameters ─────────────────────────────────────────────────
DEFAULT_ALTITUDES    = [3.0, 6.0, 9.0, 12.0, 15.0]   # metres above home
DEFAULT_HOLD_S       = 60.0   # seconds at each altitude
DEFAULT_PICS         = 5      # frames captured per level
CLIMB_TIMEOUT_S      = 30.0   # max seconds to reach each target altitude
ALTITUDE_TOLERANCE   = 0.25   # metres — within this = "at altitude"

# ── Services to pause ─────────────────────────────────────────────────────────
# Bench tests now always keep collision avoidance active. Only the watchdog
# is stopped; dronepi-main stays up so collision_monitor.py continues
# publishing obstacle distances and AGL to PX4 throughout the test.
_SERVICES_WATCHDOG   = ["drone-watchdog.service"]

# ── Process commands ──────────────────────────────────────────────────────────
_POINTLIO_LOG = Path("/tmp/hover_test_pointlio.log")

def _ros_source() -> str:
    return f"source {_ROS_SETUP} && source {_WS_SETUP} && "

_POINTLIO_CMD = _ros_source() + "ros2 launch point_lio mapping_unilidar.launch.py"


# ══════════════════════════════════════════════════════════════════════════════
# FOV calculator
# ══════════════════════════════════════════════════════════════════════════════

def _load_intrinsics() -> tuple[float, float, int, int]:
    """Load fx, fy, width, height from calibration YAML if available."""
    try:
        import yaml
        with open(_CALIB_PATH) as f:
            cal = yaml.safe_load(f)
        cam = cal.get("camera", {})
        return (
            float(cam.get("fx",           _DEFAULT_FX)),
            float(cam.get("fy",           _DEFAULT_FY)),
            int  (cam.get("image_width",  _DEFAULT_W)),
            int  (cam.get("image_height", _DEFAULT_H)),
        )
    except Exception:
        return _DEFAULT_FX, _DEFAULT_FY, _DEFAULT_W, _DEFAULT_H


def footprint_at(alt_m: float) -> tuple[float, float]:
    """
    Return (width_m, height_m) ground footprint at the given altitude.

    Uses the pinhole model: footprint = (sensor_size / focal_length) * altitude
    which in pixel terms is: footprint_w = (image_width / fx) * altitude

    Source: standard pinhole camera ground sampling distance formula,
    identical to what QGroundControl uses for survey planning.
    """
    fx, fy, w, h = _load_intrinsics()
    fw = (w / fx) * alt_m
    fh = (h / fy) * alt_m
    return fw, fh


# ══════════════════════════════════════════════════════════════════════════════
# ServiceManager
# ══════════════════════════════════════════════════════════════════════════════

class ServiceManager:
    def __init__(self, services: list[str]) -> None:
        self._services    = services
        self._was_active: dict[str, bool] = {}

    def stop_all(self) -> None:
        print("\n[Services] Pausing DronePi services...")
        for svc in self._services:
            active = subprocess.run(
                ["systemctl", "is-active", "--quiet", svc],
                capture_output=True,
            ).returncode == 0
            self._was_active[svc] = active
            if active:
                subprocess.run(["sudo", "systemctl", "stop", svc], check=False)
                print(f"  stopped : {svc}")
            else:
                print(f"  skipped : {svc}  (was not running)")
        time.sleep(1.5)

    def restore_all(self) -> None:
        print("\n[Services] Restoring DronePi services...")
        subprocess.run(["sudo", "systemctl", "daemon-reload"], check=False)
        for svc in self._services:
            if self._was_active.get(svc, False):
                subprocess.run(["sudo", "systemctl", "start", svc], check=False)
                print(f"  started : {svc}")
            else:
                print(f"  left stopped: {svc}")


# ══════════════════════════════════════════════════════════════════════════════
# RpicamCapture — direct rpicam-vid MJPEG stdout capture
# ══════════════════════════════════════════════════════════════════════════════

class RpicamCapture:
    """
    In-process camera capture using rpicam-vid MJPEG stdout pipe.

    Mirrors the frame extraction logic from ArducamNode but runs
    inside the test process — no ROS topic, no DDS discovery, no
    subprocess-to-subprocess IPC. rpicam-vid is the only subprocess.

    rpicam-vid --codec mjpeg -o - writes raw concatenated JPEGs.
    Each JPEG starts with FF D8 (SOI) and ends with FF D9 (EOI).
    The reader thread scans for these markers to extract complete frames.
    Source: JPEG specification ISO/IEC 10918-1.
    """

    _SOI = b"\xff\xd8"
    _EOI = b"\xff\xd9"
    _CHUNK = 65536

    def __init__(self, width: int = 1280, height: int = 960, fps: int = 10) -> None:
        self._width   = width
        self._height  = height
        self._fps     = fps
        self._proc:   Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None
        self._lock    = threading.Lock()
        self._frame:  Optional[bytes] = None
        self._count   = 0
        self._stop    = threading.Event()

    def start(self) -> bool:
        """Launch rpicam-vid and start the MJPEG reader thread. Returns True on success."""
        cmd = [
            "rpicam-vid",
            "--codec",     "mjpeg",
            "-o",          "-",
            "--width",     str(self._width),
            "--height",    str(self._height),
            "--framerate", str(self._fps),
            "--timeout",   "0",
            "--nopreview",
            "--flush",
        ]
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
                preexec_fn=os.setsid,
            )
        except FileNotFoundError:
            print("  [Camera] rpicam-vid not found — camera checks will be skipped")
            return False
        except Exception as exc:
            print(f"  [Camera] rpicam-vid launch failed: {exc}")
            return False

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="rpicam_reader"
        )
        self._thread.start()

        # Wait up to 6s for the first frame before continuing
        deadline = time.time() + 6.0
        while time.time() < deadline:
            with self._lock:
                if self._count > 0:
                    print(f"  [Camera] rpicam-vid ready  "
                          f"{self._width}×{self._height} @ {self._fps} fps  ✓")
                    return True
            time.sleep(0.1)

        print("  [Camera] rpicam-vid started — no frames yet, continuing")
        return True

    def capture(self, timeout: float = 3.0) -> "Optional[np.ndarray]":
        """
        Return the latest frame as a BGR numpy array, or None on timeout.

        Waits until a frame newer than the last seen arrives.
        """
        import cv2
        with self._lock:
            prev = self._count

        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._count > prev and self._frame is not None:
                    jpeg = bytes(self._frame)
                    break
            time.sleep(0.05)
        else:
            return None

        arr = np.frombuffer(jpeg, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)   # BGR or None if corrupt

    def stop(self) -> None:
        """Stop the reader thread and terminate rpicam-vid cleanly."""
        self._stop.set()
        if self._proc is not None and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        print(f"  [Camera] stopped  ({self._count} frames captured)")

    @property
    def frame_count(self) -> int:
        with self._lock:
            return self._count

    def _reader_loop(self) -> None:
        buf = b""
        start_idx = -1
        while not self._stop.is_set():
            try:
                chunk = self._proc.stdout.read(self._CHUNK)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk
            while True:
                if start_idx < 0:
                    idx = buf.find(self._SOI)
                    if idx < 0:
                        buf = buf[-1:]
                        break
                    start_idx = idx
                eoi_idx = buf.find(self._EOI, start_idx + 2)
                if eoi_idx < 0:
                    break
                frame_bytes = buf[start_idx : eoi_idx + 2]
                with self._lock:
                    self._frame = frame_bytes
                    self._count += 1
                buf = buf[eoi_idx + 2:]
                start_idx = -1


# ══════════════════════════════════════════════════════════════════════════════
# Process helpers
# ══════════════════════════════════════════════════════════════════════════════

def _launch(name: str, cmd: str, log_path: Path) -> subprocess.Popen:
    print(f"  Starting {name}  (log → {log_path})")
    with open(log_path, "w") as lf:
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=lf, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    return proc


def _stop_proc(name: str, proc: Optional[subprocess.Popen]) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=8)
        print(f"  stopped : {name}")
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            print(f"  killed  : {name}")
        except ProcessLookupError:
            pass


def _wait_topic(topic: str, timeout: float = 25.0, min_hz: float = 1.0) -> bool:
    print(f"  Waiting for {topic}  ({timeout:.0f}s timeout)", end="", flush=True)
    deadline = time.time() + timeout

    # sensor_msgs/Image from ArducamNode is published with BEST_EFFORT QoS.
    # Use matching QoS in ros2 topic hz or the CLI subscriber may see zero
    # samples even while the node is publishing correctly.
    qos = ""
    if topic == "/arducam/image_raw":
        qos = " --qos-reliability best_effort --qos-durability volatile"

    while time.time() < deadline:
        r = subprocess.run(
            ["bash", "-c",
             _ros_source() + f"timeout 3 ros2 topic hz {topic} --window 5{qos} 2>&1"],
            capture_output=True, text=True,
        )
        for line in r.stdout.splitlines():
            if "average rate:" in line:
                try:
                    hz = float(line.split("average rate:")[1].strip())
                    if hz >= min_hz:
                        print(f" {hz:.1f} Hz  ✓")
                        return True
                except ValueError:
                    pass
        print(".", end="", flush=True)
        time.sleep(1.5)
    print(" TIMEOUT")
    return False


def _wait_arducam_ready(proc: "Optional[subprocess.Popen]", log_path: "Path", timeout: float = 25.0) -> bool:
    """Wait for ArducamNode readiness via log file polling.

    ArducamNode publishes with BEST_EFFORT QoS. In this environment,
    ros2 topic list does not discover the topic reliably within the
    readiness window, and the QoS override flags required by ros2 topic hz
    are not supported by the installed ROS 2 CLI build.

    The log file is the authoritative readiness signal:
      "ArducamNode started" -- node initialised, rpicam-vid launched (~1s)
      "Published N frames"  -- frames flowing through the ROS pipeline (~5s)

    The ros2 topic list check is intentionally omitted. It spawns a bash
    subprocess that sources two setup files per poll iteration, consuming
    1-3s per call and exhausting the timeout before the log signals appear.
    """
    print(f"  Waiting for ArducamNode  ({timeout:.0f}s timeout)", end="", flush=True)
    deadline = time.time() + timeout
    started_seen = False

    # Allow the subprocess and rpicam-vid time to write the first log line
    # before beginning polls. Without this, the first 1-2 iterations read
    # an empty file and print spurious dots.
    time.sleep(1.5)

    while time.time() < deadline:
        # Fail fast if the process has already exited abnormally
        if proc is not None and proc.poll() is not None:
            print(" PROCESS EXITED")
            return False

        try:
            if log_path.exists():
                tail = log_path.read_text(errors="ignore")
                if not started_seen and "ArducamNode started" in tail:
                    started_seen = True
                    print(" [node up]", end="", flush=True)
                # "Published N frames" appears at frame 50 (~5s at 10 Hz).
                # This is the definitive signal that the full pipeline
                # (rpicam-vid -> MJPEG parser -> cv2.imdecode -> ROS publish)
                # is working end-to-end.
                if "Published" in tail:
                    print("  ✓")
                    return True
        except Exception:
            pass

        print(".", end="", flush=True)
        time.sleep(1.0)

    print(" TIMEOUT")
    return False

class BagRecorder:
    """
    Manages ros2 bag record subprocess for one altitude level.

    Records the topics needed for post-flight texture projection:
      /arducam/image_raw        — camera frames
      /aft_mapped_to_init       — LiDAR SLAM poses
      /cloud_registered         — LiDAR point cloud
      /mavros/local_position/pose — drone position log

    Each level gets its own directory named by altitude and timestamp
    so bags are individually processable by postprocess_mesh.py.
    """

    # Topics recorded at every level
    TOPICS = [
        "/arducam/image_raw",
        "/aft_mapped_to_init",
        "/cloud_registered",
        "/mavros/local_position/pose",
    ]

    def __init__(self, alt_m: float) -> None:
        self._alt_m  = alt_m
        self._proc:  Optional[subprocess.Popen] = None
        self._path:  Optional[Path] = None

    def start(self) -> Path:
        """Start recording. Returns the bag directory path."""
        BAG_DIR.mkdir(parents=True, exist_ok=True)
        ts       = time.strftime("%Y%m%d_%H%M%S")
        alt_tag  = f"{self._alt_m:04.1f}m".replace(".", "_")
        bag_name = f"hover_alt_{alt_tag}_{ts}"
        bag_path = BAG_DIR / bag_name

        topics_str = " ".join(self.TOPICS)
        cmd = (
            _ros_source()
            + f"ros2 bag record {topics_str} "
            + f"-o {bag_path} "
            + f"--storage sqlite3"
        )

        log_path = Path(f"/tmp/hover_bag_{alt_tag}.log")
        self._proc = _launch(f"BagRecorder {self._alt_m}m", cmd, log_path)
        self._path = bag_path

        # Give ros2 bag a moment to open all topics
        time.sleep(2.0)
        print(f"  Recording → {bag_path.name}")
        return bag_path

    def stop(self) -> Optional[Path]:
        """Stop recording. Returns the bag directory path."""
        if self._proc is not None:
            _stop_proc(f"BagRecorder {self._alt_m}m", self._proc)
            self._proc = None
            # ros2 bag needs ~2s to finalise and write the metadata.yaml
            time.sleep(2.0)
            print(f"  Bag closed : {self._path.name if self._path else 'unknown'}")
        return self._path

    @property
    def path(self) -> Optional[Path]:
        return self._path



# ══════════════════════════════════════════════════════════════════════════════
# ZoneCharacteriser — per-level collision zone analysis and export
# ══════════════════════════════════════════════════════════════════════════════

class ZoneCharacteriser:
    """
    Classifies accumulated /cloud_registered points into collision zones and
    exports CSV + PLY output matching the collision_zone_test.py format.

    Output per altitude level (written to OUTPUT_DIR/alt_XXm/):
      zone_stats.csv      — every point with x, y, z, range_2d, zone columns
      zones_cloud.ply     — colour-coded PLY viewable in meshview.html
      zone_summary.txt    — zone count percentages with ASCII bar chart

    Zone radii (must match collision_monitor.py and collision_zone_test.py):
      IGNORE   < 0.70m   — drone body/legs
      OBSTACLE 0.70-2.00m — prop tip + braking distance
      CAUTION  2.00-3.50m — early warning ring
      CLEAR    > 3.50m   — safe

    Usage within the flight node:
      # Enable collection at start of hold period
      node.start_zone_collection()

      # Spin the ROS node for ZONE_COLLECT_MSGS messages (happens automatically
      # during the normal hold period stream)

      # After bag stops, export results
      charser = ZoneCharacteriser(alt_m, node.get_cloud_snapshot())
      charser.export(OUTPUT_DIR / alt_tag)
      charser.print_live_summary()
    """

    def __init__(self, alt_m: float, points: list) -> None:
        """
        Args:
            alt_m:  Altitude above home in metres — used in output labelling.
            points: List of (x, y, z) tuples from the cloud buffer.
        """
        self._alt_m  = alt_m
        self._points = points

    # ── Public API ─────────────────────────────────────────────────────────────

    def export(self, out_dir: Path) -> dict:
        """
        Classify all points, export CSV + PLY, print summary.

        Returns a stats dict with keys: total, ignore, obstacle, caution, clear,
        pct_obstacle, pct_caution, pct_clear — used in the flight report.
        """
        import numpy as np

        out_dir.mkdir(parents=True, exist_ok=True)
        pts = self._points

        if not pts:
            print(f"    [WARN] No cloud points collected — zone export skipped")
            return {"total": 0}

        arr = np.array(pts, dtype=np.float32)
        r2d = np.sqrt(arr[:, 0]**2 + arr[:, 1]**2)

        # Classify
        zones   = self._classify_array(r2d)
        n_total = len(zones)
        counts  = {z: int((zones == z).sum()) for z in
                   ["IGNORE", "OBSTACLE", "CAUTION", "CLEAR"]}
        pcts    = {z: 100.0 * counts[z] / max(n_total, 1) for z in counts}

        # Export CSV
        self._write_csv(arr, r2d, zones, out_dir)

        # Export PLY
        self._write_ply(arr, zones, out_dir)

        # Print live summary matching collision_zone_test.py style
        self._print_summary(counts, pcts, n_total)

        return {
            "total":        n_total,
            "ignore":       counts["IGNORE"],
            "obstacle":     counts["OBSTACLE"],
            "caution":      counts["CAUTION"],
            "clear":        counts["CLEAR"],
            "pct_obstacle": pcts["OBSTACLE"],
            "pct_caution":  pcts["CAUTION"],
            "pct_clear":    pcts["CLEAR"],
        }

    # ── Private ────────────────────────────────────────────────────────────────

    @staticmethod
    def _classify_array(r2d) -> object:
        import numpy as np
        zones = np.full(len(r2d), "CLEAR   ", dtype="<U8")
        zones[r2d < CAUTION_RADIUS]  = "CAUTION "
        zones[r2d < OBSTACLE_RADIUS] = "OBSTACLE"
        zones[r2d < IGNORE_RADIUS]   = "IGNORE  "
        return np.char.strip(zones)

    def _write_csv(self, arr, r2d, zones, out_dir: Path) -> None:
        import numpy as np
        lines = ["x,y,z,range_2d,zone"]
        for i in range(len(arr)):
            lines.append(
                f"{arr[i,0]:.4f},{arr[i,1]:.4f},{arr[i,2]:.4f},"
                f"{r2d[i]:.4f},{zones[i]}"
            )
        path = out_dir / "zone_stats.csv"
        path.write_text("\n".join(lines) + "\n")
        print(f"    Zone CSV   → {path.name}  ({len(arr):,} points)")

    def _write_ply(self, arr, zones, out_dir: Path) -> None:
        colour_map = {
            "IGNORE":   COLOUR_IGNORE,
            "OBSTACLE": COLOUR_OBSTACLE,
            "CAUTION":  COLOUR_CAUTION,
            "CLEAR":    COLOUR_CLEAR,
        }
        path = out_dir / "zones_cloud.ply"
        with open(path, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"comment DronePi altitude {self._alt_m:.0f}m zone characterisation\n")
            f.write(f"comment IGNORE={IGNORE_RADIUS}m "
                    f"OBSTACLE={OBSTACLE_RADIUS}m "
                    f"CAUTION={CAUTION_RADIUS}m\n")
            f.write(f"element vertex {len(arr)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            for i in range(len(arr)):
                r, g, b = colour_map.get(zones[i], COLOUR_CLEAR)
                f.write(f"{arr[i,0]:.4f} {arr[i,1]:.4f} {arr[i,2]:.4f} "
                        f"{r} {g} {b}\n")
        print(f"    Zone PLY   → {path.name}  (drag into meshview.html)")

    def _print_summary(self, counts: dict, pcts: dict, total: int) -> None:
        print(f"\n    Zone Summary @ {self._alt_m:.0f}m  "
              f"({total:,} points, {ZONE_COLLECT_MSGS} msgs)")
        for zone, label, colour in [
            ("IGNORE",   "grey   — drone body/legs", "⬜"),
            ("OBSTACLE", "red    — < 2.0m danger",   "🔴"),
            ("CAUTION",  "yellow — 2.0–3.5m warn",   "🟡"),
            ("CLEAR",    "green  — safe > 3.5m",      "🟢"),
        ]:
            n   = counts[zone]
            pct = pcts[zone]
            bar = "█" * int(pct / 2)
            print(f"    {colour} {zone:<10}  {n:>8,} pts  {pct:5.1f}%  {bar}")


# ══════════════════════════════════════════════════════════════════════════════
# ROS 2 flight node
# ══════════════════════════════════════════════════════════════════════════════

class HoverCameraNode:
    """
    ROS 2 node that executes the multi-altitude hover and capture sequence.
    """

    def __init__(self, args: argparse.Namespace, cam_capture: RpicamCapture = None) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos  import (QoSProfile, ReliabilityPolicy,
                                 HistoryPolicy, DurabilityPolicy)
        from geometry_msgs.msg  import PoseStamped
        from mavros_msgs.msg    import State
        from mavros_msgs.srv    import CommandBool, SetMode
        from nav_msgs.msg       import Odometry
        from sensor_msgs.msg    import Image, Range, PointCloud2
        from std_msgs.msg       import String

        self._rclpy = rclpy
        self._args  = args
        self._cam   = cam_capture

        rclpy.init()

        class _Inner(Node):
            def __init__(inner_self):
                super().__init__("hover_camera_test")
                reliable = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=10)
                sensor = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=5,
                    durability=DurabilityPolicy.VOLATILE)

                inner_self.fcu_state:   Optional[State]       = None
                inner_self.local_pose:  Optional[PoseStamped] = None
                inner_self.last_image:  Optional[Image]       = None
                inner_self.lidar_ts:    Optional[float]       = None
                inner_self.collision_zone: Optional[str]      = None  # CLEAR|CAUTION|OBSTACLE
                inner_self.agl_m:          Optional[float]    = None  # metres from downward cone
                inner_self._cloud_pts:     list               = []    # (x,y,z) tuples
                inner_self._cloud_msgs:    int                = 0     # message count
                inner_self._collecting:    bool               = False # gate
                inner_self._lock = threading.Lock()

                inner_self.create_subscription(
                    State,       "/mavros/state",
                    lambda m: setattr(inner_self, "fcu_state", m), reliable)
                inner_self.create_subscription(
                    PoseStamped, "/mavros/local_position/pose",
                    lambda m: setattr(inner_self, "local_pose", m), sensor)
                inner_self.create_subscription(
                    Odometry, "/aft_mapped_to_init",
                    inner_self._cb_lidar, sensor)
                inner_self.create_subscription(
                    PointCloud2, "/cloud_registered",
                    inner_self._cb_cloud, sensor)
                # Collision monitor topics — only populated when dronepi-main
                # is running (--keep-collision flag).  Silently absent otherwise.
                inner_self.create_subscription(
                    String, TOPIC_COLLISION_ZONE,
                    inner_self._cb_zone, reliable)
                inner_self.create_subscription(
                    Range, TOPIC_AGL_RANGE,
                    inner_self._cb_agl, sensor)

                inner_self._sp_pub = inner_self.create_publisher(
                    PoseStamped, "/mavros/setpoint_position/local", reliable)
                inner_self._arm_cli  = inner_self.create_client(
                    CommandBool, "/mavros/cmd/arming")
                inner_self._mode_cli = inner_self.create_client(
                    SetMode, "/mavros/set_mode")

            def _cb_lidar(inner_self, msg):
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                with inner_self._lock:
                    inner_self.lidar_ts = ts

            def _cb_zone(inner_self, msg):
                with inner_self._lock:
                    inner_self.collision_zone = msg.data.strip()

            def _cb_agl(inner_self, msg):
                with inner_self._lock:
                    # Range.range is NaN when out of sensor range — guard it
                    import math as _math
                    if not _math.isnan(msg.range) and msg.range > 0:
                        inner_self.agl_m = float(msg.range)

            def _cb_cloud(inner_self, msg):
                # Accumulate XYZ points while _collecting gate is open
                with inner_self._lock:
                    if not inner_self._collecting:
                        return
                    if inner_self._cloud_msgs >= ZONE_COLLECT_MSGS:
                        return
                try:
                    import struct as _s
                    import numpy as _np
                    fields = {f.name: f.offset for f in msg.fields}
                    if not all(k in fields for k in ('x', 'y', 'z')):
                        return
                    x_off, y_off, z_off = fields['x'], fields['y'], fields['z']
                    ps   = msg.point_step
                    data = bytes(msg.data)
                    n    = msg.width * msg.height
                    if x_off == 0 and y_off == 4 and z_off == 8:
                        raw = _np.frombuffer(data, dtype=_np.float32).reshape(-1, ps // 4)
                        xyz = raw[:, :3].copy()
                    else:
                        xyz = _np.empty((n, 3), dtype=_np.float32)
                        for i in range(n):
                            b = i * ps
                            xyz[i, 0] = _s.unpack_from('<f', data, b + x_off)[0]
                            xyz[i, 1] = _s.unpack_from('<f', data, b + y_off)[0]
                            xyz[i, 2] = _s.unpack_from('<f', data, b + z_off)[0]
                    valid = _np.isfinite(xyz).all(axis=1)
                    pts   = [(float(r[0]), float(r[1]), float(r[2])) for r in xyz[valid]]
                    with inner_self._lock:
                        inner_self._cloud_pts.extend(pts)
                        inner_self._cloud_msgs += 1
                except Exception:
                    pass

        self._node     = _Inner()
        self._results: list[dict] = []   # all per-frame results across all levels
        self._bags:    list[Path] = []   # bag paths produced
        self._zone_snapshots: dict = {}  # per-level raw cloud points (alt_m -> list of (x,y,z)), exported post-landing
        self._zone_stats:     dict = {}  # per-level ZoneCharacteriser stats after export
        self._PoseStamped = PoseStamped
        self._CommandBool = CommandBool
        self._SetMode     = SetMode

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        s = self._node.fcu_state
        return s is not None and s.connected

    @property
    def armed(self) -> bool:
        s = self._node.fcu_state
        return s is not None and s.armed

    def _z(self) -> float:
        p = self._node.local_pose
        return p.pose.position.z if p else 0.0

    # ── Setpoint helpers ──────────────────────────────────────────────────────

    def _pub_sp(self, x: float, y: float, z: float) -> None:
        msg = self._PoseStamped()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self._node._sp_pub.publish(msg)

    def _spin(self, secs: float) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=secs)

    def _stream(self, x: float, y: float, z: float,
                dur: float, hz: float = 20.0) -> None:
        deadline = time.time() + dur
        while time.time() < deadline:
            self._pub_sp(x, y, z)
            self._spin(1.0 / hz)

    # ── FCU service calls ─────────────────────────────────────────────────────

    def set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        req = self._SetMode.Request()
        req.custom_mode = mode
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._node._mode_cli.service_is_ready():
                f = self._node._mode_cli.call_async(req)
                self._rclpy.spin_until_future_complete(
                    self._node, f, timeout_sec=2.0)
                if f.result() and f.result().mode_sent:
                    return True
            time.sleep(0.2)
        return False

    def arm(self, timeout: float = 10.0) -> bool:
        req = self._CommandBool.Request()
        req.value = True
        deadline  = time.time() + timeout
        while time.time() < deadline:
            if self._node._arm_cli.service_is_ready():
                f = self._node._arm_cli.call_async(req)
                self._rclpy.spin_until_future_complete(
                    self._node, f, timeout_sec=2.0)
                if f.result() and f.result().success:
                    return True
            time.sleep(0.3)
        return False

    def disarm(self) -> None:
        req = self._CommandBool.Request()
        req.value = False
        if self._node._arm_cli.service_is_ready():
            f = self._node._arm_cli.call_async(req)
            self._rclpy.spin_until_future_complete(
                self._node, f, timeout_sec=2.0)

    # ── Navigation ────────────────────────────────────────────────────────────

    def _fly_to(self, x: float, y: float, z: float,
                tol: float = 0.25, timeout: float = CLIMB_TIMEOUT_S) -> bool:
        """Stream setpoint and wait until within tol metres of target."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._pub_sp(x, y, z)
            self._spin(0.05)
            p = self._node.local_pose
            if p:
                dx = p.pose.position.x - x
                dy = p.pose.position.y - y
                dz = p.pose.position.z - z
                if math.sqrt(dx*dx + dy*dy + dz*dz) < tol:
                    return True
        return False

    # ── Frame capture ─────────────────────────────────────────────────────────

    # ── Zone collection control ─────────────────────────────────────────────

    def start_zone_collection(self) -> None:
        """Open the cloud buffer gate — /cloud_registered points accumulate."""
        with self._node._lock:
            self._node._cloud_pts   = []
            self._node._cloud_msgs  = 0
            self._node._collecting  = True

    def stop_zone_collection(self) -> list:
        """Close the gate and return the accumulated (x, y, z) tuples."""
        with self._node._lock:
            self._node._collecting = False
            return list(self._node._cloud_pts)

    def zone_msg_count(self) -> int:
        """Return number of cloud messages received since last start."""
        with self._node._lock:
            return self._node._cloud_msgs

    def _wait_new_frame(self, timeout: float = 3.0) -> "Optional[np.ndarray]":
        """Capture a frame directly from rpicam-vid via RpicamCapture."""
        if self._cam is None:
            return None
        return self._cam.capture(timeout=timeout)

    def capture_frame(
        self,
        alt_m:    float,
        frame_idx: int,
        z_target: float,
    ) -> dict:
        """
        Capture one frame while holding position at (0, 0, z_target).

        Returns a result dict keyed by altitude and frame index.
        Image saved to OUTPUT_DIR/alt_XXm/frame_NN.jpg.
        """
        import cv2
        import numpy as np

        alt_tag = f"alt_{alt_m:04.1f}m".replace(".", "_")
        label   = f"{alt_tag}_f{frame_idx:02d}"

        result = dict(
            alt_m=alt_m, frame_idx=frame_idx, label=label,
            sharpness=None, sharp_ok=False,
            sync_gap_s=None, sync_ok=False,
            image_path=None, passed=False,
            collision_zone=None, agl_m=None, agl_ok=None,
        )

        # Wait for a genuinely fresh frame
        frame = self._wait_new_frame(timeout=4.0)

        if frame is None:
            print(f"    [FAIL] No new frame received (timeout)")
            return result

        frame_ts = frame.header.stamp.sec + frame.header.stamp.nanosec * 1e-9

        # ── LiDAR sync ────────────────────────────────────────────────────────
        with self._node._lock:
            lidar_ts = self._node.lidar_ts

        if lidar_ts is not None:
            gap = abs(frame_ts - lidar_ts)
            result["sync_gap_s"] = gap
            result["sync_ok"]    = gap < SYNC_TOLERANCE_S
            icon = "✓" if result["sync_ok"] else "✗"
            print(f"    LiDAR gap   : {gap*1000:.1f}ms  {icon}")
        else:
            print("    [WARN] No LiDAR timestamp — sync skipped")

        # ── Focus ─────────────────────────────────────────────────────────────
        img = frame   # already BGR numpy array from RpicamCapture
        if img is not None:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            lap  = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            result["sharpness"] = lap
            result["sharp_ok"]  = lap >= SHARPNESS_THRESHOLD
            icon = "✓" if result["sharp_ok"] else "✗  (blurry)"
            print(f"    Sharpness   : {lap:.1f}  {icon}")

            # Save image
            level_dir = OUTPUT_DIR / alt_tag
            level_dir.mkdir(parents=True, exist_ok=True)
            out = level_dir / f"frame_{frame_idx:02d}.jpg"
            cv2.imwrite(str(out), img)
            result["image_path"] = str(out)
            print(f"    Saved       : {alt_tag}/frame_{frame_idx:02d}.jpg")
        else:
            print(f"    [FAIL] Cannot decode frame (encoding='{frame.encoding}')")

        # ── Collision zone snapshot (only if --keep-collision active) ───────
        with self._node._lock:
            zone  = self._node.collision_zone
            agl_m = self._node.agl_m

        if zone is not None:
            result["collision_zone"] = zone
            expected_zone = "CLEAR"
            zone_ok = zone in EXPECTED_ZONES   # at least a valid value
            icon = "✓" if zone == expected_zone else "⚠"
            print(f"    Collision zone : {zone}  {icon}")

        if agl_m is not None:
            result["agl_m"] = agl_m
            agl_err = abs(agl_m - alt_m)
            result["agl_ok"] = agl_err < AGL_TOLERANCE_M
            icon = "✓" if result["agl_ok"] else "✗"
            print(f"    AGL reported   : {agl_m:.2f}m  "
                  f"(commanded {alt_m:.1f}m  Δ{agl_err:.2f}m)  {icon}")

        result["passed"] = result["sharp_ok"] and (
            result["sync_ok"] if result["sync_gap_s"] is not None else True
        )
        return result



    # ── Main flight sequence ───────────────────────────────────────────────────

    def run(self) -> bool:
        """
        Execute the full multi-altitude hover sequence.

        For each altitude in args.altitudes:
          - Climb to altitude
          - Start bag
          - Hold for args.hold_s seconds
          - Capture args.pics_per_level frames, evenly spaced
          - Stop bag
          - Proceed to next altitude

        Returns True if the flight completed without a hard abort.
        Individual frame failures are recorded but do not abort the flight.
        """
        args      = self._args
        altitudes = args.altitudes
        hold_s    = args.hold_s
        n_pics    = args.pics_per_level
        home_z    = 0.0

        # Interval between frames within a hold period.
        # Leave a 3s settling buffer at the start and end of each hold.
        SETTLE_S  = 3.0
        pic_interval = (hold_s - 2 * SETTLE_S) / max(n_pics - 1, 1) if n_pics > 1 else hold_s / 2

        current_bag: Optional[BagRecorder] = None

        def _emergency_land() -> None:
            nonlocal current_bag
            if current_bag is not None:
                current_bag.stop()
                current_bag = None
            self.set_mode("AUTO.LAND")
            time.sleep(3.0)
            self.disarm()

        try:
            # ── [1] FCU connection ────────────────────────────────────────────
            print("\n[1] Waiting for FCU connection...")
            deadline = time.time() + 20.0
            while not self.connected and time.time() < deadline:
                self._spin(0.5)
            if not self.connected:
                print("  [FAIL] FCU not connected — is MAVROS running?")
                return False
            print("  [OK] FCU connected")

            # ── [2] Home Z ────────────────────────────────────────────────────
            for _ in range(30):
                self._spin(0.1)
            home_z = self._z()
            print(f"  Home Z = {home_z:.3f}m")

            # ── [3] Sensor liveness ───────────────────────────────────────────
            print("\n[2] Sensor liveness check...")
            deadline = time.time() + 10.0
            cam = lidar = False
            while time.time() < deadline:
                self._spin(0.2)
                # Camera liveness via RpicamCapture (no ROS subscription)
                cam   = (self._cam is not None and self._cam.frame_count > 0)
                with self._node._lock:
                    lidar = self._node.lidar_ts is not None
                if cam and lidar:
                    break

            print(f"  Camera  : {'✓ OK' if cam   else '✗ NOT PUBLISHING'}")
            print(f"  LiDAR   : {'✓ OK' if lidar else '✗ NOT PUBLISHING'}")

            if not cam and not args.dry_run:
                print("  [FAIL] Camera not publishing — aborting")
                return False

            # ── [4] Pre-stream ────────────────────────────────────────────────
            print("\n[3] Pre-streaming setpoints (3s)...")
            self._stream(0.0, 0.0, home_z, dur=3.0)
            print("  [OK] Setpoint stream established")

            if args.dry_run:
                print("\n  --dry-run: stopping before arm.  All pre-flight checks passed.")
                # Print FOV table so it is visible in dry-run output
                self._print_fov_table(altitudes)
                return True

            # ── [5] Arm + OFFBOARD ────────────────────────────────────────────
            print("\n[4] Arming...")
            if not self.arm():
                print("  [FAIL] Arm rejected")
                return False
            print("  [OK] Armed")

            print("  Switching to OFFBOARD...")
            if not self.set_mode("OFFBOARD"):
                print("  [FAIL] OFFBOARD rejected")
                self.disarm()
                return False
            print("  [OK] OFFBOARD")

            # ── [6] Multi-altitude loop ────────────────────────────────────────
            total_levels = len(altitudes)
            for level_idx, alt_m in enumerate(altitudes):
                target_z = home_z + alt_m
                fw, fh   = footprint_at(alt_m)

                print(f"\n{'='*55}")
                print(f"  Level {level_idx+1}/{total_levels}  —  "
                      f"{alt_m:.0f}m above home  "
                      f"(footprint {fw:.1f}×{fh:.1f}m)")
                print(f"{'='*55}")

                # ── Climb ─────────────────────────────────────────────────────
                print(f"\n  Climbing to {alt_m:.0f}m  (target Z={target_z:.2f}m)...")
                reached = self._fly_to(
                    0.0, 0.0, target_z,
                    tol=ALTITUDE_TOLERANCE,
                    timeout=CLIMB_TIMEOUT_S,
                )
                if not reached:
                    print(f"  [WARN] Did not reach {alt_m:.0f}m within "
                          f"{CLIMB_TIMEOUT_S:.0f}s  "
                          f"(at {self._z() - home_z:.1f}m) — continuing anyway")
                else:
                    print(f"  [OK] At altitude: {self._z() - home_z:.1f}m")

                # ── Start bag ─────────────────────────────────────────────────
                print(f"\n  Starting bag for {alt_m:.0f}m level...")
                current_bag = BagRecorder(alt_m)
                bag_path    = current_bag.start()

                # ── Hover and capture ─────────────────────────────────────────
                print(f"\n  Hovering {hold_s:.0f}s  —  "
                      f"capturing {n_pics} frames "
                      f"(interval {pic_interval:.1f}s)...")

                # Open zone collection gate — accumulates /cloud_registered
                # points throughout the hold period for post-hold PLY export
                self.start_zone_collection()

                # Initial settling pause — let the drone stabilise
                self._stream(0.0, 0.0, target_z, dur=SETTLE_S)

                for pic_idx in range(1, n_pics + 1):
                    print(f"\n  Frame {pic_idx}/{n_pics} @ {alt_m:.0f}m")
                    r = self.capture_frame(alt_m, pic_idx, target_z)
                    self._results.append(r)
                    status = "PASS" if r["passed"] else "FAIL"
                    print(f"  → {status}")

                    # Stream setpoints during the inter-frame interval
                    # (except after the last frame — final settling instead)
                    if pic_idx < n_pics:
                        self._stream(0.0, 0.0, target_z, dur=pic_interval)

                # Final settling pause at end of hold period
                remaining = hold_s - SETTLE_S - (n_pics - 1) * pic_interval - SETTLE_S
                if remaining > 0:
                    print(f"\n  Remaining hold: {remaining:.0f}s...")
                    self._stream(0.0, 0.0, target_z, dur=remaining)

                self._stream(0.0, 0.0, target_z, dur=SETTLE_S)

                # ── Stop bag ──────────────────────────────────────────────────
                print(f"\n  Stopping bag for {alt_m:.0f}m level...")
                current_bag.stop()
                self._bags.append(bag_path)
                current_bag = None
                print(f"  Bag saved → {bag_path.name}")

                # ── Zone snapshot — store points in memory, export after landing ──────
                # PLY/CSV generation is deferred to post-landing to avoid stalling
                # the flight loop with CPU-bound ASCII PLY writes mid-flight.
                cloud_pts  = self.stop_zone_collection()
                msg_count  = self.zone_msg_count()
                print(f"\n  Zone snapshot: {len(cloud_pts):,} pts "
                      f"from {msg_count} msgs — export queued for post-landing")
                self._zone_snapshots[alt_m] = cloud_pts

            # ── [7] Descend and land ──────────────────────────────────────────
            print(f"\n[5] All levels complete — descending and landing...")
            # Descend slowly — go to 2m above home first then AUTO.LAND
            descent_z = home_z + 2.0
            print(f"  Descending to 2m...")
            self._fly_to(0.0, 0.0, descent_z, tol=0.30, timeout=40.0)
            self._stream(0.0, 0.0, descent_z, dur=2.0)
            print("  Switching to AUTO.LAND...")
            self.set_mode("AUTO.LAND")

            deadline = time.time() + 40.0
            while self.armed and time.time() < deadline:
                self._spin(0.3)
            if not self.armed:
                print("  [OK] Landed and disarmed")
            else:
                print("  [WARN] Still armed — disarming manually")
                self.disarm()


            # ── Post-landing zone export ──────────────────────────────────────
            # All PLY and CSV files are generated here, after the drone is on
            # the ground, so no CPU work competes with the flight loop.
            if self._zone_snapshots:
                print(f"\n[Post-flight] Exporting zone characterisation "
                      f"for {len(self._zone_snapshots)} levels...")
                for snap_alt, snap_pts in sorted(self._zone_snapshots.items()):
                    alt_tag   = f"alt_{snap_alt:04.1f}m".replace('.', '_')
                    level_dir = OUTPUT_DIR / alt_tag
                    print(f"\n  Level {snap_alt:.0f}m  "
                          f"({len(snap_pts):,} pts)")
                    charser    = ZoneCharacteriser(snap_alt, snap_pts)
                    zone_stats = charser.export(level_dir)
                    self._zone_stats[snap_alt] = zone_stats
                print("\n  Zone export complete — open zones_cloud.ply "
                      "in meshview.html per level")

            return True

        except KeyboardInterrupt:
            print("\n[ABORT] Ctrl+C — emergency landing")
            _emergency_land()
            return False

        except Exception as exc:
            import traceback
            print(f"\n[FAIL] Unexpected exception: {exc}")
            traceback.print_exc()
            _emergency_land()
            return False

    def shutdown(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()

    # ── Report ────────────────────────────────────────────────────────────────

    def write_report(self) -> bool:
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        lines = [
            "=" * 65,
            "  DronePi — Multi-Altitude Stationary Hover Validation Report",
            f"  {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"  Altitudes: {[f'{a:.0f}m' for a in self._args.altitudes]}",
            f"  Hold: {self._args.hold_s:.0f}s/level   "
            f"Pics: {self._args.pics_per_level}/level",
            "=" * 65, "",
        ]

        # ── FOV table ─────────────────────────────────────────────────────────
        lines.append("Camera footprint at each altitude")
        lines.append(f"  {'Alt':>5}  {'Width':>8}  {'Height':>8}  {'px/m²':>8}")
        lines.append(f"  {'-'*5}  {'-'*8}  {'-'*8}  {'-'*8}")
        fx, fy, w, h = _load_intrinsics()
        for alt_m in self._args.altitudes:
            fw, fh  = footprint_at(alt_m)
            px_per_m2 = (w * h) / (fw * fh)
            lines.append(
                f"  {alt_m:>4.0f}m  {fw:>7.2f}m  {fh:>7.2f}m  {px_per_m2:>7.0f}"
            )
        lines.append("")

        # ── Per-level results ──────────────────────────────────────────────────
        from itertools import groupby
        by_alt = {}
        for r in self._results:
            by_alt.setdefault(r["alt_m"], []).append(r)

        all_pass = True
        for alt_m in self._args.altitudes:
            frames = by_alt.get(alt_m, [])
            fw, fh = footprint_at(alt_m)
            lines.append(f"Altitude {alt_m:.0f}m  "
                         f"(footprint {fw:.1f}×{fh:.1f}m)")

            if not frames:
                lines.append("  No frames captured at this level")
                lines.append("")
                all_pass = False
                continue

            pass_count = sum(1 for r in frames if r["passed"])
            lines.append(f"  Frames passed: {pass_count}/{len(frames)}")

            sharp_vals = [r["sharpness"] for r in frames if r["sharpness"] is not None]
            if sharp_vals:
                lines.append(
                    f"  Sharpness: min={min(sharp_vals):.0f}  "
                    f"avg={sum(sharp_vals)/len(sharp_vals):.0f}  "
                    f"max={max(sharp_vals):.0f}  "
                    f"(threshold {SHARPNESS_THRESHOLD:.0f})"
                )

            sync_vals = [r["sync_gap_s"]*1000 for r in frames
                         if r["sync_gap_s"] is not None]
            if sync_vals:
                lines.append(
                    f"  Sync gap:  min={min(sync_vals):.1f}ms  "
                    f"avg={sum(sync_vals)/len(sync_vals):.1f}ms  "
                    f"max={max(sync_vals):.1f}ms  "
                    f"(limit {SYNC_TOLERANCE_S*1000:.0f}ms)"
                )

            # Collision zone summary
            zones = [r["collision_zone"] for r in frames if r["collision_zone"]]
            if zones:
                zone_counts = {z: zones.count(z) for z in set(zones)}
                zone_str = "  ".join(f"{z}×{n}" for z, n in sorted(zone_counts.items()))
                lines.append(f"  Collision zones: {zone_str}")

            # AGL accuracy summary
            agl_vals = [r["agl_m"] for r in frames if r["agl_m"] is not None]
            if agl_vals:
                agl_errs = [abs(v - alt_m) for v in agl_vals]
                lines.append(
                    f"  AGL accuracy:  avg_err={sum(agl_errs)/len(agl_errs):.2f}m  "
                    f"max_err={max(agl_errs):.2f}m  "
                    f"(tolerance {AGL_TOLERANCE_M:.1f}m)"
                )

            for r in frames:
                sharp_s = f"{r['sharpness']:.0f}" if r["sharpness"] else "—"
                sync_s  = (f"{r['sync_gap_s']*1000:.1f}ms"
                           if r["sync_gap_s"] is not None else "—")
                zone_s  = r["collision_zone"] or "—"
                agl_s   = f"{r['agl_m']:.1f}m" if r["agl_m"] is not None else "—"
                status  = "✓" if r["passed"] else "✗"
                lines.append(
                    f"  Frame {r['frame_idx']:02d}: {status}  "
                    f"sharp={sharp_s:>6}  sync={sync_s:>8}  "
                    f"zone={zone_s:<9}  agl={agl_s:>6}  "
                    f"{Path(r['image_path']).name if r['image_path'] else 'no image'}"
                )

            if pass_count < len(frames):
                all_pass = False
            lines.append("")

        # ── Bag list ──────────────────────────────────────────────────────────
        if self._bags:
            lines.append("Bags recorded (ready for postprocess_mesh.py)")
            for bag in self._bags:
                lines.append(f"  {bag}")
            lines.append("")
            lines.append("Run texture pipeline on any level:")
            lines.append(
                "  python3 utils/postprocess_mesh.py "
                "--bag <bag_path> --texture-frames 20"
            )
            lines.append("")

        # ── Final verdict ──────────────────────────────────────────────────────
        lines += [
            "=" * 65,
            f"  Final: {'ALL PASS ✓' if all_pass else 'ONE OR MORE FAILED ✗'}",
            "=" * 65,
        ]

        # ── Diagnostic hints ──────────────────────────────────────────────────
        if not all_pass:
            blurry = [r for r in self._results
                      if r["sharpness"] is not None and not r["sharp_ok"]]
            bad_sync = [r for r in self._results
                        if r["sync_gap_s"] is not None and not r["sync_ok"]]
            if blurry:
                lines.append("\nFocus failures:")
                for r in blurry:
                    lines.append(
                        f"  {r['label']}: sharpness={r['sharpness']:.0f} "
                        f"at {r['alt_m']:.0f}m\n"
                        f"    Check lens, focus ring, or vibration damping.\n"
                        f"    IMX477 fixed-focus is sharp to infinity above ~0.5m."
                    )
            if bad_sync:
                lines.append("\nSync failures:")
                for r in bad_sync:
                    lines.append(
                        f"  {r['label']}: gap={r['sync_gap_s']*1000:.0f}ms "
                        f"at {r['alt_m']:.0f}m\n"
                        f"    ArducamNode must stamp with self.get_clock().now()."
                    )

        text = "\n".join(lines)
        print("\n" + text)
        out = OUTPUT_DIR / "report.txt"
        out.write_text(text + "\n")
        print(f"\n  Report saved → {out}")
        return all_pass

    @staticmethod
    def _print_fov_table(altitudes: list[float]) -> None:
        """Print FOV table during dry run so it is visible without flying."""
        fx, fy, w, h = _load_intrinsics()
        print(f"\n  Camera footprint preview:")
        print(f"  {'Alt':>5}  {'Width':>8}  {'Height':>8}  {'px/m²':>8}")
        print(f"  {'-'*5}  {'-'*8}  {'-'*8}  {'-'*8}")
        for alt_m in altitudes:
            fw, fh    = footprint_at(alt_m)
            px_per_m2 = (w * h) / (fw * fh)
            print(f"  {alt_m:>4.0f}m  {fw:>7.2f}m  {fh:>7.2f}m  {px_per_m2:>7.0f}")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Multi-altitude stationary hover: camera focus, LiDAR sync, FOV, collision zones."
    )
    parser.add_argument(
        "--altitudes", type=float, nargs="+",
        default=DEFAULT_ALTITUDES,
        help=f"Altitude levels in metres above home "
             f"(default: {DEFAULT_ALTITUDES})"
    )
    parser.add_argument(
        "--hold", dest="hold_s", type=float, default=DEFAULT_HOLD_S,
        help=f"Hold time in seconds at each altitude (default {DEFAULT_HOLD_S:.0f})"
    )
    parser.add_argument(
        "--pics", dest="pics_per_level", type=int, default=DEFAULT_PICS,
        help=f"Frames to capture per altitude level (default {DEFAULT_PICS})"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Start services, check sensors, print FOV table, stop before arming"
    )
    parser.add_argument(
        "--no-service-stop", action="store_true",
        help="Skip service management (services already stopped)"
    )
    parser.add_argument(
        "--keep-collision", action="store_true",
        help="Legacy flag — collision avoidance is now always kept active"
    )
    args = parser.parse_args()

    # Validate altitudes — must be ascending, max 15m (L2 reliable range)
    if any(a > 15.0 for a in args.altitudes):
        print("[WARN] Altitudes above 15m approach the Unitree L2 reliable range "
              "limit of 20m — proceed with caution")
    if args.altitudes != sorted(args.altitudes):
        args.altitudes = sorted(args.altitudes)
        print(f"[INFO] Altitudes reordered ascending: {args.altitudes}")

    # Estimated total flight time
    n_levels    = len(args.altitudes)
    climb_est_s = sum(a / 1.5 for a in args.altitudes)   # ~1.5 m/s climb
    total_est_s = climb_est_s + n_levels * args.hold_s + 30   # 30s descent buffer
    total_est_m = total_est_s / 60

    print("=" * 60)
    print("  DronePi — Multi-Altitude Stationary Hover Validation")
    print(f"  Altitudes       : {[f'{a:.0f}m' for a in args.altitudes]}")
    print(f"  Hold per level  : {args.hold_s:.0f}s")
    print(f"  Pics per level  : {args.pics_per_level}")
    print(f"  Levels          : {n_levels}")
    print(f"  Est. flight time: ~{total_est_m:.0f} min")
    print(f"  Dry run         : {'YES' if args.dry_run else 'NO'}")
    collision_mode = ("SKIP (--no-service-stop)" if args.no_service_stop
                     else "watchdog only (collision ACTIVE)")
    print(f"  Service mgmt    : {collision_mode}")
    print("=" * 60)

    if not args.dry_run:
        print(f"\n  *** LIVE FLIGHT — {total_est_m:.0f} min estimated ***")
        print("  Ctrl+C within 5s to abort.")
        for i in range(5, 0, -1):
            print(f"  {i}...", end="\r", flush=True)
            time.sleep(1.0)
        print()

        # Write bench_scan lock BEFORE stopping services and arming.
        # Without this, main.py sees armed+OFFBOARD and enters autonomous mode,
        # launching its own Point-LIO and bag recorder on top of ours.
        try:
            import json as _json
            MISSION_LOCK.write_text(_json.dumps({"mode": "bench_scan"}))
            print(f"  [Lock] bench_scan written → {MISSION_LOCK}")
        except Exception as _e:
            print(f"  [WARN] Could not write mission lock: {_e}")
            print("         main.py may conflict with OFFBOARD arming")

    _svc_list = _SERVICES_WATCHDOG
    svc = ServiceManager(_svc_list)
    if not args.no_service_stop:
        svc.stop_all()
        print("  [INFO] dronepi-main kept running — collision avoidance ACTIVE")
        print("  [INFO] Verify no mission lock: ls /tmp/dronepi_mission.lock")

    plio_proc:    Optional[subprocess.Popen] = None
    arducam_proc: Optional[subprocess.Popen] = None
    flight        = None
    flight_ok     = False

    using_existing_pointlio = False

    try:
        print("\n[Setup 1/3] Checking Point-LIO...")
        # Prefer an already-running standby Point-LIO instance when available.
        # This avoids double-opening /dev/ttyUSB0 during bench tests while still
        # allowing the script to self-start SLAM if nothing is publishing yet.
        if _wait_topic("/aft_mapped_to_init", timeout=2.0):
            using_existing_pointlio = True
            print("  [OK] Reusing already-running Point-LIO on /aft_mapped_to_init")
        else:
            print("  [INFO] No active Point-LIO detected — starting local instance...")
            plio_proc = _launch("Point-LIO", _POINTLIO_CMD, _POINTLIO_LOG)
            print(f"  PID: {plio_proc.pid}   log: tail -f {_POINTLIO_LOG}")
            if not _wait_topic("/aft_mapped_to_init", timeout=30.0):
                print(f"  [FAIL] Point-LIO not publishing after 30s")
                return

        print("\n[Setup 2/3] Starting camera (rpicam-vid direct)...")
        arducam_proc = None   # not used; camera managed by RpicamCapture
        cam_capture  = RpicamCapture(width=1280, height=960, fps=10)
        if not cam_capture.start():
            print("  [WARN] Camera unavailable — capture checks will be skipped")

        print("\n[Setup 3/3] Initialising ROS 2 node...")
        flight    = HoverCameraNode(args, cam_capture)
        flight_ok = flight.run()

        if not args.dry_run:
            flight.write_report()

    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl+C during setup")

    finally:
        if flight is not None:
            flight.shutdown()
        print("\n[Teardown] Stopping test processes...")
        if 'cam_capture' in dir() and cam_capture is not None:
            cam_capture.stop()
        _stop_proc("Point-LIO",   plio_proc)
        if not args.no_service_stop:
            svc.restore_all()
        # Always clear the mission lock so main.py and watchdog resume normally
        try:
            MISSION_LOCK.unlink(missing_ok=True)
            print(f"  [Lock] Cleared {MISSION_LOCK}")
        except Exception:
            pass

    print("\n" + "=" * 60)
    if args.dry_run:
        print("  Dry run complete — services started, sensors verified.")
        print("  Run without --dry-run for live flight.")
        print(f"  Note: live run writes bench_scan lock → {MISSION_LOCK}")
        print("        This prevents main.py from conflicting with OFFBOARD arming.")
    elif flight is not None:
        total   = len(flight._results)
        passed  = sum(1 for r in flight._results if r.get("passed"))
        n_bags  = len(flight._bags)
        print(f"  {passed}/{total} frames passed across {n_bags} bags")
        if passed == total and flight_ok:
            print("  ✓ All levels passed — camera and LiDAR validated.")
            if flight._bags:
                print(f"\n  Run postprocess_mesh.py on any bag:")
                for bag in flight._bags:
                    print(f"    python3 utils/postprocess_mesh.py "
                          f"--bag {bag} --texture-frames 20")
        else:
            print(f"  See report: {OUTPUT_DIR / 'report.txt'}")
    print("=" * 60)


if __name__ == "__main__":
    main()
