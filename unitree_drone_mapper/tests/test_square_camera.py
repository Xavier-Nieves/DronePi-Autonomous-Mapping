#!/usr/bin/env python3
"""
test_square_camera.py — Square pattern camera coverage and LiDAR sync test.

Flies a four-corner square at a single configurable altitude. At each corner
the drone holds position, captures multiple frames, and evaluates focus and
LiDAR timestamp sync. One bag is recorded for the entire flight so the full
square path with its point cloud is available for postprocess_mesh.py.

This test is complementary to test_altitude_validation.py:
  test_altitude_validation.py  — stationary hover at multiple altitudes
  test_square_camera.py        — lateral coverage pattern at one altitude

What it validates
-----------------
  Focus        — Laplacian variance > SHARPNESS_THRESHOLD (80) per frame.
                 Validates the lens stays sharp across lateral positions,
                 not just directly below the takeoff point.

  LiDAR sync   — Camera timestamp gap to LiDAR pose < 45ms per frame.
                 The sync gap may increase at corners if the drone is still
                 settling — a large gap here means vibration is introducing
                 timestamp jitter.

  Coverage     — Computed ground footprint at the test altitude. Shows how
                 much overlap exists between adjacent corner frames.

  Collision    — Zone classification and AGL at each corner when
                 --keep-collision is active.

Flight path  (top-down, body +X = forward)
------------------------------------------
    C2 ──────── C3
    │     ↑      │
    │   home     │    side = --side metres (default 4.0m)
    │            │
    C1 ──────── C4

    Default: 4m × 4m square at 5m altitude, 3 frames per corner.
    At 5m the camera footprint is ~4.2×3.2m so adjacent corners
    have approximately 50% lateral overlap.

Output
------
  tests/square_camera_output/
    corner_C1/
      frame_01.jpg ... frame_03.jpg
    corner_C2/
      frame_01.jpg ... frame_03.jpg
    corner_C3/ ...
    corner_C4/ ...
    report.txt
    bags/
      square_<ts>/    ← single bag for the entire flight

Usage
-----
  # Dry run — starts services, checks sensors, stops before arming
  python3 tests/test_square_camera.py --dry-run

  # Default: 5m altitude, 4m square, 3 frames per corner, 5s hold per corner
  python3 tests/test_square_camera.py

  # Custom
  python3 tests/test_square_camera.py --alt 3 --side 3 --pics 5 --hold 8

  # With collision avoidance active (stops only watchdog, keeps dronepi-main)
  python3 tests/test_square_camera.py --keep-collision

  # Skip service management
  python3 tests/test_square_camera.py --no-service-stop

Service management
------------------
  Default: stops drone-watchdog and dronepi-main.
  --keep-collision: stops only drone-watchdog, leaves dronepi-main running
  so collision_monitor.py publishes obstacle distances and AGL to PX4.

  dronepi-main does NOT launch Point-LIO. This script does. No conflict.

  The bench_scan mission lock is written before arming to prevent main.py
  from entering autonomous mode when it detects armed=True + OFFBOARD.

Safety
------
  AUTO.LAND + disarm on any exception or Ctrl+C.
  Bag is stopped cleanly before emergency land.
  Services and mission lock always restored in finally block.
  RC override kills OFFBOARD immediately.

Prerequisites
-------------
  MAVROS must already be running (mavros.service).
  /tmp/dronepi_mission.lock must be absent or will be overwritten.
"""

from __future__ import annotations

import argparse
import json
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
_ARDUCAM_NODE = _MAPPER_DIR / "nodes" / "arducam_node.py"
_CALIB_PATH   = _MAPPER_DIR / "config" / "camera_calibration.yaml"
OUTPUT_DIR    = _TESTS_DIR / "square_camera_output"
BAG_DIR       = OUTPUT_DIR / "bags"

# ── Mission lock ──────────────────────────────────────────────────────────────
MISSION_LOCK  = Path("/tmp/dronepi_mission.lock")

# ── Camera intrinsics defaults (overridden from calibration YAML at runtime) ──
_DEFAULT_FX = 4805.613
_DEFAULT_FY = 4773.894
_DEFAULT_W  = 4056
_DEFAULT_H  = 3040

# ── Thresholds ────────────────────────────────────────────────────────────────
SHARPNESS_THRESHOLD = 80.0
SYNC_TOLERANCE_S    = 0.045
AGL_TOLERANCE_M     = 1.5

# ── Collision topics ──────────────────────────────────────────────────────────
TOPIC_COLLISION_ZONE = "/dronepi/collision_zone"
TOPIC_AGL_RANGE      = "/mavros/distance_sensor/lidar_down"

# ── Default flight parameters ─────────────────────────────────────────────────
DEFAULT_ALT_M        = 5.0    # metres above home
DEFAULT_SIDE_M       = 4.0    # square side length in metres
DEFAULT_PICS         = 3      # frames per corner
DEFAULT_HOLD_S       = 5.0    # hold time per corner in seconds
CLIMB_TIMEOUT_S      = 25.0
NAV_TIMEOUT_S        = 15.0
ALTITUDE_TOLERANCE   = 0.25
NAV_TOLERANCE        = 0.20
SETTLE_S             = 2.0    # settle time at each corner before first frame

# ── Services ──────────────────────────────────────────────────────────────────
_SERVICES_ALL      = ["drone-watchdog.service", "dronepi-main.service"]
_SERVICES_WATCHDOG = ["drone-watchdog.service"]

# ── Process commands ──────────────────────────────────────────────────────────
_POINTLIO_LOG = Path("/tmp/square_test_pointlio.log")
_ARDUCAM_LOG  = Path("/tmp/square_test_arducam.log")

def _ros_source() -> str:
    return f"source {_ROS_SETUP} && source {_WS_SETUP} && "

_POINTLIO_CMD = _ros_source() + "ros2 launch point_lio mapping_unilidar.launch.py"
_ARDUCAM_CMD  = _ros_source() + f"python3 {_ARDUCAM_NODE}"


# ══════════════════════════════════════════════════════════════════════════════
# Camera helpers
# ══════════════════════════════════════════════════════════════════════════════

def _load_intrinsics() -> tuple[float, float, int, int]:
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
    Ground footprint (width_m, height_m) at alt_m using pinhole model.
    footprint_w = (image_width / fx) * altitude
    """
    fx, fy, w, h = _load_intrinsics()
    return (w / fx) * alt_m, (h / fy) * alt_m


# ══════════════════════════════════════════════════════════════════════════════
# ServiceManager
# ══════════════════════════════════════════════════════════════════════════════

class ServiceManager:
    def __init__(self, services: list[str]) -> None:
        self._services   = services
        self._was_active: dict[str, bool] = {}

    def stop_all(self) -> None:
        print("\n[Services] Pausing services...")
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
        print("\n[Services] Restoring services...")
        subprocess.run(["sudo", "systemctl", "daemon-reload"], check=False)
        for svc in self._services:
            if self._was_active.get(svc, False):
                subprocess.run(["sudo", "systemctl", "start", svc], check=False)
                print(f"  started : {svc}")
            else:
                print(f"  left stopped: {svc}")


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
    while time.time() < deadline:
        r = subprocess.run(
            ["bash", "-c",
             _ros_source() + f"timeout 3 ros2 topic hz {topic} --window 5 2>&1"],
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


# ══════════════════════════════════════════════════════════════════════════════
# Bag recorder — single bag for the whole flight
# ══════════════════════════════════════════════════════════════════════════════

class BagRecorder:
    """
    Records one ros2 bag for the entire square flight.

    Unlike the altitude validation test which uses one bag per level,
    the square test uses a single bag covering the full path — this
    makes it directly usable as a survey bag for postprocess_mesh.py.
    """

    TOPICS = [
        "/arducam/image_raw",
        "/aft_mapped_to_init",
        "/cloud_registered",
        "/mavros/local_position/pose",
    ]

    def __init__(self) -> None:
        self._proc: Optional[subprocess.Popen] = None
        self._path: Optional[Path]             = None

    def start(self) -> Path:
        BAG_DIR.mkdir(parents=True, exist_ok=True)
        ts       = time.strftime("%Y%m%d_%H%M%S")
        bag_name = f"square_{ts}"
        bag_path = BAG_DIR / bag_name

        topics_str = " ".join(self.TOPICS)
        cmd = (
            _ros_source()
            + f"ros2 bag record {topics_str} "
            + f"-o {bag_path} --storage sqlite3"
        )
        log_path    = Path("/tmp/square_test_bag.log")
        self._proc  = _launch("BagRecorder", cmd, log_path)
        self._path  = bag_path
        time.sleep(2.0)
        print(f"  Recording → {bag_path.name}")
        return bag_path

    def stop(self) -> Optional[Path]:
        if self._proc is not None:
            _stop_proc("BagRecorder", self._proc)
            self._proc = None
            time.sleep(2.0)
            print(f"  Bag closed : {self._path.name if self._path else 'unknown'}")
        return self._path

    @property
    def path(self) -> Optional[Path]:
        return self._path


# ══════════════════════════════════════════════════════════════════════════════
# ROS 2 flight node
# ══════════════════════════════════════════════════════════════════════════════

class SquareCameraNode:
    """
    Executes the square pattern and captures frames at each corner.

    Four corners at (±half, ±half) relative to home, all at the same
    altitude. Visits C1→C2→C3→C4 in sequence, captures N frames per
    corner with a settling pause before the first frame.
    """

    CORNERS = [
        ("C1", -1, -1),
        ("C2", -1, +1),
        ("C3", +1, +1),
        ("C4", +1, -1),
    ]

    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos  import (QoSProfile, ReliabilityPolicy,
                                 HistoryPolicy, DurabilityPolicy)
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg   import State
        from mavros_msgs.srv   import CommandBool, SetMode
        from nav_msgs.msg      import Odometry
        from sensor_msgs.msg   import Image, Range
        from std_msgs.msg      import String

        self._rclpy = rclpy
        self._args  = args
        rclpy.init()

        class _Inner(Node):
            def __init__(inner_self):
                super().__init__("square_camera_test")

                reliable = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST, depth=10)
                sensor = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST, depth=5,
                    durability=DurabilityPolicy.VOLATILE)

                inner_self.fcu_state:      Optional[State]       = None
                inner_self.local_pose:     Optional[PoseStamped] = None
                inner_self.last_image:     Optional[Image]       = None
                inner_self.lidar_ts:       Optional[float]       = None
                inner_self.collision_zone: Optional[str]         = None
                inner_self.agl_m:          Optional[float]       = None
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
                    Image, "/arducam/image_raw",
                    inner_self._cb_image, sensor)
                inner_self.create_subscription(
                    String, TOPIC_COLLISION_ZONE,
                    inner_self._cb_zone, reliable)
                inner_self.create_subscription(
                    Range, TOPIC_AGL_RANGE,
                    inner_self._cb_agl, sensor)

                inner_self._sp_pub   = inner_self.create_publisher(
                    PoseStamped, "/mavros/setpoint_position/local", reliable)
                inner_self._arm_cli  = inner_self.create_client(
                    CommandBool, "/mavros/cmd/arming")
                inner_self._mode_cli = inner_self.create_client(
                    SetMode, "/mavros/set_mode")

            def _cb_lidar(inner_self, msg):
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                with inner_self._lock:
                    inner_self.lidar_ts = ts

            def _cb_image(inner_self, msg):
                with inner_self._lock:
                    inner_self.last_image = msg

            def _cb_zone(inner_self, msg):
                with inner_self._lock:
                    inner_self.collision_zone = msg.data.strip()

            def _cb_agl(inner_self, msg):
                import math as _m
                with inner_self._lock:
                    if not _m.isnan(msg.range) and msg.range > 0:
                        inner_self.agl_m = float(msg.range)

        self._node     = _Inner()
        self._results: list[dict] = []
        self._bag_path: Optional[Path] = None
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

    # ── FCU helpers ───────────────────────────────────────────────────────────

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
                tol: float = NAV_TOLERANCE,
                timeout: float = NAV_TIMEOUT_S) -> bool:
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

    def _wait_new_frame(self, timeout: float = 3.0) -> Optional[object]:
        """Wait for a frame newer than the currently cached one."""
        with self._node._lock:
            pre_ts = (
                self._node.last_image.header.stamp.sec
                + self._node.last_image.header.stamp.nanosec * 1e-9
                if self._node.last_image else 0.0
            )
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._spin(0.05)
            with self._node._lock:
                img = self._node.last_image
            if img is not None:
                ts = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9
                if ts > pre_ts:
                    return img
        return None

    def capture_frame(
        self, corner: str, frame_idx: int, z_target: float,
        cx: float, cy: float,
    ) -> dict:
        """
        Capture one frame at a corner, evaluate focus, sync, and collision.

        Args:
            corner:     Corner label, e.g. "C1"
            frame_idx:  1-indexed frame number within this corner's hold
            z_target:   Target Z in map frame
            cx, cy:     Corner X,Y in map frame (for setpoint streaming)
        """
        import cv2
        import numpy as np

        label  = f"{corner}_f{frame_idx:02d}"
        result = dict(
            corner=corner, frame_idx=frame_idx, label=label,
            sharpness=None, sharp_ok=False,
            sync_gap_s=None, sync_ok=False,
            image_path=None, passed=False,
            collision_zone=None, agl_m=None, agl_ok=None,
        )

        frame = self._wait_new_frame(timeout=4.0)
        if frame is None:
            print(f"    [FAIL] No new frame (timeout)")
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
            print(f"    Sync gap    : {gap*1000:.1f}ms  {icon}")
        else:
            print("    [WARN] No LiDAR timestamp — sync skipped")

        # ── Focus ─────────────────────────────────────────────────────────────
        img = self._decode(frame)
        if img is not None:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            lap  = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            result["sharpness"] = lap
            result["sharp_ok"]  = lap >= SHARPNESS_THRESHOLD
            icon = "✓" if result["sharp_ok"] else "✗  (blurry)"
            print(f"    Sharpness   : {lap:.1f}  {icon}")

            out_dir = OUTPUT_DIR / f"corner_{corner}"
            out_dir.mkdir(parents=True, exist_ok=True)
            out = out_dir / f"frame_{frame_idx:02d}.jpg"
            cv2.imwrite(str(out), img)
            result["image_path"] = str(out)
            print(f"    Saved       : corner_{corner}/frame_{frame_idx:02d}.jpg")
        else:
            print(f"    [FAIL] Cannot decode frame (encoding='{frame.encoding}')")

        # ── Collision snapshot ────────────────────────────────────────────────
        with self._node._lock:
            zone  = self._node.collision_zone
            agl_m = self._node.agl_m

        if zone is not None:
            result["collision_zone"] = zone
            icon = "✓" if zone == "CLEAR" else "⚠"
            print(f"    Collision   : {zone}  {icon}")

        if agl_m is not None:
            result["agl_m"] = agl_m
            agl_err = abs(agl_m - self._args.alt)
            result["agl_ok"] = agl_err < AGL_TOLERANCE_M
            icon = "✓" if result["agl_ok"] else "✗"
            print(f"    AGL         : {agl_m:.2f}m  "
                  f"(Δ{agl_err:.2f}m commanded {self._args.alt:.1f}m)  {icon}")

        result["passed"] = result["sharp_ok"] and (
            result["sync_ok"] if result["sync_gap_s"] is not None else True
        )
        return result

    @staticmethod
    def _decode(msg) -> Optional[object]:
        import cv2
        import numpy as np
        enc = msg.encoding.lower()
        h, w = int(msg.height), int(msg.width)
        raw  = bytes(msg.data)
        if enc == "bgr8":
            return np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3).copy()
        if enc == "rgb8":
            rgb = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if enc == "mono8":
            gray = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return None

    # ── Main flight sequence ──────────────────────────────────────────────────

    def run(self, bag: BagRecorder) -> bool:
        """
        Fly the square, capture frames at each corner, return to home, land.

        The bag recorder is passed in and remains active for the full flight,
        capturing the complete square path as one contiguous dataset.
        """
        args   = self._args
        half   = args.side / 2.0
        alt    = args.alt
        n_pics = args.pics
        hold_s = args.hold
        home_z = 0.0

        # Scale corners by half-side
        corners = [(n, cx * half, cy * half) for n, cx, cy in self.CORNERS]

        # Interval between frames, with settling at start
        pic_interval = (hold_s - SETTLE_S) / max(n_pics, 1)

        bag_proc = None

        def _emergency_land():
            bag.stop()
            self.set_mode("AUTO.LAND")
            time.sleep(3.0)
            self.disarm()

        try:
            # ── [1] FCU connection ────────────────────────────────────────────
            print("\n[1/6] Waiting for FCU connection...")
            deadline = time.time() + 20.0
            while not self.connected and time.time() < deadline:
                self._spin(0.5)
            if not self.connected:
                print("  [FAIL] FCU not connected")
                return False
            print("  [OK] FCU connected")

            # ── [2] Home Z ────────────────────────────────────────────────────
            for _ in range(30):
                self._spin(0.1)
            home_z   = self._z()
            target_z = home_z + alt
            print(f"  Home Z={home_z:.3f}m  target={target_z:.3f}m")

            # ── [3] Sensor liveness ───────────────────────────────────────────
            print("\n[2/6] Sensor liveness check...")
            deadline = time.time() + 10.0
            cam = lidar = False
            while time.time() < deadline:
                self._spin(0.2)
                with self._node._lock:
                    cam   = self._node.last_image is not None
                    lidar = self._node.lidar_ts   is not None
                if cam and lidar:
                    break

            print(f"  Camera  : {'✓ OK' if cam   else '✗ NOT PUBLISHING'}")
            print(f"  LiDAR   : {'✓ OK' if lidar else '✗ NOT PUBLISHING'}")

            if not cam and not args.dry_run:
                print("  [FAIL] Camera not publishing — aborting")
                return False

            # ── [4] Pre-stream ────────────────────────────────────────────────
            print("\n[3/6] Pre-streaming setpoints (3s)...")
            self._stream(0.0, 0.0, home_z, dur=3.0)
            print("  [OK] Setpoint stream established")

            if args.dry_run:
                fw, fh = footprint_at(alt)
                print(f"\n  --dry-run: stopping before arm.")
                print(f"  Footprint at {alt}m: {fw:.2f}m × {fh:.2f}m")
                print(f"  Square side {args.side}m — corner separation {args.side:.1f}m")
                overlap_pct = max(0.0, (fw - args.side) / fw * 100)
                print(f"  Lateral overlap estimate: {overlap_pct:.0f}%")
                return True

            # ── [5] Arm + OFFBOARD ────────────────────────────────────────────
            print("\n[4/6] Arming...")
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

            # ── [6] Climb ─────────────────────────────────────────────────────
            print(f"\n[5/6] Climbing to {alt}m (target Z={target_z:.2f}m)...")
            if not self._fly_to(0.0, 0.0, target_z,
                                tol=ALTITUDE_TOLERANCE,
                                timeout=CLIMB_TIMEOUT_S):
                print(f"  [WARN] Climb timeout — at {self._z()-home_z:.1f}m, continuing")
            else:
                print(f"  [OK] Altitude: {self._z()-home_z:.1f}m")

            # ── [7] Square pattern ────────────────────────────────────────────
            fw, fh = footprint_at(alt)
            print(f"\n[6/6] Square pattern  "
                  f"side={args.side}m  alt={alt}m  "
                  f"footprint={fw:.1f}×{fh:.1f}m  "
                  f"{n_pics} frames/corner  {hold_s:.0f}s hold...")

            for name, cx, cy in corners:
                print(f"\n  ─── Corner {name}  ({cx:+.2f}, {cy:+.2f}) ───")

                # Navigate to corner
                reached = self._fly_to(cx, cy, target_z, timeout=NAV_TIMEOUT_S)
                if not reached:
                    print(f"  [WARN] Did not fully converge to {name}")

                # Settling pause
                self._stream(cx, cy, target_z, dur=SETTLE_S)

                # Capture N frames, spaced across the hold period
                for pic_idx in range(1, n_pics + 1):
                    print(f"\n    Frame {pic_idx}/{n_pics}")
                    r = self.capture_frame(name, pic_idx, target_z, cx, cy)
                    self._results.append(r)
                    print(f"    → {'PASS' if r['passed'] else 'FAIL'}")

                    # Stream setpoints between frames
                    if pic_idx < n_pics:
                        self._stream(cx, cy, target_z, dur=pic_interval)

                # Remaining hold time after last frame
                remaining = hold_s - SETTLE_S - n_pics * pic_interval
                if remaining > 0:
                    self._stream(cx, cy, target_z, dur=remaining)

            # ── Return + land ─────────────────────────────────────────────────
            print("\n  Returning to home and landing...")
            self._fly_to(0.0, 0.0, target_z, tol=0.30, timeout=15.0)
            self._stream(0.0, 0.0, target_z, dur=2.0)

            # Stop bag before AUTO.LAND so the bag closes cleanly
            bag.stop()
            self._bag_path = bag.path

            self.set_mode("AUTO.LAND")
            deadline = time.time() + 40.0
            while self.armed and time.time() < deadline:
                self._spin(0.3)
            if not self.armed:
                print("  [OK] Landed and disarmed")
            else:
                print("  [WARN] Still armed — disarming manually")
                self.disarm()

            return True

        except KeyboardInterrupt:
            print("\n[ABORT] Ctrl+C — emergency landing")
            _emergency_land()
            return False

        except Exception as exc:
            import traceback
            print(f"\n[FAIL] Exception: {exc}")
            traceback.print_exc()
            _emergency_land()
            return False

    def shutdown(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()

    # ── Report ────────────────────────────────────────────────────────────────

    def write_report(self) -> bool:
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        args   = self._args
        fw, fh = footprint_at(args.alt)
        overlap = max(0.0, (fw - args.side) / fw * 100)

        lines = [
            "=" * 65,
            "  DronePi — Square Camera Coverage Test Report",
            f"  {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"  Altitude: {args.alt}m   Side: {args.side}m   "
            f"Pics/corner: {args.pics}   Hold: {args.hold}s",
            f"  Footprint: {fw:.2f}×{fh:.2f}m   "
            f"Lateral overlap: ~{overlap:.0f}%",
            "=" * 65, "",
        ]

        if self._bag_path:
            lines += [
                f"Bag: {self._bag_path}",
                "Run pipeline:",
                f"  python3 utils/postprocess_mesh.py "
                f"--bag {self._bag_path} --texture-frames 20",
                "",
            ]

        all_pass = True
        for corner_name, _, _ in self.CORNERS:
            frames = [r for r in self._results if r["corner"] == corner_name]
            if not frames:
                lines.append(f"Corner {corner_name}: no data")
                all_pass = False
                continue

            pass_count = sum(1 for r in frames if r["passed"])
            lines.append(f"Corner {corner_name}")
            lines.append(f"  Passed   : {pass_count}/{len(frames)}")

            sharp_vals = [r["sharpness"] for r in frames if r["sharpness"] is not None]
            if sharp_vals:
                lines.append(
                    f"  Sharpness: avg={sum(sharp_vals)/len(sharp_vals):.0f}  "
                    f"min={min(sharp_vals):.0f}  max={max(sharp_vals):.0f}"
                )
            sync_vals = [r["sync_gap_s"]*1000 for r in frames
                         if r["sync_gap_s"] is not None]
            if sync_vals:
                lines.append(
                    f"  Sync gap : avg={sum(sync_vals)/len(sync_vals):.1f}ms  "
                    f"max={max(sync_vals):.1f}ms  "
                    f"(limit {SYNC_TOLERANCE_S*1000:.0f}ms)"
                )
            zones = [r["collision_zone"] for r in frames if r["collision_zone"]]
            if zones:
                zone_counts = {z: zones.count(z) for z in set(zones)}
                lines.append(
                    f"  Zones    : "
                    + "  ".join(f"{z}×{n}" for z, n in sorted(zone_counts.items()))
                )
            agl_vals = [r["agl_m"] for r in frames if r["agl_m"] is not None]
            if agl_vals:
                lines.append(
                    f"  AGL      : avg={sum(agl_vals)/len(agl_vals):.2f}m  "
                    f"(commanded {args.alt:.1f}m)"
                )

            for r in frames:
                sharp_s = f"{r['sharpness']:.0f}" if r["sharpness"] else "—"
                sync_s  = (f"{r['sync_gap_s']*1000:.1f}ms"
                           if r["sync_gap_s"] is not None else "—")
                zone_s  = r["collision_zone"] or "—"
                status  = "✓" if r["passed"] else "✗"
                lines.append(
                    f"  Frame {r['frame_idx']:02d}: {status}  "
                    f"sharp={sharp_s:>6}  sync={sync_s:>8}  zone={zone_s}"
                )
            lines.append("")
            if pass_count < len(frames):
                all_pass = False

        lines += [
            "=" * 65,
            f"  Final: {'ALL PASS ✓' if all_pass else 'ONE OR MORE FAILED ✗'}",
            "=" * 65,
        ]

        text = "\n".join(lines)
        print("\n" + text)
        out = OUTPUT_DIR / "report.txt"
        out.write_text(text + "\n")
        print(f"\n  Report saved → {out}")
        return all_pass


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Square pattern camera coverage + LiDAR sync test."
    )
    parser.add_argument("--alt",  type=float, default=DEFAULT_ALT_M,
                        help=f"Altitude above home in metres (default {DEFAULT_ALT_M})")
    parser.add_argument("--side", type=float, default=DEFAULT_SIDE_M,
                        help=f"Square side length in metres (default {DEFAULT_SIDE_M})")
    parser.add_argument("--pics", type=int, default=DEFAULT_PICS,
                        help=f"Frames per corner (default {DEFAULT_PICS})")
    parser.add_argument("--hold", type=float, default=DEFAULT_HOLD_S,
                        help=f"Hold time per corner in seconds (default {DEFAULT_HOLD_S})")
    parser.add_argument("--dry-run",         action="store_true",
                        help="Start services, check sensors, stop before arming")
    parser.add_argument("--keep-collision",  action="store_true",
                        help="Stop only watchdog, keep dronepi-main for collision avoidance")
    parser.add_argument("--no-service-stop", action="store_true",
                        help="Skip service management entirely")
    args = parser.parse_args()

    fw, fh   = footprint_at(args.alt)
    overlap  = max(0.0, (fw - args.side) / fw * 100)
    # Estimated time: climb + 4 corners × (nav ~5s + settle + hold) + descent
    est_s    = (args.alt / 1.5) + 4 * (5 + SETTLE_S + args.hold) + 20
    est_min  = est_s / 60

    coll_mode = ("SKIP" if args.no_service_stop
                 else "watchdog only (collision ACTIVE)" if args.keep_collision
                 else "full (watchdog + main stopped)")

    print("=" * 60)
    print("  DronePi — Square Camera Coverage Test")
    print(f"  Altitude        : {args.alt}m")
    print(f"  Square side     : {args.side}m")
    print(f"  Footprint       : {fw:.2f}×{fh:.2f}m  (overlap ~{overlap:.0f}%)")
    print(f"  Pics per corner : {args.pics}")
    print(f"  Hold per corner : {args.hold}s")
    print(f"  Est. flight time: ~{est_min:.0f} min")
    print(f"  Dry run         : {'YES' if args.dry_run else 'NO'}")
    print(f"  Service mgmt    : {coll_mode}")
    print("=" * 60)

    if not args.dry_run:
        print(f"\n  *** LIVE FLIGHT — ~{est_min:.0f} min ***")
        print("  Ctrl+C within 5s to abort.")
        for i in range(5, 0, -1):
            print(f"  {i}...", end="\r", flush=True)
            time.sleep(1.0)
        print()

        # Write bench_scan lock before arming — prevents main.py from entering
        # autonomous mode when it detects armed=True + OFFBOARD
        try:
            MISSION_LOCK.write_text(json.dumps({"mode": "bench_scan"}))
            print(f"  [Lock] bench_scan written → {MISSION_LOCK}")
        except Exception as e:
            print(f"  [WARN] Could not write mission lock: {e}")

    _svc_list = _SERVICES_WATCHDOG if args.keep_collision else _SERVICES_ALL
    svc = ServiceManager(_svc_list)
    if not args.no_service_stop:
        svc.stop_all()
        if args.keep_collision:
            print("  [INFO] dronepi-main running — collision avoidance ACTIVE")
            print("  [INFO] Verify: ls /tmp/dronepi_mission.lock → should show bench_scan")

    plio_proc:    Optional[subprocess.Popen] = None
    arducam_proc: Optional[subprocess.Popen] = None
    flight        = None
    flight_ok     = False

    try:
        print("\n[Setup 1/3] Starting Point-LIO...")
        plio_proc = _launch("Point-LIO", _POINTLIO_CMD, _POINTLIO_LOG)
        print(f"  PID: {plio_proc.pid}   log: tail -f {_POINTLIO_LOG}")
        if not _wait_topic("/aft_mapped_to_init", timeout=30.0):
            print("  [FAIL] Point-LIO not publishing after 30s")
            return

        print("\n[Setup 2/3] Starting ArducamNode...")
        if not _ARDUCAM_NODE.exists():
            print(f"  [FAIL] Not found: {_ARDUCAM_NODE}")
            return
        arducam_proc = _launch("ArducamNode", _ARDUCAM_CMD, _ARDUCAM_LOG)
        print(f"  PID: {arducam_proc.pid}   log: tail -f {_ARDUCAM_LOG}")
        if not _wait_topic("/arducam/image_raw", timeout=15.0):
            print("  [FAIL] ArducamNode not publishing after 15s")
            return

        print("\n[Setup 3/3] Initialising ROS 2 node...")
        flight = SquareCameraNode(args)

        if args.dry_run:
            flight_ok = flight.run(BagRecorder())  # bag never started in dry run
        else:
            bag       = BagRecorder()
            bag.start()
            flight_ok = flight.run(bag)

        if not args.dry_run:
            flight.write_report()

    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl+C during setup")

    finally:
        if flight is not None:
            flight.shutdown()
        print("\n[Teardown] Stopping test processes...")
        _stop_proc("ArducamNode", arducam_proc)
        _stop_proc("Point-LIO",   plio_proc)
        if not args.no_service_stop:
            svc.restore_all()
        try:
            MISSION_LOCK.unlink(missing_ok=True)
            print(f"  [Lock] Cleared {MISSION_LOCK}")
        except Exception:
            pass

    print("\n" + "=" * 60)
    if args.dry_run:
        print("  Dry run complete — services started, sensors verified.")
        print(f"  Footprint at {args.alt}m: {fw:.2f}×{fh:.2f}m")
        print(f"  Lateral overlap at {args.side}m side: ~{overlap:.0f}%")
        print("  Run without --dry-run for live flight.")
    elif flight is not None:
        total  = len(flight._results)
        passed = sum(1 for r in flight._results if r.get("passed"))
        print(f"  {passed}/{total} frames passed across 4 corners")
        if passed == total and flight_ok:
            print("  ✓ All corners passed — camera coverage validated.")
            if flight._bag_path:
                print(f"\n  Run pipeline:")
                print(f"    python3 utils/postprocess_mesh.py "
                      f"--bag {flight._bag_path} --texture-frames 20")
        else:
            print(f"  See: {OUTPUT_DIR / 'report.txt'}")
    print("=" * 60)


if __name__ == "__main__":
    main()
