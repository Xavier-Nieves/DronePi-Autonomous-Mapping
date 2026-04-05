#!/usr/bin/env python3
"""main.py — DronePi Flight Mode Orchestrator.

Started automatically by the dronepi-main systemd service on boot.
Monitors drone state and routes to the correct flight mode via a
four-state machine. Coordinates with drone_watchdog.py via a shared
lock file at /tmp/dronepi_mission.lock.

State Machine
-------------

  IDLE
    LiDAR absent + armed        → MODE 1  (no scan)
    LiDAR present + armed       → DEBOUNCE (10 s timer starts)

  DEBOUNCE  (10 seconds)
    OFFBOARD detected           → MODE 3  (autonomous scan)   [lock written]
    10 s elapsed, no OFFBOARD   → MODE 2  (manual scan)       [lock written]
    Disarm during debounce      → IDLE    (cancelled)

  MODE 1 — no scan, locked until disarm
    Logs state only; no stack launched.
    Disarm                      → IDLE

  MODE 2 — manual scan, locked until disarm
    Writes lock: manual_scan
    Watchdog reads lock → starts Point-LIO + bag recorder
    OFFBOARD detected mid-flight → IGNORED (mode is locked)
    Disarm                      → watchdog stops bag + triggers postflight → IDLE

  MODE 3 — autonomous scan, locked until disarm
    Writes lock: autonomous
    main.py launches Point-LIO + SLAM bridge + bag recorder + camera
    Executes QGC waypoints with LiDAR gap fill + IMX477 capture per waypoint
    OFFBOARD lost               → mission paused, RC has control
      Restored within 30 s      → mission resumes from current waypoint
      Not restored after 30 s   → AUTO.RTL
    Disarm                      → watchdog stops bag + triggers postflight → IDLE

Lock File
---------
  /tmp/dronepi_mission.lock  — JSON written when a mode is committed.
    manual_scan   watchdog starts stack; main.py monitors only
    autonomous    watchdog yields; main.py owns the stack
    absent        watchdog operates in default RC-toggle mode

Camera (MODE 3 only)
--------------------
  CameraCapture (flight/camera_capture.py) runs as a background thread.
  One JPEG + sidecar JSON is saved per waypoint arrival.
  Output: /mnt/ssd/flights/<session>/frame_NNNN.{jpg,json} + capture_log.csv
  If Picamera2 is unavailable (e.g. camera not connected), the mission
  continues without captures — camera failure is non-fatal.

Logging
-------
  All output captured by journalctl:
    sudo journalctl -u dronepi-main -f
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from enum import Enum
from pathlib import Path

# ── Project Paths ─────────────────────────────────────────────────────────────

PROJECT_ROOT = Path(__file__).parent
FLIGHT_DIR   = PROJECT_ROOT / "flight"
UTILS_DIR    = PROJECT_ROOT / "utils"

ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)

BRIDGE_SCRIPT       = FLIGHT_DIR / "_slam_bridge.py"
COLLISION_SCRIPT    = FLIGHT_DIR / "collision_monitor.py"
CAMERA_SCRIPT       = FLIGHT_DIR / "camera_capture.py"
POSTFLIGHT_SCRIPT   = UTILS_DIR  / "run_postflight.py"
HAILO_FLIGHT_SCRIPT = PROJECT_ROOT / "hailo" / "hailo_flight_node.py"
LIDAR_PORT          = Path("/dev/ttyUSB0")
HAILO_DEVICE      = Path("/dev/hailo0")
MISSION_LOCK      = Path("/tmp/dronepi_mission.lock")
HAILO_LOCK        = Path("/tmp/dronepi_hailo.lock")
ROSBAG_DIR        = Path("/mnt/ssd/rosbags")
FLIGHT_IMG_DIR    = Path("/mnt/ssd/flights")

# ── Constants ─────────────────────────────────────────────────────────────────

DEBOUNCE_S        = 10      # Seconds after arm to wait before committing a mode
HOME_TIMEOUT      = 30.0    # Seconds to wait for GPS home position (non-blocking — WARN only)
WP_TIMEOUT        = 15.0    # Seconds to wait for QGC waypoints
EKF_STABLE_COUNT  = 20      # Consecutive stable Z readings required for EKF pass
EKF_TIMEOUT       = 40.0    # Seconds to wait for EKF stability
OFFBOARD_RESUME_S = 30      # Seconds to wait for OFFBOARD restore before RTL
POLL_HZ           = 2       # State machine poll rate
SETPOINT_HZ       = 20      # OFFBOARD setpoint publish rate (PX4 requires > 2 Hz)
POINTLIO_INIT_S   = 5       # Fixed sleep fallback — topic verification preferred
BRIDGE_INIT_S     = 2       # Fixed sleep fallback — topic verification preferred
COLLISION_INIT_S  = 2       # Seconds to wait for collision monitor initialisation
CAMERA_INIT_S     = 2       # Seconds to allow AEC/AWB convergence after camera start
GRACEFUL_KILL_S   = 5       # Seconds before SIGKILL after SIGINT
DEFAULT_FOV_DEG   = 70.0    # Diagonal camera FOV fallback (degrees)
DEFAULT_MIN_DENS  = 5       # Minimum LiDAR points per grid cell before gap fill
WP_TOLERANCE_M    = 0.5     # Waypoint arrival tolerance (metres)

# SLAM chain verification — replaces fixed sleeps after process launch
SLAM_TOPIC_TIMEOUT_S  = 30.0  # Max seconds to wait for /aft_mapped_to_init
BRIDGE_TOPIC_TIMEOUT_S = 15.0 # Max seconds to wait for /mavros/vision_pose/pose
SLAM_MIN_HZ           = 5.0   # Minimum acceptable publish rate (Hz) for SLAM topic
BRIDGE_MIN_HZ         = 8.0   # Minimum acceptable publish rate (Hz) for vision pose

# Hailo in-flight integration
HAILO_STARTUP_TIMEOUT_S = 20.0  # Max seconds to wait for Hailo node to publish
HAILO_FLOW_TOPIC        = "/hailo/optical_flow"
HAILO_GROUND_TOPIC      = "/hailo/ground_class"
HAILO_ENV               = os.path.expanduser("~/hailo_inference_env/bin/python3")

# Velocity scale factors applied by fly_to() based on collision zone status.
SPEED_SCALE_CLEAR    = 1.0
SPEED_SCALE_CAUTION  = 0.5
SPEED_SCALE_OBSTACLE = 0.2

BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
    "/mavros/distance_sensor/lidar_down",
    "/mavros/vision_pose/pose",       # SLAM bridge output — EKF2 fusion input
    "/hailo/optical_flow",            # Hailo velocity output — logged when active
    "/hailo/ground_class",            # Hailo ground classifier — logged when active
]

# ── Flight Modes ──────────────────────────────────────────────────────────────

class FlightMode(Enum):
    IDLE       = "IDLE"
    DEBOUNCE   = "DEBOUNCE"
    NO_SCAN    = "NO_SCAN"
    MANUAL     = "MANUAL_SCAN"
    AUTONOMOUS = "AUTONOMOUS"

# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def log_mode(mode: FlightMode, detail: str = "") -> None:
    sep = f"  {detail}" if detail else ""
    log(f"[{mode.value}]{sep}")


# ── Main Status Writer ───────────────────────────────────────────────────────
# main.py does NOT touch GPIO. It writes system status to /tmp/main_status.json.
# led_service.py observes this file (and watchdog_status.json + mission lock)
# and derives the correct LED state autonomously.
#
# To add a new LED-visible condition from main.py:
#   1. Add a field to the payload dict in _write_main_status() below.
#   2. Read it in led_service.py _read_snapshot() and _derive_state().
#   3. Add a _Pattern entry and priority rule in led_service.py.

_MAIN_STATUS_FILE     = "/tmp/main_status.json"
_MAIN_STATUS_FILE_TMP = _MAIN_STATUS_FILE + ".tmp"


def _write_main_status(
    hailo_active:   bool = False,
    hailo_degraded: bool = False,
    hailo_failed:   bool = False,
) -> None:
    """
    Write main.py heartbeat and Hailo status to /tmp/main_status.json.

    Must be called on every poll cycle during autonomous flight so
    led_service.py sees a live timestamp. If the timestamp goes stale
    beyond MAIN_DEAD_S (8s) while autonomous lock is active, led_service.py
    transitions to MAIN_DEAD.

    Parameters
    ----------
    hailo_active   : Hailo node publishing and augmenting EKF2
    hailo_degraded : FlowBridge degraded — consecutive rejection threshold exceeded
    hailo_failed   : Hailo process exited unexpectedly mid-flight
    """
    try:
        payload = json.dumps({
            "ts":            time.time(),
            "hailo_active":  hailo_active,
            "hailo_degraded":hailo_degraded,
            "hailo_failed":  hailo_failed,
        })
        with open(_MAIN_STATUS_FILE_TMP, "w") as f:
            f.write(payload)
        os.replace(_MAIN_STATUS_FILE_TMP, _MAIN_STATUS_FILE)
    except Exception as exc:
        log(f"[STATUS] Main status write failed: {exc}")


def _clear_main_status() -> None:
    """Remove main status file on clean shutdown."""
    try:
        Path(_MAIN_STATUS_FILE).unlink(missing_ok=True)
        Path(_MAIN_STATUS_FILE_TMP).unlink(missing_ok=True)
    except Exception:
        pass


# ── Lock File ─────────────────────────────────────────────────────────────────

def write_lock(mode: str, extra: dict = None) -> None:
    payload = {"mode": mode, "started_at": datetime.now().isoformat()}
    if extra:
        payload.update(extra)
    MISSION_LOCK.write_text(json.dumps(payload))
    log(f"Lock written: {mode}")


def clear_lock() -> None:
    if MISSION_LOCK.exists():
        MISSION_LOCK.unlink()
        log("Lock cleared — watchdog resuming normal operation")

# ── Process Management ────────────────────────────────────────────────────────

def start_proc(name: str, cmd: str) -> subprocess.Popen:
    log(f"Starting {name}...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log(f"{name} started (PID {proc.pid})")
    return proc


def stop_proc(name: str, proc: subprocess.Popen) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly")
    except subprocess.TimeoutExpired:
        log(f"{name} did not exit — sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    except ProcessLookupError:
        pass

# ── LiDAR Detection ───────────────────────────────────────────────────────────

def lidar_present() -> bool:
    return LIDAR_PORT.exists()

# ── Coordinate Conversion ─────────────────────────────────────────────────────

def haversine_to_enu(
    lat: float, lon: float, alt: float,
    hlat: float, hlon: float, halt: float,
) -> tuple:
    """Convert GPS (lat, lon, alt) to local ENU metres relative to home.

    Uses the flat-earth approximation, accurate to < 1 mm for distances
    under 10 km — sufficient for all survey mission scales.

    Returns:
        (east, north, up) in metres.
    """
    R    = 6371000.0
    dlat = math.radians(lat  - hlat)
    dlon = math.radians(lon  - hlon)
    lm   = math.radians((lat + hlat) / 2.0)
    return dlon * R * math.cos(lm), dlat * R, alt - halt

# ── ROS Node ──────────────────────────────────────────────────────────────────

class MainNode:
    """Minimal ROS 2 node shared across all flight mode handlers.

    Provides thread-safe access to drone state, position, home position,
    GPS fix, and QGC waypoints. Also implements setpoint publishing and
    mode switching for use during MODE 3 autonomous execution.
    """

    def __init__(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import State, WaypointList, HomePosition
            from mavros_msgs.srv import SetMode
            from sensor_msgs.msg import NavSatFix
            from std_msgs.msg import String
            from geometry_msgs.msg import TwistStamped
        except ImportError as exc:
            log(f"[FAIL] ROS 2 not sourced: {exc}")
            sys.exit(1)

        self._rclpy = rclpy
        self._PS    = PoseStamped
        self._lock  = threading.Lock()

        self._state          = State()
        self._pose           = None
        self._home           = None
        self._waypoints      = []
        self._gps_fix        = None   # sensor_msgs/NavSatFix — live GPS position
        self._collision_zone = "CLEAR"
        self.vision_pose_received  = False
        self._vision_pose_stamp    = None  # rclpy.time.Time of last vision pose msg
        self.hailo_active          = False  # True once Hailo flight node is publishing

        rclpy.init()
        self._node = Node("dronepi_main")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(
            State, "/mavros/state", self._state_cb, 10)
        self._node.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, sensor_qos)
        self._node.create_subscription(
            WaypointList,
            "/mavros/mission/waypoints", self._wp_cb, 10)
        self._node.create_subscription(
            HomePosition, "/mavros/home_position/home",
            self._home_cb, sensor_qos)
        self._node.create_subscription(
            String,
            "/dronepi/collision_zone",
            self._zone_cb, reliable_qos)
        self._node.create_subscription(
            PoseStamped,
            "/mavros/vision_pose/pose",
            self._vision_cb, reliable_qos)

        # Live GPS position — used for camera sidecar metadata
        self._node.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._gps_cb, sensor_qos)

        # Hailo in-flight topics — optional, non-fatal if not publishing
        self._node.create_subscription(
            TwistStamped,
            HAILO_FLOW_TOPIC,
            self._hailo_flow_cb, sensor_qos)
        self._node.create_subscription(
            String,
            HAILO_GROUND_TOPIC,
            self._hailo_ground_cb, reliable_qos)

        self._sp_pub = self._node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self._mode_client = self._node.create_client(
            SetMode, "/mavros/set_mode")

        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()
        log("ROS 2 node started (dronepi_main)")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg):
        with self._lock: self._state = msg

    def _pose_cb(self, msg):
        with self._lock: self._pose = msg

    def _wp_cb(self, msg):
        with self._lock: self._waypoints = msg.waypoints

    def _home_cb(self, msg):
        with self._lock: self._home = msg

    def _zone_cb(self, msg):
        with self._lock: self._collision_zone = msg.data

    def _vision_cb(self, msg):
        with self._lock:
            self.vision_pose_received = True
            self._vision_pose_stamp   = self._node.get_clock().now()

    def _gps_cb(self, msg):
        with self._lock: self._gps_fix = msg

    def _hailo_flow_cb(self, msg):
        with self._lock: self.hailo_active = True

    def _hailo_ground_cb(self, msg):
        with self._lock: self.hailo_active = True

    # ── State Properties ──────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        with self._lock: return self._state.armed

    @property
    def mode(self) -> str:
        with self._lock: return self._state.mode

    @property
    def connected(self) -> bool:
        with self._lock: return self._state.connected

    @property
    def collision_zone(self) -> str:
        with self._lock: return self._collision_zone

    def get_pos(self) -> tuple:
        with self._lock:
            if self._pose is None:
                return 0.0, 0.0, 0.0
            p = self._pose.pose.position
            return p.x, p.y, p.z

    def get_home(self):
        with self._lock: return self._home

    def get_gps(self) -> tuple:
        """Return current GPS fix as (lat, lon, alt) or (None, None, None)."""
        with self._lock:
            if self._gps_fix is None:
                return None, None, None
            return (
                self._gps_fix.latitude,
                self._gps_fix.longitude,
                self._gps_fix.altitude,
            )

    def get_nav_waypoints(self) -> list:
        with self._lock:
            return [w for w in self._waypoints if w.command == 16]

    def vision_pose_age_sec(self) -> float:
        """Seconds since the last /mavros/vision_pose/pose message was received.

        Returns float('inf') if no message has ever been received.
        Used by preflight checks and EKF monitor to verify the SLAM chain is live.
        """
        with self._lock:
            if self._vision_pose_stamp is None:
                return float("inf")
            elapsed = self._node.get_clock().now() - self._vision_pose_stamp
            return elapsed.nanoseconds / 1e9

    # ── Setpoint Publishing ───────────────────────────────────────────────────

    def publish_sp(self, ex: float, ey: float, ez: float, yaw: float = 0.0) -> None:
        msg = self._PS()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = ex
        msg.pose.position.y = ey
        msg.pose.position.z = ez
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def stream_sp(
        self, ex: float, ey: float, ez: float, yaw: float, duration: float
    ) -> None:
        end = time.time() + duration
        while time.time() < end:
            self.publish_sp(ex, ey, ez, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def fly_to(
        self, ex: float, ey: float, ez: float, yaw: float, timeout: float
    ) -> bool:
        """Publish setpoints toward target until within WP_TOLERANCE_M or timeout.

        Speed is automatically scaled by the current collision zone:
          CLEAR    → SPEED_SCALE_CLEAR    (full speed)
          CAUTION  → SPEED_SCALE_CAUTION  (50%)
          OBSTACLE → SPEED_SCALE_OBSTACLE (20% creep — PX4 CP_DIST is backstop)

        Returns True if the target was reached, False on timeout.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            zone = self.collision_zone
            scale = {
                "OBSTACLE": SPEED_SCALE_OBSTACLE,
                "CAUTION":  SPEED_SCALE_CAUTION,
            }.get(zone, SPEED_SCALE_CLEAR)

            cx, cy, cz = self.get_pos()
            dist = math.sqrt((cx - ex)**2 + (cy - ey)**2 + (cz - ez)**2)

            if dist < WP_TOLERANCE_M:
                print()
                return True

            tx = cx + (ex - cx) * scale
            ty = cy + (ey - cy) * scale
            tz = cz + (ez - cz) * scale
            self.publish_sp(tx, ty, tz, yaw)

            print(
                f"\r    dist={dist:.2f}m  zone={zone}  scale={scale:.0%}    ",
                end="", flush=True,
            )
            time.sleep(1.0 / SETPOINT_HZ)

        print()
        return False

    # ── Mode Control ──────────────────────────────────────────────────────────

    def set_mode(self, mode: str) -> bool:
        from mavros_msgs.srv import SetMode
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self._mode_client.call_async(req)
        self._rclpy.spin_until_future_complete(
            self._node, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().mode_sent)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        if self._rclpy.ok():
            self._rclpy.shutdown()

# ── Pre-flight Checks ─────────────────────────────────────────────────────────

def _wait_for_topic_hz(topic: str, min_hz: float, timeout_s: float) -> bool:
    """Verify a ROS 2 topic is publishing above min_hz within timeout_s.

    Shells out to `ros2 topic hz` with a short window sample. Used to
    confirm the SLAM chain and Hailo node are actually publishing before
    the flight stack is declared ready — replacing the previous fixed-sleep
    approach which made a timing assumption rather than a health check.

    Returns True if the measured rate meets or exceeds min_hz.
    Returns False on timeout or if the topic is not publishing.
    """
    import subprocess as _sp
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            result = _sp.run(
                ["bash", "-c",
                 f"source {ROS_SETUP} && source {WS_SETUP} && "
                 f"timeout 3 ros2 topic hz {topic} --window 5 2>/dev/null "
                 f"| grep -oP 'average rate: \\K[0-9.]+'"],
                capture_output=True, text=True, timeout=5,
            )
            hz_str = result.stdout.strip()
            if hz_str:
                hz = float(hz_str)
                if hz >= min_hz:
                    return True
        except Exception:
            pass
        time.sleep(2.0)
    return False


def run_preflight_checks(node: MainNode) -> bool:
    """Run all pre-flight checks required before MODE 3 mission start.

    GPS lock is a WARNING only — SLAM is the primary position source.
    The SLAM chain (Point-LIO → bridge → vision pose) is a HARD requirement.
    All other checks must pass. Any hard failure aborts the mission to IDLE.

    Checks:
        1/7  FCU connected via MAVROS
        2/7  GPS home position — WARN only (non-blocking, SLAM is primary)
        3/7  EKF locally stable (Z drift < 0.03 m for 20 consecutive readings)
        4/7  SSD mounted and writable at /mnt/ssd/rosbags
        5/7  Point-LIO launch file present
        6/7  At least one NAV_WAYPOINT uploaded via QGC
        7/7  SLAM bridge publishing vision pose at adequate rate
    """
    log("=" * 50)
    log("[MODE 3] PRE-FLIGHT CHECKS")
    log("=" * 50)
    all_pass = True

    # 1 — FCU connection
    if node.connected:
        log(f"[CHECK 1/7] FCU connected  mode={node.mode}")
    else:
        log("[CHECK 1/7] FAIL — FCU not connected")
        log("            Check Pixhawk USB cable and MAVROS service")
        all_pass = False

    # 2 — GPS / home position — WARN only, non-blocking
    # GPS is not the primary position source (EKF2_GPS_CTRL=0 or 1).
    # SLAM provides position. GPS coordinates are logged for geotagging only.
    log("[CHECK 2/7] Checking GPS home position (non-blocking)...")
    deadline = time.time() + min(HOME_TIMEOUT, 10.0)  # shorter wait — it's a WARN
    while time.time() < deadline:
        if node.get_home() is not None:
            break
        time.sleep(0.5)
    h = node.get_home()
    if h is not None:
        log(f"[CHECK 2/7] GPS home set  "
            f"lat={h.geo.latitude:.6f}  "
            f"lon={h.geo.longitude:.6f}  "
            f"alt={h.geo.altitude:.1f}m")
    else:
        log("[CHECK 2/7] WARN — GPS home not set (non-fatal — SLAM is position source)")
        log("            Camera geotagging will use SLAM origin. Open sky improves accuracy.")

    # 3 — EKF stability
    log("[CHECK 3/7] Waiting for EKF stability...")
    prev_z   = None
    stable   = 0
    deadline = time.time() + EKF_TIMEOUT
    while time.time() < deadline:
        _, _, z = node.get_pos()
        if prev_z is not None:
            stable = stable + 1 if abs(z - prev_z) < 0.03 else 0
        prev_z = z
        if stable >= EKF_STABLE_COUNT:
            break
        time.sleep(0.1)
    if stable >= EKF_STABLE_COUNT:
        log(f"[CHECK 3/7] EKF stable  z={prev_z:.3f}m")
    else:
        log("[CHECK 3/7] FAIL — EKF not stable within timeout")
        log("            Wait longer after boot or check IMU calibration")
        all_pass = False

    # 4 — SSD writable
    ssd_path  = ROSBAG_DIR
    test_file = ssd_path / ".write_test"
    if ssd_path.exists():
        try:
            test_file.touch()
            test_file.unlink()
            log(f"[CHECK 4/7] SSD writable at {ssd_path}")
        except OSError:
            log("[CHECK 4/7] FAIL — SSD not writable")
            all_pass = False
    else:
        log("[CHECK 4/7] FAIL — SSD not mounted at /mnt/ssd")
        log("            Run: sudo mount -a")
        all_pass = False

    # 5 — Point-LIO launch file
    lf = Path(os.path.expanduser(LAUNCH_FILE))
    if lf.exists():
        log(f"[CHECK 5/7] Launch file present: {lf.name}")
    else:
        log(f"[CHECK 5/7] FAIL — Launch file not found: {LAUNCH_FILE}")
        all_pass = False

    # 6 — Mission waypoints
    log("[CHECK 6/7] Waiting for mission waypoints...")
    deadline = time.time() + WP_TIMEOUT
    while time.time() < deadline:
        if node.get_nav_waypoints():
            break
        time.sleep(0.5)
    nav_wps = node.get_nav_waypoints()
    if nav_wps:
        log(f"[CHECK 6/7] {len(nav_wps)} NAV_WAYPOINT(s) received from PX4")
    else:
        log("[CHECK 6/7] FAIL — No waypoints found")
        log("            Upload a survey mission in QGC first")
        all_pass = False

    # 7 — SLAM bridge vision pose — rate-verified with fast path
    # If pointlio-standby.service had Point-LIO running before arm,
    # the vision pose topic will already be publishing. The fast path
    # (2s window) catches this case and skips the 30s Point-LIO wait.
    # If Point-LIO is not yet running, falls through to the full wait.
    log("[CHECK 7/7] Verifying SLAM chain...")
    slam_ok = False

    # Fast path — covers pre-arm standby case and already-running bridge
    age = node.vision_pose_age_sec()
    if age < 1.0:
        log(f"[CHECK 7/7] Vision pose fresh (age={age:.2f}s) — verifying rate...")
        slam_ok = _wait_for_topic_hz(
            "/mavros/vision_pose/pose", BRIDGE_MIN_HZ, 2.0
        )
    else:
        # Check if Point-LIO is up but bridge not yet started
        lio_pre = _wait_for_topic_hz("/aft_mapped_to_init", SLAM_MIN_HZ, 2.0)
        if lio_pre:
            log("[CHECK 7/7] Point-LIO pre-running — checking SLAM bridge rate...")
            slam_ok = _wait_for_topic_hz(
                "/mavros/vision_pose/pose", BRIDGE_MIN_HZ, BRIDGE_TOPIC_TIMEOUT_S
            )
        else:
            # Full wait — neither Point-LIO nor bridge are running yet
            log("[CHECK 7/7] Waiting for Point-LIO /aft_mapped_to_init "
                f"(up to {SLAM_TOPIC_TIMEOUT_S:.0f}s)...")
            lio_ok = _wait_for_topic_hz(
                "/aft_mapped_to_init", SLAM_MIN_HZ, SLAM_TOPIC_TIMEOUT_S
            )
            if lio_ok:
                log("[CHECK 7/7] Point-LIO confirmed — checking SLAM bridge rate...")
                slam_ok = _wait_for_topic_hz(
                    "/mavros/vision_pose/pose", BRIDGE_MIN_HZ, BRIDGE_TOPIC_TIMEOUT_S
                )
            else:
                log("[CHECK 7/7] FAIL — Point-LIO not publishing on /aft_mapped_to_init")
                log("            Ensure LiDAR is connected (/dev/ttyUSB0) and "
                    "pointlio-standby.service is enabled or Point-LIO launched manually")

    if slam_ok:
        log(f"[CHECK 7/7] SLAM chain verified — vision pose >= {BRIDGE_MIN_HZ} Hz")
    else:
        log("[CHECK 7/7] FAIL — Vision pose rate insufficient for EKF2 fusion")
        log("            Check: Is Point-LIO running?  Is _slam_bridge.py active?")
        log(f"           Required: >= {BRIDGE_MIN_HZ} Hz on /mavros/vision_pose/pose")
        all_pass = False

    log("=" * 50)
    if all_pass:
        log("[MODE 3] ALL CHECKS PASSED — mission starting")
    else:
        log("[MODE 3] PRE-FLIGHT CHECKS FAILED — returning to IDLE")
        log("         Disarm, fix the issues above, and re-arm to retry")
    log("=" * 50)
    return all_pass

# ── Camera Helpers ────────────────────────────────────────────────────────────

def _start_camera(session_id: str):
    """Import and start CameraCapture.  Returns instance or None if unavailable.

    Camera failure is non-fatal — the mission proceeds without captures.
    A None return value is handled gracefully at every call site.
    """
    try:
        sys.path.insert(0, str(FLIGHT_DIR))
        from camera_capture import CameraCapture
        cam = CameraCapture(session_id=session_id)
        ok  = cam.start()
        if ok:
            log(f"[CAM] IMX477 started — output: {cam.output_dir}")
            return cam
        else:
            log("[CAM] WARN — CameraCapture.start() failed (picamera2 unavailable?)")
            log("       Mission continues without image capture")
            return None
    except Exception as exc:
        log(f"[CAM] WARN — Could not load camera_capture.py: {exc}")
        log("       Mission continues without image capture")
        return None


def _stop_camera(cam) -> None:
    """Stop CameraCapture safely, tolerating None."""
    if cam is None:
        return
    try:
        cam.stop()
        log(f"[CAM] Stopped — {cam._frame_index} frame(s) saved to {cam.output_dir}")
    except Exception as exc:
        log(f"[CAM] Error during stop: {exc}")


def _trigger_camera(cam, waypoint_index: int, enu: tuple, node: MainNode) -> None:
    """Fire one camera trigger at waypoint arrival.

    Builds the metadata context dict from current node state and posts it
    to CameraCapture.trigger().  Runs synchronously — trigger() returns
    within ~25ms (one frame period at 40fps).

    Parameters
    ----------
    cam           : CameraCapture instance or None
    waypoint_index: zero-based index into the mission waypoint list
    enu           : (east, north, up) metres — current waypoint ENU coords
    node          : MainNode — provides live GPS fix for sidecar metadata
    """
    if cam is None:
        return

    gps_lat, gps_lon, gps_alt = node.get_gps()
    context = {
        "waypoint_index": waypoint_index,
        "enu":            enu,
        "gps":            (gps_lat, gps_lon, gps_alt),
    }
    saved = cam.trigger(context)
    if saved:
        gps_str = (f"{gps_lat:.5f},{gps_lon:.5f}"
                   if gps_lat is not None else "no-fix")
        log(f"  [CAM] WP {waypoint_index + 1} → {saved.name}  (gps={gps_str})")
    else:
        log(f"  [CAM] WP {waypoint_index + 1} trigger timeout — frame skipped")

# ── Mode Handlers ─────────────────────────────────────────────────────────────

def handle_no_scan(node: MainNode) -> None:
    """MODE 1 — LiDAR absent. Monitor state and log only until disarm."""
    log_mode(FlightMode.NO_SCAN,
             "LiDAR absent — no scanning active. Flying freely.")
    while node.armed:
        log_mode(FlightMode.NO_SCAN, f"mode={node.mode}")
        time.sleep(1.0 / POLL_HZ)
    log_mode(FlightMode.NO_SCAN, "Disarmed — returning to IDLE")


def handle_manual_scan(node: MainNode) -> None:
    """MODE 2 — Manual scan. Write lock and monitor until disarm.

    The watchdog service reads the lock file and takes responsibility for
    launching Point-LIO, the SLAM bridge, and the bag recorder. main.py
    monitors armed state only and ignores any OFFBOARD mode changes while
    the mode is locked.
    """
    log_mode(FlightMode.MANUAL,
             "LiDAR present, manual flight — watchdog starting Point-LIO + bag.")
    write_lock("manual_scan")

    while node.armed:
        if node.mode == "OFFBOARD":
            log("[LOCKED] OFFBOARD detected — ignored. Disarm to change mode.")
        log_mode(FlightMode.MANUAL, f"mode={node.mode}  scanning active")
        time.sleep(1.0 / POLL_HZ)

    log_mode(FlightMode.MANUAL,
             "Disarmed — watchdog will stop bag and run post-flight processing")


def handle_autonomous(node: MainNode) -> None:
    """MODE 3 — Autonomous survey. main.py owns the full flight stack.

    Execution order:
        1. Run pre-flight checks (abort to IDLE on any failure).
        2. Write mission lock.
        3. Launch Point-LIO → SLAM bridge → Hailo (non-fatal) →
           collision monitor → bag recorder.
        4. Start CameraCapture background thread (non-fatal if unavailable).
        5. Stream pre-arm setpoints (3 s) required before PX4 accepts OFFBOARD.
        6. Initialise GapDetector for real-time coverage monitoring.
        7. Execute each NAV_WAYPOINT:
               a. Handle OFFBOARD loss — pause / RTL after OFFBOARD_RESUME_S.
               b. fly_to() with collision-zone speed scaling.
               c. Gap-fill sub-waypoints if density below threshold.
               d. Trigger IMX477 capture with ENU + GPS metadata.
        8. AUTO.RTL on mission complete.
        9. Wait for disarm, then tear down all processes and camera.

    Args:
        node: MainNode — shared ROS 2 node.
    """
    log_mode(FlightMode.AUTONOMOUS, "Starting autonomous survey mission")

    if not run_preflight_checks(node):
        log("[MODE 3] Aborting — disarm and resolve pre-flight failures to retry")
        while node.armed:
            time.sleep(0.5)
        return

    write_lock("autonomous")

    # ── 3. Launch Flight Stack ────────────────────────────────────────────────
    pointlio_proc = start_proc(
        "Point-LIO",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0",
    )
    # Verify Point-LIO is actually publishing — not a fixed sleep
    log("Waiting for Point-LIO /aft_mapped_to_init ...")
    if not _wait_for_topic_hz("/aft_mapped_to_init", SLAM_MIN_HZ, SLAM_TOPIC_TIMEOUT_S):
        log("[FAIL] Point-LIO did not publish within timeout — aborting mission")
        stop_proc("Point-LIO", pointlio_proc)
        clear_lock()
        while node.armed:
            time.sleep(0.5)
        return

    bridge_proc = start_proc(
        "SLAM bridge",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"python3 {BRIDGE_SCRIPT}",
    )
    # Verify bridge is forwarding to MAVROS — not a fixed sleep
    log("Waiting for SLAM bridge /mavros/vision_pose/pose ...")
    if not _wait_for_topic_hz("/mavros/vision_pose/pose", BRIDGE_MIN_HZ, BRIDGE_TOPIC_TIMEOUT_S):
        log("[FAIL] SLAM bridge did not publish vision pose within timeout — aborting mission")
        stop_proc("SLAM bridge",  bridge_proc)
        stop_proc("Point-LIO",    pointlio_proc)
        clear_lock()
        while node.armed:
            time.sleep(0.5)
        return
    log("SLAM chain verified — EKF2 has a valid position source")

    # ── 3b. Launch Hailo Flight Node (non-fatal) ──────────────────────────────
    # Runs in hailo_inference_env — separate Python env from the Conda stack.
    # Publishes /hailo/optical_flow and /hailo/ground_class via ROS 2 IPC.
    # If /dev/hailo0 is absent or the node fails to start, flight continues
    # without Hailo — optical flow velocity augmentation is degraded gracefully.
    hailo_proc = None
    if HAILO_DEVICE.exists() and HAILO_FLIGHT_SCRIPT.exists():
        hailo_proc = start_proc(
            "Hailo flight node",
            f"{HAILO_ENV} {HAILO_FLIGHT_SCRIPT}",
        )
        log(f"Waiting up to {HAILO_STARTUP_TIMEOUT_S:.0f}s for Hailo node...")
        if _wait_for_topic_hz(HAILO_FLOW_TOPIC, 5.0, HAILO_STARTUP_TIMEOUT_S):
            log("Hailo flight node active — optical flow augmentation enabled")
            HAILO_LOCK.write_text("flight")
            _write_main_status(hailo_active=True)
        else:
            log("[WARN] Hailo node did not publish within timeout — continuing without it")
            log("       CPU load will be higher. Check /dev/hailo0 and hailo_inference_env")
            stop_proc("Hailo flight node", hailo_proc)
            hailo_proc = None
            _write_main_status(hailo_failed=True)
    else:
        if not HAILO_DEVICE.exists():
            log("[INFO] /dev/hailo0 not found — Hailo flight node skipped")
        else:
            log(f"[INFO] {HAILO_FLIGHT_SCRIPT} not found — Hailo flight node skipped")

    collision_proc = None
    if COLLISION_SCRIPT.exists():
        collision_proc = start_proc(
            "Collision monitor",
            f"source {ROS_SETUP} && source {WS_SETUP} && "
            f"python3 {COLLISION_SCRIPT}",
        )
        time.sleep(COLLISION_INIT_S)
    else:
        log("[WARN] collision_monitor.py not found — obstacle avoidance disabled")

    ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_out = ROSBAG_DIR / f"scan_{ts}"
    bag_proc = start_proc(
        "Bag recorder",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 bag record -o {bag_out} {' '.join(BAG_TOPICS)}",
    )
    log("Flight stack running")

    # ── 3c. Wire FlowBridge into MainNode ─────────────────────────────────────
    # FlowBridge subscribes to /hailo/optical_flow and forwards velocity to
    # /mavros/odometry/out for EKF2 fusion. Only active if Hailo node is up.
    flow_bridge = None
    if hailo_proc is not None:
        try:
            sys.path.insert(0, str(FLIGHT_DIR))
            from _flow_bridge import FlowBridge

            def _on_flow_degraded():
                """Called by FlowBridge after FALLBACK_ESCALATE_COUNT rejections."""
                log("[HAILO] Flow bridge degraded — consecutive rejections exceeded threshold")
                _write_main_status(hailo_active=True, hailo_degraded=True)

            flow_bridge = FlowBridge(
                node=node._node,
                degraded_callback=_on_flow_degraded,
            )
            log("FlowBridge active — Hailo optical flow feeding EKF2 velocity")
        except Exception as exc:
            log(f"[WARN] FlowBridge failed to initialise: {exc} — velocity augmentation disabled")

    # ── 4. Start Camera ───────────────────────────────────────────────────────
    # Session ID matches the bag timestamp so images and bag align by name.
    cam = _start_camera(f"scan_{ts}")
    if cam is not None:
        # Allow AEC/AWB to converge while we load waypoints
        log(f"[CAM] Waiting {CAMERA_INIT_S}s for AEC/AWB convergence...")
        time.sleep(CAMERA_INIT_S)

    # ── Load Waypoints ────────────────────────────────────────────────────────
    home    = node.get_home()
    nav_wps = node.get_nav_waypoints()

    if not nav_wps or home is None:
        log("[MODE 3] Aborting — No waypoints or home position — switching to AUTO.RTL")
        node.set_mode("AUTO.RTL")
        _stop_camera(cam)
        stop_proc("Bag recorder",      bag_proc)
        stop_proc("Collision monitor", collision_proc)
        stop_proc("SLAM bridge",       bridge_proc)
        stop_proc("Point-LIO",         pointlio_proc)
        if hailo_proc:
            stop_proc("Hailo flight node", hailo_proc)
            HAILO_LOCK.unlink(missing_ok=True)
        return

    enu_wps = [
        haversine_to_enu(
            wp.x_lat, wp.y_long, wp.z_alt,
            home.geo.latitude, home.geo.longitude, home.geo.altitude,
        )
        for wp in nav_wps
    ]
    log(f"Mission: {len(enu_wps)} waypoints loaded")

    # ── 5. Pre-stream Setpoints ───────────────────────────────────────────────
    hx, hy, hz = node.get_pos()
    log("Pre-streaming setpoints (3 s)...")
    node.stream_sp(hx, hy, hz, 0.0, 3.0)

    # ── 6. Initialise Gap Detector ────────────────────────────────────────────
    gap_fill_enabled = False
    gap_det = None
    try:
        sys.path.insert(0, str(FLIGHT_DIR))
        from gap_detector import GapDetector

        fov_half    = math.radians(DEFAULT_FOV_DEG / 2.0)
        alt         = enu_wps[0][2] if enu_wps else 10.0
        footprint_r = alt * math.tan(fov_half)
        gap_det     = GapDetector(
            cell_size_m=footprint_r * 2.0,
            min_density=DEFAULT_MIN_DENS,
        )
        gap_det.start(node._node)
        gap_fill_enabled = True
        log(f"Gap detector active  footprint_r={footprint_r:.1f}m  "
            f"cell={footprint_r * 2.0:.1f}m  min_density={DEFAULT_MIN_DENS}")
    except ImportError:
        log("[WARN] gap_detector.py not found — gap fill disabled")

    # ── 7. Waypoint Execution ─────────────────────────────────────────────────
    offboard_lost_at = None

    for i, (ex, ey, ez) in enumerate(enu_wps):
        log(f"Waypoint {i + 1}/{len(enu_wps)}: ENU=({ex:.1f}, {ey:.1f}, {ez:.1f})")

        # Handle OFFBOARD loss
        while node.mode != "OFFBOARD" and node.armed:
            if offboard_lost_at is None:
                offboard_lost_at = time.time()
                log(f"[MODE 3] OFFBOARD lost — RC has control. "
                    f"Resuming if restored within {OFFBOARD_RESUME_S}s.")

            elapsed = time.time() - offboard_lost_at
            if elapsed > OFFBOARD_RESUME_S:
                log(f"[MODE 3] OFFBOARD not restored — switching to AUTO.RTL")
                node.set_mode("AUTO.RTL")
                while node.armed:
                    time.sleep(0.5)
                _stop_camera(cam)
                stop_proc("Bag recorder",      bag_proc)
                stop_proc("Collision monitor", collision_proc)
                stop_proc("SLAM bridge",       bridge_proc)
                stop_proc("Point-LIO",         pointlio_proc)
                if hailo_proc:
                    stop_proc("Hailo flight node", hailo_proc)
                    HAILO_LOCK.unlink(missing_ok=True)
                return
            time.sleep(0.5)

        if offboard_lost_at is not None:
            log(f"[MODE 3] OFFBOARD restored — resuming from waypoint {i + 1}")
            offboard_lost_at = None

        if not node.armed:
            log("[MODE 3] Disarmed mid-mission — stopping")
            break

        # ── Hailo health check per waypoint ───────────────────────────────────
        # Check if the Hailo node process exited unexpectedly between waypoints.
        # This is a background process — it can crash without stopping the mission.
        if hailo_proc is not None and hailo_proc.poll() is not None:
            log(f"[HAILO] Flight node exited unexpectedly "
                f"(code: {hailo_proc.poll()}) — continuing without Hailo")
            hailo_proc = None
            flow_bridge = None
            HAILO_LOCK.unlink(missing_ok=True)
            _write_main_status(hailo_failed=True)

        # Fly to waypoint
        reached = node.fly_to(ex, ey, ez, 0.0, timeout=120.0)
        if not reached:
            log(f"[WARN] Waypoint {i + 1} not reached within timeout — continuing")

        # Gap fill
        if gap_fill_enabled and gap_det is not None:
            cx, cy, cz  = node.get_pos()
            fov_half    = math.radians(DEFAULT_FOV_DEG / 2.0)
            footprint_r = cz * math.tan(fov_half)
            gaps        = gap_det.find_gaps_near(cx, cy, radius_m=footprint_r)

            if gaps:
                log(f"  {len(gaps)} gap(s) detected — inserting gap-fill waypoints")
                for gx, gy in gaps[:3]:
                    node.fly_to(gx, gy, ez, 0.0, timeout=60.0)
                    node.fly_to(ex, ey, ez, 0.0, timeout=60.0)

        # ── Camera trigger — one capture per waypoint arrival ─────────────────
        # Dwell at the waypoint for one stream cycle (50ms) before firing the
        # trigger so the frame is captured with the drone stationary.
        node.stream_sp(ex, ey, ez, 0.0, 0.05)
        _trigger_camera(cam, i, (ex, ey, ez), node)

    # ── 8. Mission Complete → RTL ─────────────────────────────────────────────
    log("Mission complete — switching to AUTO.RTL")
    node.set_mode("AUTO.RTL")

    log("Waiting for landing and disarm...")
    while node.armed:
        time.sleep(0.5)
    log("Disarmed — watchdog will stop bag and run post-flight processing")

    # ── 9. Tear Down ──────────────────────────────────────────────────────────
    _stop_camera(cam)
    stop_proc("Bag recorder",      bag_proc)
    stop_proc("Collision monitor", collision_proc)
    stop_proc("SLAM bridge",       bridge_proc)
    stop_proc("Point-LIO",         pointlio_proc)
    if hailo_proc:
        stop_proc("Hailo flight node", hailo_proc)
        HAILO_LOCK.unlink(missing_ok=True)
    log("[MODE 3] Flight stack torn down cleanly")

# ── Main State Machine ────────────────────────────────────────────────────────

def main() -> None:
    log("=" * 55)
    log("DronePi Main Orchestrator — boot")
    log("Modes: NO_SCAN | MANUAL_SCAN | AUTONOMOUS")
    log("=" * 55)

    clear_lock()

    # ── LED controller — non-fatal if unavailable ─────────────────────────────
    _write_main_status()

    node = MainNode()

    log("Waiting for FCU connection...")
    deadline = time.time() + 30.0
    while time.time() < deadline:
        if node.connected:
            break
        time.sleep(0.5)

    if node.connected:
        log(f"FCU connected  mode={node.mode}")
    else:
        log("[WARN] FCU not connected — will retry on each arm detection")

    def _shutdown(signum=None, frame=None) -> None:
        log("Shutdown signal received")
        clear_lock()
        _clear_main_status()
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    current_mode   = FlightMode.IDLE
    debounce_start = None

    while True:
        armed  = node.armed
        mode   = node.mode
        lidar  = lidar_present()

        if current_mode == FlightMode.IDLE:
            if armed:
                if not lidar:
                    log_mode(FlightMode.IDLE,
                             "Armed, LiDAR absent → MODE 1 (no scan)")
                    current_mode = FlightMode.NO_SCAN
                    handle_no_scan(node)
                    current_mode = FlightMode.IDLE
                    clear_lock()
                    _write_main_status()
                else:
                    if debounce_start is None:
                        debounce_start = time.time()
                        log_mode(FlightMode.DEBOUNCE,
                                 f"Armed + LiDAR present — "
                                 f"waiting {DEBOUNCE_S}s to detect OFFBOARD")
                    current_mode = FlightMode.DEBOUNCE
            else:
                debounce_start = None

                _write_main_status()
                log_mode(FlightMode.IDLE,
                         f"armed={armed}  mode={mode}  "
                         f"lidar={'yes' if lidar else 'no'}")

        elif current_mode == FlightMode.DEBOUNCE:
            if not armed:
                log_mode(FlightMode.DEBOUNCE,
                         "Disarmed during debounce — cancelled, returning to IDLE")
                current_mode   = FlightMode.IDLE
                debounce_start = None

            elif mode == "OFFBOARD":
                log_mode(FlightMode.DEBOUNCE,
                         "OFFBOARD detected — committing MODE 3 (autonomous)")
                current_mode   = FlightMode.AUTONOMOUS
                debounce_start = None
                handle_autonomous(node)
                current_mode = FlightMode.IDLE
                clear_lock()

            elif time.time() - debounce_start >= DEBOUNCE_S:
                log_mode(FlightMode.DEBOUNCE,
                         f"{DEBOUNCE_S}s elapsed, no OFFBOARD — "
                         f"committing MODE 2 (manual scan)")
                current_mode   = FlightMode.MANUAL
                debounce_start = None
                handle_manual_scan(node)
                current_mode = FlightMode.IDLE

            else:
                elapsed   = time.time() - debounce_start
                remaining = DEBOUNCE_S - elapsed
                _write_main_status()
                log_mode(FlightMode.DEBOUNCE,
                         f"mode={mode}  {remaining:.1f}s remaining...")

        time.sleep(1.0 / POLL_HZ)


if __name__ == "__main__":
    main()
