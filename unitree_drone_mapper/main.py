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

GPS Augmentation (MODE 3 only)
-------------------------------
  GpsReader (flight/gps_reader.py) runs as a background polling thread.
  It provides two independent services:

    ① EXIF / sidecar geotag — get_fix() returns the best available GPS
      fix at waypoint trigger. The fix is passed into CameraCapture.trigger()
      context so every JPEG and sidecar JSON carries GPS coordinates alongside
      the SLAM ENU position. Fix quality (HDOP, satellite count) is recorded
      in the sidecar so post-processing can weight or discard marginal fixes.

    ② Drift monitor — check_drift(slam_x, slam_y) compares the Pi GPS
      position to the SLAM-derived ENU position after each fly_to(). A
      disagreement > GPS_DRIFT_THRESHOLD_M (default 5.0 m) is logged as a
      WARNING. At GPS_DRIFT_CRITICAL_M (default 10.0 m) the mission is
      paused and the operator is alerted via buzzer. GPS is never fused
      into EKF2 — SLAM remains the sole authoritative position source.

  GpsReader failure is non-fatal: if no GPS backend is available the
  mission continues with SLAM-only geotagging (same as original behaviour).

Buzzer (MODE 3 only)
--------------------
  MainNode.play_tune() publishes PlayTuneV2 to /mavros/play_tune, using the
  same Pixhawk endpoint as the watchdog's MavrosReader. A ScanBeeper from
  watchdog_core/buzzer.py runs throughout the autonomous mission, giving the
  operator the same 1 Hz audio heartbeat as a manual scan. TUNE_SCAN_START
  is played once at mission commitment; TUNE_SCAN_FINISHED is played when the
  waypoint loop exits. All tones use QBASIC Format 1.

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
HAILO_DEVICE        = Path("/dev/hailo0")
MISSION_LOCK        = Path("/tmp/dronepi_mission.lock")
HAILO_LOCK          = Path("/tmp/dronepi_hailo.lock")
ROSBAG_DIR          = Path("/mnt/ssd/rosbags")
FLIGHT_IMG_DIR      = Path("/mnt/ssd/flights")

# ── Constants ─────────────────────────────────────────────────────────────────

DEBOUNCE_S        = 10      # Seconds after arm to wait before committing a mode
HOME_TIMEOUT      = 30.0    # Seconds to wait for GPS home position (non-blocking)
WP_TIMEOUT        = 15.0    # Seconds to wait for QGC waypoints
EKF_STABLE_COUNT  = 20      # Consecutive stable Z readings required for EKF pass
EKF_TIMEOUT       = 40.0    # Seconds to wait for EKF stability
OFFBOARD_RESUME_S = 30      # Seconds to wait for OFFBOARD restore before RTL
POLL_HZ           = 2       # State machine poll rate
SETPOINT_HZ       = 20      # OFFBOARD setpoint publish rate (PX4 requires > 2 Hz)
POINTLIO_INIT_S   = 5       # Fixed sleep fallback
BRIDGE_INIT_S     = 2       # Fixed sleep fallback
COLLISION_INIT_S  = 2       # Seconds to wait for collision monitor init
CAMERA_INIT_S     = 2       # Seconds to allow AEC/AWB convergence
GRACEFUL_KILL_S   = 5       # Seconds before SIGKILL after SIGINT
DEFAULT_FOV_DEG   = 70.0    # Diagonal camera FOV fallback (degrees)
DEFAULT_MIN_DENS  = 5       # Minimum LiDAR points per grid cell before gap fill
WP_TOLERANCE_M    = 0.5     # Waypoint arrival tolerance (metres)

# SLAM chain verification
SLAM_TOPIC_TIMEOUT_S   = 30.0
BRIDGE_TOPIC_TIMEOUT_S = 15.0
SLAM_MIN_HZ            = 5.0
BRIDGE_MIN_HZ          = 8.0

# Hailo in-flight integration
HAILO_STARTUP_TIMEOUT_S = 20.0
HAILO_FLOW_TOPIC        = "/hailo/optical_flow"
HAILO_GROUND_TOPIC      = "/hailo/ground_class"
HAILO_ENV               = os.path.expanduser("~/hailo_inference_env/bin/python3")

# Velocity scale factors applied by fly_to() based on collision zone status
SPEED_SCALE_CLEAR    = 1.0
SPEED_SCALE_CAUTION  = 0.5
SPEED_SCALE_OBSTACLE = 0.2

# Autonomous test flight constants
HOVER_ALTITUDE_M     = 1.5
HOVER_DWELL_S        = 10.0
TAKEOFF_TIMEOUT_S    = 20.0
TAKEOFF_TOLERANCE_M  = 0.25

# Background setpoint streamer
IDLE_STREAM_HZ = 10

# ── GPS Augmentation Constants ────────────────────────────────────────────────
# These govern the drift monitor only. They do NOT affect EKF2 or SLAM.
# SLAM remains the sole authoritative position source at all times.
#
# GPS_DRIFT_THRESHOLD_M — disagreement level that produces a WARNING log entry.
#   Set above the GPS noise floor (±3–5 m CEP) to suppress false alarms.
#   Default 5.0 m is the minimum safe value for consumer GPS.
#
# GPS_DRIFT_CRITICAL_M — disagreement level that pauses the mission, sounds
#   a buzzer alert, and logs a CRITICAL entry. Operator can override by
#   restoring OFFBOARD. Default 10.0 m represents a plausible SLAM map loss
#   scenario where corrective action is warranted.
#
# GPS_DRIFT_CHECK_ENABLED — master switch. Set False to disable the drift
#   check entirely while keeping GPS active for EXIF tagging only.
#
GPS_DRIFT_THRESHOLD_M  = float(os.environ.get("GPS_DRIFT_THRESHOLD_M", "5.0"))
GPS_DRIFT_CRITICAL_M   = float(os.environ.get("GPS_DRIFT_CRITICAL_M",  "10.0"))
GPS_DRIFT_CHECK_ENABLED = os.environ.get("GPS_DRIFT_CHECK", "1") != "0"

# GPS startup — maximum seconds to wait for first reliable fix before
# declaring GPS unavailable and continuing without it.
GPS_FIX_WAIT_S = float(os.environ.get("GPS_FIX_WAIT_S", "15.0"))

BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
    "/mavros/distance_sensor/lidar_down",
    "/mavros/vision_pose/pose",
    "/hailo/optical_flow",
    "/hailo/ground_class",
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

# ── Background Setpoint Streamer ─────────────────────────────────────────────

class SetpointStreamer:
    """
    Publishes a hold-position setpoint continuously at IDLE_STREAM_HZ.

    Runs as a daemon thread during IDLE and DEBOUNCE states so PX4 always
    has an active setpoint stream. Without this, switching to OFFBOARD mode
    is rejected because PX4 requires the stream to be present before the
    mode switch is accepted.

    The streamer holds the position captured at start() — it does not track
    the drone's current position. This is intentional: during IDLE the drone
    is on the ground and the setpoint is simply "stay at ground origin".

    Usage:
        streamer = SetpointStreamer(node)
        streamer.start()          # begin streaming
        streamer.stop()           # stop before handle_autonomous() takes over
    """

    def __init__(self, node: "MainNode") -> None:
        self._node    = node
        self._thread  = None
        self._stop_ev = threading.Event()

    def start(self) -> None:
        """Capture current position and begin streaming."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_ev.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="setpoint_streamer"
        )
        self._thread.start()
        log("SetpointStreamer started — holding position for OFFBOARD readiness")

    def stop(self) -> None:
        """Stop the background stream. Blocks until thread exits."""
        self._stop_ev.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        log("SetpointStreamer stopped — handle_autonomous() owns setpoints")

    def _loop(self) -> None:
        interval = 1.0 / IDLE_STREAM_HZ
        while not self._stop_ev.is_set():
            hx, hy, hz = self._node.get_pos()
            self._node.publish_sp(hx, hy, hz, 0.0)
            time.sleep(interval)


# ── Main Status Writer ────────────────────────────────────────────────────────

_MAIN_STATUS_FILE     = "/tmp/main_status.json"
_MAIN_STATUS_FILE_TMP = _MAIN_STATUS_FILE + ".tmp"


def _write_main_status(
    hailo_active:   bool = False,
    hailo_degraded: bool = False,
    hailo_failed:   bool = False,
    gps_reliable:   bool = False,
    gps_hdop:       float = 99.0,
    gps_satellites: int   = 0,
    gps_drift_m:    float = 0.0,
) -> None:
    """
    Write main.py heartbeat, Hailo status, and GPS augmentation status
    to /tmp/main_status.json.

    Must be called on every poll cycle during autonomous flight so
    led_service.py sees a live timestamp. The GPS fields are informational —
    they are not consumed by led_service.py but are available for future
    LED states or monitoring tools.

    Parameters
    ----------
    hailo_active   : Hailo node publishing and augmenting EKF2
    hailo_degraded : FlowBridge degraded
    hailo_failed   : Hailo process exited unexpectedly
    gps_reliable   : Pi GPS fix passes quality gate (HDOP, sat count, fix type)
    gps_hdop       : Current HDOP value from Pi GPS reader
    gps_satellites : Satellite count from Pi GPS reader
    gps_drift_m    : Last GPS↔SLAM disagreement in metres (0.0 if not checked)
    """
    try:
        payload = json.dumps({
            "ts":             time.time(),
            "hailo_active":   hailo_active,
            "hailo_degraded": hailo_degraded,
            "hailo_failed":   hailo_failed,
            "gps_reliable":   gps_reliable,
            "gps_hdop":       round(gps_hdop, 2),
            "gps_satellites": gps_satellites,
            "gps_drift_m":    round(gps_drift_m, 2),
        })
        with open(_MAIN_STATUS_FILE_TMP, "w") as f:
            f.write(payload)
        os.replace(_MAIN_STATUS_FILE_TMP, _MAIN_STATUS_FILE)
    except Exception as exc:
        log(f"[STATUS] Main status write failed: {exc}")


def _clear_main_status() -> None:
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
    GPS fix, and QGC waypoints. Also implements setpoint publishing, mode
    switching, and buzzer output for use during MODE 3 autonomous execution.
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

        self._state              = State()
        self._pose               = None
        self._home               = None
        self._waypoints          = []
        self._gps_fix            = None
        self._collision_zone     = "CLEAR"
        self.vision_pose_received = False
        self._vision_pose_stamp  = None
        self.hailo_active        = False

        self._tune_pub = None

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
        self._node.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._gps_cb, sensor_qos)
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
        """Return current MAVROS GPS fix as (lat, lon, alt) or (None, None, None)."""
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
        """Seconds since the last /mavros/vision_pose/pose message.
        Returns float('inf') if no message has ever been received.
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
          OBSTACLE → SPEED_SCALE_OBSTACLE (20% creep)

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

    # ── Buzzer ────────────────────────────────────────────────────────────────

    def play_tune(self, tune: str) -> None:
        """Publish a QBASIC Format 1 tune to the Pixhawk buzzer via MAVROS."""
        try:
            from mavros_msgs.msg import PlayTuneV2
            if self._tune_pub is None:
                self._tune_pub = self._node.create_publisher(
                    PlayTuneV2, "/mavros/play_tune", 10
                )
            msg        = PlayTuneV2()
            msg.format = 1
            msg.tune   = tune
            self._tune_pub.publish(msg)
        except Exception as exc:
            log(f"[BUZZER] play_tune failed: {exc}")

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
    """Verify a ROS 2 topic is publishing above min_hz within timeout_s."""
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
    log("[CHECK 2/7] Checking GPS home position (non-blocking)...")
    deadline = time.time() + min(HOME_TIMEOUT, 10.0)
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

    # 7 — SLAM bridge vision pose
    log("[CHECK 7/7] Verifying SLAM chain...")
    slam_ok = False

    age = node.vision_pose_age_sec()
    if age < 1.0:
        log(f"[CHECK 7/7] Vision pose fresh (age={age:.2f}s) — verifying rate...")
        slam_ok = _wait_for_topic_hz(
            "/mavros/vision_pose/pose", BRIDGE_MIN_HZ, 2.0
        )
    else:
        lio_pre = _wait_for_topic_hz("/aft_mapped_to_init", SLAM_MIN_HZ, 2.0)
        if lio_pre:
            log("[CHECK 7/7] Point-LIO pre-running — checking SLAM bridge rate...")
            slam_ok = _wait_for_topic_hz(
                "/mavros/vision_pose/pose", BRIDGE_MIN_HZ, BRIDGE_TOPIC_TIMEOUT_S
            )
        else:
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

# ── GPS Augmentation Helpers ──────────────────────────────────────────────────

def _start_gps_reader():
    """Import and start GpsReader. Returns instance or None if unavailable.

    Failure is non-fatal. If the GPS reader cannot start, the mission
    continues with SLAM-only geotagging — the same behaviour as before
    this module was added.
    """
    try:
        sys.path.insert(0, str(PROJECT_ROOT))
        from flight.gps_reader import GpsReader
        gps = GpsReader()
        ok  = gps.start()
        if ok:
            log(f"[GPS] Reader started — backend={gps.backend}  "
                f"HDOP_max={2.5}  sats_min=6")
            return gps
        else:
            log("[GPS] WARN — GpsReader.start() failed (gpsd not running? pyserial missing?)")
            log("      Mission continues with SLAM-only geotagging")
            log("      Install: sudo apt install gpsd python3-gps  OR  pip install pyserial")
            return None
    except Exception as exc:
        log(f"[GPS] WARN — Could not load gps_reader.py: {exc}")
        log("      Mission continues with SLAM-only geotagging")
        return None


def _stop_gps_reader(gps) -> None:
    """Stop GpsReader safely, tolerating None."""
    if gps is None:
        return
    try:
        stats = gps.get_stats()
        gps.stop()
        log(f"[GPS] Stopped — {stats['fix_count']} reliable fixes accepted  "
            f"{stats['reject_count']} rejected  backend={stats['backend']}")
    except Exception as exc:
        log(f"[GPS] Error during stop: {exc}")


def _wait_for_gps_home(gps, timeout_s: float) -> bool:
    """Wait for first reliable GPS fix and call set_home().

    Non-blocking on failure — returns True if home was set, False if
    no reliable fix arrived within timeout_s. Caller continues regardless.
    """
    if gps is None:
        return False

    log(f"[GPS] Waiting up to {timeout_s:.0f}s for reliable fix to set home datum...")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if gps.is_reliable():
            ok = gps.set_home()
            if ok:
                fix = gps.get_reliable_fix()
                log(f"[GPS] Home datum set  "
                    f"lat={fix.lat:.7f}  lon={fix.lon:.7f}  "
                    f"alt={fix.alt:.1f}m  HDOP={fix.hdop:.2f}  "
                    f"sats={fix.satellites}")
                return True
        time.sleep(0.5)

    log(f"[GPS] WARN — No reliable fix within {timeout_s:.0f}s "
        f"— geotagging will use MAVROS GPS fix if available, otherwise SLAM origin")
    return False


def _get_geotag_gps(gps, node: MainNode) -> tuple:
    """Return best available GPS coordinate for EXIF/sidecar tagging.

    Priority:
        1. Pi GpsReader reliable fix (best metadata — includes HDOP)
        2. MAVROS NavSatFix from /mavros/global_position/global (fallback)
        3. (None, None, None) — no GPS available

    The Pi GPS fix is preferred because it includes HDOP and satellite count
    in the returned GpsFix object, which the camera sidecar JSON records for
    post-processing quality assessment. The MAVROS fix is a fallback for when
    the Pi GPS is unavailable or unreliable.
    """
    if gps is not None:
        fix = gps.get_fix()   # get_fix() not get_reliable_fix() — always log
        if fix is not None and not fix.is_stale():
            return fix.lat, fix.lon, fix.alt

    # Fall through to MAVROS GPS
    return node.get_gps()


def _run_drift_check(
    gps,
    slam_x: float,
    slam_y: float,
    node: MainNode,
    _play,
    _buzzer_available: bool,
) -> float:
    """Run GPS↔SLAM drift check at current SLAM position.

    Returns the disagreement distance in metres (0.0 if check skipped).
    Logs WARNING at GPS_DRIFT_THRESHOLD_M, CRITICAL at GPS_DRIFT_CRITICAL_M.
    At critical level, sounds a warning buzzer tone — does NOT abort mission
    autonomously since GPS accuracy alone cannot confirm SLAM failure.
    The operator receives the alert and decides whether to intervene.
    """
    if gps is None or not GPS_DRIFT_CHECK_ENABLED:
        return 0.0

    if not gps.is_reliable():
        return 0.0

    delta = gps.get_enu_delta()
    if delta is None:
        return 0.0

    gps_east, gps_north, _ = delta
    disagreement = math.sqrt(
        (gps_east  - slam_x) ** 2 +
        (gps_north - slam_y) ** 2
    )

    fix = gps.get_reliable_fix()
    hdop_str = f"HDOP={fix.hdop:.2f}" if fix else ""

    if disagreement >= GPS_DRIFT_CRITICAL_M:
        log(f"[GPS][CRITICAL] GPS↔SLAM disagreement={disagreement:.1f}m "
            f"(threshold={GPS_DRIFT_CRITICAL_M:.1f}m)  {hdop_str}  "
            f"GPS=({gps_east:.1f},{gps_north:.1f})  "
            f"SLAM=({slam_x:.1f},{slam_y:.1f})  "
            f"— SLAM may have degraded. Monitor closely.")
        if _buzzer_available:
            try:
                from flight.watchdog_core.buzzer import TUNE_WARNING
                _play(TUNE_WARNING)
            except Exception:
                pass

    elif disagreement >= GPS_DRIFT_THRESHOLD_M:
        log(f"[GPS][WARN] GPS↔SLAM disagreement={disagreement:.1f}m "
            f"(threshold={GPS_DRIFT_THRESHOLD_M:.1f}m)  {hdop_str}  "
            f"GPS=({gps_east:.1f},{gps_north:.1f})  "
            f"SLAM=({slam_x:.1f},{slam_y:.1f})")

    return round(disagreement, 2)

# ── Camera Helpers ────────────────────────────────────────────────────────────

def _start_camera(session_id: str):
    """Import and start CameraCapture. Returns instance or None if unavailable."""
    try:
        sys.path.insert(0, str(PROJECT_ROOT))
        from flight.camera_capture import CameraCapture
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


def _trigger_camera(
    cam,
    waypoint_index: int,
    enu: tuple,
    gps_coords: tuple,
    gps_quality: dict,
    node: "MainNode | None" = None,     # ← added: ROS node for clock access
) -> None:
    """Fire one camera trigger at waypoint arrival.

    Parameters
    ----------
    cam           : CameraCapture instance or None
    waypoint_index: Zero-based waypoint index
    enu           : (east, north, up) in metres — SLAM-derived position
    gps_coords    : (lat, lon, alt) — best available GPS coordinates
    gps_quality   : dict with keys hdop, satellites, reliable, source
    node          : MainNode instance — used to read ROS clock for ros_timestamp.
                    If None, ros_timestamp is omitted from the sidecar (non-fatal).
                    frame_ingestor.py will log a warning but continue with
                    wall-clock matching as a degraded fallback.
    """
    if cam is None:
        return

    # Obtain ROS clock timestamp for SLAM pose matching in frame_ingestor.py.
    # ROS clock is preferred over time.time() because PoseInterpolator keys
    # on ROS timestamps from the SLAM trajectory, not wall clock.
    ros_ts = None
    if node is not None:
        try:
            ros_ts = node._node.get_clock().now().nanoseconds * 1e-9
        except Exception:
            pass   # Non-fatal — sidecar will have ros_timestamp: null

    gps_lat, gps_lon, gps_alt = gps_coords
    context = {
        "waypoint_index": waypoint_index,
        "enu":            enu,
        "gps":            (gps_lat, gps_lon, gps_alt),
        "gps_quality":    gps_quality,
        "ros_timestamp":  ros_ts,           # ← added: written to frame sidecar JSON
    }
    saved = cam.trigger(context)

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
        1.  Run pre-flight checks (abort to IDLE on any failure).
        2.  Write mission lock.
        3.  Launch Point-LIO → SLAM bridge → Hailo (non-fatal) →
            collision monitor → bag recorder.
        3b. Wire FlowBridge into MainNode for Hailo EKF2 velocity augmentation.
        4.  Start CameraCapture background thread (non-fatal if unavailable).
        4b. Start GpsReader background thread (non-fatal if unavailable).
            Wait GPS_FIX_WAIT_S for first reliable fix and set home datum.
        5.  Stream pre-arm setpoints (3 s) required before PX4 accepts OFFBOARD.
        6.  Initialise GapDetector for real-time coverage monitoring.
        7.  Execute each NAV_WAYPOINT:
               a. Handle OFFBOARD loss — pause / RTL after OFFBOARD_RESUME_S.
               b. fly_to() with collision-zone speed scaling.
               c. Gap-fill sub-waypoints if density below threshold.
               d. GPS drift check — WARN at 5 m, CRITICAL + buzzer at 10 m.
               e. Trigger IMX477 capture with ENU + GPS metadata + quality.
               f. Update main status file with GPS telemetry.
        8.  AUTO.RTL on mission complete.
        9.  Wait for disarm, then tear down all processes, camera, GPS reader.

    GPS Augmentation (non-fatal throughout):
        GpsReader runs in a background thread and is never in the critical
        path. If it fails to start, returns no fix, or loses signal mid-flight,
        all GPS-dependent operations degrade gracefully:
          - EXIF tagging falls back to MAVROS NavSatFix
          - Drift check is skipped when no reliable fix is available
          - Mission execution is never paused or aborted due to GPS issues

    Buzzer:
        ScanBeeper runs throughout via node.play_tune(). TUNE_WARNING fires
        on GPS critical drift events. TUNE_SCAN_FINISHED fires on all exits.
    """
    try:
        sys.path.insert(0, str(PROJECT_ROOT / "flight"))
        from flight.watchdog_core.buzzer import (
            ScanBeeper,
            TUNE_SCAN_START,
            TUNE_SCAN_FINISHED,
            TUNE_WARNING,
        )
        _buzzer_available = True
    except ImportError as exc:
        log(f"[BUZZER] watchdog_core.buzzer not importable: {exc} — running silent")
        _buzzer_available = False

    def _play(tune: str) -> None:
        if _buzzer_available:
            node.play_tune(tune)

    log_mode(FlightMode.AUTONOMOUS, "Starting autonomous survey mission")

    if not run_preflight_checks(node):
        log("[MODE 3] Aborting — disarm and resolve pre-flight failures to retry")
        while node.armed:
            time.sleep(0.5)
        return

    write_lock("autonomous")

    scan_beeper = (
        ScanBeeper(play_tune_fn=node.play_tune) if _buzzer_available else None
    )

    # ── Teardown helper — called on every exit path ────────────────────────────
    def _teardown(
        bag_proc, collision_proc, bridge_proc, pointlio_proc, hailo_proc, cam, gps
    ) -> None:
        if scan_beeper is not None:
            scan_beeper.stop()
        _play(TUNE_SCAN_FINISHED if _buzzer_available else "")
        log("[BUZZER] autonomous scan_finished")
        _stop_camera(cam)
        _stop_gps_reader(gps)
        stop_proc("Bag recorder",      bag_proc)
        stop_proc("Collision monitor", collision_proc)
        stop_proc("SLAM bridge",       bridge_proc)
        stop_proc("Point-LIO",         pointlio_proc)
        if hailo_proc:
            stop_proc("Hailo flight node", hailo_proc)
            HAILO_LOCK.unlink(missing_ok=True)

    # ── 3. Launch Flight Stack ────────────────────────────────────────────────
    pointlio_proc = start_proc(
        "Point-LIO",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0",
    )
    log("Waiting for Point-LIO /aft_mapped_to_init ...")
    if not _wait_for_topic_hz("/aft_mapped_to_init", SLAM_MIN_HZ, SLAM_TOPIC_TIMEOUT_S):
        log("[FAIL] Point-LIO did not publish within timeout — aborting mission")
        stop_proc("Point-LIO", pointlio_proc)
        clear_lock()
        if scan_beeper is not None:
            scan_beeper.stop()
        while node.armed:
            time.sleep(0.5)
        return

    bridge_proc = start_proc(
        "SLAM bridge",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"python3 {BRIDGE_SCRIPT}",
    )
    log("Waiting for SLAM bridge /mavros/vision_pose/pose ...")
    if not _wait_for_topic_hz("/mavros/vision_pose/pose", BRIDGE_MIN_HZ, BRIDGE_TOPIC_TIMEOUT_S):
        log("[FAIL] SLAM bridge did not publish vision pose within timeout — aborting mission")
        stop_proc("SLAM bridge",  bridge_proc)
        stop_proc("Point-LIO",    pointlio_proc)
        clear_lock()
        if scan_beeper is not None:
            scan_beeper.stop()
        while node.armed:
            time.sleep(0.5)
        return
    log("SLAM chain verified — EKF2 has a valid position source")

    # ── 3b. Launch Hailo Flight Node (non-fatal) ──────────────────────────────
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

    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_out  = ROSBAG_DIR / f"scan_{ts}"
    bag_proc = start_proc(
        "Bag recorder",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 bag record -o {bag_out} {' '.join(BAG_TOPICS)}",
    )
    log("Flight stack running")

    # ── 3c. Wire FlowBridge into MainNode ─────────────────────────────────────
    flow_bridge = None
    if hailo_proc is not None:
        try:
            sys.path.insert(0, str(PROJECT_ROOT))
            from flight._flow_bridge import FlowBridge

            def _on_flow_degraded():
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
    cam = _start_camera(f"scan_{ts}")
    if cam is not None:
        log(f"[CAM] Waiting {CAMERA_INIT_S}s for AEC/AWB convergence...")
        time.sleep(CAMERA_INIT_S)

    # ── 4b. Start GPS Reader ──────────────────────────────────────────────────
    # GpsReader is started after the camera to keep the startup sequence
    # consistent — camera has the longer blocking init (AEC/AWB sleep).
    # GPS is non-fatal: if it fails here the mission continues unchanged.
    gps = _start_gps_reader()
    gps_home_set = _wait_for_gps_home(gps, GPS_FIX_WAIT_S)
    if not gps_home_set:
        log("[GPS] Home datum not set — drift check will be skipped. "
            "EXIF will use MAVROS GPS fallback if Pi GPS unavailable.")

    # ── Load Waypoints ────────────────────────────────────────────────────────
    home    = node.get_home()
    nav_wps = node.get_nav_waypoints()

    if not nav_wps or home is None:
        # ── No-waypoints test flight ──────────────────────────────────────────
        log("[MODE 3] No waypoints — executing hover test "
            f"(alt={HOVER_ALTITUDE_M}m  dwell={HOVER_DWELL_S}s)")

        hx, hy, hz = node.get_pos()
        hover_z     = hz + HOVER_ALTITUDE_M

        log("Pre-streaming hold setpoint (3s)...")
        node.stream_sp(hx, hy, hz, 0.0, 3.0)

        log(f"Climbing to {hover_z:.2f}m...")
        reached = node.fly_to(hx, hy, hover_z, 0.0, timeout=TAKEOFF_TIMEOUT_S)
        if not reached:
            log("[WARN] Takeoff altitude not reached within timeout — landing")
        else:
            log(f"Hover altitude reached — holding for {HOVER_DWELL_S}s")
            node.stream_sp(hx, hy, hover_z, 0.0, HOVER_DWELL_S)

        log("Hover test complete — switching to AUTO.LAND")
        node.set_mode("AUTO.LAND")

        log("Waiting for landing and disarm...")
        while node.armed:
            time.sleep(0.5)

        _teardown(bag_proc, collision_proc, bridge_proc, pointlio_proc, hailo_proc, cam, gps)
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

    # ── Start scan beeper — mission is now committed ──────────────────────────
    _play(TUNE_SCAN_START)
    log("[BUZZER] autonomous scan_start")
    if scan_beeper is not None:
        scan_beeper.start()

    # ── 6. Initialise Gap Detector ────────────────────────────────────────────
    gap_fill_enabled = False
    gap_det = None
    try:
        sys.path.insert(0, str(PROJECT_ROOT))
        from flight.gap_detector import GapDetector

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
    last_drift_m     = 0.0   # Tracks most recent GPS↔SLAM disagreement for status

    for i, (ex, ey, ez) in enumerate(enu_wps):
        log(f"Waypoint {i + 1}/{len(enu_wps)}: ENU=({ex:.1f}, {ey:.1f}, {ez:.1f})")

        # ── Handle OFFBOARD loss ───────────────────────────────────────────────
        while node.mode != "OFFBOARD" and node.armed:
            if offboard_lost_at is None:
                offboard_lost_at = time.time()
                log(f"[MODE 3] OFFBOARD lost — RC has control. "
                    f"Resuming if restored within {OFFBOARD_RESUME_S}s.")

            elapsed = time.time() - offboard_lost_at
            if elapsed > OFFBOARD_RESUME_S:
                log("[MODE 3] OFFBOARD not restored — switching to AUTO.RTL")
                node.set_mode("AUTO.RTL")
                while node.armed:
                    time.sleep(0.5)
                _teardown(bag_proc, collision_proc, bridge_proc, pointlio_proc, hailo_proc, cam, gps)
                return
            time.sleep(0.5)

        if offboard_lost_at is not None:
            log(f"[MODE 3] OFFBOARD restored — resuming from waypoint {i + 1}")
            offboard_lost_at = None

        if not node.armed:
            log("[MODE 3] Disarmed mid-mission — stopping")
            break

        # ── Hailo health check per waypoint ───────────────────────────────────
        if hailo_proc is not None and hailo_proc.poll() is not None:
            log(f"[HAILO] Flight node exited unexpectedly "
                f"(code: {hailo_proc.poll()}) — continuing without Hailo")
            hailo_proc  = None
            flow_bridge = None
            HAILO_LOCK.unlink(missing_ok=True)
            _write_main_status(hailo_failed=True)

        # ── Fly to waypoint ────────────────────────────────────────────────────
        reached = node.fly_to(ex, ey, ez, 0.0, timeout=120.0)
        if not reached:
            log(f"[WARN] Waypoint {i + 1} not reached within timeout — continuing")

        # ── Gap fill ───────────────────────────────────────────────────────────
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

        # ── GPS drift check ────────────────────────────────────────────────────
        # Run after fly_to() confirms arrival so the position comparison is
        # against the settled SLAM position, not a position mid-transit.
        # Skipped automatically if GPS has no reliable fix or home not set.
        cx, cy, _ = node.get_pos()
        last_drift_m = _run_drift_check(
            gps, cx, cy, node, _play, _buzzer_available
        )

        # ── Dwell + camera trigger ─────────────────────────────────────────────
        node.stream_sp(ex, ey, ez, 0.0, 0.05)

        # Build GPS geotag from best available source
        gps_coords  = _get_geotag_gps(gps, node)
        gps_quality = {}
        if gps is not None:
            fix = gps.get_fix()
            if fix is not None:
                gps_quality = {
                    "hdop":       fix.hdop,
                    "satellites": fix.satellites,
                    "reliable":   fix.reliable,
                    "source":     fix.source,
                }

        _trigger_camera(cam, i, (ex, ey, ez), gps_coords, gps_quality)

        # ── Update main status ─────────────────────────────────────────────────
        hailo_ok   = hailo_proc is not None and hailo_proc.poll() is None
        hailo_deg  = flow_bridge.is_degraded() if flow_bridge is not None else False
        fix_now    = gps.get_fix() if gps is not None else None
        _write_main_status(
            hailo_active=hailo_ok and not hailo_deg,
            hailo_degraded=hailo_deg,
            hailo_failed=(hailo_proc is None and flow_bridge is not None),
            gps_reliable=fix_now.reliable if fix_now else False,
            gps_hdop=fix_now.hdop if fix_now else 99.0,
            gps_satellites=fix_now.satellites if fix_now else 0,
            gps_drift_m=last_drift_m,
        )

    # ── 8. Mission Complete → RTL ─────────────────────────────────────────────
    log("Mission complete — switching to AUTO.RTL")
    node.set_mode("AUTO.RTL")

    log("Waiting for landing and disarm...")
    while node.armed:
        time.sleep(0.5)
    log("Disarmed — watchdog will stop bag and run post-flight processing")

    # ── 9. Tear Down ──────────────────────────────────────────────────────────
    _teardown(bag_proc, collision_proc, bridge_proc, pointlio_proc, hailo_proc, cam, gps)
    log("[MODE 3] Flight stack torn down cleanly")

# ── Main State Machine ────────────────────────────────────────────────────────

def main() -> None:
    log("=" * 55)
    log("DronePi Main Orchestrator — boot")
    log("Modes: NO_SCAN | MANUAL_SCAN | AUTONOMOUS")
    log("=" * 55)

    clear_lock()
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
        streamer.stop()
        clear_lock()
        _clear_main_status()
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    current_mode   = FlightMode.IDLE
    debounce_start = None

    streamer = SetpointStreamer(node)
    streamer.start()

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
                    streamer.stop()
                    handle_no_scan(node)
                    streamer.start()
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
                streamer.stop()
                handle_autonomous(node)
                streamer.start()
                current_mode = FlightMode.IDLE
                clear_lock()

            elif time.time() - debounce_start >= DEBOUNCE_S:
                log_mode(FlightMode.DEBOUNCE,
                         f"{DEBOUNCE_S}s elapsed, no OFFBOARD — "
                         f"committing MODE 2 (manual scan)")
                current_mode   = FlightMode.MANUAL
                debounce_start = None
                streamer.stop()
                handle_manual_scan(node)
                streamer.start()
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
