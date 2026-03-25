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
    main.py launches Point-LIO + SLAM bridge + bag recorder
    Executes QGC waypoints with LiDAR gap fill
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

BRIDGE_SCRIPT     = FLIGHT_DIR / "_slam_bridge.py"
POSTFLIGHT_SCRIPT = UTILS_DIR  / "run_postflight.py"
LIDAR_PORT        = Path("/dev/ttyUSB0")
MISSION_LOCK      = Path("/tmp/dronepi_mission.lock")
ROSBAG_DIR        = Path("/mnt/ssd/rosbags")

# ── Constants ─────────────────────────────────────────────────────────────────

DEBOUNCE_S        = 10      # Seconds after arm to wait before committing a mode
HOME_TIMEOUT      = 30.0    # Seconds to wait for GPS home position
WP_TIMEOUT        = 15.0    # Seconds to wait for QGC waypoints
EKF_STABLE_COUNT  = 20      # Consecutive stable Z readings required for EKF pass
EKF_TIMEOUT       = 40.0    # Seconds to wait for EKF stability
OFFBOARD_RESUME_S = 30      # Seconds to wait for OFFBOARD restore before RTL
POLL_HZ           = 2       # State machine poll rate
SETPOINT_HZ       = 20      # OFFBOARD setpoint publish rate (PX4 requires > 2 Hz)
POINTLIO_INIT_S   = 5       # Seconds to wait for Point-LIO initialisation
BRIDGE_INIT_S     = 2       # Seconds to wait for SLAM bridge initialisation
GRACEFUL_KILL_S   = 5       # Seconds before SIGKILL after SIGINT
DEFAULT_FOV_DEG   = 70.0    # Diagonal camera FOV fallback (degrees)
DEFAULT_MIN_DENS  = 5       # Minimum LiDAR points per grid cell before gap fill
WP_TOLERANCE_M    = 0.5     # Waypoint arrival tolerance (metres)

BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
]

# ── Flight Modes ──────────────────────────────────────────────────────────────

class FlightMode(Enum):
    IDLE       = "IDLE"
    DEBOUNCE   = "DEBOUNCE"
    NO_SCAN    = "NO_SCAN"       # MODE 1
    MANUAL     = "MANUAL_SCAN"   # MODE 2
    AUTONOMOUS = "AUTONOMOUS"    # MODE 3

# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def log_mode(mode: FlightMode, detail: str = "") -> None:
    sep = f"  {detail}" if detail else ""
    log(f"[{mode.value}]{sep}")

# ── Lock File ─────────────────────────────────────────────────────────────────

def write_lock(mode: str, extra: dict = None) -> None:
    """Write mission lock file. Watchdog reads this on every loop."""
    payload = {"mode": mode, "started_at": datetime.now().isoformat()}
    if extra:
        payload.update(extra)
    MISSION_LOCK.write_text(json.dumps(payload))
    log(f"Lock written: {mode}")


def clear_lock() -> None:
    """Remove mission lock file. Watchdog returns to normal RC-toggle mode."""
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
    """Return True if the LiDAR USB device node exists."""
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
    and QGC waypoints. Also implements setpoint publishing and mode switching
    for use during MODE 3 autonomous execution.
    """

    def __init__(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import State, WaypointList, HomePosition
            from mavros_msgs.srv import SetMode
        except ImportError as exc:
            log(f"[FAIL] ROS 2 not sourced: {exc}")
            sys.exit(1)

        self._rclpy = rclpy
        self._PS    = PoseStamped
        self._lock  = threading.Lock()

        self._state     = State()
        self._pose      = None
        self._home      = None
        self._waypoints = []

        rclpy.init()
        self._node = Node("dronepi_main")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(
            State, "/mavros/state", self._state_cb, 10)
        self._node.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, sensor_qos)
        self._node.create_subscription(
            __import__("mavros_msgs").msg.WaypointList,
            "/mavros/mission/waypoints", self._wp_cb, 10)
        self._node.create_subscription(
            HomePosition, "/mavros/home_position/home",
            self._home_cb, sensor_qos)

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

    def get_pos(self) -> tuple:
        with self._lock:
            if self._pose is None:
                return 0.0, 0.0, 0.0
            p = self._pose.pose.position
            return p.x, p.y, p.z

    def get_home(self):
        with self._lock: return self._home

    def get_nav_waypoints(self) -> list:
        """Return only NAV_WAYPOINT (command == 16) items from the mission."""
        with self._lock:
            return [w for w in self._waypoints if w.command == 16]

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
        """Continuously publish a setpoint for the given duration (seconds)."""
        end = time.time() + duration
        while time.time() < end:
            self.publish_sp(ex, ey, ez, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def fly_to(
        self, ex: float, ey: float, ez: float, yaw: float, timeout: float
    ) -> bool:
        """Publish setpoints toward target until within WP_TOLERANCE_M or timeout.

        Returns:
            True if target reached, False if timeout expired.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.publish_sp(ex, ey, ez, yaw)
            cx, cy, cz = self.get_pos()
            dist = math.sqrt((cx - ex)**2 + (cy - ey)**2 + (cz - ez)**2)
            print(f"\r    dist={dist:.2f}m", end="", flush=True)
            if dist < WP_TOLERANCE_M:
                print()
                return True
            time.sleep(1.0 / SETPOINT_HZ)
        print()
        return False

    # ── Mode Control ──────────────────────────────────────────────────────────

    def set_mode(self, mode: str) -> bool:
        """Send a mode change request to PX4 via MAVROS.

        Returns:
            True if the mode change was accepted.
        """
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

def run_preflight_checks(node: MainNode) -> bool:
    """Run all six pre-flight checks required before MODE 3 mission start.

    All checks must pass. Any single failure causes an immediate return of
    False and the mission is aborted back to IDLE.

    Checks:
        1. FCU connected via MAVROS
        2. GPS lock and home position received
        3. EKF locally stable (Z drift < 0.03 m for 20 consecutive readings)
        4. SSD mounted and writable at /mnt/ssd/rosbags
        5. Point-LIO launch file present
        6. At least one NAV_WAYPOINT uploaded via QGC
    """
    log("=" * 50)
    log("[MODE 3] PRE-FLIGHT CHECKS")
    log("=" * 50)
    all_pass = True

    # 1 — FCU connection
    if node.connected:
        log(f"[CHECK 1/6] FCU connected  mode={node.mode}")
    else:
        log("[CHECK 1/6] FAIL — FCU not connected")
        log("            Check Pixhawk USB cable and MAVROS service")
        all_pass = False

    # 2 — GPS lock and home position
    log("[CHECK 2/6] Waiting for GPS home position...")
    deadline = time.time() + HOME_TIMEOUT
    while time.time() < deadline:
        if node.get_home() is not None:
            break
        time.sleep(0.5)
    h = node.get_home()
    if h is not None:
        log(f"[CHECK 2/6] GPS lock OK  "
            f"lat={h.geo.latitude:.6f}  "
            f"lon={h.geo.longitude:.6f}  "
            f"alt={h.geo.altitude:.1f}m")
    else:
        log("[CHECK 2/6] FAIL — GPS not locked")
        log("            Move to open sky and wait for fix")
        all_pass = False

    # 3 — EKF stability
    log("[CHECK 3/6] Waiting for EKF stability...")
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
        log(f"[CHECK 3/6] EKF stable  z={prev_z:.3f}m")
    else:
        log("[CHECK 3/6] FAIL — EKF not stable within timeout")
        log("            Wait longer after boot or check IMU calibration")
        all_pass = False

    # 4 — SSD writable
    ssd_path  = ROSBAG_DIR
    test_file = ssd_path / ".write_test"
    if ssd_path.exists():
        try:
            test_file.touch()
            test_file.unlink()
            log(f"[CHECK 4/6] SSD writable at {ssd_path}")
        except OSError:
            log("[CHECK 4/6] FAIL — SSD not writable")
            all_pass = False
    else:
        log("[CHECK 4/6] FAIL — SSD not mounted at /mnt/ssd")
        log("            Run: sudo mount -a")
        all_pass = False

    # 5 — Point-LIO launch file
    lf = Path(os.path.expanduser(LAUNCH_FILE))
    if lf.exists():
        log(f"[CHECK 5/6] Launch file present: {lf.name}")
    else:
        log(f"[CHECK 5/6] FAIL — Launch file not found: {LAUNCH_FILE}")
        all_pass = False

    # 6 — Mission waypoints from QGC
    log("[CHECK 6/6] Waiting for mission waypoints...")
    deadline = time.time() + WP_TIMEOUT
    while time.time() < deadline:
        if node.get_nav_waypoints():
            break
        time.sleep(0.5)
    nav_wps = node.get_nav_waypoints()
    if nav_wps:
        log(f"[CHECK 6/6] {len(nav_wps)} NAV_WAYPOINT(s) received from PX4")
    else:
        log("[CHECK 6/6] FAIL — No waypoints found")
        log("            Upload a survey mission in QGC first")
        all_pass = False

    log("=" * 50)
    if all_pass:
        log("[MODE 3] ALL CHECKS PASSED — mission starting")
    else:
        log("[MODE 3] PRE-FLIGHT CHECKS FAILED — returning to IDLE")
        log("         Disarm, fix the issues above, and re-arm to retry")
    log("=" * 50)
    return all_pass

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
    # Lock is intentionally left for the watchdog to clear after post-flight


def handle_autonomous(node: MainNode) -> None:
    """MODE 3 — Autonomous survey. main.py owns the full flight stack.

    Pre-flight checks must all pass before the stack is launched. The lock
    file is written to signal the watchdog to yield. GapDetector is imported
    from flight/gap_detector.py for real-time coverage monitoring. OFFBOARD
    loss is tolerated for up to OFFBOARD_RESUME_S seconds before RTL.
    """
    log_mode(FlightMode.AUTONOMOUS, "Starting autonomous survey mission")

    if not run_preflight_checks(node):
        log("[MODE 3] Aborting — disarm and resolve pre-flight failures to retry")
        while node.armed:
            time.sleep(0.5)
        return

    write_lock("autonomous")

    # ── Launch Flight Stack ───────────────────────────────────────────────────
    pointlio_proc = start_proc(
        "Point-LIO",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0",
    )
    log(f"Waiting {POINTLIO_INIT_S:.0f}s for Point-LIO initialisation...")
    time.sleep(POINTLIO_INIT_S)

    bridge_proc = start_proc(
        "SLAM bridge",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"python3 {BRIDGE_SCRIPT}",
    )
    time.sleep(BRIDGE_INIT_S)

    ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_out = ROSBAG_DIR / f"scan_{ts}"
    bag_proc = start_proc(
        "Bag recorder",
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 bag record -o {bag_out} {' '.join(BAG_TOPICS)}",
    )
    log("Flight stack running")

    # ── Load Waypoints ────────────────────────────────────────────────────────
    home    = node.get_home()
    nav_wps = node.get_nav_waypoints()

    if not nav_wps or home is None:
        log("[FAIL] No waypoints or home position — switching to AUTO.RTL")
        node.set_mode("AUTO.RTL")
        stop_proc("Bag recorder", bag_proc)
        stop_proc("SLAM bridge",  bridge_proc)
        stop_proc("Point-LIO",    pointlio_proc)
        return

    enu_wps = [
        haversine_to_enu(
            wp.x_lat, wp.y_long, wp.z_alt,
            home.geo.latitude, home.geo.longitude, home.geo.altitude,
        )
        for wp in nav_wps
    ]
    log(f"Mission: {len(enu_wps)} waypoints loaded")

    # ── Pre-stream Setpoints (required before OFFBOARD is accepted) ───────────
    hx, hy, hz = node.get_pos()
    log("Pre-streaming setpoints (3 s)...")
    node.stream_sp(hx, hy, hz, 0.0, 3.0)

    # ── Initialise Gap Detector ───────────────────────────────────────────────
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

    # ── Waypoint Execution ────────────────────────────────────────────────────
    offboard_lost_at = None

    for i, (ex, ey, ez) in enumerate(enu_wps):
        log(f"Waypoint {i + 1}/{len(enu_wps)}: ENU=({ex:.1f}, {ey:.1f}, {ez:.1f})")

        # Handle OFFBOARD loss — pause mission, give RC control, wait or RTL
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
                stop_proc("Bag recorder", bag_proc)
                stop_proc("SLAM bridge",  bridge_proc)
                stop_proc("Point-LIO",    pointlio_proc)
                return
            time.sleep(0.5)

        if offboard_lost_at is not None:
            log(f"[MODE 3] OFFBOARD restored — resuming from waypoint {i + 1}")
            offboard_lost_at = None

        if not node.armed:
            log("[MODE 3] Disarmed mid-mission — stopping")
            break

        # Fly to waypoint
        reached = node.fly_to(ex, ey, ez, 0.0, timeout=120.0)
        if not reached:
            log(f"[WARN] Waypoint {i + 1} not reached within timeout — continuing")

        # Gap fill at this waypoint
        if gap_fill_enabled and gap_det is not None:
            cx, cy, cz  = node.get_pos()
            fov_half    = math.radians(DEFAULT_FOV_DEG / 2.0)
            footprint_r = cz * math.tan(fov_half)
            gaps        = gap_det.find_gaps_near(cx, cy, radius_m=footprint_r)

            if gaps:
                log(f"  {len(gaps)} gap(s) detected — inserting gap-fill waypoints")
                for gx, gy in gaps[:3]:   # Limit to 3 gap fills per waypoint
                    node.fly_to(gx, gy, ez, 0.0, timeout=60.0)
                    node.fly_to(ex, ey, ez, 0.0, timeout=60.0)

        # Camera trigger stub — replaced when IMX477 is reinstalled
        node.stream_sp(ex, ey, ez, 0.0, 2.0)
        log(f"  [TRIGGER] WP {i + 1} ENU=({ex:.1f}, {ey:.1f}, {ez:.1f})")

    # ── Mission Complete → RTL ────────────────────────────────────────────────
    log("Mission complete — switching to AUTO.RTL")
    node.set_mode("AUTO.RTL")

    log("Waiting for landing and disarm...")
    while node.armed:
        time.sleep(0.5)
    log("Disarmed — watchdog will stop bag and run post-flight processing")

    stop_proc("Bag recorder", bag_proc)
    stop_proc("SLAM bridge",  bridge_proc)
    stop_proc("Point-LIO",    pointlio_proc)

# ── Main State Machine ────────────────────────────────────────────────────────

def main() -> None:
    log("=" * 55)
    log("DronePi Main Orchestrator — boot")
    log("Modes: NO_SCAN | MANUAL_SCAN | AUTONOMOUS")
    log("=" * 55)

    clear_lock()   # Ensure no stale lock from a previous session

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

        # ── IDLE ──────────────────────────────────────────────────────────────
        if current_mode == FlightMode.IDLE:
            if armed:
                if not lidar:
                    log_mode(FlightMode.IDLE,
                             "Armed, LiDAR absent → MODE 1 (no scan)")
                    current_mode = FlightMode.NO_SCAN
                    handle_no_scan(node)
                    current_mode = FlightMode.IDLE
                    clear_lock()
                else:
                    if debounce_start is None:
                        debounce_start = time.time()
                        log_mode(FlightMode.DEBOUNCE,
                                 f"Armed + LiDAR present — "
                                 f"waiting {DEBOUNCE_S}s to detect OFFBOARD")
                    current_mode = FlightMode.DEBOUNCE
            else:
                debounce_start = None
                log_mode(FlightMode.IDLE,
                         f"armed={armed}  mode={mode}  "
                         f"lidar={'yes' if lidar else 'no'}")

        # ── DEBOUNCE ──────────────────────────────────────────────────────────
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
                # Lock is cleared by watchdog after post-flight completes

            else:
                elapsed   = time.time() - debounce_start
                remaining = DEBOUNCE_S - elapsed
                log_mode(FlightMode.DEBOUNCE,
                         f"mode={mode}  {remaining:.1f}s remaining...")

        time.sleep(1.0 / POLL_HZ)


if __name__ == "__main__":
    main()
