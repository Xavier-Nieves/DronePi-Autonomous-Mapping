#!/usr/bin/env python3
"""
main.py — DronePi Flight Mode Orchestrator

Started automatically by dronepi-main.service on boot.
Monitors drone state and routes to the correct flight mode.

STATE MACHINE
-------------

IDLE
  LiDAR absent + armed       -> MODE 1  (no scan)
  LiDAR present + armed      -> DEBOUNCE (10s timer)

DEBOUNCE (10 seconds)
  OFFBOARD detected           -> MODE 3  (autonomous scan)  [lock applied]
  10s elapsed, no OFFBOARD    -> MODE 2  (manual scan)      [lock applied]
  disarm during debounce      -> IDLE    (cancelled)

MODE 1 — no scan, locked until disarm
  monitors state, logs only
  disarm                      -> IDLE

MODE 2 — manual scan, locked until disarm
  writes mission lock: manual_scan
  watchdog reads lock -> starts Point-LIO + bag
  OFFBOARD detected           -> IGNORED (mode locked)
  disarm                      -> watchdog stops bag + postflight -> IDLE

MODE 3 — autonomous scan, locked until disarm
  writes mission lock: autonomous
  main.py starts Point-LIO + SLAM bridge + bag
  executes QGC waypoints with gap fill
  OFFBOARD lost               -> mission paused, RC has control
    restored within 30s       -> mission resumes from current waypoint
    not restored after 30s    -> AUTO.RTL
  OFFBOARD change (non-lost)  -> IGNORED (mode locked)
  disarm                      -> watchdog stops bag + postflight -> IDLE

LOCK FILE
---------
/tmp/dronepi_mission.lock — JSON file written when a mode is committed.
Watchdog reads this on every loop to determine supervised behaviour.

  manual_scan  -> watchdog starts stack normally, skips postflight trigger
                  (main.py monitors, watchdog handles disarm + postflight)
  autonomous   -> watchdog skips stack launch entirely
                  (main.py owns the stack, watchdog handles disarm + postflight)
  absent       -> watchdog runs normally (standard arm+OFFBOARD detection)

LOGGING
-------
All output goes to stdout, captured by journalctl:
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

# ── paths ─────────────────────────────────────────────────────────────────────

PROJECT_ROOT  = Path(__file__).parent
FLIGHT_DIR    = PROJECT_ROOT / "flight"
UTILS_DIR     = PROJECT_ROOT / "utils"

ROS_SETUP     = "/opt/ros/jazzy/setup.bash"
WS_SETUP      = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE   = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
BRIDGE_SCRIPT      = FLIGHT_DIR / "_slam_bridge.py"
POSTFLIGHT_SCRIPT  = UTILS_DIR  / "run_postflight.py"
MISSION_EXEC       = FLIGHT_DIR / "mission_executor.py"

LIDAR_PORT    = Path("/dev/ttyUSB0")
MISSION_LOCK  = Path("/tmp/dronepi_mission.lock")

# ── constants ─────────────────────────────────────────────────────────────────

DEBOUNCE_S          = 10     # seconds after arm before mode is committed
HOME_TIMEOUT        = 30.0   # seconds to wait for GPS home position
WP_TIMEOUT          = 15.0   # seconds to wait for QGC waypoints
EKF_STABLE_COUNT    = 20     # consecutive stable Z readings required
EKF_TIMEOUT         = 40.0   # seconds to wait for EKF stability
OFFBOARD_RESUME_S   = 30     # seconds to wait for OFFBOARD restore before RTL
POLL_HZ             = 2      # state machine poll rate
SETPOINT_HZ         = 20     # OFFBOARD setpoint publish rate
POINTLIO_INIT_S     = 5      # seconds to wait for Point-LIO init
BRIDGE_INIT_S       = 2      # seconds to wait for SLAM bridge init
GRACEFUL_KILL_S     = 5      # seconds before SIGKILL after SIGINT
DEFAULT_FOV_DEG     = 70.0
DEFAULT_MIN_DENSITY = 5
WP_TOLERANCE_M      = 0.5

# ── flight modes ──────────────────────────────────────────────────────────────

class FlightMode(Enum):
    IDLE       = "IDLE"
    DEBOUNCE   = "DEBOUNCE"
    NO_SCAN    = "NO_SCAN"       # MODE 1
    MANUAL     = "MANUAL_SCAN"   # MODE 2
    AUTONOMOUS = "AUTONOMOUS"    # MODE 3

# ── logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)

def log_mode(mode: FlightMode, detail: str = ""):
    sep = f"  {detail}" if detail else ""
    log(f"[{mode.value}]{sep}")

# ── lock file ─────────────────────────────────────────────────────────────────

def write_lock(mode: str, extra: dict = None):
    """
    Write mission lock file.
    Watchdog reads this to determine supervised behaviour.
    mode: 'manual_scan' or 'autonomous'
    """
    payload = {"mode": mode, "started_at": datetime.now().isoformat()}
    if extra:
        payload.update(extra)
    MISSION_LOCK.write_text(json.dumps(payload))
    log(f"Lock written: {mode}")


def clear_lock():
    """Remove mission lock file. Watchdog returns to normal operation."""
    if MISSION_LOCK.exists():
        MISSION_LOCK.unlink()
        log("Lock cleared -- watchdog resuming normal operation")


def lock_exists() -> bool:
    return MISSION_LOCK.exists()

# ── process management ────────────────────────────────────────────────────────

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


def stop_proc(name: str, proc: subprocess.Popen):
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly")
    except subprocess.TimeoutExpired:
        log(f"{name} did not exit -- sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    except ProcessLookupError:
        pass

# ── LiDAR detection ───────────────────────────────────────────────────────────

def lidar_present() -> bool:
    """Check if LiDAR USB device exists."""
    return LIDAR_PORT.exists()


def lidar_publishing(node, timeout: float = 8.0) -> bool:
    """
    Wait for /unilidar/cloud to publish.
    Returns True if healthy, False if device exists but no data
    (indicates hardware issue -- loose cable, firmware crash).
    """
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

    received = threading.Event()

    def _cb(msg):
        received.set()

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    sub = node._node.create_subscription(
        PointCloud2, "/unilidar/cloud", _cb, qos)

    result = received.wait(timeout=timeout)
    node._node.destroy_subscription(sub)
    return result

# ── coordinate conversion ─────────────────────────────────────────────────────

def haversine_to_enu(lat, lon, alt, hlat, hlon, halt):
    R     = 6371000.0
    dlat  = math.radians(lat - hlat)
    dlon  = math.radians(lon - hlon)
    lm    = math.radians((lat + hlat) / 2.0)
    return dlon * R * math.cos(lm), dlat * R, alt - halt

# ── ROS node ──────────────────────────────────────────────────────────────────

class MainNode:
    """
    Minimal ROS 2 node for state monitoring and setpoint publishing.
    Shared across all flight modes.
    """

    def __init__(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import State, WaypointList, HomePosition
            from mavros_msgs.srv import SetMode
        except ImportError as e:
            log(f"[FAIL] ROS 2 not sourced: {e}")
            sys.exit(1)

        self._rclpy = rclpy
        self._PS    = PoseStamped
        self._lock  = threading.Lock()

        self.state     = State()
        self.pose      = None
        self.home      = None
        self.waypoints = []

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
            WaypointList, "/mavros/mission/waypoints",
            self._wp_cb, 10)
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
        log("ROS 2 node started")

    def _state_cb(self, msg):
        with self._lock: self.state = msg

    def _pose_cb(self, msg):
        with self._lock: self.pose = msg

    def _wp_cb(self, msg):
        with self._lock: self.waypoints = msg.waypoints

    def _home_cb(self, msg):
        with self._lock: self.home = msg

    @property
    def armed(self) -> bool:
        with self._lock: return self.state.armed

    @property
    def mode(self) -> str:
        with self._lock: return self.state.mode

    @property
    def connected(self) -> bool:
        with self._lock: return self.state.connected

    def get_pos(self):
        with self._lock:
            if self.pose is None:
                return 0.0, 0.0, 0.0
            p = self.pose.pose.position
            return p.x, p.y, p.z

    def get_home(self):
        with self._lock: return self.home

    def get_nav_waypoints(self):
        with self._lock:
            return [w for w in self.waypoints if w.command == 16]

    def publish_sp(self, ex, ey, ez, yaw=0.0):
        msg = self._PS()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = ex
        msg.pose.position.y = ey
        msg.pose.position.z = ez
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def stream_sp(self, ex, ey, ez, yaw, duration):
        end = time.time() + duration
        while time.time() < end:
            self.publish_sp(ex, ey, ez, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def fly_to(self, ex, ey, ez, yaw, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.publish_sp(ex, ey, ez, yaw)
            cx, cy, cz = self.get_pos()
            dist = math.sqrt((cx-ex)**2+(cy-ey)**2+(cz-ez)**2)
            print(f"\r    dist={dist:.2f}m", end="", flush=True)
            if dist < WP_TOLERANCE_M:
                print()
                return True
            time.sleep(1.0 / SETPOINT_HZ)
        print()
        return False

    def set_mode(self, mode: str) -> bool:
        from mavros_msgs.srv import SetMode
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self._mode_client.call_async(req)
        self._rclpy.spin_until_future_complete(
            self._node, fut, timeout_sec=5.0)
        return fut.result() and fut.result().mode_sent

    def shutdown(self):
        if self._rclpy.ok():
            self._rclpy.shutdown()

# ── mode handlers ─────────────────────────────────────────────────────────────

def run_preflight_checks(node: MainNode) -> bool:
    """
    Run pre-flight checks before autonomous mission.
    ALL six checks must pass -- any failure returns False
    and the mission is aborted back to IDLE.

    Checks:
      1. FCU connected
      2. GPS lock + home position
      3. EKF locally stable
      4. SSD mounted and writable
      5. Point-LIO launch file exists
      6. Mission waypoints uploaded in QGC
    """
    log("=" * 50)
    log("[MODE 3] PRE-FLIGHT CHECKS")
    log("=" * 50)
    all_pass = True

    # 1 -- FCU connected
    if node.connected:
        log(f"[CHECK 1/6] FCU connected  mode={node.mode}")
    else:
        log("[CHECK 1/6] FAIL -- FCU not connected")
        log("            Check Pixhawk USB cable")
        all_pass = False

    # 2 -- GPS lock + home position
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
        log("[CHECK 2/6] FAIL -- GPS not locked")
        log("            Move to open sky and wait for fix")
        all_pass = False

    # 3 -- EKF stability
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
        log("[CHECK 3/6] FAIL -- EKF not stable")
        log("            Wait longer after boot or check IMU")
        all_pass = False

    # 4 -- SSD mounted and writable
    ssd_path = Path("/mnt/ssd/rosbags")
    if ssd_path.exists():
        test_file = ssd_path / ".write_test"
        try:
            test_file.touch()
            test_file.unlink()
            log(f"[CHECK 4/6] SSD mounted and writable at {ssd_path}")
        except OSError:
            log("[CHECK 4/6] FAIL -- SSD not writable")
            all_pass = False
    else:
        log("[CHECK 4/6] FAIL -- SSD not mounted at /mnt/ssd")
        log("            Run: sudo mount -a")
        all_pass = False

    # 5 -- Point-LIO launch file
    lf = Path(os.path.expanduser(LAUNCH_FILE))
    if lf.exists():
        log(f"[CHECK 5/6] Launch file present: {lf.name}")
    else:
        log(f"[CHECK 5/6] FAIL -- Launch file not found: {LAUNCH_FILE}")
        all_pass = False

    # 6 -- Mission waypoints
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
        log("[CHECK 6/6] FAIL -- No waypoints")
        log("            Upload a survey mission in QGC first")
        all_pass = False

    log("=" * 50)
    if all_pass:
        log("[MODE 3] ALL CHECKS PASSED -- mission starting")
    else:
        log("[MODE 3] PRE-FLIGHT CHECKS FAILED -- returning to IDLE")
        log("         Fix the issues above and re-arm to retry")
    log("=" * 50)
    return all_pass


def handle_no_scan(node: MainNode):
    """
    MODE 1 — LiDAR absent, armed.
    Monitors state, logs only. Locked until disarm.
    """
    log_mode(FlightMode.NO_SCAN,
             "LiDAR absent -- no scanning active. Flying freely.")
    while node.armed:
        log_mode(FlightMode.NO_SCAN,
                 f"mode={node.mode}  (no scan)")
        time.sleep(1.0 / POLL_HZ)
    log_mode(FlightMode.NO_SCAN, "Disarmed -- returning to IDLE")


def handle_manual_scan(node: MainNode):
    """
    MODE 2 — LiDAR present, armed, OFFBOARD not active.
    Writes lock file so watchdog starts Point-LIO + bag.
    Monitors state. Ignores OFFBOARD changes (mode locked).
    Locked until disarm -- watchdog handles bag stop + postflight.
    """
    log_mode(FlightMode.MANUAL,
             "LiDAR present, manual flight. "
             "Watchdog starting Point-LIO + bag.")
    write_lock("manual_scan")

    while node.armed:
        if node.mode == "OFFBOARD":
            log(f"[LOCKED] OFFBOARD detected mid-flight -- ignored. "
                f"Disarm to change mode.")
        log_mode(FlightMode.MANUAL,
                 f"mode={node.mode}  scanning active")
        time.sleep(1.0 / POLL_HZ)

    log_mode(FlightMode.MANUAL,
             "Disarmed -- watchdog will stop bag and run postflight")
    # Lock is cleared by watchdog after postflight completes


def handle_autonomous(node: MainNode):
    """
    MODE 3 — LiDAR present, armed, OFFBOARD confirmed.
    main.py owns the full flight stack.
    Writes lock file so watchdog skips launch but handles disarm.
    Executes QGC waypoints with gap fill.
    Handles OFFBOARD loss: pauses mission, resumes within 30s or RTL.
    Locked until disarm.
    """
    log_mode(FlightMode.AUTONOMOUS, "Starting autonomous survey mission")

    # ── pre-flight checks -- must all pass before stack launches ─────────
    if not run_preflight_checks(node):
        log("[MODE 3] Aborting -- disarm and fix issues to retry")
        # Wait for disarm so state machine returns cleanly to IDLE
        while node.armed:
            time.sleep(0.5)
        return

    write_lock("autonomous")

    # ── launch flight stack ───────────────────────────────────────────────
    pointlio_cmd = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0"
    )
    bridge_cmd = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"python3 {BRIDGE_SCRIPT}"
    )
    bag_cmd = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 bag record -o /mnt/ssd/rosbags/scan_"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')} "
        f"/cloud_registered /aft_mapped_to_init /unilidar/imu "
        f"/mavros/state /mavros/local_position/pose "
        f"/mavros/global_position/global"
    )

    pointlio_proc = start_proc("Point-LIO",   pointlio_cmd)
    log(f"Waiting {POINTLIO_INIT_S:.0f}s for Point-LIO init...")
    time.sleep(POINTLIO_INIT_S)

    bridge_proc = start_proc("SLAM bridge",  bridge_cmd)
    time.sleep(BRIDGE_INIT_S)

    bag_proc = start_proc("Bag recorder", bag_cmd)
    log("Flight stack running")

    # ── get waypoints ─────────────────────────────────────────────────────
    home    = node.get_home()
    nav_wps = node.get_nav_waypoints()

    if not nav_wps or home is None:
        log("[FAIL] No waypoints or home position -- switching to AUTO.RTL")
        node.set_mode("AUTO.RTL")
        stop_proc("Bag recorder", bag_proc)
        stop_proc("SLAM bridge",  bridge_proc)
        stop_proc("Point-LIO",    pointlio_proc)
        return

    enu_wps = []
    for wp in nav_wps:
        e, n, u = haversine_to_enu(
            wp.x_lat, wp.y_long, wp.z_alt,
            home.geo.latitude, home.geo.longitude, home.geo.altitude)
        enu_wps.append((e, n, u))

    log(f"Mission: {len(enu_wps)} waypoints loaded")

    # ── pre-stream setpoints ──────────────────────────────────────────────
    hx, hy, hz = node.get_pos()
    log("Pre-streaming setpoints (3s)...")
    node.stream_sp(hx, hy, hz, 0.0, 3.0)

    # ── execute waypoints ─────────────────────────────────────────────────
    sys.path.insert(0, str(FLIGHT_DIR))
    try:
        from mission_executor import GapDetector
        fov_half    = math.radians(DEFAULT_FOV_DEG / 2.0)
        alt         = enu_wps[0][2] if enu_wps else 10.0
        footprint_r = alt * math.tan(fov_half)
        gap_det     = GapDetector(footprint_r * 2.0, DEFAULT_MIN_DENSITY)
        gap_det.start(node._node)
        gap_fill_available = True
    except ImportError:
        log("[WARN] GapDetector not available -- gap fill disabled")
        gap_fill_available = False

    offboard_lost_at = None

    for i, (ex, ey, ez) in enumerate(enu_wps):
        log(f"Waypoint {i+1}/{len(enu_wps)}: ENU=({ex:.1f},{ey:.1f},{ez:.1f})")

        # ── OFFBOARD loss handling ────────────────────────────────────────
        while node.mode != "OFFBOARD" and node.armed:
            if offboard_lost_at is None:
                offboard_lost_at = time.time()
                log(f"[MODE 3] OFFBOARD lost -- RC has control. "
                    f"Resuming in {OFFBOARD_RESUME_S}s if restored.")
            elapsed = time.time() - offboard_lost_at
            if elapsed > OFFBOARD_RESUME_S:
                log(f"[MODE 3] OFFBOARD not restored after "
                    f"{OFFBOARD_RESUME_S}s -- switching to AUTO.RTL")
                node.set_mode("AUTO.RTL")
                # Wait for disarm -- watchdog handles bag + postflight
                while node.armed:
                    time.sleep(0.5)
                stop_proc("Bag recorder", bag_proc)
                stop_proc("SLAM bridge",  bridge_proc)
                stop_proc("Point-LIO",    pointlio_proc)
                return
            time.sleep(0.5)

        if offboard_lost_at is not None:
            log(f"[MODE 3] OFFBOARD restored -- "
                f"resuming from waypoint {i+1}")
            offboard_lost_at = None

        if not node.armed:
            log("[MODE 3] Disarmed mid-mission -- stopping")
            break

        # ── fly to waypoint ───────────────────────────────────────────────
        reached = node.fly_to(ex, ey, ez, 0.0, timeout=120.0)
        if not reached:
            log(f"[WARN] WP {i+1} not reached -- continuing")

        # ── gap detection ─────────────────────────────────────────────────
        if gap_fill_available:
            cx, cy, cz = node.get_pos()
            gaps = gap_det.find_gaps_near(cx, cy, footprint_r)
            if gaps:
                log(f"  {len(gaps)} gap(s) -- inserting gap fills")
                for gx, gy in gaps[:3]:
                    node.fly_to(gx, gy, ez, 0.0, timeout=60.0)
                    node.fly_to(ex, ey, ez, 0.0, timeout=60.0)

        # ── camera trigger stub ───────────────────────────────────────────
        # Replaced by actual camera_trigger.py call when IMX477 reinstalled
        node.stream_sp(ex, ey, ez, 0.0, 2.0)
        log(f"  [TRIGGER] WP {i+1} ENU=({ex:.1f},{ey:.1f},{ez:.1f})")

    # ── mission complete → RTL ────────────────────────────────────────────
    log("Mission complete -- switching to AUTO.RTL")
    node.set_mode("AUTO.RTL")

    # Wait for disarm (landing)
    log("Waiting for landing and disarm...")
    while node.armed:
        time.sleep(0.5)
    log("Disarmed -- watchdog will stop bag and run postflight")

    # Stop autonomous stack (watchdog handles bag via lock file)
    stop_proc("Bag recorder", bag_proc)
    stop_proc("SLAM bridge",  bridge_proc)
    stop_proc("Point-LIO",    pointlio_proc)

# ── main state machine ────────────────────────────────────────────────────────

def main():
    log("=" * 55)
    log("DronePi Main Orchestrator -- boot")
    log("Modes: NO_SCAN | MANUAL_SCAN | AUTONOMOUS")
    log("=" * 55)

    # Ensure no stale lock from previous session
    clear_lock()

    node = MainNode()

    # Wait for FCU connection
    log("Waiting for FCU connection...")
    deadline = time.time() + 30.0
    while time.time() < deadline:
        if node.connected:
            break
        time.sleep(0.5)
    if node.connected:
        log(f"FCU connected  mode={node.mode}")
    else:
        log("[WARN] FCU not connected -- will retry on each arm detection")

    def _shutdown(signum=None, frame=None):
        log("Shutdown signal received")
        clear_lock()
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── state machine loop ────────────────────────────────────────────────
    current_mode  = FlightMode.IDLE
    debounce_start = None

    while True:
        armed   = node.armed
        mode    = node.mode
        lidar   = lidar_present()

        # ── IDLE ──────────────────────────────────────────────────────────
        if current_mode == FlightMode.IDLE:
            if armed:
                if not lidar:
                    # MODE 1 — no scan, commit immediately
                    log_mode(FlightMode.IDLE,
                             "Armed, LiDAR absent -> MODE 1 (no scan)")
                    current_mode = FlightMode.NO_SCAN
                    handle_no_scan(node)
                    current_mode = FlightMode.IDLE
                    clear_lock()
                else:
                    # Start debounce timer
                    if debounce_start is None:
                        debounce_start = time.time()
                        log_mode(FlightMode.DEBOUNCE,
                                 f"Armed + LiDAR present -- "
                                 f"waiting {DEBOUNCE_S}s to detect OFFBOARD")
                    current_mode = FlightMode.DEBOUNCE
            else:
                debounce_start = None
                log_mode(FlightMode.IDLE,
                         f"armed={armed}  mode={mode}  "
                         f"lidar={'yes' if lidar else 'no'}")

        # ── DEBOUNCE ──────────────────────────────────────────────────────
        elif current_mode == FlightMode.DEBOUNCE:
            if not armed:
                log_mode(FlightMode.DEBOUNCE,
                         "Disarmed during debounce -- cancelled, back to IDLE")
                current_mode  = FlightMode.IDLE
                debounce_start = None

            elif mode == "OFFBOARD":
                log_mode(FlightMode.DEBOUNCE,
                         "OFFBOARD detected -- committing MODE 3 (autonomous)")
                current_mode  = FlightMode.AUTONOMOUS
                debounce_start = None
                handle_autonomous(node)
                current_mode  = FlightMode.IDLE
                clear_lock()

            elif time.time() - debounce_start >= DEBOUNCE_S:
                log_mode(FlightMode.DEBOUNCE,
                         f"{DEBOUNCE_S}s elapsed, no OFFBOARD -- "
                         f"committing MODE 2 (manual scan)")
                current_mode  = FlightMode.MANUAL
                debounce_start = None
                handle_manual_scan(node)
                current_mode  = FlightMode.IDLE
                # Lock cleared by watchdog after postflight

            else:
                elapsed = time.time() - debounce_start
                remaining = DEBOUNCE_S - elapsed
                log_mode(FlightMode.DEBOUNCE,
                         f"mode={mode}  waiting {remaining:.1f}s more...")

        time.sleep(1.0 / POLL_HZ)


if __name__ == "__main__":
    main()
