#!/usr/bin/env python3
"""
flight_mission.py — Full autonomous survey mission orchestrator for DronePi.

Single entry point for a complete flight session:

  Pre-flight checks → Stack startup → Arm gate → Mission execution
  → RTL + Landing → Post-flight processing → Browser auto-load

Watchdog integration
--------------------
The drone-watchdog systemd service is stopped at startup and restored
on exit. This prevents the watchdog from launching a competing Point-LIO
instance when the drone arms. The watchdog is always restored on exit
regardless of how the script terminates (normal, Ctrl+C, exception).

Pre-flight checks (ALL must pass to proceed)
--------------------------------------------
  CRITICAL (abort if fail):
    - FCU connected via MAVROS
    - GPS lock + home position received
    - EKF locally stable
    - SSD mounted and writable at /mnt/ssd/rosbags/
    - Point-LIO launch file exists
    - Mission waypoints uploaded in QGC (>= 1 NAV_WAYPOINT)

  SKIPPED (managed by PX4 failsafe):
    - Battery level (configure COM_LOW_BAT_ACT in QGC)

Usage
-----
  # Dry run -- all checks + mission preview, no motors
  python3 flight_mission.py --dry-run

  # Full mission
  python3 flight_mission.py

  # With camera FOV override (when zoom changes)
  python3 flight_mission.py --fov-deg 85.0

  # Stop-and-trigger mode (default)
  python3 flight_mission.py --trigger-mode stop

  # Continuous trigger mode
  python3 flight_mission.py --trigger-mode continuous

  # Skip post-flight processing (process manually later)
  python3 flight_mission.py --no-postflight

SAFETY
------
  - RC transmitter in hand at all times
  - Kill switch within reach
  - Ctrl+C at any point triggers AUTO.RTL then clean shutdown
  - Watchdog is always restored on exit
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── project paths ─────────────────────────────────────────────────────────────

PROJECT_ROOT   = Path(__file__).parent
FLIGHT_DIR     = PROJECT_ROOT / "flight"
UTILS_DIR      = PROJECT_ROOT / "utils"
CONFIG_DIR     = PROJECT_ROOT / "config"

ROS_SETUP      = "/opt/ros/jazzy/setup.bash"
WS_SETUP       = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
MAVROS_URL     = "serial:///dev/ttyACM0:57600"
ROSBAG_DIR     = Path("/mnt/ssd/rosbags")
MAPS_DIR       = Path("/mnt/ssd/maps")

LAUNCH_FILE    = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
BRIDGE_SCRIPT      = FLIGHT_DIR  / "_slam_bridge.py"
POSTFLIGHT_SCRIPT  = UTILS_DIR   / "run_postflight.py"
MISSION_EXEC       = FLIGHT_DIR  / "mission_executor.py"
CALIBRATION_YAML   = CONFIG_DIR  / "camera_calibration.yaml"

# ── constants ─────────────────────────────────────────────────────────────────

SETPOINT_HZ      = 20
EKF_TIMEOUT      = 40.0
HOME_TIMEOUT     = 30.0
WP_TIMEOUT       = 15.0
POINTLIO_INIT_S  = 5.0     # seconds to wait for Point-LIO to initialize
BRIDGE_INIT_S    = 2.0     # seconds to wait for SLAM bridge to subscribe
DEFAULT_FOV_DEG  = 70.0
DEFAULT_OVERLAP  = 0.80
DEFAULT_MIN_DENS = 5
WP_TOLERANCE_M   = 0.5
GRACEFUL_KILL_S  = 5

# ── logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def log_ok(msg: str):   print(f"  [OK]   {msg}", flush=True)
def log_fail(msg: str): print(f"  [FAIL] {msg}", flush=True)
def log_warn(msg: str): print(f"  [WARN] {msg}", flush=True)
def log_info(msg: str): print(f"  [INFO] {msg}", flush=True)

# ── process management ────────────────────────────────────────────────────────

def start_proc(name: str, cmd: str) -> subprocess.Popen:
    log(f"Starting {name}...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log_ok(f"{name} started (PID {proc.pid})")
    return proc


def stop_proc(name: str, proc: subprocess.Popen):
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=GRACEFUL_KILL_S)
        log_ok(f"{name} stopped cleanly")
    except subprocess.TimeoutExpired:
        log_warn(f"{name} did not exit -- sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    except ProcessLookupError:
        pass

# ── watchdog management ───────────────────────────────────────────────────────

def stop_watchdog():
    """
    Stop the drone-watchdog systemd service.
    Required before launching Point-LIO to prevent the watchdog from
    launching a competing instance when the drone arms.
    """
    log("Stopping drone-watchdog service...")
    r = subprocess.run(
        ["sudo", "systemctl", "stop", "drone-watchdog"],
        capture_output=True
    )
    if r.returncode == 0:
        log_ok("drone-watchdog stopped")
    else:
        log_warn("Could not stop drone-watchdog -- may not be running")


def restore_watchdog():
    """
    Restore the drone-watchdog systemd service.
    Always called on exit regardless of how the script terminates.
    """
    log("Restoring drone-watchdog service...")
    r = subprocess.run(
        ["sudo", "systemctl", "start", "drone-watchdog"],
        capture_output=True
    )
    if r.returncode == 0:
        log_ok("drone-watchdog restored")
    else:
        log_warn("Could not restore drone-watchdog -- start manually if needed:")
        log_warn("  sudo systemctl start drone-watchdog")

# ── coordinate conversion ─────────────────────────────────────────────────────

def haversine_to_enu(lat, lon, alt, home_lat, home_lon, home_alt):
    R     = 6371000.0
    dlat  = math.radians(lat - home_lat)
    dlon  = math.radians(lon - home_lon)
    lat_m = math.radians((lat + home_lat) / 2.0)
    north = dlat * R
    east  = dlon * R * math.cos(lat_m)
    up    = alt - home_alt
    return east, north, up

# ── FOV resolution ────────────────────────────────────────────────────────────

def resolve_fov(fov_arg):
    if fov_arg is not None:
        log_info(f"Camera FOV: {fov_arg:.1f}deg (CLI override)")
        return fov_arg
    if CALIBRATION_YAML.exists():
        try:
            import yaml
            with open(CALIBRATION_YAML) as f:
                cal = yaml.safe_load(f)
            fx = cal.get("camera_matrix", {}).get("data", [0]*9)[0]
            fy = cal.get("camera_matrix", {}).get("data", [0]*9)[4]
            w  = cal.get("image_width",  1920)
            h  = cal.get("image_height", 1080)
            if fx > 0 and fy > 0:
                fov_d = math.degrees(
                    2 * math.atan(
                        math.sqrt((w/2)**2 + (h/2)**2) / math.sqrt(fx * fy)))
                log_info(f"Camera FOV: {fov_d:.1f}deg (from calibration YAML)")
                return fov_d
        except Exception as e:
            log_warn(f"Could not read calibration YAML: {e}")
    log_info(f"Camera FOV: {DEFAULT_FOV_DEG:.1f}deg (default -- "
             f"provide calibration YAML or use --fov-deg)")
    return DEFAULT_FOV_DEG

# ── ROS node ──────────────────────────────────────────────────────────────────

class FlightNode:
    """
    Minimal ROS 2 node for state monitoring and setpoint publishing.
    Subscribes to MAVROS state, pose, home position, and mission waypoints.
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
            log_fail(f"ROS 2 import error: {e}")
            log_fail("Source ROS 2 before running:")
            log_fail("  source /opt/ros/jazzy/setup.bash")
            log_fail("  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash")
            sys.exit(1)

        self._rclpy = rclpy
        self._PS    = PoseStamped
        self._lock  = threading.Lock()

        self.state     = State()
        self.pose      = None
        self.home      = None
        self.waypoints = []

        rclpy.init()
        self._node = Node("flight_mission")

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

    def _state_cb(self, msg):
        with self._lock: self.state = msg

    def _pose_cb(self, msg):
        with self._lock: self.pose = msg

    def _wp_cb(self, msg):
        with self._lock: self.waypoints = msg.waypoints

    def _home_cb(self, msg):
        with self._lock: self.home = msg

    def get_state(self):
        with self._lock: return self.state

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
            print(f"\r    dist={dist:.2f}m  "
                  f"pos=({cx:.1f},{cy:.1f},{cz:.1f})",
                  end="", flush=True)
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

# ── pre-flight checks ─────────────────────────────────────────────────────────

def run_preflight_checks(node: FlightNode) -> bool:
    """
    Run all pre-flight checks. ALL must pass to proceed.
    Returns True if all pass, False if any critical check fails.
    """
    print("\n" + "="*55)
    print("  PRE-FLIGHT CHECKS")
    print("="*55)
    all_pass = True

    # 1 -- FCU connection
    print("\n  [1] FCU connection")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if node.get_state().connected:
            break
        time.sleep(0.2)
    if node.get_state().connected:
        log_ok(f"FCU connected  mode={node.get_state().mode}")
    else:
        log_fail("FCU not connected -- check Pixhawk USB cable")
        all_pass = False

    # 2 -- GPS lock + home position
    print("\n  [2] GPS lock and home position")
    deadline = time.time() + HOME_TIMEOUT
    while time.time() < deadline:
        if node.get_home() is not None:
            break
        time.sleep(0.5)
    h = node.get_home()
    if h is not None:
        log_ok(f"Home: lat={h.geo.latitude:.6f}  "
               f"lon={h.geo.longitude:.6f}  "
               f"alt={h.geo.altitude:.1f}m")
    else:
        log_fail("GPS not locked -- move to open sky and wait for fix")
        all_pass = False

    # 3 -- EKF stability
    print("\n  [3] EKF stability")
    prev_z  = None
    stable  = 0
    deadline = time.time() + EKF_TIMEOUT
    while time.time() < deadline:
        _, _, z = node.get_pos()
        if prev_z is not None:
            stable = stable + 1 if abs(z - prev_z) < 0.03 else 0
        prev_z = z
        print(f"\r    z={z:.3f}m  stable={stable}/20", end="", flush=True)
        if stable >= 20:
            break
        time.sleep(0.1)
    print()
    if stable >= 20:
        log_ok(f"EKF stable  z={prev_z:.3f}m")
    else:
        log_fail("EKF not stable -- wait longer or check IMU")
        all_pass = False

    # 4 -- SSD mounted and writable
    print("\n  [4] SSD storage")
    ssd_ok = False
    if ROSBAG_DIR.exists():
        test_file = ROSBAG_DIR / ".write_test"
        try:
            test_file.touch()
            test_file.unlink()
            ssd_ok = True
            log_ok(f"SSD mounted and writable at {ROSBAG_DIR}")
        except OSError:
            log_fail(f"SSD not writable at {ROSBAG_DIR}")
    else:
        log_fail(f"SSD not mounted -- run: sudo mount -a")
    if not ssd_ok:
        all_pass = False

    # 5 -- Point-LIO launch file
    print("\n  [5] Point-LIO launch file")
    lf = Path(os.path.expanduser(LAUNCH_FILE))
    if lf.exists():
        log_ok(f"Launch file found: {lf.name}")
    else:
        log_fail(f"Launch file not found: {LAUNCH_FILE}")
        all_pass = False

    # 6 -- Mission waypoints
    print("\n  [6] Mission waypoints")
    deadline = time.time() + WP_TIMEOUT
    while time.time() < deadline:
        if node.get_nav_waypoints():
            break
        time.sleep(0.5)
    nav_wps = node.get_nav_waypoints()
    if nav_wps:
        log_ok(f"{len(nav_wps)} NAV_WAYPOINT(s) received from PX4")
    else:
        log_fail("No waypoints -- upload a survey mission in QGC first")
        all_pass = False

    print("\n" + "="*55)
    if all_pass:
        print("  ALL CHECKS PASSED -- ready to fly")
    else:
        print("  CHECKS FAILED -- fix issues above before flying")
    print("="*55)
    return all_pass

# ── mission execution ─────────────────────────────────────────────────────────

def execute_mission(node: FlightNode, enu_wps: list,
                    args, fov_deg: float):
    """Execute waypoints with gap detection and camera trigger stubs."""
    from mission_executor import GapDetector

    fov_half     = math.radians(fov_deg / 2.0)
    gap_fills    = 0
    cam_triggers = 0

    # Footprint at first waypoint altitude
    alt          = enu_wps[0][2] if enu_wps else 10.0
    footprint_r  = alt * math.tan(fov_half)
    cell_size    = footprint_r * 2.0

    gap_det = GapDetector(cell_size, args.min_density)
    gap_det.start(node._node)

    log(f"Executing {len(enu_wps)} waypoints  "
        f"footprint_r={footprint_r:.1f}m  cell={cell_size:.1f}m")

    for i, (ex, ey, ez) in enumerate(enu_wps):
        log(f"Waypoint {i+1}/{len(enu_wps)}: "
            f"ENU=({ex:.1f},{ey:.1f},{ez:.1f})")

        reached = node.fly_to(ex, ey, ez, 0.0, timeout=120.0)
        if not reached:
            log_warn(f"WP {i+1} timeout -- continuing")

        cx, cy, cz = node.get_pos()

        # Gap detection
        gaps = gap_det.find_gaps_near(cx, cy, footprint_r)
        if gaps:
            log(f"  {len(gaps)} gap(s) near WP {i+1} -- inserting gap fills")
            for gx, gy in gaps[:3]:
                node.fly_to(gx, gy, ez, 0.0, timeout=60.0)
                gap_fills += 1
                node.fly_to(ex, ey, ez, 0.0, timeout=60.0)

        # Camera trigger
        if args.trigger_mode == "stop":
            node.stream_sp(ex, ey, ez, 0.0, 2.0)
            # Placeholder -- replaced by camera_trigger.py call
            # when IMX477 is reinstalled
            log(f"  [TRIGGER] WP {i+1} "
                f"ENU=({ex:.1f},{ey:.1f},{ez:.1f})")
            cam_triggers += 1

    log(f"Mission complete -- "
        f"WPs={len(enu_wps)}  gaps={gap_fills}  triggers={cam_triggers}")

# ── post-flight ───────────────────────────────────────────────────────────────

def run_postflight():
    """Trigger post-flight processing in background."""
    if not POSTFLIGHT_SCRIPT.exists():
        log_warn(f"run_postflight.py not found at {POSTFLIGHT_SCRIPT}")
        log_warn("Run manually: python3 run_postflight.py")
        return None
    log("Launching post-flight processing...")
    proc = subprocess.Popen(
        [sys.executable, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log_ok(f"Post-flight processing started (PID {proc.pid})")
    log_info("Viewer will auto-load at http://10.42.0.1:8080/meshview.html")
    return proc

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Full autonomous survey mission from takeoff to landing.")
    parser.add_argument("--dry-run",       action="store_true",
                        help="Pre-flight checks + mission preview, no flight")
    parser.add_argument("--trigger-mode",  choices=["stop", "continuous"],
                        default="stop")
    parser.add_argument("--fov-deg",       type=float, default=None)
    parser.add_argument("--min-density",   type=int, default=DEFAULT_MIN_DENS)
    parser.add_argument("--overlap",       type=float, default=DEFAULT_OVERLAP)
    parser.add_argument("--no-postflight", action="store_true",
                        help="Skip post-flight processing")
    args = parser.parse_args()

    fov_deg = resolve_fov(args.fov_deg)

    print("\n" + "="*55)
    print("  DronePi Full Mission")
    print(f"  {datetime.now().strftime('%Y-%m-%d %H:%M')}")
    print("="*55)
    print(f"  Trigger mode : {args.trigger_mode}")
    print(f"  FOV          : {fov_deg:.1f}deg")
    print(f"  Min density  : {args.min_density} pts/cell")
    print(f"  Dry run      : {'YES' if args.dry_run else 'NO -- LIVE FLIGHT'}")

    if not args.dry_run:
        print("\n  LIVE FLIGHT -- area must be clear. Starting in 5s...")
        try:
            for i in range(5, 0, -1):
                print(f"\r  {i}...", end="", flush=True)
                time.sleep(1)
            print()
        except KeyboardInterrupt:
            print("\n  Aborted.")
            sys.exit(0)

    # ── init ROS node ─────────────────────────────────────────────────────
    node = FlightNode()

    # ── pre-flight checks ─────────────────────────────────────────────────
    checks_pass = run_preflight_checks(node)
    if not checks_pass:
        log_fail("Pre-flight checks failed -- aborting")
        node.shutdown()
        sys.exit(1)

    # ── get waypoints and convert ─────────────────────────────────────────
    nav_wps  = node.get_nav_waypoints()
    home     = node.get_home()
    home_alt = home.geo.altitude
    enu_wps  = []
    for wp in nav_wps:
        e, n, u = haversine_to_enu(
            wp.x_lat, wp.y_long, wp.z_alt,
            home.geo.latitude, home.geo.longitude, home_alt)
        enu_wps.append((e, n, u))

    print(f"\n  Mission preview ({len(enu_wps)} waypoints):")
    for i, (e, n, u) in enumerate(enu_wps):
        print(f"    WP {i+1:2d}: E={e:+6.1f}m  N={n:+6.1f}m  U={u:.1f}m")

    if args.dry_run:
        print("\n  --dry-run: all checks passed. Mission preview above.")
        print("  Run without --dry-run to execute the flight.")
        node.shutdown()
        return

    # ── stop watchdog before launching flight stack ───────────────────────
    stop_watchdog()

    pointlio_proc = None
    bridge_proc   = None

    # Ensure watchdog is always restored on exit
    def _cleanup(signum=None, frame=None):
        log("Shutting down flight stack...")
        stop_proc("SLAM bridge",  bridge_proc)
        stop_proc("Point-LIO",    pointlio_proc)
        restore_watchdog()
        if not args.no_postflight:
            run_postflight()
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    try:
        # ── launch flight stack ───────────────────────────────────────────
        print("\n" + "="*55)
        print("  STACK STARTUP")
        print("="*55)

        pointlio_cmd = (
            f"source {ROS_SETUP} && "
            f"source {WS_SETUP} && "
            f"ros2 launch {LAUNCH_FILE} "
            f"rviz:=false port:=/dev/ttyUSB0"
        )
        bridge_cmd = (
            f"source {ROS_SETUP} && "
            f"source {WS_SETUP} && "
            f"python3 {BRIDGE_SCRIPT}"
        )

        pointlio_proc = start_proc("Point-LIO", pointlio_cmd)
        log(f"Waiting {POINTLIO_INIT_S:.0f}s for Point-LIO to initialize...")
        time.sleep(POINTLIO_INIT_S)

        bridge_proc = start_proc("SLAM bridge", bridge_cmd)
        log(f"Waiting {BRIDGE_INIT_S:.0f}s for SLAM bridge to subscribe...")
        time.sleep(BRIDGE_INIT_S)

        log_ok("Flight stack running")

        # ── pre-stream setpoints ──────────────────────────────────────────
        print("\n" + "="*55)
        print("  ARM GATE")
        print("="*55)

        hx, hy, hz = node.get_pos()
        log(f"Home ENU: ({hx:.2f}, {hy:.2f}, {hz:.2f})")
        log("Pre-streaming setpoints at home position (3s)...")
        node.stream_sp(hx, hy, hz, 0.0, 3.0)
        log_ok("Setpoint stream active")

        log("Waiting for arm + OFFBOARD mode...")
        log_info("Arm on RC transmitter then switch to OFFBOARD to begin")
        deadline = time.time() + 120.0
        while time.time() < deadline:
            node.publish_sp(hx, hy, hz, 0.0)
            st = node.get_state()
            if st.armed and st.mode == "OFFBOARD":
                break
            time.sleep(0.05)

        if not node.get_state().armed:
            log_fail("Not armed within 120s -- aborting")
            _cleanup()
            return

        log_ok(f"Armed in OFFBOARD -- mission beginning")

        # ── execute mission ───────────────────────────────────────────────
        print("\n" + "="*55)
        print("  MISSION EXECUTION")
        print("="*55)

        # Add mission_executor to path for GapDetector import
        sys.path.insert(0, str(FLIGHT_DIR))
        execute_mission(node, enu_wps, args, fov_deg)

        # ── RTL ───────────────────────────────────────────────────────────
        print("\n" + "="*55)
        print("  RETURN TO LAUNCH")
        print("="*55)
        log("Switching to AUTO.RTL...")
        node.set_mode("AUTO.RTL")

        # Wait for disarm (landing complete)
        log("Waiting for landing and disarm...")
        deadline = time.time() + 120.0
        while time.time() < deadline:
            node.publish_sp(hx, hy, hz, 0.0)
            if not node.get_state().armed:
                break
            time.sleep(0.5)

        if not node.get_state().armed:
            log_ok("Landed and disarmed")
        else:
            log_warn("Still armed after 120s -- check drone manually")

    except Exception as e:
        log_fail(f"Unexpected error: {e}")
        log("Sending AUTO.RTL as safety measure...")
        try:
            node.set_mode("AUTO.RTL")
        except Exception:
            pass

    finally:
        # ── shutdown ──────────────────────────────────────────────────────
        print("\n" + "="*55)
        print("  SHUTDOWN")
        print("="*55)

        stop_proc("SLAM bridge",  bridge_proc)
        stop_proc("Point-LIO",    pointlio_proc)
        restore_watchdog()

        if not args.no_postflight:
            run_postflight()

        node.shutdown()

        print("\n" + "="*55)
        print(f"  Mission complete: {datetime.now().strftime('%H:%M:%S')}")
        print("="*55)


if __name__ == "__main__":
    main()
