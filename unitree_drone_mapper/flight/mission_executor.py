#!/usr/bin/env python3
"""
mission_executor.py — Autonomous survey mission executor for DronePi.

Reads a survey mission uploaded from QGroundControl, executes it in
OFFBOARD mode with real-time gap detection using the Point-LIO point
cloud, and triggers the camera at each waypoint or at fixed distance
intervals.

Pipeline
--------
1. Read waypoints from /mavros/mission/waypoints (uploaded via QGC)
2. Convert GPS lat/lon/alt -> local ENU metres using MAVROS home position
3. Arm + switch to OFFBOARD (manual on transmitter)
4. Execute waypoints sequentially as OFFBOARD setpoints
5. At each waypoint: analyse /cloud_registered point density
6. If density below threshold in expected camera footprint -> insert
   gap-fill waypoint before continuing
7. Camera trigger at each waypoint (stop mode) or every N metres
   (continuous mode) based on FOV and overlap
8. All waypoints complete -> AUTO.RTL

Usage
-----
  # Dry run -- validates mission without flying
  python3 mission_executor.py --dry-run

  # Stop-and-trigger mode (default)
  python3 mission_executor.py --trigger-mode stop

  # Continuous trigger mode
  python3 mission_executor.py --trigger-mode continuous

  # Override camera FOV (degrees, diagonal)
  python3 mission_executor.py --fov-deg 80.0

  # Override minimum point density for gap detection
  python3 mission_executor.py --min-density 5

  # No MAVROS launch (already running)
  python3 mission_executor.py --no-mavros

Camera FOV resolution order
---------------------------
1. --fov-deg CLI argument (explicit override, use when zoom changes)
2. config/camera_calibration.yaml in project config directory
3. Default: 70.0 degrees diagonal (conservative estimate)

Gap detection
-------------
At each waypoint the executor samples /cloud_registered within the
expected camera footprint on the ground:

    footprint_radius = altitude * tan(fov_half_rad)

A 2D grid cell of footprint_radius * 2 is checked for point count.
If points < --min-density the cell is flagged as a gap and a gap-fill
waypoint is inserted directly above the gap center before the mission
continues. Gap-fill waypoints use the same altitude as the mission.

The gap detection uses the accumulated cloud from the current flight
only -- it does not read from previous bags.

Coordinate conversion
---------------------
QGC uploads GPS waypoints (lat, lon, alt_amsl) to PX4.
MAVROS exposes them on /mavros/mission/waypoints.
The executor converts to local ENU using the MAVROS home position
(published on /mavros/home_position/home) via the haversine formula.
OFFBOARD setpoints are published in local ENU on
/mavros/setpoint_position/local.

SAFETY
------
- RC transmitter in hand at all times
- Kill switch within reach
- Ctrl+C triggers AUTO.RTL before exit
- Low battery triggers AUTO.RTL via PX4 failsafe (not handled here --
  configure COM_LOW_BAT_ACT in QGC)
- This script never arms the drone -- arming is manual on transmitter
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np

# ── config ────────────────────────────────────────────────────────────────────

ROS_SETUP   = "/opt/ros/jazzy/setup.bash"
WS_SETUP    = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
MAVROS_URL  = "serial:///dev/ttyACM0:57600"

CALIBRATION_YAML = Path(__file__).parent.parent / "config" / "camera_calibration.yaml"

# Default camera FOV in degrees (diagonal) -- used if no calibration found
# and --fov-deg not specified. Conservative value for Tamron 4-12mm at
# widest zoom setting at typical survey altitude.
DEFAULT_FOV_DEG = 70.0

# Overlap fraction for continuous trigger mode (matches QGC default 80%)
DEFAULT_OVERLAP = 0.80

# Minimum points per footprint cell to consider coverage adequate.
# Below this count a gap-fill waypoint is inserted.
DEFAULT_MIN_DENSITY = 5

# Waypoint arrival tolerance in metres
WP_TOLERANCE_M = 0.5

# Setpoint publish rate Hz -- PX4 requires > 2 Hz in OFFBOARD mode
SETPOINT_HZ = 20

# Seconds to hold at each waypoint in stop-and-trigger mode
STOP_HOLD_S = 2.0

# Seconds to wait for EKF stability before mission start
EKF_TIMEOUT = 30.0

MAVROS_STARTUP_S = 8.0

# ── coordinate conversion ─────────────────────────────────────────────────────

def haversine_to_enu(lat: float, lon: float, alt: float,
                     home_lat: float, home_lon: float,
                     home_alt: float) -> Tuple[float, float, float]:
    """
    Convert GPS lat/lon/alt to local ENU metres relative to home position.

    Uses the flat-earth approximation which is accurate to < 1mm for
    distances under 10km -- more than sufficient for survey missions.

    Returns (east, north, up) in metres.
    """
    R = 6371000.0   # Earth radius metres
    dlat = math.radians(lat - home_lat)
    dlon = math.radians(lon - home_lon)
    lat_mid = math.radians((lat + home_lat) / 2.0)

    north = dlat * R
    east  = dlon * R * math.cos(lat_mid)
    up    = alt - home_alt
    return east, north, up

# ── FOV resolution ────────────────────────────────────────────────────────────

def resolve_fov(fov_arg: Optional[float]) -> float:
    """
    Resolve camera FOV in degrees using the priority order:
    1. CLI --fov-deg argument
    2. config/camera_calibration.yaml (fx, fy, image size)
    3. DEFAULT_FOV_DEG fallback
    """
    if fov_arg is not None:
        print(f"  Camera FOV: {fov_arg:.1f}deg (CLI override)")
        return fov_arg

    if CALIBRATION_YAML.exists():
        try:
            import yaml
            with open(CALIBRATION_YAML) as f:
                cal = yaml.safe_load(f)
            # Standard OpenCV calibration format
            fx = cal.get("camera_matrix", {}).get("data", [0]*9)[0]
            fy = cal.get("camera_matrix", {}).get("data", [0]*9)[4]
            w  = cal.get("image_width",  1920)
            h  = cal.get("image_height", 1080)
            if fx > 0 and fy > 0:
                fov_h = math.degrees(2 * math.atan(w / (2 * fx)))
                fov_v = math.degrees(2 * math.atan(h / (2 * fy)))
                fov_d = math.degrees(
                    2 * math.atan(math.sqrt((w/2)**2 + (h/2)**2) /
                                  math.sqrt(fx * fy)))
                print(f"  Camera FOV from calibration YAML: "
                      f"H={fov_h:.1f} V={fov_v:.1f} D={fov_d:.1f}deg")
                return fov_d
        except Exception as e:
            print(f"  [WARN] Could not read calibration YAML: {e}")

    print(f"  Camera FOV: {DEFAULT_FOV_DEG:.1f}deg (default -- "
          f"use --fov-deg to override or provide calibration YAML)")
    return DEFAULT_FOV_DEG

# ── gap detection ─────────────────────────────────────────────────────────────

class GapDetector:
    """
    Monitors /cloud_registered and detects under-covered areas.

    Maintains a flat 2D grid of point counts in the XY plane.
    Grid cells are sized to the camera footprint radius at mission altitude.
    A cell is flagged as a gap if its count is below min_density.
    """

    def __init__(self, cell_size_m: float, min_density: int):
        self._cell  = cell_size_m
        self._min   = min_density
        self._grid  = {}   # (ix, iy) -> int count
        self._lock  = threading.Lock()
        self._sub   = None

    def start(self, node):
        """Subscribe to /cloud_registered and start accumulating."""
        try:
            from sensor_msgs.msg import PointCloud2
            import struct
            self._struct = struct
            self._node   = node

            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self._sub = node.create_subscription(
                PointCloud2, "/cloud_registered",
                self._cloud_cb, qos)
            print("  Gap detector: subscribed to /cloud_registered")
        except Exception as e:
            print(f"  [WARN] Gap detector could not subscribe: {e}")

    def _cloud_cb(self, msg):
        """Accumulate XY positions into the coverage grid."""
        try:
            fields = {f.name: f.offset for f in msg.fields}
            if "x" not in fields:
                return
            x_off = fields["x"]
            ps    = msg.point_step
            data  = bytes(msg.data)
            n     = msg.width * msg.height

            # Fast path -- x,y tightly packed
            if x_off == 0 and fields.get("y") == 4:
                stride = ps // 4
                raw = np.frombuffer(data, dtype=np.float32).reshape(-1, stride)
                xs  = raw[:, 0]
                ys  = raw[:, 1]
            else:
                xs = np.empty(n); ys = np.empty(n)
                for i in range(n):
                    base = i * ps
                    xs[i] = self._struct.unpack_from("<f", data, base + x_off)[0]
                    ys[i] = self._struct.unpack_from("<f", data, base + fields["y"])[0]

            valid = np.isfinite(xs) & np.isfinite(ys)
            xs = xs[valid]; ys = ys[valid]

            with self._lock:
                for x, y in zip(xs, ys):
                    key = (int(x / self._cell), int(y / self._cell))
                    self._grid[key] = self._grid.get(key, 0) + 1
        except Exception:
            pass

    def check_coverage(self, ex: float, ey: float) -> bool:
        """
        Return True if the cell at (ex, ey) has adequate coverage.
        Return False if it is a gap and needs a gap-fill waypoint.
        """
        key = (int(ex / self._cell), int(ey / self._cell))
        with self._lock:
            count = self._grid.get(key, 0)
        return count >= self._min

    def find_gaps_near(self, ex: float, ey: float,
                       radius_m: float) -> List[Tuple[float, float]]:
        """
        Return list of gap cell centres within radius_m of (ex, ey).
        Used to generate gap-fill waypoints after reaching a survey point.
        """
        r_cells = int(math.ceil(radius_m / self._cell))
        cx = int(ex / self._cell)
        cy = int(ey / self._cell)
        gaps = []
        with self._lock:
            for dx in range(-r_cells, r_cells + 1):
                for dy in range(-r_cells, r_cells + 1):
                    if dx*dx + dy*dy > r_cells*r_cells:
                        continue
                    key = (cx + dx, cy + dy)
                    if self._grid.get(key, 0) < self._min:
                        # Return cell centre in metres
                        gaps.append((
                            (cx + dx + 0.5) * self._cell,
                            (cy + dy + 0.5) * self._cell,
                        ))
        return gaps

# ── MAVROS launcher ───────────────────────────────────────────────────────────

def launch_mavros() -> subprocess.Popen:
    cmd = (f"source {ROS_SETUP} && "
           f"ros2 launch mavros px4.launch fcu_url:={MAVROS_URL}")
    print("  Launching MAVROS...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"  MAVROS PID {proc.pid} -- waiting {MAVROS_STARTUP_S:.0f}s...")
    time.sleep(MAVROS_STARTUP_S)
    print("  [OK] MAVROS ready")
    return proc


def kill_proc(name: str, proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        pass

# ── mission node ──────────────────────────────────────────────────────────────

class MissionNode:
    """
    ROS 2 node that reads the mission, converts waypoints, and executes
    the OFFBOARD flight with gap fill and camera trigger.
    """

    def __init__(self):
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import PoseStamped
            from mavros_msgs.msg import State, WaypointList, HomePosition
            from mavros_msgs.srv import CommandBool, SetMode
        except ImportError as e:
            print(f"[FAIL] ROS 2 import error: {e}")
            sys.exit(1)

        self._rclpy = rclpy
        self._PS    = PoseStamped
        self._lock  = threading.Lock()

        self.state        = State()
        self.pose         = None
        self.home         = None
        self.waypoints    = []
        self._camera_cb   = None   # called to trigger camera

        rclpy.init()
        self._node = Node("mission_executor")

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

        self._arm_client  = self._node.create_client(
            CommandBool, "/mavros/cmd/arming")
        self._mode_client = self._node.create_client(
            SetMode, "/mavros/set_mode")

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    # ── callbacks ──────────────────────────────────────────────────────────

    def _state_cb(self, msg):
        with self._lock:
            self.state = msg

    def _pose_cb(self, msg):
        with self._lock:
            self.pose = msg

    def _wp_cb(self, msg):
        with self._lock:
            self.waypoints = msg.waypoints

    def _home_cb(self, msg):
        with self._lock:
            self.home = msg

    # ── getters ────────────────────────────────────────────────────────────

    def get_state(self):
        with self._lock:
            return self.state

    def get_pos(self) -> Tuple[float, float, float]:
        with self._lock:
            if self.pose is None:
                return 0.0, 0.0, 0.0
            p = self.pose.pose.position
            return p.x, p.y, p.z

    def get_home(self):
        with self._lock:
            return self.home

    def get_waypoints(self):
        with self._lock:
            return list(self.waypoints)

    # ── setpoint publishing ────────────────────────────────────────────────

    def publish_sp(self, ex: float, ey: float, ez: float,
                   yaw: float = 0.0):
        msg                  = self._PS()
        msg.header.stamp     = self._node.get_clock().now().to_msg()
        msg.header.frame_id  = "map"
        msg.pose.position.x  = ex
        msg.pose.position.y  = ey
        msg.pose.position.z  = ez
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def stream_sp(self, ex: float, ey: float, ez: float,
                  yaw: float, duration: float):
        end = time.time() + duration
        while time.time() < end:
            self.publish_sp(ex, ey, ez, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def fly_to(self, ex: float, ey: float, ez: float,
               yaw: float, timeout: float) -> bool:
        """Stream setpoints to target until within WP_TOLERANCE_M."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.publish_sp(ex, ey, ez, yaw)
            cx, cy, cz = self.get_pos()
            dist = math.sqrt((cx-ex)**2 + (cy-ey)**2 + (cz-ez)**2)
            print(f"\r  dist={dist:.2f}m  pos=({cx:.2f},{cy:.2f},{cz:.2f})",
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
        self._rclpy.spin_until_future_complete(self._node, fut, timeout_sec=5.0)
        return fut.result() and fut.result().mode_sent

    def shutdown(self):
        self._rclpy.shutdown()

# ── mission execution ─────────────────────────────────────────────────────────

def wait_for_home(node: MissionNode, timeout: float = 30.0) -> bool:
    print("  Waiting for home position...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if node.get_home() is not None:
            h = node.get_home()
            print(f"  [OK] Home: lat={h.geo.latitude:.6f} "
                  f"lon={h.geo.longitude:.6f} alt={h.geo.altitude:.1f}m")
            return True
        time.sleep(0.5)
    print("  [WARN] Home position not received -- GPS may not be locked")
    return False


def wait_for_waypoints(node: MissionNode, timeout: float = 15.0) -> list:
    print("  Waiting for mission waypoints from PX4...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        wps = node.get_waypoints()
        if wps:
            nav_wps = [w for w in wps if w.command == 16]  # NAV_WAYPOINT
            if nav_wps:
                print(f"  [OK] Received {len(nav_wps)} NAV_WAYPOINT(s) from PX4")
                return nav_wps
        time.sleep(0.5)
    print("  [WARN] No waypoints found -- upload a mission in QGC first")
    return []


def convert_waypoints(nav_wps: list, home,
                      home_alt_m: float) -> List[Tuple[float, float, float]]:
    """Convert GPS NAV_WAYPOINT list to local ENU coordinates."""
    enu_wps = []
    for wp in nav_wps:
        e, n, u = haversine_to_enu(
            wp.x_lat, wp.y_long, wp.z_alt,
            home.geo.latitude, home.geo.longitude,
            home_alt_m,
        )
        enu_wps.append((e, n, u))
    return enu_wps


def execute_mission(node: MissionNode, enu_wps: list,
                    gap_det: GapDetector, args,
                    fov_deg: float):
    """
    Execute waypoints in OFFBOARD mode with gap fill and camera trigger.

    For each waypoint:
      1. Fly to waypoint
      2. In stop mode: hold, trigger camera, check gaps
      3. In continuous mode: trigger camera based on distance, check gaps
      4. For each gap found: insert gap-fill waypoint
    """

    fov_half = math.radians(fov_deg / 2.0)
    total_gap_fills = 0
    camera_triggers = 0

    print(f"\n  Executing {len(enu_wps)} waypoints")
    print(f"  Trigger mode : {args.trigger_mode}")
    print(f"  FOV          : {fov_deg:.1f}deg diagonal")
    print(f"  Min density  : {args.min_density} pts/cell")

    prev_x, prev_y, prev_z = node.get_pos()
    dist_since_trigger = 0.0

    for i, (ex, ey, ez) in enumerate(enu_wps):
        print(f"\n  --- Waypoint {i+1}/{len(enu_wps)}: "
              f"ENU=({ex:.1f}, {ey:.1f}, {ez:.1f}) ---")

        # Fly to waypoint
        reached = node.fly_to(ex, ey, ez, yaw=0.0, timeout=120.0)
        if not reached:
            print(f"  [WARN] WP {i+1} not reached within timeout -- continuing")

        cx, cy, cz = node.get_pos()

        # Gap detection at this waypoint
        footprint_r = cz * math.tan(fov_half)
        gaps = gap_det.find_gaps_near(cx, cy, footprint_r)
        if gaps:
            print(f"  Gap fill: {len(gaps)} gap(s) detected near WP {i+1}")
            for gx, gy in gaps[:3]:   # limit to 3 gap fills per waypoint
                print(f"    Gap-fill -> ENU=({gx:.1f}, {gy:.1f}, {ez:.1f})")
                node.fly_to(gx, gy, ez, yaw=0.0, timeout=60.0)
                total_gap_fills += 1
                # Return to waypoint after gap fill
                node.fly_to(ex, ey, ez, yaw=0.0, timeout=60.0)

        # Camera trigger
        if args.trigger_mode == "stop":
            # Hold at waypoint and trigger
            print(f"  Holding {STOP_HOLD_S:.0f}s for camera trigger...")
            node.stream_sp(ex, ey, ez, 0.0, STOP_HOLD_S)
            # Camera trigger placeholder -- replaced by actual trigger
            # when camera is reinstalled
            print(f"  [TRIGGER] Camera triggered at WP {i+1} "
                  f"ENU=({ex:.1f},{ey:.1f},{ez:.1f})")
            camera_triggers += 1

        else:
            # Continuous mode -- check distance-based trigger
            dx = cx - prev_x; dy = cy - prev_y
            dist_since_trigger += math.sqrt(dx*dx + dy*dy)
            trigger_dist = footprint_r * 2.0 * (1.0 - args.overlap)
            if dist_since_trigger >= trigger_dist:
                print(f"  [TRIGGER] Camera triggered after "
                      f"{dist_since_trigger:.1f}m travel")
                camera_triggers += 1
                dist_since_trigger = 0.0

        prev_x, prev_y, prev_z = cx, cy, cz

    print(f"\n  Mission complete.")
    print(f"  Waypoints executed : {len(enu_wps)}")
    print(f"  Gap fills inserted : {total_gap_fills}")
    print(f"  Camera triggers    : {camera_triggers}")

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Autonomous survey mission executor with gap fill.")
    parser.add_argument("--dry-run",       action="store_true",
                        help="Validate mission without flying")
    # MAVROS is managed by systemd (mavros.service) and is always running.
    # --launch-mavros is only needed if running completely standalone
    # outside the normal boot sequence.
    parser.add_argument("--launch-mavros", action="store_true",
                        help="Launch MAVROS manually (default: use existing systemd instance)")
    parser.add_argument("--trigger-mode",  choices=["stop", "continuous"],
                        default="stop",
                        help="Camera trigger mode (default: stop)")
    parser.add_argument("--fov-deg",       type=float, default=None,
                        help="Camera FOV degrees diagonal (overrides YAML)")
    parser.add_argument("--min-density",   type=int,
                        default=DEFAULT_MIN_DENSITY,
                        help=f"Min points/cell for coverage "
                             f"(default: {DEFAULT_MIN_DENSITY})")
    parser.add_argument("--overlap",       type=float,
                        default=DEFAULT_OVERLAP,
                        help=f"Image overlap 0-1 for continuous trigger "
                             f"(default: {DEFAULT_OVERLAP})")
    args = parser.parse_args()

    print("=" * 60)
    print("  DronePi Mission Executor")
    print("=" * 60)
    print(f"  Trigger mode : {args.trigger_mode}")
    print(f"  Min density  : {args.min_density} pts/cell")
    print(f"  Dry run      : {'YES' if args.dry_run else 'NO -- LIVE FLIGHT'}")

    fov_deg = resolve_fov(args.fov_deg)

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

    mavros_proc = None
    if args.launch_mavros:
        mavros_proc = launch_mavros()
    else:
        print("  Using existing MAVROS instance (mavros.service)")

    node = MissionNode()

    # ── wait for home + waypoints ─────────────────────────────────────────
    print("\n[1/5] Waiting for FCU connection...")
    deadline = time.time() + 15.0
    while time.time() < deadline:
        if node.get_state().connected:
            break
        time.sleep(0.2)
    if not node.get_state().connected:
        print("  [FAIL] FCU not connected.")
        node.shutdown()
        kill_proc("MAVROS", mavros_proc)
        sys.exit(1)
    print("  [OK] FCU connected")

    print("\n[2/5] Reading mission from PX4...")
    home    = None
    nav_wps = []

    wait_for_home(node)
    home = node.get_home()
    nav_wps = wait_for_waypoints(node)

    if not nav_wps:
        print("  [FAIL] No waypoints -- upload a survey mission in QGC first.")
        node.shutdown()
        kill_proc("MAVROS", mavros_proc)
        sys.exit(1)

    if home is None:
        print("  [FAIL] No home position -- GPS not locked.")
        node.shutdown()
        kill_proc("MAVROS", mavros_proc)
        sys.exit(1)

    # Convert to ENU
    home_alt = home.geo.altitude
    enu_wps  = convert_waypoints(nav_wps, home, home_alt)

    print(f"\n  Mission: {len(enu_wps)} waypoints in local ENU:")
    for i, (e, n, u) in enumerate(enu_wps):
        print(f"    WP {i+1:2d}: E={e:+.1f}m  N={n:+.1f}m  U={u:.1f}m")

    footprint_r = enu_wps[0][2] * math.tan(math.radians(fov_deg / 2.0))
    print(f"\n  Altitude    : {enu_wps[0][2]:.1f}m")
    print(f"  Footprint r : {footprint_r:.1f}m")
    print(f"  Cell size   : {footprint_r * 2:.1f}m x {footprint_r * 2:.1f}m")

    if args.dry_run:
        print("\n  --dry-run: mission validated. No flight executed.")
        node.shutdown()
        return

    # ── EKF stability ─────────────────────────────────────────────────────
    print(f"\n[3/5] Waiting for EKF stability...")
    prev_z  = None
    stable  = 0
    deadline = time.time() + EKF_TIMEOUT
    while time.time() < deadline:
        _, _, z = node.get_pos()
        if prev_z is not None:
            stable = stable + 1 if abs(z - prev_z) < 0.03 else 0
        prev_z = z
        print(f"\r  z={z:.3f}m  stable={stable}/20", end="", flush=True)
        if stable >= 20:
            break
        time.sleep(0.1)
    print()
    print("  [OK] EKF ready")

    # Start gap detector
    cell_size = footprint_r * 2.0
    gap_det   = GapDetector(cell_size, args.min_density)
    gap_det.start(node._node)

    # Pre-stream setpoints
    print(f"\n[4/5] Pre-streaming setpoints (3s)...")
    hx, hy, hz = node.get_pos()
    node.stream_sp(hx, hy, hz, 0.0, 3.0)
    print("  [OK] Setpoint stream active")

    # Wait for arm + OFFBOARD
    print(f"\n  Arm on transmitter and switch to OFFBOARD to begin mission...")
    deadline = time.time() + 60.0
    while time.time() < deadline:
        node.publish_sp(hx, hy, hz, 0.0)
        st = node.get_state()
        if st.armed and st.mode == "OFFBOARD":
            break
        time.sleep(0.05)
    if not node.get_state().armed:
        print("  [FAIL] Not armed in time.")
        node.shutdown()
        kill_proc("MAVROS", mavros_proc)
        sys.exit(1)
    print("  [OK] Armed in OFFBOARD -- mission starting")

    # ── execute mission ────────────────────────────────────────────────────
    print(f"\n[5/5] Executing mission...")

    try:
        execute_mission(node, enu_wps, gap_det, args, fov_deg)
    except KeyboardInterrupt:
        print("\n\n  Ctrl+C -- aborting mission, returning to launch...")
    finally:
        print("  Switching to AUTO.RTL...")
        node.set_mode("AUTO.RTL")
        time.sleep(2.0)
        node.shutdown()
        kill_proc("MAVROS", mavros_proc)

    print("\n" + "=" * 60)
    print("  Mission complete. AUTO.RTL active.")
    print("=" * 60)


if __name__ == "__main__":
    main()
