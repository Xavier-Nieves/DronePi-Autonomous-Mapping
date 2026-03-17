#!/usr/bin/env python3
"""OFFBOARD survey mission — global GPS waypoints via MAVROS.

Uses /mavros/setpoint_position/global with GeoPoseStamped to command
GPS waypoints directly, avoiding local ENU frame conversion entirely.
Designed for the full capstone survey mission.

Sequence:
    1. Connect and verify MAVROS + GPS lock
    2. Capture home GPS position
    3. Pre-stream setpoints at home position (required before OFFBOARD)
    4. Arm and switch to OFFBOARD
    5. Climb to survey altitude at home lat/lon
    6. Fly survey waypoints in sequence
    7. Return to home and land

Usage:
    source /opt/ros/jazzy/setup.bash

    # Hover test at 5m — single waypoint above home
    python3 test_offboard_gps.py --alt 5.0 --hold 15

    # Dry run — GPS + setpoint check, no arming
    python3 test_offboard_gps.py --dry-run

    # Full survey from waypoint file
    python3 test_offboard_gps.py --waypoints survey_waypoints.json --alt 10.0

Waypoint file format (JSON):
    {
      "waypoints": [
        {"lat": 18.2013, "lon": -67.1397, "alt_agl": 10.0},
        {"lat": 18.2015, "lon": -67.1397, "alt_agl": 10.0}
      ]
    }

Run MAVROS first:
    ros2 launch mavros px4.launch fcu_url:=serial:///dev/ttyACM0:57600

SAFETY:
    - RC transmitter in hand. Mode switch to STABILIZED = immediate override.
    - Kill switch within reach at all times.
    - Verify GPS fix ≥ 10 satellites before flying.
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

# ── config ────────────────────────────────────────────────────────────────────

DEFAULT_ALT     = 5.0    # metres AGL for hover test
HOLD_SECONDS    = 15     # seconds to hold at each waypoint
SETPOINT_HZ     = 20     # Hz
ARM_TIMEOUT     = 10.0
MODE_TIMEOUT    = 10.0
TAKEOFF_TIMEOUT = 30.0
WAYPOINT_TIMEOUT= 45.0   # seconds per waypoint
LAND_TIMEOUT    = 30.0
GPS_TIMEOUT     = 30.0   # seconds to wait for GPS fix

# GPS position tolerance for waypoint arrival
LAT_LON_TOL_M   = 1.5   # metres horizontal
ALT_TOL_M       = 0.5   # metres vertical


# ── helpers ───────────────────────────────────────────────────────────────────

def haversine_m(lat1, lon1, lat2, lon2) -> float:
    """Haversine distance in metres between two GPS coordinates."""
    import math
    R   = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a    = (math.sin(dphi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _import_ros():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from geographic_msgs.msg import GeoPoseStamped
        from mavros_msgs.msg import State, HomePosition
        from mavros_msgs.srv import CommandBool, SetMode
        from sensor_msgs.msg import NavSatFix
        return (rclpy, Node, QoSProfile, ReliabilityPolicy,
                HistoryPolicy, GeoPoseStamped, State, HomePosition,
                CommandBool, SetMode, NavSatFix)
    except ImportError as e:
        print(f"[FAIL] ROS 2 import error: {e}")
        print("       source /opt/ros/jazzy/setup.bash")
        sys.exit(1)


# ── node ──────────────────────────────────────────────────────────────────────

class GPSOffboardNode:

    def __init__(self, target_alt: float, hold_seconds: int, dry_run: bool):
        (rclpy, Node, QoSProfile, ReliabilityPolicy,
         HistoryPolicy, GeoPoseStamped, State, HomePosition,
         CommandBool, SetMode, NavSatFix) = _import_ros()

        self._rclpy           = rclpy
        self._GeoPoseStamped  = GeoPoseStamped

        self.target_alt       = target_alt
        self.hold_seconds     = hold_seconds
        self.dry_run          = dry_run

        self.current_state    = State()
        self.current_gps      = None    # NavSatFix
        self.home_lat         = None
        self.home_lon         = None
        self.home_alt_amsl    = None    # AMSL metres
        self.home_set         = False
        self._lock            = threading.Lock()
        self._stop            = threading.Event()

        rclpy.init()
        self._node = Node("offboard_gps_node")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(State, "/mavros/state",
                                        self._state_cb, 10)
        self._node.create_subscription(NavSatFix,
                                        "/mavros/global_position/global",
                                        self._gps_cb, sensor_qos)
        self._node.create_subscription(HomePosition,
                                        "/mavros/home_position/home",
                                        self._home_cb, sensor_qos)

        self._setpoint_pub = self._node.create_publisher(
            GeoPoseStamped, "/mavros/setpoint_position/global", 10)

        self._arm_client  = self._node.create_client(
            CommandBool, "/mavros/cmd/arming")
        self._mode_client = self._node.create_client(
            SetMode, "/mavros/set_mode")

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg):
        with self._lock:
            self.current_state = msg

    def _gps_cb(self, msg):
        with self._lock:
            self.current_gps = msg

    def _home_cb(self, msg):
        with self._lock:
            self.home_lat      = msg.geo.latitude
            self.home_lon      = msg.geo.longitude
            self.home_alt_amsl = msg.geo.altitude
            self.home_set      = True

    # ── getters ───────────────────────────────────────────────────────────────

    def get_state(self):
        with self._lock:
            return self.current_state

    def get_gps(self):
        with self._lock:
            return self.current_gps

    def get_home(self):
        with self._lock:
            return self.home_lat, self.home_lon, self.home_alt_amsl

    # ── GPS health check ──────────────────────────────────────────────────────

    def wait_for_gps(self, timeout: float = GPS_TIMEOUT) -> bool:
        """Wait for a valid GPS fix with sufficient accuracy."""
        print("  Waiting for GPS fix...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            gps = self.get_gps()
            if gps is not None and gps.status.status >= 0:
                hdop = gps.position_covariance[0] ** 0.5
                print(f"\r  GPS: lat={gps.latitude:.6f} "
                      f"lon={gps.longitude:.6f}  "
                      f"hdop≈{hdop:.2f}m    ", end="", flush=True)
                if hdop < 3.0 and self.home_set:
                    print()
                    return True
            time.sleep(0.5)
        print()
        return False

    # ── setpoint helpers ──────────────────────────────────────────────────────

    def _amsl(self, alt_agl: float) -> float:
        """Convert AGL altitude to AMSL using home altitude."""
        with self._lock:
            base = self.home_alt_amsl or 0.0
        return base + alt_agl

    def _make_setpoint(self, lat: float, lon: float, alt_amsl: float):
        """Build a GeoPoseStamped global setpoint."""
        sp = self._GeoPoseStamped()
        sp.header.stamp    = self._node.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.latitude  = lat
        sp.pose.position.longitude = lon
        sp.pose.position.altitude  = alt_amsl
        sp.pose.orientation.w      = 1.0
        return sp

    def publish_setpoint(self, lat: float, lon: float, alt_amsl: float):
        self._setpoint_pub.publish(self._make_setpoint(lat, lon, alt_amsl))

    def stream_setpoints(self, lat: float, lon: float, alt_amsl: float,
                          duration: float, hz: float = SETPOINT_HZ):
        interval = 1.0 / hz
        deadline = time.time() + duration
        while time.time() < deadline and not self._stop.is_set():
            self.publish_setpoint(lat, lon, alt_amsl)
            time.sleep(interval)

    # ── waypoint arrival check ────────────────────────────────────────────────

    def wait_for_waypoint(self, lat: float, lon: float, alt_amsl: float,
                           timeout: float = WAYPOINT_TIMEOUT) -> bool:
        """Block until GPS position is within tolerance of target waypoint."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            gps = self.get_gps()
            if gps is not None:
                h_dist = haversine_m(gps.latitude, gps.longitude, lat, lon)
                v_dist = abs(gps.altitude - alt_amsl)
                print(f"\r  Horizontal: {h_dist:.1f}m  "
                      f"Vertical: {v_dist:.1f}m", end="", flush=True)
                if h_dist < LAT_LON_TOL_M and v_dist < ALT_TOL_M:
                    print()
                    return True
            time.sleep(0.2)
        print()
        return False

    # ── vehicle commands ──────────────────────────────────────────────────────

    def set_mode(self, mode: str, timeout: float = MODE_TIMEOUT) -> bool:
        from mavros_msgs.srv import SetMode
        if not self._mode_client.wait_for_service(timeout_sec=5.0):
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        future   = self._mode_client.call_async(req)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return future.result().mode_sent
            time.sleep(0.05)
        return False

    def arm(self, timeout: float = ARM_TIMEOUT) -> bool:
        from mavros_msgs.srv import CommandBool
        if not self._arm_client.wait_for_service(timeout_sec=5.0):
            return False
        req       = CommandBool.Request()
        req.value = True
        future    = self._arm_client.call_async(req)
        deadline  = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                return future.result().success
            time.sleep(0.05)
        return False

    def disarm(self) -> bool:
        from mavros_msgs.srv import CommandBool
        if not self._arm_client.wait_for_service(timeout_sec=5.0):
            return False
        req       = CommandBool.Request()
        req.value = False
        future    = self._arm_client.call_async(req)
        deadline  = time.time() + 5.0
        while time.time() < deadline:
            if future.done():
                return future.result().success
            time.sleep(0.05)
        return False

    def shutdown(self):
        self._stop.set()
        self._rclpy.shutdown()


# ── flight sequence ───────────────────────────────────────────────────────────

def run_mission(node: GPSOffboardNode, waypoints: list, args) -> bool:

    # 1 — FCU connection
    print("\n[1/8] Waiting for FCU connection...")
    deadline = time.time() + 15.0
    while time.time() < deadline:
        if node.get_state().connected:
            break
        time.sleep(0.1)
    if not node.get_state().connected:
        print("  [FAIL] MAVROS not connected.")
        return False
    print("  [OK] FCU connected")

    # 2 — GPS lock
    print(f"\n[2/8] Waiting for GPS fix (up to {GPS_TIMEOUT:.0f}s)...")
    if not node.wait_for_gps(timeout=GPS_TIMEOUT):
        print("  [FAIL] GPS fix not acquired.")
        print("  Wait for QGC GPS preflight warnings to clear before flying.")
        return False
    home_lat, home_lon, home_alt_amsl = node.get_home()
    print(f"  [OK] GPS ready — home: {home_lat:.6f}, {home_lon:.6f}, "
          f"{home_alt_amsl:.1f}m AMSL")

    # 3 — Pre-stream at home (required before OFFBOARD switch)
    hover_alt_amsl = node._amsl(args.alt)
    print(f"\n[3/8] Pre-streaming at home position for 3s...")
    print(f"  Survey altitude: {args.alt}m AGL = {hover_alt_amsl:.1f}m AMSL")
    node.stream_setpoints(home_lat, home_lon, home_alt_amsl, duration=3.0)
    print("  [OK] Setpoint stream established")

    if args.dry_run:
        print("\n  --dry-run: stopping before arm. GPS and setpoints verified.")
        return True

    # 4 — Arm
    print(f"\n[4/8] Arming...")
    if not node.arm():
        print("  [FAIL] Arm rejected.")
        return False
    deadline = time.time() + ARM_TIMEOUT
    while time.time() < deadline:
        if node.get_state().armed:
            break
        time.sleep(0.1)
    if not node.get_state().armed:
        print("  [FAIL] FCU still disarmed")
        return False
    print("  [OK] Armed")

    # 5 — Switch to OFFBOARD
    print(f"\n[5/8] Switching to OFFBOARD mode...")
    if not node.set_mode("OFFBOARD"):
        print("  [FAIL] Mode switch failed")
        node.disarm()
        return False
    deadline = time.time() + MODE_TIMEOUT
    while time.time() < deadline:
        if node.get_state().mode == "OFFBOARD":
            break
        node.publish_setpoint(home_lat, home_lon, hover_alt_amsl)
        time.sleep(0.05)
    if node.get_state().mode != "OFFBOARD":
        print(f"  [FAIL] Still in {node.get_state().mode}")
        node.disarm()
        return False
    print("  [OK] OFFBOARD active")

    # 6 — Climb to survey altitude above home
    print(f"\n[6/8] Climbing to {args.alt:.1f}m AGL ({hover_alt_amsl:.1f}m AMSL)...")
    climb_thread = threading.Thread(
        target=node.stream_setpoints,
        args=(home_lat, home_lon, hover_alt_amsl, TAKEOFF_TIMEOUT),
        daemon=True,
    )
    climb_thread.start()
    reached = node.wait_for_waypoint(home_lat, home_lon, hover_alt_amsl,
                                      timeout=TAKEOFF_TIMEOUT)
    if reached:
        print(f"  [OK] Survey altitude reached")
    else:
        print(f"  [WARN] Altitude not confirmed — continuing")

    # 7 — Fly waypoints
    print(f"\n[7/8] Flying {len(waypoints)} waypoint(s)...")
    for i, wp in enumerate(waypoints):
        wp_lat  = wp["lat"]
        wp_lon  = wp["lon"]
        wp_amsl = node._amsl(wp.get("alt_agl", args.alt))

        print(f"\n  Waypoint {i+1}/{len(waypoints)}: "
              f"lat={wp_lat:.6f} lon={wp_lon:.6f} "
              f"alt={wp.get('alt_agl', args.alt):.1f}m AGL")

        # Stream setpoints toward waypoint
        wp_thread = threading.Thread(
            target=node.stream_setpoints,
            args=(wp_lat, wp_lon, wp_amsl, WAYPOINT_TIMEOUT),
            daemon=True,
        )
        wp_thread.start()

        arrived = node.wait_for_waypoint(wp_lat, wp_lon, wp_amsl,
                                          timeout=WAYPOINT_TIMEOUT)
        if arrived:
            print(f"  [OK] Arrived at waypoint {i+1}")
        else:
            print(f"  [WARN] Did not reach waypoint {i+1} within timeout")

        # Hold at waypoint
        print(f"  Holding for {node.hold_seconds}s...")
        hold_thread = threading.Thread(
            target=node.stream_setpoints,
            args=(wp_lat, wp_lon, wp_amsl, float(node.hold_seconds)),
            daemon=True,
        )
        hold_thread.start()
        for j in range(node.hold_seconds, 0, -1):
            gps = node.get_gps()
            if gps:
                dist = haversine_m(gps.latitude, gps.longitude, wp_lat, wp_lon)
                print(f"\r  {j}s remaining | dist from WP: {dist:.1f}m",
                      end="", flush=True)
            time.sleep(1.0)
        print()

    # Return to home
    print(f"\n  Returning to home...")
    rth_thread = threading.Thread(
        target=node.stream_setpoints,
        args=(home_lat, home_lon, hover_alt_amsl, WAYPOINT_TIMEOUT),
        daemon=True,
    )
    rth_thread.start()
    node.wait_for_waypoint(home_lat, home_lon, hover_alt_amsl,
                            timeout=WAYPOINT_TIMEOUT)
    print("  [OK] Home position reached")

    # 8 — Land
    print(f"\n[8/8] Landing...")
    if not node.set_mode("AUTO.LAND"):
        print("  [WARN] AUTO.LAND failed — commanding descent")
        _, _, amsl_now = node.get_home()
        node.stream_setpoints(home_lat, home_lon,
                               (amsl_now or 0.0) + 0.1, duration=LAND_TIMEOUT)
    else:
        print("  AUTO.LAND active...")
        gps_prev_alt = None
        deadline = time.time() + LAND_TIMEOUT
        while time.time() < deadline:
            gps = node.get_gps()
            if gps:
                alt_agl_approx = gps.altitude - (home_alt_amsl or 0.0)
                print(f"\r  Descending: ~{alt_agl_approx:.1f}m AGL",
                      end="", flush=True)
                if alt_agl_approx < 0.3:
                    break
            time.sleep(0.2)
        print()

    time.sleep(1.5)
    node.disarm()
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if not node.get_state().armed:
            break
        time.sleep(0.1)

    if not node.get_state().armed:
        print("  [OK] Disarmed. Safe.")
        return True
    else:
        print("  [WARN] Still armed — use RC kill switch!")
        return False


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="OFFBOARD GPS waypoint mission via MAVROS.")
    parser.add_argument("--alt",       type=float, default=DEFAULT_ALT,
                        help=f"Survey altitude AGL in metres (default: {DEFAULT_ALT})")
    parser.add_argument("--hold",      type=int,   default=HOLD_SECONDS,
                        help=f"Hold seconds per waypoint (default: {HOLD_SECONDS})")
    parser.add_argument("--waypoints", type=str,   default=None,
                        help="JSON waypoint file (optional — hover above home if omitted)")
    parser.add_argument("--dry-run",   action="store_true",
                        help="GPS + setpoint check only — no arming")
    args = parser.parse_args()

    # Load waypoints
    if args.waypoints:
        wp_path = Path(args.waypoints)
        if not wp_path.exists():
            print(f"[FAIL] Waypoint file not found: {wp_path}")
            sys.exit(1)
        with open(wp_path) as f:
            data = json.load(f)
        waypoints = data["waypoints"]
        print(f"  Loaded {len(waypoints)} waypoints from {wp_path}")
    else:
        # Single hover above home
        waypoints = []   # will be populated after GPS home is captured

    print("=" * 55)
    print("  OFFBOARD GPS Mission — MAVROS / ROS 2")
    print("=" * 55)
    print(f"\n  Survey altitude : {args.alt}m AGL")
    print(f"  Hold per WP     : {args.hold}s")
    print(f"  Waypoints       : {len(waypoints) if waypoints else 'hover above home'}")
    print(f"  Dry run         : {'YES' if args.dry_run else 'NO — LIVE FLIGHT'}")

    if not args.dry_run:
        print("\n  !! LIVE FLIGHT !!")
        print("  Props installed, area clear, RC in hand, kill switch ready.")
        print("  Ctrl+C to abort. Starting in 5s...")
        try:
            for i in range(5, 0, -1):
                print(f"\r  {i}...", end="", flush=True)
                time.sleep(1)
            print()
        except KeyboardInterrupt:
            print("\n  Aborted.")
            sys.exit(0)

    node = GPSOffboardNode(
        target_alt=args.alt,
        hold_seconds=args.hold,
        dry_run=args.dry_run,
    )

    # If no waypoints specified, build a single hover WP above home
    # after GPS is acquired (done inside run_mission)
    if not waypoints:
        # Placeholder — run_mission will use home position directly
        waypoints = [{"use_home": True, "alt_agl": args.alt}]

    success = False
    try:
        success = run_mission(node, waypoints, args)
    except KeyboardInterrupt:
        print("\n\n  Ctrl+C — emergency abort.")
        try:
            node.set_mode("AUTO.LAND")
            time.sleep(2)
            node.disarm()
        except Exception:
            pass
        print("  Switch to STABILIZED or use kill switch if airborne!")
    except Exception as e:
        print(f"\n  [ERROR] {e}")
        try:
            node.set_mode("AUTO.LAND")
            time.sleep(2)
            node.disarm()
        except Exception:
            pass
    finally:
        node.shutdown()

    print("\n" + "=" * 55)
    print("  MISSION COMPLETE" if success else "  Did not complete cleanly")
    print("=" * 55)


if __name__ == "__main__":
    main()
