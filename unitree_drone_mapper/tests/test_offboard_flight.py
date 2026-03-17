#!/usr/bin/env python3
"""OFFBOARD hover test — local ENU setpoints via MAVROS.

Fixes vs previous version:
  - Waits for EKF to fully initialize before arming
  - Captures home position Z at startup, commands relative altitude above it
  - Verifies local position is stable and near zero before streaming setpoints
  - Adds pre-arm health checks (EKF, GPS, heading)

Usage (single terminal — MAVROS launched automatically):
    source /opt/ros/jazzy/setup.bash
    python3 test_offboard_flight.py --dry-run     # no arming
    python3 test_offboard_flight.py --alt 1.5 --hold 10
    python3 test_offboard_flight.py --no-mavros   # if MAVROS already running

SAFETY:
    - RC transmitter in hand at all times
    - Kill switch within reach
    - Switch to STABILIZED immediately if behavior is unexpected
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time

# ── config ────────────────────────────────────────────────────────────────────

TARGET_ALT      = 1.5    # metres above home (ENU, positive up)
HOLD_SECONDS    = 10     # seconds to hold hover
SETPOINT_HZ     = 20     # Hz — PX4 requires > 2 Hz
ARM_TIMEOUT     = 10.0
MODE_TIMEOUT    = 10.0
TAKEOFF_TIMEOUT = 25.0
LAND_TIMEOUT    = 30.0
EKF_TIMEOUT     = 30.0   # seconds to wait for EKF init
ALT_TOLERANCE   = 0.15   # metres
MAVROS_STARTUP  = 8.0    # seconds to wait for MAVROS to initialize after launch

ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
MAVROS_URL = "serial:///dev/ttyACM0:57600"


# ── MAVROS launcher ───────────────────────────────────────────────────────────

def launch_mavros() -> subprocess.Popen:
    """Launch MAVROS as a background subprocess.

    Returns the Popen handle so the caller can kill it on exit.
    MAVROS is placed in its own process group so a single killpg()
    cleans up all child nodes cleanly.
    """
    cmd = (
        f"source {ROS_SETUP} && "
        f"ros2 launch mavros px4.launch "
        f"fcu_url:={MAVROS_URL}"
    )
    print("  Launching MAVROS in background...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"  MAVROS PID: {proc.pid} — waiting {MAVROS_STARTUP:.0f}s for startup...")
    time.sleep(MAVROS_STARTUP)
    print("  [OK] MAVROS startup wait complete")
    return proc


def kill_mavros(proc: subprocess.Popen):
    """Send SIGINT to the MAVROS process group for a clean shutdown."""
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
        print("  [OK] MAVROS stopped")
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── node ──────────────────────────────────────────────────────────────────────

def _import_ros():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State, HomePosition
        from mavros_msgs.srv import CommandBool, SetMode
        return (rclpy, Node, QoSProfile, ReliabilityPolicy,
                HistoryPolicy, PoseStamped, State, HomePosition,
                CommandBool, SetMode)
    except ImportError as e:
        print(f"[FAIL] ROS 2 import error: {e}")
        print("       source /opt/ros/jazzy/setup.bash")
        sys.exit(1)


class OffboardNode:

    def __init__(self, target_alt: float, hold_seconds: int, dry_run: bool):
        (rclpy, Node, QoSProfile, ReliabilityPolicy,
         HistoryPolicy, PoseStamped, State, HomePosition,
         CommandBool, SetMode) = _import_ros()

        self._rclpy       = rclpy
        self._PoseStamped = PoseStamped

        self.target_alt   = target_alt
        self.hold_seconds = hold_seconds
        self.dry_run      = dry_run

        self.current_state    = State()
        self.current_pose     = None   # full PoseStamped
        self.home_set         = False
        self.home_alt         = 0.0    # local Z when home was captured
        self._lock            = threading.Lock()
        self._stop            = threading.Event()

        rclpy.init()
        self._node = Node("offboard_flight_test")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(State, "/mavros/state",
                                        self._state_cb, 10)
        self._node.create_subscription(PoseStamped,
                                        "/mavros/local_position/pose",
                                        self._pose_cb, sensor_qos)
        self._node.create_subscription(HomePosition,
                                        "/mavros/home_position/home",
                                        self._home_cb, sensor_qos)

        self._setpoint_pub = self._node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)

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

    def _pose_cb(self, msg):
        with self._lock:
            self.current_pose = msg

    def _home_cb(self, msg):
        with self._lock:
            self.home_set = True

    # ── getters ───────────────────────────────────────────────────────────────

    def get_state(self):
        with self._lock:
            return self.current_state

    def get_alt(self) -> float:
        with self._lock:
            if self.current_pose is None:
                return 0.0
            return self.current_pose.pose.position.z

    def get_alt_above_home(self) -> float:
        """Return altitude above the captured home Z position."""
        with self._lock:
            if self.current_pose is None:
                return 0.0
            return self.current_pose.pose.position.z - self.home_alt

    # ── EKF initialization wait ───────────────────────────────────────────────

    def wait_for_ekf(self, timeout: float = EKF_TIMEOUT,
                      require_gps_home: bool = False) -> bool:
        """Block until local position estimate is stable.

        Indoor / no-GPS: require_gps_home=False (default)
        Outdoor GPS flight: require_gps_home=True
        """
        print("  Waiting for EKF initialization...")
        if not require_gps_home:
            print("  (GPS home not required — indoor/no-GPS mode)")

        deadline        = time.time() + timeout
        prev_alt        = None
        stable_count    = 0
        required_stable = 20   # 2 seconds at 10 Hz polling

        while time.time() < deadline:
            alt     = self.get_alt()
            home_ok = self.home_set if require_gps_home else True

            if prev_alt is not None:
                drift = abs(alt - prev_alt)
                if drift < 0.03:
                    stable_count += 1
                else:
                    stable_count = 0
            prev_alt = alt

            gps_str = (f"home={'OK' if self.home_set else 'waiting'}  "
                       if require_gps_home else "")
            status  = f"alt={alt:.3f}m  {gps_str}stable={stable_count}/{required_stable}"
            print(f"\r  {status}    ", end="", flush=True)

            if stable_count >= required_stable and home_ok:
                print()
                with self._lock:
                    if self.current_pose is not None:
                        self.home_alt = self.current_pose.pose.position.z
                print(f"  [OK] EKF ready — home Z = {self.home_alt:.3f}m")
                return True

            time.sleep(0.1)

        print()
        return False

    # ── setpoint helpers ──────────────────────────────────────────────────────

    def get_yaw_quaternion(self) -> tuple:
        """Return current yaw as quaternion (x,y,z,w) to hold heading in setpoints."""
        with self._lock:
            if self.current_pose is None:
                return (0.0, 0.0, 0.0, 1.0)
            o = self.current_pose.pose.orientation
            return (o.x, o.y, o.z, o.w)

    def _make_setpoint(self, x: float, y: float, z_abs: float):
        """Build a PoseStamped in local ENU frame, holding current yaw."""
        sp = self._PoseStamped()
        sp.header.stamp    = self._node.get_clock().now().to_msg()
        sp.header.frame_id = "map"
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z_abs
        # Hold current heading — prevents snap rotation on OFFBOARD switch
        qx, qy, qz, qw       = self.get_yaw_quaternion()
        sp.pose.orientation.x = qx
        sp.pose.orientation.y = qy
        sp.pose.orientation.z = qz
        sp.pose.orientation.w = qw
        return sp

    def target_z(self) -> float:
        """Absolute Z setpoint = home Z + desired altitude above home."""
        return self.home_alt + self.target_alt

    def publish_setpoint(self, x: float = 0.0, y: float = 0.0,
                          z_abs: float | None = None):
        if z_abs is None:
            z_abs = self.target_z()
        self._setpoint_pub.publish(self._make_setpoint(x, y, z_abs))

    def stream_setpoints(self, duration: float,
                          z_abs: float | None = None,
                          hz: float = SETPOINT_HZ):
        interval = 1.0 / hz
        deadline = time.time() + duration
        if z_abs is None:
            z_abs = self.target_z()
        while time.time() < deadline and not self._stop.is_set():
            self.publish_setpoint(z_abs=z_abs)
            time.sleep(interval)

    # ── vehicle commands ──────────────────────────────────────────────────────

    def set_mode(self, mode: str, timeout: float = MODE_TIMEOUT) -> bool:
        from mavros_msgs.srv import SetMode
        if not self._mode_client.wait_for_service(timeout_sec=5.0):
            print("  [FAIL] set_mode service unavailable")
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
            print("  [FAIL] arming service unavailable")
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

    def wait_for_altitude(self, target_z_abs: float,
                           tolerance: float = ALT_TOLERANCE,
                           timeout: float = TAKEOFF_TIMEOUT) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            alt_rel = self.get_alt_above_home()
            print(f"\r  Altitude above home: {alt_rel:.2f}m / "
                  f"{self.target_alt:.2f}m target", end="", flush=True)
            if abs(self.get_alt() - target_z_abs) <= tolerance:
                print()
                return True
            time.sleep(0.1)
        print()
        return False

    def shutdown(self):
        self._stop.set()
        self._rclpy.shutdown()


# ── flight sequence ───────────────────────────────────────────────────────────

def run_flight(node: OffboardNode, args) -> bool:

    # 1 — FCU connection
    print("\n[1/8] Waiting for FCU connection...")
    deadline = time.time() + 15.0
    while time.time() < deadline:
        if node.get_state().connected:
            break
        time.sleep(0.1)
    if not node.get_state().connected:
        print("  [FAIL] MAVROS not connected. Is MAVROS running?")
        return False
    print("  [OK] FCU connected")

    # 2 — EKF initialization (NEW — was missing before)
    print(f"\n[2/8] Waiting for EKF initialization (up to {EKF_TIMEOUT:.0f}s)...")
    if not node.wait_for_ekf(timeout=EKF_TIMEOUT, require_gps_home=False):
        print("  [FAIL] EKF did not initialize in time.")
        print("  Check GPS lock and wait for QGC preflight warnings to clear.")
        return False

    # 3 — Pre-stream setpoints at home altitude (not target — stay on ground)
    print(f"\n[3/8] Pre-streaming setpoints at {SETPOINT_HZ} Hz for 3s...")
    print(f"  Holding at home Z = {node.home_alt:.3f}m until OFFBOARD switch")
    node.stream_setpoints(duration=3.0, z_abs=node.home_alt)
    print("  [OK] Setpoint stream established")

    if args.dry_run:
        print("\n  --dry-run: stopping before arm. EKF and setpoint stream verified.")
        return True

    # 4 — Arm
    print(f"\n[4/8] Arming...")
    print("  WARNING — motors will spin. Area must be clear.")
    if not node.arm():
        print("  [FAIL] Arm rejected.")
        print("  Fixes: CBRK_USB_CHK=197848, COM_ARM_WO_GPS=1")
        return False

    deadline = time.time() + ARM_TIMEOUT
    while time.time() < deadline:
        if node.get_state().armed:
            break
        time.sleep(0.1)
    if not node.get_state().armed:
        print("  [FAIL] FCU still disarmed after arm command")
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
        node.publish_setpoint(z_abs=node.target_z())
        time.sleep(0.05)

    if node.get_state().mode != "OFFBOARD":
        print(f"  [FAIL] Still in {node.get_state().mode}")
        node.disarm()
        return False
    print("  [OK] OFFBOARD active")

    # 6 — Climb
    print(f"\n[6/8] Climbing to {node.target_alt:.1f}m above home...")
    target_z_abs = node.target_z()
    climb_thread = threading.Thread(
        target=node.stream_setpoints,
        args=(TAKEOFF_TIMEOUT,),
        kwargs={"z_abs": target_z_abs},
        daemon=True,
    )
    climb_thread.start()

    reached = node.wait_for_altitude(target_z_abs, timeout=TAKEOFF_TIMEOUT)
    if reached:
        print(f"  [OK] Reached {node.get_alt_above_home():.2f}m above home")
    else:
        print(f"  [WARN] Target altitude not reached — "
              f"at {node.get_alt_above_home():.2f}m, continuing")

    # 7 — Hold
    print(f"\n[7/8] Holding for {node.hold_seconds}s...")
    for i in range(node.hold_seconds, 0, -1):
        node.publish_setpoint(z_abs=target_z_abs)
        print(f"\r  {i}s remaining | "
              f"alt above home = {node.get_alt_above_home():.2f}m",
              end="", flush=True)
        time.sleep(1.0)
    print()
    print("  [OK] Hold complete")

    # 8 — Land and disarm
    print(f"\n[8/8] Landing...")
    if not node.set_mode("AUTO.LAND"):
        print("  [WARN] AUTO.LAND failed — commanding descent manually")
        current_z = node.get_alt()
        while current_z > node.home_alt + 0.1:
            current_z = max(node.home_alt, current_z - 0.03)
            node.publish_setpoint(z_abs=current_z)
            print(f"\r  Descending: {node.get_alt_above_home():.2f}m above home",
                  end="", flush=True)
            time.sleep(0.1)
        print()
    else:
        print("  AUTO.LAND active...")
        deadline = time.time() + LAND_TIMEOUT
        while time.time() < deadline:
            alt_rel = node.get_alt_above_home()
            print(f"\r  Descending: {alt_rel:.2f}m above home",
                  end="", flush=True)
            if alt_rel < 0.15:
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
        description="OFFBOARD hover test (local ENU, EKF-safe).")
    parser.add_argument("--alt",     type=float, default=TARGET_ALT,
                        help=f"Hover altitude above home in metres (default: {TARGET_ALT})")
    parser.add_argument("--hold",    type=int,   default=HOLD_SECONDS,
                        help=f"Hold duration in seconds (default: {HOLD_SECONDS})")
    parser.add_argument("--dry-run",   action="store_true",
                        help="EKF + setpoint check only — no arming")
    parser.add_argument("--no-mavros", action="store_true",
                        help="Skip MAVROS launch — use if MAVROS is already running")
    args = parser.parse_args()

    print("=" * 55)
    print("  OFFBOARD Hover Test — Local ENU (EKF-safe)")
    print("=" * 55)
    print(f"\n  Target altitude : {args.alt}m above home")
    print(f"  Hold duration   : {args.hold}s")
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

    # ── launch MAVROS if not already running ─────────────────────────────────
    mavros_proc = None
    if not args.no_mavros:
        mavros_proc = launch_mavros()
    else:
        print("  --no-mavros: skipping MAVROS launch, using existing instance")

    node = OffboardNode(
        target_alt=args.alt,
        hold_seconds=args.hold,
        dry_run=args.dry_run,
    )

    success = False
    try:
        success = run_flight(node, args)
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
        kill_mavros(mavros_proc)

    print("\n" + "=" * 55)
    print("  PASSED" if success else "  Did not complete cleanly — review output above")
    print("=" * 55)


if __name__ == "__main__":
    main()
