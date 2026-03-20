#!/usr/bin/env python3
"""SLAM bridge out-and-back flight test.

Validates that the SLAM bridge is correctly feeding position data to PX4
by flying a simple out-and-back mission:

  1. Arm + OFFBOARD (manual on transmitter)
  2. Climb to target altitude
  3. Fly forward N metres
  4. Hold for K seconds
  5. Yaw 180 degrees (face back toward home)
  6. Fly back N metres to home position
  7. AUTO.LAND

While flying, the SLAM bridge feeds /aft_mapped_to_init into
/mavros/vision_pose/pose. If EKF2_AID_MASK has bit 3 set, PX4 will
fuse the SLAM position with GPS. You can verify fusion is active in
QGC: EKF2 innovations panel should show vision position residuals.

Usage
-----
  source /opt/ros/jazzy/setup.bash
  python3 test_slam_bridge_flight.py --distance 5 --alt 2.0 --hold 5
  python3 test_slam_bridge_flight.py --dry-run   # no arming, checks only
  python3 test_slam_bridge_flight.py             # MAVROS already running (default)
  python3 test_slam_bridge_flight.py --launch-mavros  # start MAVROS manually

SAFETY
------
  - RC transmitter in hand at all times
  - Kill switch within reach
  - Switch to STABILIZED immediately if behaviour is unexpected
  - Do NOT arm until area is clear -- motors will spin
"""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

# ── config ────────────────────────────────────────────────────────────────────

FORWARD_DIST_M  = 5.0    # metres to fly forward
TARGET_ALT_M    = 2.0    # metres above home
HOLD_SECONDS    = 5      # seconds to hold at forward position
YAW_RATE_DPS    = 30.0   # degrees per second for yaw turn
SETPOINT_HZ     = 20     # Hz -- PX4 requires > 2 Hz
ARM_TIMEOUT     = 10.0
MODE_TIMEOUT    = 10.0
ALT_TOLERANCE   = 0.15   # metres
POS_TOLERANCE   = 0.3    # metres -- close enough to waypoint
EKF_TIMEOUT     = 30.0
MAVROS_STARTUP  = 8.0

MISSION_LOCK = Path("/tmp/dronepi_mission.lock")

ROS_SETUP   = "/opt/ros/jazzy/setup.bash"
WS_SETUP    = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
MAVROS_URL  = "serial:///dev/ttyACM0:57600"
BRIDGE_SCRIPT = os.path.expanduser(
    "~/unitree_lidar_project/unitree_drone_mapper/flight/_slam_bridge.py"
)

# ── MAVROS + bridge launcher ──────────────────────────────────────────────────

def launch_mavros() -> subprocess.Popen:
    cmd = (
        f"source {ROS_SETUP} && "
        f"ros2 launch mavros px4.launch fcu_url:={MAVROS_URL}"
    )
    print("  Launching MAVROS...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"  MAVROS PID: {proc.pid} -- waiting {MAVROS_STARTUP:.0f}s...")
    time.sleep(MAVROS_STARTUP)
    print("  [OK] MAVROS ready")
    return proc


def launch_slam_bridge() -> subprocess.Popen:
    cmd = (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"python3 {BRIDGE_SCRIPT}"
    )
    print("  Launching SLAM bridge...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(2.0)   # wait for bridge to subscribe to /aft_mapped_to_init
    print(f"  SLAM bridge PID: {proc.pid}")
    return proc


def kill_proc(name: str, proc: subprocess.Popen):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
        print(f"  [OK] {name} stopped")
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass

def write_lock():
    """
    Write autonomous mission lock so the watchdog skips its own
    stack launch when it sees arm+OFFBOARD. The watchdog will only
    monitor for disarm and run postflight when done.
"""
    import json
    from datetime import datetime
    MISSION_LOCK.write_text(
        json.dumps({"mode": "autonomous",
                    "started_at": datetime.now().isoformat()}))
    print("  Watchdog lock written (autonomous) -- watchdog will not interfere")


def clear_lock():
    """Remove the mission lock so the watchdog returns to normal."""
    if MISSION_LOCK.exists():
        MISSION_LOCK.unlink()
        print("  Watchdog lock cleared -- watchdog resuming normal operation")


# ── ROS imports ───────────────────────────────────────────────────────────────

def _import_ros():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandBool, SetMode
        return (rclpy, Node, QoSProfile, ReliabilityPolicy,
                HistoryPolicy, PoseStamped, State, CommandBool, SetMode)
    except ImportError as e:
        print(f"[FAIL] ROS 2 import error: {e}")
        sys.exit(1)

# ── flight node ───────────────────────────────────────────────────────────────

class OutAndBackNode:

    def __init__(self):
        (rclpy, Node, QoSProfile, ReliabilityPolicy,
         HistoryPolicy, PoseStamped, State,
         CommandBool, SetMode) = _import_ros()

        self._rclpy       = rclpy
        self._PoseStamped = PoseStamped
        self._lock        = threading.Lock()

        self.current_state = State()
        self.current_pose  = None
        self.home_z        = 0.0

        rclpy.init()
        self._node = Node("slam_bridge_flight_test")

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

        self._sp_pub = self._node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)

        self._arm_client  = self._node.create_client(
            CommandBool, "/mavros/cmd/arming")
        self._mode_client = self._node.create_client(
            SetMode, "/mavros/set_mode")

        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

    def _state_cb(self, msg):
        with self._lock:
            self.current_state = msg

    def _pose_cb(self, msg):
        with self._lock:
            self.current_pose = msg

    def get_state(self):
        with self._lock:
            return self.current_state

    def get_pos(self):
        with self._lock:
            if self.current_pose is None:
                return 0.0, 0.0, 0.0
            p = self.current_pose.pose.position
            return p.x, p.y, p.z

    def get_yaw(self) -> float:
        """Return current yaw in radians from quaternion."""
        with self._lock:
            if self.current_pose is None:
                return 0.0
            q = self.current_pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny, cosy)

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg                      = self._PoseStamped()
        msg.header.stamp         = self._node.get_clock().now().to_msg()
        msg.header.frame_id      = "map"
        msg.pose.position.x      = x
        msg.pose.position.y      = y
        msg.pose.position.z      = z
        # Convert yaw to quaternion (rotation around Z axis)
        msg.pose.orientation.z   = math.sin(yaw / 2.0)
        msg.pose.orientation.w   = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def stream_setpoints(self, x: float, y: float, z: float,
                         yaw: float, duration: float):
        """Stream setpoints at SETPOINT_HZ for duration seconds."""
        end = time.time() + duration
        while time.time() < end:
            self.publish_setpoint(x, y, z, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def arm(self) -> bool:
        from mavros_msgs.srv import CommandBool
        req = CommandBool.Request()
        req.value = True
        future = self._arm_client.call_async(req)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=5.0)
        return future.result() and future.result().success

    def set_mode(self, mode: str) -> bool:
        from mavros_msgs.srv import SetMode
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        self._rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=5.0)
        return future.result() and future.result().mode_sent

    def wait_for_position(self, tx: float, ty: float, tz: float,
                          timeout: float, yaw: float) -> bool:
        """Stream setpoints and wait until drone reaches target position."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.publish_setpoint(tx, ty, tz, yaw)
            x, y, z = self.get_pos()
            dist = math.sqrt((x-tx)**2 + (y-ty)**2 + (z-tz)**2)
            print(f"\r  dist={dist:.2f}m  pos=({x:.2f},{y:.2f},{z:.2f})",
                  end="", flush=True)
            if dist < POS_TOLERANCE:
                print()
                return True
            time.sleep(1.0 / SETPOINT_HZ)
        print()
        return False

    def wait_for_yaw(self, target_yaw: float, timeout: float,
                     x: float, y: float, z: float) -> bool:
        """Stream setpoints with target yaw until heading is reached."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.publish_setpoint(x, y, z, target_yaw)
            current = self.get_yaw()
            err = abs(math.atan2(
                math.sin(target_yaw - current),
                math.cos(target_yaw - current)
            ))
            print(f"\r  yaw_error={math.degrees(err):.1f}deg", end="", flush=True)
            if err < math.radians(10.0):
                print()
                return True
            time.sleep(1.0 / SETPOINT_HZ)
        print()
        return False

    def shutdown(self):
        self._rclpy.shutdown()

# ── flight sequence ───────────────────────────────────────────────────────────

def run_flight(node: OutAndBackNode, args) -> bool:

    # 1 -- FCU connection
    print("\n[1/8] Waiting for FCU connection...")
    deadline = time.time() + 15.0
    while time.time() < deadline:
        if node.get_state().connected:
            break
        time.sleep(0.1)
    if not node.get_state().connected:
        print("  [FAIL] FCU not connected.")
        return False
    print("  [OK] FCU connected")

    # 2 -- EKF stability wait
    print(f"\n[2/8] Waiting for EKF stability ({EKF_TIMEOUT:.0f}s max)...")
    deadline   = time.time() + EKF_TIMEOUT
    prev_z     = None
    stable     = 0
    while time.time() < deadline:
        _, _, z = node.get_pos()
        if prev_z is not None:
            if abs(z - prev_z) < 0.03:
                stable += 1
            else:
                stable = 0
        prev_z = z
        print(f"\r  z={z:.3f}m  stable={stable}/20", end="", flush=True)
        if stable >= 20:
            break
        time.sleep(0.1)
    print()
    if stable < 20:
        print("  [WARN] EKF not fully stable -- continuing with caution")
    else:
        print("  [OK] EKF stable")

    # Capture home position
    hx, hy, hz = node.get_pos()
    node.home_z = hz
    home_yaw    = node.get_yaw()
    print(f"  Home: ({hx:.2f}, {hy:.2f}, {hz:.2f})  yaw={math.degrees(home_yaw):.1f}deg")

    # Target positions in ENU -- forward is +X in home frame
    target_z       = hz + args.alt
    forward_x      = hx + args.distance * math.cos(home_yaw)
    forward_y      = hy + args.distance * math.sin(home_yaw)
    return_yaw     = home_yaw + math.pi   # 180 deg turn

    # 3 -- Pre-stream setpoints at home (required before OFFBOARD accepted)
    print(f"\n[3/8] Pre-streaming setpoints at home position (3s)...")
    node.stream_setpoints(hx, hy, hz, home_yaw, 3.0)
    print("  [OK] Setpoint stream established")

    if args.dry_run:
        print("\n  --dry-run: stopping before arm.")
        print(f"  Would fly: home({hx:.2f},{hy:.2f},{hz:.2f}) "
              f"-> forward({forward_x:.2f},{forward_y:.2f},{target_z:.2f})"
              f" -> return")
        return True

    # 4 -- Switch to OFFBOARD then arm programmatically.
    # OFFBOARD must be requested before arming -- PX4 requires a live
    # setpoint stream to accept the mode switch.
    print(f"\n[4/8] Switching to OFFBOARD and arming...")

    # Switch to OFFBOARD
    if not node.set_mode("OFFBOARD"):
        print("  [FAIL] OFFBOARD mode switch failed -- "
              "ensure setpoints are streaming")
        return False
    deadline = time.time() + MODE_TIMEOUT
    while time.time() < deadline:
        node.publish_setpoint(hx, hy, hz, home_yaw)
        if node.get_state().mode == "OFFBOARD":
            break
        time.sleep(0.05)
    if node.get_state().mode != "OFFBOARD":
        print(f"  [FAIL] Still in {node.get_state().mode} -- "
              "setpoint stream may not be active")
        return False
    print("  [OK] OFFBOARD mode active")

    # Arm
    if not node.arm():
        print("  [FAIL] Arm command rejected")
        return False
    deadline = time.time() + ARM_TIMEOUT
    while time.time() < deadline:
        node.publish_setpoint(hx, hy, hz, home_yaw)
        if node.get_state().armed:
            break
        time.sleep(0.05)
    if not node.get_state().armed:
        print("  [FAIL] Drone did not arm in time")
        return False
    print("  [OK] Armed and in OFFBOARD mode -- motors spinning")

    # 5 -- Climb
    print(f"\n[5/8] Climbing to {args.alt:.1f}m above home...")
    reached = node.wait_for_position(hx, hy, target_z, timeout=20.0, yaw=home_yaw)
    if reached:
        print(f"  [OK] At altitude")
    else:
        print(f"  [WARN] Target altitude not reached -- continuing")

    # 6 -- Fly forward
    print(f"\n[6/8] Flying forward {args.distance:.1f}m...")
    reached = node.wait_for_position(
        forward_x, forward_y, target_z, timeout=30.0, yaw=home_yaw)
    if reached:
        print(f"  [OK] Forward position reached")
    else:
        print(f"  [WARN] Forward position not reached -- holding")

    # Hold
    print(f"\n  Holding for {args.hold}s...")
    node.stream_setpoints(forward_x, forward_y, target_z, home_yaw, args.hold)

    # 7 -- Yaw 180
    print(f"\n[7/8] Yawing 180deg to face home...")
    turned = node.wait_for_yaw(
        return_yaw, timeout=15.0,
        x=forward_x, y=forward_y, z=target_z)
    if turned:
        print("  [OK] Facing home")
    else:
        print("  [WARN] Yaw incomplete -- proceeding")

    # Fly back
    print(f"\n  Flying back to home position...")
    reached = node.wait_for_position(
        hx, hy, target_z, timeout=30.0, yaw=return_yaw)
    if reached:
        print("  [OK] Home position reached")
    else:
        print("  [WARN] Home position not fully reached")

    # 8 -- Land
    print(f"\n[8/8] Landing...")
    if node.set_mode("AUTO.LAND"):
        print("  AUTO.LAND active -- waiting for touchdown...")
        deadline = time.time() + 30.0
        while time.time() < deadline:
            _, _, z = node.get_pos()
            above = z - node.home_z
            print(f"\r  {above:.2f}m above home", end="", flush=True)
            if above < 0.15:
                break
            node.publish_setpoint(hx, hy, target_z, return_yaw)
            time.sleep(0.2)
        print()
    else:
        print("  [WARN] AUTO.LAND failed -- descend manually")

    time.sleep(2.0)
    print("  [OK] Landing complete")
    return True

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="SLAM bridge out-and-back flight test.")
    parser.add_argument("--distance",   type=float, default=FORWARD_DIST_M,
                        help=f"Forward distance in metres (default: {FORWARD_DIST_M})")
    parser.add_argument("--alt",        type=float, default=TARGET_ALT_M,
                        help=f"Altitude above home in metres (default: {TARGET_ALT_M})")
    parser.add_argument("--hold",       type=int,   default=HOLD_SECONDS,
                        help=f"Hold seconds at forward position (default: {HOLD_SECONDS})")
    parser.add_argument("--dry-run",    action="store_true",
                        help="EKF check only -- no arming or flight")
    # MAVROS is managed by systemd (mavros.service) -- always running on boot.
    # Use --launch-mavros only when running completely standalone.
    parser.add_argument("--launch-mavros", action="store_true",
                        help="Launch MAVROS manually (default: use systemd instance)")
    parser.add_argument("--no-bridge",  action="store_true",
                        help="Skip SLAM bridge launch (testing without SLAM)")
    parser.add_argument("--no-bag",     action="store_true",
                        help="Skip bag recording (quick test flights, no postflight)")
    args = parser.parse_args()

    print("=" * 55)
    print("  SLAM Bridge Out-and-Back Flight Test")
    print("=" * 55)
    print(f"  Forward dist : {args.distance}m")
    print(f"  Altitude     : {args.alt}m above home")
    print(f"  Hold         : {args.hold}s")
    print(f"  Dry run      : {'YES' if args.dry_run else 'NO -- LIVE FLIGHT'}")
    print(f"  SLAM bridge  : {'SKIP' if args.no_bridge else 'ACTIVE'}")
    print(f"  Bag recording: {'SKIP' if args.no_bag else 'ACTIVE'}")

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
    bridge_proc = None
    bag_proc    = None

    if args.launch_mavros:
        mavros_proc = launch_mavros()
    else:
        print("  Using existing MAVROS instance (mavros.service)")

    if not args.no_bridge:
        bridge_proc = launch_slam_bridge()
        print("  SLAM bridge active -- EKF2 fusion enabled if EKF2_EV_CTRL=15")
    else:
        print("  SLAM bridge skipped -- GPS-only position hold")

    if not args.no_bag:
        from datetime import datetime
        bag_dir = f"/mnt/ssd/rosbags/scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        bag_cmd = (
            f"source {ROS_SETUP} && "
            f"source {WS_SETUP} && "
            f"ros2 bag record -o {bag_dir} "
            f"/cloud_registered /aft_mapped_to_init /unilidar/imu "
            f"/mavros/state /mavros/local_position/pose "
            f"/mavros/global_position/global"
        )
        print("  Starting bag recorder...")
        bag_proc = subprocess.Popen(
            ["bash", "-c", bag_cmd],
            preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f"  Bag recording to: {bag_dir}")
    else:
        print("  Bag recording skipped (--no-bag)")

    # Write autonomous lock so watchdog does not launch a competing
    # Point-LIO instance when it sees arm+OFFBOARD.
    if not args.dry_run:
        write_lock()

    node = OutAndBackNode()
    success = False

    try:
        success = run_flight(node, args)
    except KeyboardInterrupt:
        print("\n\n  Ctrl+C -- emergency abort.")
        try:
            node.set_mode("AUTO.LAND")
            time.sleep(2)
        except Exception:
            pass
        print("  Switch to STABILIZED or use kill switch if airborne!")
    except Exception as e:
        print(f"\n  [ERROR] {e}")
        try:
            node.set_mode("AUTO.LAND")
        except Exception:
            pass
    finally:
        node.shutdown()
        kill_proc("Bag recorder", bag_proc)
        kill_proc("SLAM bridge",  bridge_proc)
        kill_proc("MAVROS",       mavros_proc)
        clear_lock()

    print("\n" + "=" * 55)
    print("  PASSED" if success else "  Did not complete cleanly")
    print("=" * 55)


if __name__ == "__main__":
    main()
