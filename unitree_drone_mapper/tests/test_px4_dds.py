#!/usr/bin/env python3
"""PX4 DDS communication test via ROS 2.

Tests the full DDS stack:
  1. Starts MicroXRCEAgent (serial bridge to Pixhawk)
  2. Waits for /fmu/out/vehicle_status to appear
  3. Reads arm state, mode, battery
  4. Publishes offboard control mode heartbeat
  5. Arms the drone via /fmu/in/vehicle_command
  6. Optionally bridges SLAM odometry -> /fmu/in/vehicle_visual_odometry

Prerequisites:
  - MicroXRCEAgent installed (sudo make install in /tmp/MicroXRCEAgent/build)
  - px4_msgs built in RPI5/ros2_ws
  - Source: source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash

Run:
    # Ethernet (Pixhawk 6X) — recommended
    python3 -m tests.test_px4_dds --udp
    python3 -m tests.test_px4_dds --udp --no-arm

    # Serial (USB-UART adapter on TELEM2)
    python3 -m tests.test_px4_dds --port /dev/ttyUSB1 --baud 921600

    # SLAM bridge
    python3 -m tests.test_px4_dds --udp --with-slam

IMPORTANT: Remove propellers before arming.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import threading
import time

# ── config ────────────────────────────────────────────────────────────────────
PORT          = "/dev/ttyUSB1"   # Serial: USB-UART on TELEM2
BAUD          = 921600
UDP_PORT      = 8888             # UDP: Pixhawk ethernet
AGENT_HOST    = "192.168.1.2"   # Pixhawk ethernet IP
TOPIC_TIMEOUT = 15.0   # Seconds to wait for /fmu/out topics to appear
ARM_HOLD_SEC  = 5      # Seconds to stay armed before auto-disarm

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP  = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)

# NAV state map (PX4 NavigatorState)
NAV_STATES = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
    4: "AUTO_LOITER", 5: "AUTO_RTL", 14: "OFFBOARD",
    15: "STAB", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
}
ARMING_STATES = {
    1: "STANDBY", 2: "ARMED", 3: "STANDBY_ERROR", 4: "SHUTDOWN",
}


# ══════════════════════════════════════════════════════════════════════════════
#  MicroXRCEAgent subprocess
# ══════════════════════════════════════════════════════════════════════════════

def find_xrce_agent() -> str | None:
    """Find MicroXRCEAgent binary (snap or custom build)."""
    candidates = [
        "micro-xrce-dds-agent",          # snap install (preferred)
        "MicroXRCEAgent",                 # custom build in PATH
        "/usr/local/bin/MicroXRCEAgent",  # custom build copied manually
        "/usr/bin/MicroXRCEAgent",
    ]
    for c in candidates:
        check = ["which", c] if "/" not in c else ["test", "-x", c]
        result = subprocess.run(check, capture_output=True)
        if result.returncode == 0:
            if "/" in c:
                return c
            return subprocess.run(
                ["which", c], capture_output=True, text=True
            ).stdout.strip()
    return None


def start_xrce_agent(udp: bool, port: str, baud: int,
                     udp_port: int = UDP_PORT) -> subprocess.Popen | None:
    """Launch MicroXRCEAgent as a subprocess (serial or UDP)."""
    agent_bin = find_xrce_agent()
    if not agent_bin:
        print("  [FAIL] MicroXRCEAgent not found.")
        print("         Install via snap: sudo snap install micro-xrce-dds-agent")
        return None

    if udp:
        cmd = [agent_bin, "udp4", "-p", str(udp_port)]   # PX4 docs use -p not --port
    else:
        cmd = [agent_bin, "serial", "--dev", port, "-b", str(baud)]

    print(f"  Starting MicroXRCEAgent: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    time.sleep(2)
    if proc.poll() is not None:
        print(f"  [FAIL] MicroXRCEAgent exited immediately (code {proc.returncode})")
        out, _ = proc.communicate()
        print(f"  Output: {out.decode()[:300]}")
        return None

    mode = f"UDP port {udp_port}" if udp else f"serial {port} @ {baud}"
    print(f"  [OK] MicroXRCEAgent running ({mode})  PID {proc.pid}")
    return proc


def stop_xrce_agent(proc: subprocess.Popen):
    if proc and proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        print("  MicroXRCEAgent stopped")


# ══════════════════════════════════════════════════════════════════════════════
#  PX4 DDS ROS 2 node
# ══════════════════════════════════════════════════════════════════════════════

class PX4DDSTestNode:
    """Handles PX4 DDS communication for the test."""

    def __init__(self, with_slam: bool = False):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                                QoSDurabilityPolicy, QoSHistoryPolicy)

        self._rclpy = rclpy
        rclpy.init()

        SENSOR_QOS = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        RELIABLE_QOS = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        from px4_msgs.msg import (
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
            VehicleStatus,
            VehicleLocalPosition,
            BatteryStatus,
        )

        class _Node(Node):
            def __init__(inner):
                super().__init__("px4_dds_test")

                # State
                inner.arming_state = 0
                inner.nav_state    = 0
                inner.battery_pct  = 0.0
                inner.pos_x = inner.pos_y = inner.pos_z = 0.0
                inner.status_received  = False
                inner.position_received = False

                # Publishers
                inner.offboard_pub = inner.create_publisher(
                    OffboardControlMode,
                    "/fmu/in/offboard_control_mode",
                    RELIABLE_QOS,
                )
                inner.setpoint_pub = inner.create_publisher(
                    TrajectorySetpoint,
                    "/fmu/in/trajectory_setpoint",
                    RELIABLE_QOS,
                )
                inner.command_pub = inner.create_publisher(
                    VehicleCommand,
                    "/fmu/in/vehicle_command",
                    RELIABLE_QOS,
                )

                # Subscribers
                inner.create_subscription(
                    VehicleStatus, "/fmu/out/vehicle_status",
                    inner._on_status, SENSOR_QOS,
                )
                inner.create_subscription(
                    VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                    inner._on_position, SENSOR_QOS,
                )
                inner.create_subscription(
                    BatteryStatus, "/fmu/out/battery_status",
                    inner._on_battery, SENSOR_QOS,
                )

                # SLAM bridge (optional)
                if with_slam:
                    from px4_msgs.msg import VehicleOdometry
                    from nav_msgs.msg import Odometry as ROSOdometry
                    inner.odom_pub = inner.create_publisher(
                        VehicleOdometry,
                        "/fmu/in/vehicle_visual_odometry",
                        RELIABLE_QOS,
                    )
                    inner.create_subscription(
                        ROSOdometry, "/Odometry",
                        inner._on_slam_odom, SENSOR_QOS,
                    )
                    inner.get_logger().info(
                        "SLAM bridge active: /Odometry -> /fmu/in/vehicle_visual_odometry"
                    )

                # Heartbeat at 10 Hz (required for offboard mode)
                inner.hb_timer = inner.create_timer(0.1, inner._heartbeat)

            def _on_status(inner, msg):
                inner.arming_state    = msg.arming_state
                inner.nav_state       = msg.nav_state
                inner.status_received = True

            def _on_position(inner, msg):
                inner.pos_x = msg.x
                inner.pos_y = msg.y
                inner.pos_z = msg.z
                inner.position_received = True

            def _on_battery(inner, msg):
                inner.battery_pct = msg.remaining * 100.0

            def _heartbeat(inner):
                msg = OffboardControlMode()
                msg.position     = True
                msg.velocity     = False
                msg.acceleration = False
                msg.attitude     = False
                msg.body_rate    = False
                msg.timestamp    = int(inner.get_clock().now().nanoseconds / 1000)
                inner.offboard_pub.publish(msg)

                # Also send hold setpoint (stay in place)
                sp = TrajectorySetpoint()
                sp.position  = [inner.pos_x, inner.pos_y, inner.pos_z]
                sp.yaw       = float("nan")
                sp.timestamp = msg.timestamp
                inner.setpoint_pub.publish(sp)

            def _on_slam_odom(inner, msg):
                """Forward Point-LIO odometry to PX4 as visual odometry."""
                from px4_msgs.msg import VehicleOdometry
                odom = VehicleOdometry()
                odom.timestamp = int(inner.get_clock().now().nanoseconds / 1000)
                odom.timestamp_sample = odom.timestamp
                # NED frame: Point-LIO is ENU, swap x/y, negate z
                odom.position = [
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.x,
                    -msg.pose.pose.position.z,
                ]
                q = msg.pose.pose.orientation
                odom.q = [q.w, q.x, q.y, q.z]
                odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
                inner.odom_pub.publish(odom)

            def send_command(inner, cmd_id: int, p1=0.0, p2=0.0, p7=0.0):
                msg = VehicleCommand()
                msg.command          = cmd_id
                msg.param1           = float(p1)
                msg.param2           = float(p2)
                msg.param7           = float(p7)
                msg.target_system    = 1
                msg.target_component = 1
                msg.source_system    = 1
                msg.source_component = 1
                msg.from_external    = True
                msg.timestamp        = int(inner.get_clock().now().nanoseconds / 1000)
                inner.command_pub.publish(msg)

        self._node = _Node()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def start(self):
        self._thread.start()

    def _spin(self):
        while not self._stop.is_set() and self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.05)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3)
        if self._rclpy.ok():
            self._rclpy.shutdown()

    def wait_for_status(self, timeout: float = TOPIC_TIMEOUT) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._node.status_received:
                return True
            time.sleep(0.2)
        return False

    def arm(self):
        """Send arm command (MAV_CMD_COMPONENT_ARM_DISARM = 400)."""
        self._node.send_command(400, p1=1.0, p2=21196.0)  # 21196 = force arm

    def disarm(self):
        """Send disarm command."""
        self._node.send_command(400, p1=0.0, p2=21196.0)

    def set_offboard_mode(self):
        """Switch PX4 to offboard mode (MAV_CMD_DO_SET_MODE = 176)."""
        # PX4: base_mode=1 (custom), custom_main_mode=6 (offboard)
        self._node.send_command(176, p1=1.0, p2=6.0)

    @property
    def is_armed(self) -> bool:
        return self._node.arming_state == 2

    @property
    def nav_state_name(self) -> str:
        return NAV_STATES.get(self._node.nav_state, f"STATE_{self._node.nav_state}")

    @property
    def arming_state_name(self) -> str:
        return ARMING_STATES.get(self._node.arming_state, f"STATE_{self._node.arming_state}")

    @property
    def battery_pct(self) -> float:
        return self._node.battery_pct

    @property
    def position(self) -> tuple:
        n = self._node
        return (n.pos_x, n.pos_y, n.pos_z)


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="PX4 DDS test via ROS 2")
    parser.add_argument("--udp",         action="store_true", help="Use UDP transport (ethernet) instead of serial")
    parser.add_argument("--udp-port",   default=UDP_PORT,   type=int, help=f"UDP port (default: {UDP_PORT})")
    parser.add_argument("--agent-host", default=AGENT_HOST, help=f"Pixhawk ethernet IP (default: {AGENT_HOST})")
    parser.add_argument("--port",       default=PORT,  help=f"Serial port for serial mode (default: {PORT})")
    parser.add_argument("--baud",       default=BAUD,  type=int, help=f"Baud rate (default: {BAUD})")
    parser.add_argument("--no-arm",     action="store_true", help="Status only — do not arm")
    parser.add_argument("--no-agent",   action="store_true", help="Skip launching MicroXRCEAgent (already running)")
    parser.add_argument("--with-slam",  action="store_true", help="Bridge /Odometry -> /fmu/in/vehicle_visual_odometry")
    parser.add_argument("--hold",       default=ARM_HOLD_SEC, type=int, help="Seconds to stay armed")
    args = parser.parse_args()

    print("=" * 60)
    print("  PX4 DDS Communication Test (ROS 2)")
    print("=" * 60)

    if not args.no_arm:
        print("\n  WARNING: This test will ARM the drone via DDS.")
        print("  Remove propellers. Press Ctrl+C in 3 s to cancel.\n")
        try:
            time.sleep(3)
        except KeyboardInterrupt:
            print("  Cancelled.")
            sys.exit(0)

    # ── [1] MicroXRCEAgent ────────────────────────────────────────────────────
    agent_proc = None
    if not args.no_agent:
        print("\n[1] Starting MicroXRCEAgent...")
        agent_proc = start_xrce_agent(args.udp, args.port, args.baud, args.udp_port)
        if not agent_proc:
            print("\n  Cannot proceed without DDS agent.")
            print("  Build it first:")
            print("    cd /tmp/MicroXRCEAgent/build && sudo make install")
            sys.exit(1)
    else:
        print("\n[1] Skipping MicroXRCEAgent (--no-agent set)")

    # ── [2] ROS 2 DDS node ────────────────────────────────────────────────────
    print("\n[2] Starting ROS 2 DDS node...")
    px4 = PX4DDSTestNode(with_slam=args.with_slam)
    px4.start()
    print("  Subscribing to /fmu/out/vehicle_status ...")

    if args.with_slam:
        print("  SLAM bridge: /Odometry -> /fmu/in/vehicle_visual_odometry ACTIVE")

    # ── [3] Wait for PX4 data ─────────────────────────────────────────────────
    print(f"\n[3] Waiting for PX4 DDS topics (up to {TOPIC_TIMEOUT:.0f}s)...")
    if not px4.wait_for_status(TOPIC_TIMEOUT):
        print(f"\n  [FAIL] No data from /fmu/out/vehicle_status after {TOPIC_TIMEOUT}s")
        print("\n  Troubleshooting:")
        print("  1. Check MicroXRCEAgent connected to correct port/baud")
        print("  2. PX4 firmware must be v1.14+ for native XRCE-DDS client")
        print("  3. Check PX4 param: XRCE_DDS_CFG must be set to enable DDS")
        print("     In QGroundControl: Parameters > XRCE_DDS_CFG = 1 (Serial)")
        print("  4. Check PX4 param: SER_TEL2_BAUD = 921600 (if using TELEM2)")
        px4.stop()
        stop_xrce_agent(agent_proc)
        sys.exit(1)

    print(f"  [OK] PX4 DDS connected")

    # ── [4] Status ────────────────────────────────────────────────────────────
    print("\n[4] Vehicle status:")
    print(f"  Arming state : {px4.arming_state_name}")
    print(f"  Nav mode     : {px4.nav_state_name}")
    print(f"  Battery      : {px4.battery_pct:.1f}%")
    x, y, z = px4.position
    print(f"  Position NED : x={x:.2f}  y={y:.2f}  z={z:.2f}  (m)")

    if args.no_arm:
        print("\n  --no-arm set. Monitoring for 10 s then exiting.")
        try:
            for _ in range(10):
                time.sleep(1)
                print(f"  {px4.arming_state_name:12s} | {px4.nav_state_name:15s} | "
                      f"bat={px4.battery_pct:.1f}%", end="\r")
        except KeyboardInterrupt:
            pass
        print()
        px4.stop()
        stop_xrce_agent(agent_proc)
        return

    # ── [5] Switch to offboard mode ───────────────────────────────────────────
    print("\n[5] Switching to OFFBOARD mode...")
    # Must send several heartbeats first before mode switch is accepted
    time.sleep(1.0)
    px4.set_offboard_mode()
    time.sleep(0.5)
    print(f"  Mode: {px4.nav_state_name}")

    # ── [6] Arm ───────────────────────────────────────────────────────────────
    print("\n[6] Arming...")
    px4.arm()
    time.sleep(1.0)

    if px4.is_armed:
        print(f"  [OK] ARMED  (mode: {px4.nav_state_name})")
    else:
        print(f"  [FAIL] Not armed — state: {px4.arming_state_name}")
        print("\n  Common causes:")
        print("  - Pre-arm checks failing (RC signal, gyro calib, etc.)")
        print("  - Not in OFFBOARD mode (set XRCE_DDS_CFG and retry)")
        print("  - PX4 param CBRK_USB_CHK must be 197848 to arm via USB")

    # ── [7] Hold armed ────────────────────────────────────────────────────────
    if px4.is_armed:
        print(f"\n  Holding armed for {args.hold}s — Ctrl+C to disarm early...")
        try:
            for i in range(args.hold, 0, -1):
                x, y, z = px4.position
                print(f"  {i}s  armed={px4.is_armed}  pos=({x:.2f},{y:.2f},{z:.2f})  "
                      f"bat={px4.battery_pct:.1f}%", end="\r")
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        print()

    # ── [8] Disarm ────────────────────────────────────────────────────────────
    print("\n[7] Disarming...")
    px4.disarm()
    time.sleep(1.0)
    print(f"  State: {px4.arming_state_name}")

    # ── cleanup ───────────────────────────────────────────────────────────────
    px4.stop()
    stop_xrce_agent(agent_proc)

    print("\n" + "=" * 60)
    armed_final = px4.is_armed
    if not armed_final:
        print("  Test complete. Vehicle DISARMED. Safe.")
    else:
        print("  [WARN] Vehicle may still be armed — verify manually!")
    print("=" * 60)


if __name__ == "__main__":
    main()
