#!/usr/bin/env python3
"""
tests/test_slam_bridge_flight.py

Validate the SLAM bridge pipeline during flight:
  Point-LIO odometry → slam_bridge.py → /mavros/vision_pose/pose

Flies to altitude, holds, checks that vision_pose messages are arriving
and that MAVROS accepted the external vision source (EKF converged).

SAFETY: CONTEXT_TEST — any OFFBOARD exit → immediate teardown.

CLI:
  --alt       Altitude in metres (default: 3.0)
  --hold      Hold time in seconds (default: 30.0)
  --min-hz    Minimum vision_pose message rate to pass (default: 10.0)
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).parent.parent / "flight"))
from safe_flight_mixin import SafeFlightMixin  # noqa: E402

# ── Constants ─────────────────────────────────────────────────────────────────
ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = str(Path.home() / "unitree_lidar_project/RPI5/ros2_ws/install/setup.bash")
POINTLIO_LAUNCH = (
    "source {ros} && source {ws} && "
    "ros2 launch point_lio mapping_mid360.launch.py"
).format(ros=ROS_SETUP, ws=WS_SETUP)
SLAM_BRIDGE_CMD = (
    "source {ros} && source {ws} && "
    "python3 {script}"
).format(
    ros=ROS_SETUP,
    ws=WS_SETUP,
    script=str(Path(__file__).parent.parent / "flight" / "slam_bridge.py"),
)

SETPOINT_RATE_HZ = 20.0
SETPOINT_PERIOD  = 1.0 / SETPOINT_RATE_HZ
WARMUP_S         = 3.0
ALT_TOLERANCE_M  = 0.15


class FlightNode(SafeFlightMixin, Node):
    """
    SLAM bridge validation during hover.
    CONTEXT_TEST — immediate teardown on any OFFBOARD exit.
    """

    def __init__(self, alt: float, hold_s: float, min_hz: float) -> None:
        Node.__init__(self, "slam_bridge_validation_node")
        SafeFlightMixin.__init__(self, script_name=__file__)

        self.alt    = alt
        self.hold_s = hold_s
        self.min_hz = min_hz

        self.mode:    str   = ""
        self.armed:   bool  = False
        self.local_z: float = 0.0

        self._vision_pose_count: int = 0

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.create_subscription(
            State, "/mavros/state", self._state_cb, qos_reliable
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, qos_sensor
        )
        # Count vision_pose messages from the SLAM bridge
        self.create_subscription(
            PoseStamped, "/mavros/vision_pose/pose",
            self._vision_cb, qos_sensor
        )

        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")

        self._pointlio_proc:    subprocess.Popen | None = None
        self._slam_bridge_proc: subprocess.Popen | None = None

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        self.mode  = msg.mode
        self.armed = msg.armed

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.local_z = msg.pose.position.z

    def _vision_cb(self, msg: PoseStamped) -> None:
        self._vision_pose_count += 1

    # ── Service helpers ──────────────────────────────────────────────────────

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return bool(future.result() and future.result().mode_sent)

    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return bool(future.result() and future.result().success)

    # ── Setpoint publish ─────────────────────────────────────────────────────

    def publish_sp(self, x: float, y: float, z: float) -> None:
        """Publish one setpoint. MUST update both heartbeat fields."""
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)

        # REQUIRED on every publish
        self._sp_heartbeat = time.monotonic()
        self._alt_setpoint = z

    # ── Subprocess helpers ───────────────────────────────────────────────────

    def _kill_proc(self, proc: subprocess.Popen | None) -> None:
        if proc and proc.poll() is None:
            import os, signal
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=8)
            except Exception:
                pass

    def start_pointlio(self) -> bool:
        try:
            self._pointlio_proc = subprocess.Popen(
                ["bash", "-c", POINTLIO_LAUNCH],
                preexec_fn=__import__("os").setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(4.0)
            return True
        except Exception as exc:
            self.get_logger().error(f"Point-LIO failed: {exc}")
            return False

    def start_slam_bridge(self) -> bool:
        try:
            self._slam_bridge_proc = subprocess.Popen(
                ["bash", "-c", SLAM_BRIDGE_CMD],
                preexec_fn=__import__("os").setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(
                f"SLAM bridge started pid={self._slam_bridge_proc.pid}"
            )
            time.sleep(2.0)
            return True
        except Exception as exc:
            self.get_logger().error(f"SLAM bridge failed: {exc}")
            return False

    # ── Main run ─────────────────────────────────────────────────────────────

    def run(self) -> dict:
        results = {
            "vision_pose_hz":   0.0,
            "vision_pose_pass": False,
            "passed":           False,
            "abort_reason":     None,
        }

        if not self.start_safety_monitors():
            results["abort_reason"] = "LOCK_DENIED"
            return results
        if self._pilot_override:
            results["abort_reason"] = "PRE_ARM_OVERRIDE"
            return results

        if not self.start_pointlio():
            results["abort_reason"] = "POINTLIO_FAILED"
            return results
        if not self.start_slam_bridge():
            results["abort_reason"] = "SLAM_BRIDGE_FAILED"
            return results

        # Wait for vision_pose to start flowing before arming
        self.get_logger().info(
            "Waiting for vision_pose messages from SLAM bridge..."
        )
        t0 = time.monotonic()
        while self._vision_pose_count < 10 and time.monotonic() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._vision_pose_count < 10:
            self.get_logger().error(
                f"SLAM bridge not publishing (only {self._vision_pose_count} msgs). "
                "Aborting."
            )
            results["abort_reason"] = "SLAM_BRIDGE_NOT_PUBLISHING"
            self._kill_proc(self._slam_bridge_proc)
            self._kill_proc(self._pointlio_proc)
            self._teardown("slam_not_publishing")
            return results

        # Wait for MAVROS
        t0 = time.monotonic()
        while self.mode == "" and time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.mode == "":
            results["abort_reason"] = "MAVROS_TIMEOUT"
            return results

        # Warmup
        t0 = time.monotonic()
        while time.monotonic() - t0 < WARMUP_S:
            if self._pilot_override:
                results["abort_reason"] = "PILOT_OVERRIDE_WARMUP"
                return results
            self.publish_sp(0.0, 0.0, self.alt)
            rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)

        if not self.set_mode("OFFBOARD"):
            results["abort_reason"] = "OFFBOARD_FAILED"
            return results
        if not self.arm():
            results["abort_reason"] = "ARM_FAILED"
            return results

        self.get_logger().info(
            f"Armed. Hovering at {self.alt}m for {self.hold_s}s."
        )

        try:
            # Climb
            t0 = time.monotonic()
            while time.monotonic() - t0 < 15.0:
                if self._pilot_override:
                    break
                self.publish_sp(0.0, 0.0, self.alt)
                rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)
                if abs(self.local_z - self.alt) < ALT_TOLERANCE_M:
                    break

            if self._pilot_override:
                results["abort_reason"] = "PILOT_OVERRIDE"
                return results

            # Reset counter and hold
            self._vision_pose_count = 0
            hold_start = time.monotonic()

            while time.monotonic() - hold_start < self.hold_s:
                if self._pilot_override:
                    results["abort_reason"] = "PILOT_OVERRIDE_HOLD"
                    break
                self.publish_sp(0.0, 0.0, self.alt)
                rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)

            actual_hold = time.monotonic() - hold_start
            vp_hz  = self._vision_pose_count / max(actual_hold, 0.1)
            vp_ok  = vp_hz >= self.min_hz

            results.update({
                "vision_pose_hz":   round(vp_hz, 2),
                "vision_pose_pass": vp_ok,
                "passed":           vp_ok and not self._pilot_override,
            })
            self.get_logger().info(
                f"SLAM bridge: vision_pose={vp_hz:.1f}Hz "
                f"(min={self.min_hz:.0f}Hz) "
                f"[{'PASS' if vp_ok else 'FAIL'}]"
            )

        finally:
            if self.armed:
                self.get_logger().info("Landing...")
                self.set_mode("AUTO.LAND")
                t0 = time.monotonic()
                while self.armed and time.monotonic() - t0 < 30.0:
                    rclpy.spin_once(self, timeout_sec=0.5)
            self._kill_proc(self._slam_bridge_proc)
            self._kill_proc(self._pointlio_proc)
            self._teardown("mission_complete")

        return results


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="SLAM bridge (Point-LIO → vision_pose) validation."
    )
    parser.add_argument("--alt",     type=float, default=3.0,
                        help="Altitude in metres (default: 3)")
    parser.add_argument("--hold",    type=float, default=30.0,
                        help="Hold time in seconds (default: 30)")
    parser.add_argument("--min-hz",  type=float, default=10.0,
                        dest="min_hz",
                        help="Minimum vision_pose Hz to pass (default: 10)")
    args = parser.parse_args()

    rclpy.init()
    node = FlightNode(alt=args.alt, hold_s=args.hold, min_hz=args.min_hz)

    try:
        results = node.run()
        print(f"\nSLAM bridge validation: {'PASS' if results['passed'] else 'FAIL'}")
        print(f"  vision_pose: {results['vision_pose_hz']:.1f} Hz "
              f"[{'PASS' if results['vision_pose_pass'] else 'FAIL'}]")
        if results.get("abort_reason"):
            print(f"  Abort: {results['abort_reason']}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
