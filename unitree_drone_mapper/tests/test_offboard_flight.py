#!/usr/bin/env python3
"""
tests/test_offboard_flight.py

Basic OFFBOARD hover and optional waypoint test.
SAFETY: CONTEXT_TEST — any OFFBOARD exit → immediate teardown.

CLI:
  --alt         Altitude in metres (default: 2.0)
  --hold        Hold time in seconds (default: 20.0)
  --waypoints   Optional ENU waypoints as x,y,z e.g. 1,0,2 -1,0,2
"""

import argparse, subprocess, sys, time, math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).parent.parent / "flight"))
from safe_flight_mixin import SafeFlightMixin

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP  = str(Path.home() / "unitree_lidar_project/RPI5/ros2_ws/install/setup.bash")
SETPOINT_RATE_HZ = 20.0
SETPOINT_PERIOD  = 1.0 / SETPOINT_RATE_HZ
WARMUP_S         = 3.0
POS_TOLERANCE_M  = 0.20
ALT_TOLERANCE_M  = 0.15


class FlightNode(SafeFlightMixin, Node):
    def __init__(self, alt, hold_s, waypoints):
        Node.__init__(self, "offboard_flight_node")
        SafeFlightMixin.__init__(self, script_name=__file__)
        self.alt = alt; self.hold_s = hold_s; self.waypoints = waypoints
        self.mode = ""; self.armed = False
        self.local_x = 0.0; self.local_y = 0.0; self.local_z = 0.0

        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_s = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=5)
        self.sp_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.create_subscription(State, "/mavros/state", self._state_cb, qos_r)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose_cb, qos_s)
        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")

    def _state_cb(self, msg): self.mode = msg.mode; self.armed = msg.armed
    def _pose_cb(self, msg):
        self.local_x = msg.pose.position.x
        self.local_y = msg.pose.position.y
        self.local_z = msg.pose.position.z

    def set_mode(self, mode):
        req = SetMode.Request(); req.custom_mode = mode
        f = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)
        return bool(f.result() and f.result().mode_sent)

    def arm(self):
        req = CommandBool.Request(); req.value = True
        f = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=3.0)
        return bool(f.result() and f.result().success)

    def publish_sp(self, x, y, z):
        msg = PoseStamped(); msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x; msg.pose.position.y = y
        msg.pose.position.z = z; msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)
        self._sp_heartbeat = time.monotonic()   # REQUIRED
        self._alt_setpoint = z                  # REQUIRED

    def fly_to(self, x, y, z, timeout_s=20.0):
        t0 = time.monotonic()
        while time.monotonic()-t0 < timeout_s:
            if self._pilot_override: return False
            self.publish_sp(x, y, z)
            rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)
            if (math.hypot(self.local_x-x, self.local_y-y) < POS_TOLERANCE_M
                    and abs(self.local_z-z) < ALT_TOLERANCE_M):
                return True
        return not self._pilot_override

    def run(self):
        if not self.start_safety_monitors():
            self.get_logger().error("Lock denied."); return False
        if self._pilot_override: return False

        t0 = time.monotonic()
        while self.mode == "" and time.monotonic()-t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.mode == "":
            self.get_logger().error("MAVROS timeout."); return False

        t0 = time.monotonic()
        while time.monotonic()-t0 < WARMUP_S:
            if self._pilot_override: return False
            self.publish_sp(0.0, 0.0, self.alt)
            rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)

        if not self.set_mode("OFFBOARD"):
            return False
        if not self.arm():
            return False

        self.get_logger().info(f"Armed. Hover {self.alt}m for {self.hold_s}s.")
        success = True
        try:
            if not self.fly_to(0.0, 0.0, self.alt): success = False
            if success and self.waypoints:
                for i, (wx, wy, wz) in enumerate(self.waypoints):
                    if self._pilot_override: success = False; break
                    self.get_logger().info(f"  WP {i+1}: ({wx:.1f},{wy:.1f},{wz:.1f})")
                    if not self.fly_to(wx, wy, wz): success = False; break
            if success:
                t0 = time.monotonic()
                while time.monotonic()-t0 < self.hold_s:
                    if self._pilot_override: success = False; break
                    self.publish_sp(0.0, 0.0, self.alt)
                    rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)
        finally:
            if self.armed:
                self.set_mode("AUTO.LAND")
                t0 = time.monotonic()
                while self.armed and time.monotonic()-t0 < 30.0:
                    rclpy.spin_once(self, timeout_sec=0.5)
            self._teardown("mission_complete")
        return success


def parse_wp(s):
    p = [float(v) for v in s.split(",")]
    if len(p) != 3: raise argparse.ArgumentTypeError(f"Need x,y,z — got {s}")
    return tuple(p)

def main():
    parser = argparse.ArgumentParser(description="Basic OFFBOARD hover test.")
    parser.add_argument("--alt",  type=float, default=2.0)
    parser.add_argument("--hold", type=float, default=20.0)
    parser.add_argument("--waypoints", nargs="*", type=parse_wp, default=[], metavar="x,y,z")
    args = parser.parse_args()
    rclpy.init()
    node = FlightNode(alt=args.alt, hold_s=args.hold, waypoints=args.waypoints or [])
    try:
        ok = node.run()
        print(f"\nOffboard test: {'COMPLETE' if ok else 'ABORTED'}")
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
