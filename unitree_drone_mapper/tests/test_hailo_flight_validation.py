#!/usr/bin/env python3
"""
tests/test_hailo_flight_validation.py

Hover at altitude and validate that the Hailo-8L NPU is publishing inference
results on /hailo/optical_flow and /hailo/ground_class throughout the flight.

SAFETY: CONTEXT_TEST — any OFFBOARD exit → immediate teardown.

CLI:
  --alt   Altitude in metres (default: 3.0)
  --hold  Hold time in seconds (default: 30.0)
"""

import argparse, subprocess, sys, time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header, String

sys.path.insert(0, str(Path(__file__).parent.parent / "flight"))
from safe_flight_mixin import SafeFlightMixin

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP  = str(Path.home() / "unitree_lidar_project/RPI5/ros2_ws/install/setup.bash")
POINTLIO_LAUNCH = (
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch point_lio mapping_mid360.launch.py"
)
HAILO_LAUNCH = (
    f"source {ROS_SETUP} && source {WS_SETUP} && "
    "ros2 launch hailo_ros hailo_inference.launch.py"
)
SETPOINT_RATE_HZ = 20.0
SETPOINT_PERIOD  = 1.0 / SETPOINT_RATE_HZ
WARMUP_S         = 3.0
HAILO_MIN_MSG_RATE_HZ = 5.0


class FlightNode(SafeFlightMixin, Node):
    def __init__(self, alt: float, hold_s: float) -> None:
        Node.__init__(self, "hailo_validation_node")
        SafeFlightMixin.__init__(self, script_name=__file__)
        self.alt = alt; self.hold_s = hold_s
        self.mode = ""; self.armed = False; self.local_z = 0.0
        self._optical_flow_count = 0; self._ground_class_count = 0

        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_s = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=5)

        self.sp_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", 10)
        self.create_subscription(State, "/mavros/state", self._state_cb, qos_r)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._pose_cb, qos_s)
        self.create_subscription(String, "/hailo/ground_class",
                                 lambda m: self._count("ground_class"), qos_s)
        self.create_subscription(TwistStamped, "/hailo/optical_flow",
                                 lambda m: self._count("optical_flow"), qos_s)
        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")
        self._pointlio_proc = None; self._hailo_proc = None

    def _state_cb(self, msg): self.mode = msg.mode; self.armed = msg.armed
    def _pose_cb(self, msg):  self.local_z = msg.pose.position.z
    def _count(self, which):
        if which == "ground_class": self._ground_class_count += 1
        else: self._optical_flow_count += 1

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

    def _kill(self, p):
        if p and p.poll() is None:
            import os, signal
            try: os.killpg(os.getpgid(p.pid), signal.SIGINT); p.wait(timeout=8)
            except Exception: pass

    def start_pointlio(self):
        try:
            self._pointlio_proc = subprocess.Popen(
                ["bash", "-c", POINTLIO_LAUNCH],
                preexec_fn=__import__("os").setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(4.0); return True
        except Exception as e:
            self.get_logger().error(f"Point-LIO: {e}"); return False

    def start_hailo(self):
        try:
            self._hailo_proc = subprocess.Popen(
                ["bash", "-c", HAILO_LAUNCH],
                preexec_fn=__import__("os").setsid,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info(f"Hailo started pid={self._hailo_proc.pid}")
            time.sleep(3.0); return True
        except Exception as e:
            self.get_logger().error(f"Hailo: {e}"); return False

    def run(self):
        results = {"optical_flow_hz": 0.0, "ground_class_hz": 0.0,
                   "optical_flow_pass": False, "ground_class_pass": False,
                   "passed": False, "abort_reason": None}

        if not self.start_safety_monitors():
            results["abort_reason"] = "LOCK_DENIED"; return results
        if self._pilot_override:
            results["abort_reason"] = "PRE_ARM_OVERRIDE"; return results
        if not self.start_pointlio():
            results["abort_reason"] = "POINTLIO_FAILED"; return results
        if not self.start_hailo():
            results["abort_reason"] = "HAILO_FAILED"; return results

        self.get_logger().info("Waiting for MAVROS...")
        t0 = time.monotonic()
        while self.mode == "" and time.monotonic()-t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.mode == "":
            results["abort_reason"] = "MAVROS_TIMEOUT"; return results

        t0 = time.monotonic()
        while time.monotonic()-t0 < WARMUP_S:
            if self._pilot_override:
                results["abort_reason"] = "PILOT_OVERRIDE_WARMUP"; return results
            self.publish_sp(0.0, 0.0, self.alt)
            rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)

        if not self.set_mode("OFFBOARD"):
            results["abort_reason"] = "OFFBOARD_FAILED"; return results
        if not self.arm():
            results["abort_reason"] = "ARM_FAILED"; return results

        self.get_logger().info(f"Armed. Hovering at {self.alt}m for {self.hold_s}s.")
        try:
            t0 = time.monotonic()
            while time.monotonic()-t0 < 15.0:
                if self._pilot_override: break
                self.publish_sp(0.0, 0.0, self.alt)
                rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)
                if abs(self.local_z - self.alt) < 0.15: break

            if self._pilot_override:
                results["abort_reason"] = "PILOT_OVERRIDE"; return results

            self._optical_flow_count = 0; self._ground_class_count = 0
            hold_start = time.monotonic()
            while time.monotonic()-hold_start < self.hold_s:
                if self._pilot_override:
                    results["abort_reason"] = "PILOT_OVERRIDE_HOLD"; break
                self.publish_sp(0.0, 0.0, self.alt)
                rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)

            actual = time.monotonic()-hold_start
            of_hz = self._optical_flow_count / max(actual, 0.1)
            gc_hz = self._ground_class_count / max(actual, 0.1)
            of_ok = of_hz >= HAILO_MIN_MSG_RATE_HZ
            gc_ok = gc_hz >= HAILO_MIN_MSG_RATE_HZ
            results.update({"optical_flow_hz": round(of_hz,2), "ground_class_hz": round(gc_hz,2),
                            "optical_flow_pass": of_ok, "ground_class_pass": gc_ok,
                            "passed": of_ok and gc_ok and not self._pilot_override})
            self.get_logger().info(
                f"Hailo: optical_flow={of_hz:.1f}Hz {'PASS' if of_ok else 'FAIL'}, "
                f"ground_class={gc_hz:.1f}Hz {'PASS' if gc_ok else 'FAIL'}")
        finally:
            if self.armed:
                self.set_mode("AUTO.LAND")
                t0 = time.monotonic()
                while self.armed and time.monotonic()-t0 < 30.0:
                    rclpy.spin_once(self, timeout_sec=0.5)
            self._kill(self._hailo_proc); self._kill(self._pointlio_proc)
            self._teardown("mission_complete")
        return results


def main():
    parser = argparse.ArgumentParser(description="Hailo-8L inference validation.")
    parser.add_argument("--alt",  type=float, default=3.0)
    parser.add_argument("--hold", type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = FlightNode(alt=args.alt, hold_s=args.hold)
    try:
        r = node.run()
        print(f"\nHailo validation: {'PASS' if r['passed'] else 'FAIL'}")
        print(f"  optical_flow: {r['optical_flow_hz']:.1f}Hz [{'PASS' if r['optical_flow_pass'] else 'FAIL'}]")
        print(f"  ground_class: {r['ground_class_hz']:.1f}Hz [{'PASS' if r['ground_class_pass'] else 'FAIL'}]")
        if r.get("abort_reason"): print(f"  Abort: {r['abort_reason']}")
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
