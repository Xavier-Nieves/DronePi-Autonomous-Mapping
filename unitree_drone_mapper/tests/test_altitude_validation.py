#!/usr/bin/env python3
"""
tests/test_altitude_validation.py — Altitude hold validation test.

Steps through a configurable list of altitude targets, holds at each for a
configurable duration, records actual deviation from the setpoint at each step.
Passes if max deviation across all steps stays within ALT_PASS_THRESHOLD.

SAFETY INVARIANTS (non-negotiable, enforced via SafeFlightMixin):
  - Inherits SafeFlightMixin (CONTEXT_TEST) — arming blocked if monitors fail.
  - Bag recorder started as first subprocess inside start_safety_monitors().
  - Setpoint watchdog active: self._sp_heartbeat updated on every publish.
  - self._alt_setpoint updated on every publish (altitude deviation monitor).
  - RC CH7 kill switch: immediate teardown on trigger.
  - RC override (OFFBOARD exit): immediate teardown, no recovery window.
  - No arming if self._pilot_override is True.

STANDALONE TEST — has its own main(), never imported by production code.
"""

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv   import CommandBool, SetMode

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from flight.safe_flight_mixin import SafeFlightMixin, CONTEXT_TEST

# ── Environment-configurable parameters ──────────────────────────────────────
DEFAULT_ALTITUDES    = [float(x) for x in
                        os.environ.get("TEST_ALTITUDES", "1.5 3.0").split()]
DEFAULT_HOLD_S       = float(os.environ.get("TEST_HOLD_S",      "15"))
DEFAULT_CLIMB_RATE_S = float(os.environ.get("TEST_CLIMB_RATE",  "1.5"))  # m/s
ALT_PASS_THRESHOLD   = float(os.environ.get("ALT_PASS_THRESHOLD", "0.25"))  # m
SETPOINT_HZ          = int(  os.environ.get("SETPOINT_HZ",      "20"))
PRE_ARM_OFFBOARD_S   = float(os.environ.get("PRE_ARM_OFFBOARD_S", "3.0"))


class AltValidationNode(SafeFlightMixin, Node):
    """
    Altitude validation test node.

    MRO: SafeFlightMixin must appear before Node so _teardown() and all
    monitor state are established before the ROS executor processes anything.
    """

    def __init__(
        self,
        altitudes:    list[float],
        hold_s:       float,
        climb_rate_s: float,
    ) -> None:
        Node.__init__(self, "alt_validation_node")
        SafeFlightMixin.__init__(self, script_name=__file__, context=CONTEXT_TEST)

        self._altitudes    = altitudes
        self._hold_s       = hold_s
        self._climb_rate_s = climb_rate_s

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10,
        )

        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", qos_be
        )
        self._arm_client      = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")

        self._pos_lock  = threading.Lock()
        self._current_z = 0.0
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, qos_be
        )

        self._results: list[dict] = []

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped) -> None:
        with self._pos_lock:
            self._current_z = msg.pose.position.z

    # ── MAVROS helpers ────────────────────────────────────────────────────────

    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        if not self._set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_mode service unavailable")
            return False
        fut = self._set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().mode_sent)

    def _arm(self, value: bool) -> bool:
        req = CommandBool.Request()
        req.value = value
        if not self._arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("arming service unavailable")
            return False
        fut = self._arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)

    def _publish_setpoint(self, x: float, y: float, z: float) -> None:
        """
        Publish one position setpoint.
        MUST update both _sp_heartbeat and _alt_setpoint on every call
        (feeds the setpoint watchdog and altitude deviation monitor respectively).
        """
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self._sp_pub.publish(msg)
        self._sp_heartbeat = time.monotonic()   # watchdog feed
        self._alt_setpoint = z                  # altitude monitor feed

    # ── Test execution ────────────────────────────────────────────────────────

    def run(self) -> bool:
        """
        Execute altitude steps. Returns True if all steps pass ALT_PASS_THRESHOLD.
        """
        log = self.get_logger()

        # INVARIANT: monitors must be running before any arming attempt.
        # start_safety_monitors() starts the bag recorder as its first action.
        if not self.start_safety_monitors():
            log.error("start_safety_monitors() refused — live lock owner exists.")
            return False

        if self._pilot_override:
            self._teardown("pre-arm override active")
            return False

        log.info(
            f"Alt validation — targets={self._altitudes}m  "
            f"hold={self._hold_s}s  threshold=±{ALT_PASS_THRESHOLD}m"
        )

        # Stream hold-on-ground setpoints before requesting OFFBOARD.
        # PX4 requires setpoints flowing before it accepts the mode switch.
        log.info(f"Pre-arm: streaming ground setpoints for {PRE_ARM_OFFBOARD_S:.0f}s")
        rate   = self.create_rate(SETPOINT_HZ)
        t_pre  = time.monotonic()
        while time.monotonic() - t_pre < PRE_ARM_OFFBOARD_S:
            if self._pilot_override:
                return False
            self._publish_setpoint(0.0, 0.0, 0.0)
            rate.sleep()

        if not self._set_mode("OFFBOARD"):
            log.error("Failed to set OFFBOARD mode.")
            self._teardown("offboard_set_failed")
            return False

        if not self._arm(True):
            log.error("Arming rejected.")
            self._teardown("arm_rejected")
            return False

        log.info("Armed and in OFFBOARD.")

        # ── Altitude step loop ────────────────────────────────────────────────
        all_pass = True
        for step_idx, target_z in enumerate(self._altitudes):
            step_n = step_idx + 1
            if self._pilot_override:
                log.warning("Pilot override — aborting step sequence.")
                break

            log.info(
                f"Step {step_n}/{len(self._altitudes)}: "
                f"climbing to {target_z}m at {self._climb_rate_s}m/s"
            )

            # Ramp altitude setpoint up to target
            with self._pos_lock:
                z_start = self._current_z
            t_climb = time.monotonic()
            while not self._pilot_override:
                elapsed = time.monotonic() - t_climb
                z_cmd   = min(z_start + self._climb_rate_s * elapsed, target_z)
                self._publish_setpoint(0.0, 0.0, z_cmd)
                with self._pos_lock:
                    z_now = self._current_z
                rate.sleep()
                if z_cmd >= target_z and abs(z_now - target_z) < 0.15:
                    break

            # Hold and measure deviation
            log.info(f"Holding at {target_z}m for {self._hold_s}s")
            deviations: list[float] = []
            t_hold = time.monotonic()
            while time.monotonic() - t_hold < self._hold_s:
                if self._pilot_override:
                    break
                self._publish_setpoint(0.0, 0.0, target_z)
                with self._pos_lock:
                    deviations.append(abs(self._current_z - target_z))
                rate.sleep()

            if deviations:
                max_dev = max(deviations)
                avg_dev = sum(deviations) / len(deviations)
                passed  = max_dev <= ALT_PASS_THRESHOLD
                if not passed:
                    all_pass = False
                result = {
                    "step":      step_n,
                    "target_z":  target_z,
                    "max_dev_m": round(max_dev, 4),
                    "avg_dev_m": round(avg_dev, 4),
                    "passed":    passed,
                    "threshold": ALT_PASS_THRESHOLD,
                }
                self._results.append(result)
                log.info(
                    f"  Step {step_n} result: max_dev={max_dev:.3f}m  "
                    f"avg_dev={avg_dev:.3f}m  "
                    f"{'PASS' if passed else 'FAIL (exceeds threshold)'}"
                )

        # Land and disarm
        log.info("Sequence complete. Switching to AUTO.LAND.")
        self._set_mode("AUTO.LAND")
        time.sleep(6.0)
        self._arm(False)
        self._teardown("test_complete")

        # Summary
        log.info("═" * 50)
        log.info("ALTITUDE VALIDATION SUMMARY")
        for r in self._results:
            log.info(
                f"  Step {r['step']} @ {r['target_z']}m: "
                f"max={r['max_dev_m']}m  avg={r['avg_dev_m']}m  "
                f"{'PASS' if r['passed'] else 'FAIL'}"
            )
        log.info(f"  OVERALL: {'PASS' if all_pass else 'FAIL'}")
        log.info("═" * 50)
        return all_pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="DronePi altitude validation test"
    )
    parser.add_argument(
        "--altitudes", nargs="+", type=float, default=DEFAULT_ALTITUDES,
        help="Altitude targets in metres",
    )
    parser.add_argument(
        "--hold", type=float, default=DEFAULT_HOLD_S,
        help="Hold duration at each target (seconds)",
    )
    parser.add_argument(
        "--climb-rate", type=float, default=DEFAULT_CLIMB_RATE_S,
        help="Climb rate in m/s",
    )
    args = parser.parse_args()

    rclpy.init()
    node = AltValidationNode(
        altitudes    = args.altitudes,
        hold_s       = args.hold,
        climb_rate_s = args.climb_rate,
    )

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True, name="ros_spin"
    )
    spin_thread.start()

    try:
        passed = node.run()
        sys.exit(0 if passed else 1)
    except KeyboardInterrupt:
        node._teardown("keyboard_interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
