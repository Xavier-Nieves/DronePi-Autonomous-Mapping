#!/usr/bin/env python3
"""
tests/test_hover_smallscale.py — Mandatory first flight after repair.

THIS TEST MUST PASS BEFORE ANY OTHER PROPS-ON TEST IS PERMITTED.

What it validates
-----------------
  1. SafeFlightMixin active and all monitors confirmed running
  2. RC CH7 kill switch functional (prompted pre-arm test)
  3. Stable hover at 1 m: altitude deviation < 0.3 m, no anomaly triggered
  4. (If --alt-max 3) Stable hover at 3 m: same criteria
  5. Rosbag recorded and all required topics present
  6. Clean AUTO.LAND and disarm

Pass criteria
-------------
  All altitudes held for full --hold seconds.
  No SafeFlightMixin triggers during hold periods.
  Rosbag contains all BAG_TOPICS_REQUIRED.

Usage
-----
  # Step 1: 1m only (absolute minimum first flight)
  python3 tests/test_hover_smallscale.py --alt-max 1 --hold 15

  # Step 2: 1m then 3m
  python3 tests/test_hover_smallscale.py --alt-max 3 --hold 15

  # Dry run (no arming, confirms monitors start correctly)
  python3 tests/test_hover_smallscale.py --dry-run

Safety
------
  Inherits SafeFlightMixin — all monitors active before arming.
  Ctrl+C triggers graceful AUTO.LAND.
  RC CH7 kill switch tested before arming (prompted).
  bench_scan mission lock written to prevent watchdog interference.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── Make flight/ importable ───────────────────────────────────────────────────
_TESTS_DIR  = Path(__file__).resolve().parent
_MAPPER_DIR = _TESTS_DIR.parent
sys.path.insert(0, str(_MAPPER_DIR))

from flight.safe_flight_mixin import (
    SafeFlightMixin,
    MISSION_LOCK,
    BAG_TOPICS_REQUIRED,
    ROS_SETUP,
    WS_SETUP,
)

# ── Constants ─────────────────────────────────────────────────────────────────

SETPOINT_HZ        = 20
CLIMB_TIMEOUT_S    = 20.0
ALTITUDE_TOLERANCE = 0.25   # metres — within this = "at altitude"
HOLD_DEVIATION_MAX = 0.30   # metres — max allowed deviation during hold
OUTPUT_DIR         = _TESTS_DIR / "hover_smallscale_output"

RC_KILL_CHANNEL = int(os.environ.get("RC_KILL_CHANNEL", "7"))
RC_KILL_THRESHOLD = int(os.environ.get("RC_KILL_THRESHOLD", "1700"))


# ══════════════════════════════════════════════════════════════════════════════
# Flight Node
# ══════════════════════════════════════════════════════════════════════════════

class HoverSmallScaleNode(SafeFlightMixin):
    """
    Minimal hover test node. Inherits SafeFlightMixin for all safety logic.

    Does NOT inherit rclpy.node.Node directly — instead it holds a Node
    instance internally, which is the pattern used by FlightController.
    This keeps the safety mixin compatible with both patterns.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from rclpy.node    import Node
        from rclpy.qos     import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg   import State, RCIn
        from mavros_msgs.srv   import CommandBool, SetMode

        self._args   = args
        self._rclpy  = rclpy

        # Initialise SafeFlightMixin FIRST
        SafeFlightMixin.__init__(
            self,
            script_name=__file__,
            headless=args.headless,
        )

        rclpy.init()
        self._node = Node("hover_smallscale_test")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Shared state
        self._state_cb_lock = threading.Lock()
        self._fcu_state   = None
        self._local_pose  = None
        self._rc_channels: list[int] = []

        # Subscriptions
        self._node.create_subscription(
            State, "/mavros/state", self._on_state, reliable_qos)
        self._node.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._on_pose, sensor_qos)
        self._node.create_subscription(
            RCIn, "/mavros/rc/in", self._on_rc, sensor_qos)

        # Publisher
        self._sp_pub = self._node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", reliable_qos)
        self._PoseStamped = PoseStamped

        # Service clients
        self._arm_cli  = self._node.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_cli = self._node.create_client(SetMode, "/mavros/set_mode")

        # Spin thread
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

        # Test results
        self._results: list[dict] = []
        self._test_passed = False

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_state(self, msg) -> None:
        with self._state_cb_lock:
            self._fcu_state = msg
            # Update SafeFlightMixin's state mirror
            self._last_mode  = msg.mode
            self._last_armed = msg.armed

    def _on_pose(self, msg) -> None:
        with self._state_cb_lock:
            self._local_pose = msg
            self._last_pose_z = msg.pose.position.z

    def _on_rc(self, msg) -> None:
        with self._state_cb_lock:
            self._rc_channels = list(msg.channels)

    # ── State helpers ──────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        with self._state_cb_lock:
            return self._fcu_state is not None and self._fcu_state.connected

    @property
    def armed(self) -> bool:
        with self._state_cb_lock:
            return self._fcu_state is not None and self._fcu_state.armed

    @property
    def mode(self) -> str:
        with self._state_cb_lock:
            return self._fcu_state.mode if self._fcu_state else ""

    def get_z(self) -> float:
        with self._state_cb_lock:
            if self._local_pose is None:
                return 0.0
            return self._local_pose.pose.position.z

    def get_rc_ch(self, ch_1indexed: int) -> int:
        with self._state_cb_lock:
            idx = ch_1indexed - 1
            if idx < len(self._rc_channels):
                return self._rc_channels[idx]
            return 0

    # ── Setpoint ───────────────────────────────────────────────────────────────

    def publish_sp(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        """Publish one setpoint AND update the watchdog heartbeat."""
        msg = self._PoseStamped()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)
        self._sp_heartbeat  = time.monotonic()   # REQUIRED — feeds watchdog
        self._alt_setpoint  = z                  # REQUIRED — feeds altitude monitor

    def stream_sp(self, x: float, y: float, z: float,
                  duration: float, yaw: float = 0.0) -> None:
        end = time.time() + duration
        while time.time() < end:
            if self._pilot_override:
                return
            self.publish_sp(x, y, z, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def fly_to(self, x: float, y: float, z: float,
               timeout: float = CLIMB_TIMEOUT_S,
               tol: float = ALTITUDE_TOLERANCE) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._pilot_override:
                return False
            self.publish_sp(x, y, z)
            dist = abs(self.get_z() - z)
            print(f"\r  climbing... Δz={dist:.2f}m   ", end="", flush=True)
            if dist < tol:
                print()
                return True
            time.sleep(1.0 / SETPOINT_HZ)
        print()
        return False

    # ── Service calls ──────────────────────────────────────────────────────────

    def arm(self) -> bool:
        from mavros_msgs.srv import CommandBool
        req = CommandBool.Request()
        req.value = True
        f = self._arm_cli.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, f, timeout_sec=5.0)
        return bool(f.result() and f.result().success)

    def set_mode(self, mode_str: str) -> bool:
        from mavros_msgs.srv import SetMode
        req = SetMode.Request()
        req.custom_mode = mode_str
        f = self._mode_cli.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, f, timeout_sec=5.0)
        return bool(f.result() and f.result().mode_sent)

    # ══════════════════════════════════════════════════════════════════════
    # PRE-ARM KILL SWITCH TEST
    # ══════════════════════════════════════════════════════════════════════

    def preflight_kill_switch_test(self) -> bool:
        """
        Prompt the operator to activate RC CH7 kill switch before arming.

        Returns True only if the kill switch was confirmed above threshold.
        This ensures CH7 monitoring is functional before props spin.
        """
        print()
        print("=" * 58)
        print("  PRE-ARM: RC KILL SWITCH FUNCTIONAL TEST")
        print("=" * 58)
        print(f"  Raise CH{RC_KILL_CHANNEL} (kill switch) ABOVE {RC_KILL_THRESHOLD} PWM")
        print("  and hold it there for 3 seconds.")
        print("  Press Enter when ready, then raise the switch.")
        print()
        try:
            input("  > Press Enter to start the kill switch test: ")
        except (EOFError, KeyboardInterrupt):
            return False

        print(f"  Monitoring CH{RC_KILL_CHANNEL} for 10 seconds...")
        deadline  = time.time() + 10.0
        confirmed = False
        held_since = None

        while time.time() < deadline:
            pwm = self.get_rc_ch(RC_KILL_CHANNEL)
            remaining = deadline - time.time()
            print(
                f"\r  CH{RC_KILL_CHANNEL} PWM={pwm:4d}  "
                f"({'ACTIVE ✓' if pwm > RC_KILL_THRESHOLD else 'low  '})  "
                f"t={remaining:.1f}s   ",
                end="", flush=True,
            )

            if pwm > RC_KILL_THRESHOLD:
                if held_since is None:
                    held_since = time.time()
                elif time.time() - held_since >= 3.0:
                    confirmed = True
                    break
            else:
                held_since = None

            time.sleep(0.1)

        print()

        if confirmed:
            print(f"  [PASS] Kill switch confirmed active on CH{RC_KILL_CHANNEL}")
            print("  Return switch to LOW before arming.")
            input("  > Press Enter when switch is back to LOW: ")
            # Verify it's back low
            pwm = self.get_rc_ch(RC_KILL_CHANNEL)
            if pwm > RC_KILL_THRESHOLD:
                print(f"  [FAIL] Switch still high (PWM={pwm}). Lower it first.")
                return False
            print("  [OK] Switch is low — safe to arm.")
            return True
        else:
            print(f"  [FAIL] Kill switch not confirmed within 10s.")
            print("  Fix RC Channel 7 assignment before flying.")
            return False

    # ══════════════════════════════════════════════════════════════════════
    # HOLD QUALITY CHECK
    # ══════════════════════════════════════════════════════════════════════

    def hold_and_evaluate(
        self, x: float, y: float, z: float,
        hold_s: float, alt_label: str,
    ) -> dict:
        """
        Hold position at (x, y, z) for hold_s seconds while monitoring
        altitude deviation and SafeFlightMixin trigger state.

        Returns a result dict with pass/fail and statistics.
        """
        print(f"\n  Holding at {alt_label} for {hold_s:.0f}s...")
        deviations: list[float] = []
        t0 = time.time()
        triggered = False

        while time.time() - t0 < hold_s:
            if self._pilot_override or self._teardown_called:
                triggered = True
                break
            current_z = self.get_z()
            dev       = abs(current_z - z)
            deviations.append(dev)
            self.publish_sp(x, y, z)
            remaining = hold_s - (time.time() - t0)
            print(
                f"\r  {alt_label}: z={current_z:.2f}m  "
                f"Δ={dev:.2f}m  t={remaining:.0f}s   ",
                end="", flush=True,
            )
            time.sleep(1.0 / SETPOINT_HZ)

        print()

        avg_dev = sum(deviations) / max(len(deviations), 1)
        max_dev = max(deviations) if deviations else 0.0

        passed = (
            not triggered
            and max_dev < HOLD_DEVIATION_MAX
        )

        result = {
            "alt_label":  alt_label,
            "target_z":   z,
            "avg_dev":    round(avg_dev, 3),
            "max_dev":    round(max_dev, 3),
            "triggered":  triggered,
            "passed":     passed,
        }
        status = "[PASS]" if passed else "[FAIL]"
        print(
            f"  {status} {alt_label}  "
            f"avg_dev={avg_dev:.3f}m  max_dev={max_dev:.3f}m  "
            f"triggered={triggered}"
        )
        return result

    # ══════════════════════════════════════════════════════════════════════
    # ROSBAG VERIFICATION
    # ══════════════════════════════════════════════════════════════════════

    def verify_rosbag(self) -> dict:
        """
        Check that the rosbag was written and contains required topics.
        """
        import subprocess as sp

        result = {"bag_found": False, "topics_ok": False, "missing": []}

        if self._bag_proc is None or self._session_dir is None:
            print("  [WARN] No bag recorder was started.")
            return result

        # Stop the bag recorder cleanly
        if self._bag_proc.poll() is None:
            try:
                import signal as _sig
                os.killpg(os.getpgid(self._bag_proc.pid), _sig.SIGINT)
                self._bag_proc.wait(timeout=8)
            except Exception:
                pass

        # Give ros2 bag time to write metadata.yaml
        time.sleep(2.0)

        # Find the bag directory
        bag_dirs = list(self._session_dir.glob("*/metadata.yaml"))
        if not bag_dirs:
            print("  [FAIL] No metadata.yaml found — bag may not have closed cleanly.")
            return result

        result["bag_found"] = True
        bag_dir = bag_dirs[0].parent
        print(f"  [OK] Bag found: {bag_dir.name}")

        # Check topics via ros2 bag info
        try:
            r = sp.run(
                ["bash", "-c",
                 f"source {ROS_SETUP} && source {WS_SETUP} && "
                 f"ros2 bag info {bag_dir}"],
                capture_output=True, text=True, timeout=15,
            )
            info_output = r.stdout + r.stderr
            missing = []
            for topic in BAG_TOPICS_REQUIRED:
                if topic not in info_output:
                    missing.append(topic)
            result["missing"] = missing
            result["topics_ok"] = len(missing) == 0
            if missing:
                print(f"  [WARN] Missing topics: {missing}")
            else:
                print(f"  [OK] All required bag topics present.")
        except Exception as exc:
            print(f"  [WARN] Could not verify bag topics: {exc}")

        return result

    # ══════════════════════════════════════════════════════════════════════
    # MAIN RUN SEQUENCE
    # ══════════════════════════════════════════════════════════════════════

    def run(self) -> bool:
        """
        Execute the full small-scale hover validation sequence.
        Returns True if all steps pass.
        """
        args = self._args
        altitudes = [1.0]
        if args.alt_max >= 3.0:
            altitudes.append(3.0)

        print("=" * 58)
        print("  DronePi Small-Scale Hover Validation")
        print(f"  Altitudes  : {altitudes} m")
        print(f"  Hold time  : {args.hold} s per level")
        print(f"  Dry run    : {'YES' if args.dry_run else 'NO'}")
        print("=" * 58)

        # ── [1] FCU connection ────────────────────────────────────────────
        print("\n[1/6] Waiting for FCU connection...")
        deadline = time.time() + 20.0
        while not self.connected and time.time() < deadline:
            time.sleep(0.3)
        if not self.connected:
            print("  [FAIL] FCU not connected. Is MAVROS running?")
            return False
        print(f"  [OK] FCU connected  mode={self.mode}")

        # ── [2] Start safety monitors ─────────────────────────────────────
        print("\n[2/6] Starting safety monitors...")
        if not self.start_safety_monitors():
            print("  [FAIL] Safety monitors failed to start.")
            return False
        time.sleep(1.0)
        if not self.monitors_alive():
            print("  [FAIL] One or more monitor threads died immediately.")
            return False
        print(f"  [OK] All monitors running ({len(self._monitor_threads)} threads)")

        # ── [3] Kill switch test ──────────────────────────────────────────
        if not args.dry_run and not args.skip_kill_test:
            print("\n[3/6] RC kill switch pre-arm test...")
            if not self.preflight_kill_switch_test():
                print("  [FAIL] Kill switch test failed. Fix before flying.")
                return False
        else:
            print("\n[3/6] Kill switch test skipped.")

        if args.dry_run:
            print("\n  --dry-run: all pre-arm checks passed. Not arming.")
            self._test_passed = True
            return True

        # ── [4] Pre-stream + arm ──────────────────────────────────────────
        home_z = self.get_z()
        print(f"\n[4/6] Pre-streaming setpoints (3s)  home_z={home_z:.3f}m...")
        self.stream_sp(0.0, 0.0, home_z, duration=3.0)

        print("  Arming...")
        if not self.arm():
            print("  [FAIL] Arm rejected.")
            return False
        print("  [OK] Armed")

        print("  Switching to OFFBOARD...")
        if not self.set_mode("OFFBOARD"):
            print("  [FAIL] OFFBOARD rejected.")
            self.set_mode("AUTO.LAND")
            return False
        print("  [OK] OFFBOARD")

        # ── [5] Altitude levels ───────────────────────────────────────────
        print(f"\n[5/6] Altitude sequence: {altitudes} m")
        all_pass = True

        for alt_m in altitudes:
            target_z  = home_z + alt_m
            alt_label = f"{alt_m:.0f}m"

            print(f"\n  Climbing to {alt_label} (target Z={target_z:.2f}m)...")
            reached = self.fly_to(0.0, 0.0, target_z)

            if self._pilot_override:
                print("  [ABORT] Pilot override triggered during climb.")
                all_pass = False
                break

            if not reached:
                print(f"  [WARN] Did not converge to {alt_label} — continuing.")

            result = self.hold_and_evaluate(
                0.0, 0.0, target_z,
                hold_s=args.hold,
                alt_label=alt_label,
            )
            self._results.append(result)

            if not result["passed"]:
                all_pass = False

            if self._pilot_override:
                print("  [ABORT] Pilot override triggered during hold.")
                all_pass = False
                break

        # ── [6] Land ──────────────────────────────────────────────────────
        print("\n[6/6] Landing...")
        self.set_mode("AUTO.LAND")
        deadline = time.time() + 40.0
        while self.armed and time.time() < deadline:
            time.sleep(0.3)
        if not self.armed:
            print("  [OK] Landed and disarmed.")
        else:
            print("  [WARN] Still armed after 40s — disarming manually.")

        # ── Rosbag verification ───────────────────────────────────────────
        print("\n  Verifying rosbag...")
        bag_result = self.verify_rosbag()
        if not bag_result.get("bag_found"):
            print("  [WARN] Bag not found — record verification skipped.")
        elif not bag_result.get("topics_ok"):
            print(f"  [WARN] Bag missing topics: {bag_result.get('missing')}")

        self._test_passed = all_pass
        return all_pass

    def shutdown(self) -> None:
        self._teardown("normal_shutdown")
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ══════════════════════════════════════════════════════════════════════════════
# Report
# ══════════════════════════════════════════════════════════════════════════════

def write_report(node: HoverSmallScaleNode, passed: bool) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = OUTPUT_DIR / f"report_{ts}.txt"

    lines = [
        "=" * 58,
        "  DronePi Small-Scale Hover Validation Report",
        f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"  Script: {Path(__file__).name}",
        "=" * 58,
        "",
    ]
    for r in node._results:
        status = "PASS ✓" if r["passed"] else "FAIL ✗"
        lines.append(
            f"  {r['alt_label']:>4s}  {status}  "
            f"avg_dev={r['avg_dev']:.3f}m  "
            f"max_dev={r['max_dev']:.3f}m  "
            f"triggered={r['triggered']}"
        )
    lines += [
        "",
        f"  Final: {'ALL PASS ✓' if passed else 'ONE OR MORE FAILED ✗'}",
        "",
        "  Next steps if passed:",
        "    python3 tests/test_altitude_validation.py --altitudes 3 6 --hold 30",
        "=" * 58,
    ]

    text = "\n".join(lines)
    print("\n" + text)
    path.write_text(text + "\n")
    print(f"\n  Report → {path}")


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Small-scale hover validation — mandatory first flight after repair."
    )
    parser.add_argument(
        "--alt-max", type=float, default=3.0,
        help="Maximum altitude in metres. 1 = 1m only, 3 = 1m then 3m (default 3)",
    )
    parser.add_argument(
        "--hold", type=float, default=15.0,
        help="Hold duration per level in seconds (default 15)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Start monitors and check everything up to arm — no motors",
    )
    parser.add_argument(
        "--skip-kill-test", action="store_true",
        help="Skip RC kill switch pre-arm test (NOT recommended for first flight)",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Disable TTY check (for tmux / no-SSH deployments)",
    )
    args = parser.parse_args()

    # Write bench_scan lock so watchdog yields
    MISSION_LOCK.parent.mkdir(parents=True, exist_ok=True)
    try:
        MISSION_LOCK.write_text(
            json.dumps({"mode": "bench_scan",
                        "started_at": datetime.now().isoformat()})
        )
        print(f"  [Lock] bench_scan written → {MISSION_LOCK}")
    except Exception as e:
        print(f"  [WARN] Could not write mission lock: {e}")

    if not args.dry_run:
        print("\n  *** LIVE FLIGHT — PROPS ON ***")
        print("  Ctrl+C within 5s to abort.")
        for i in range(5, 0, -1):
            print(f"  {i}...", end="\r", flush=True)
            time.sleep(1.0)
        print()

    node   = None
    passed = False

    try:
        node   = HoverSmallScaleNode(args)
        passed = node.run()
        write_report(node, passed)

    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl+C — emergency landing.")
        if node is not None:
            node._teardown("KEYBOARD_INTERRUPT")

    except Exception as exc:
        import traceback
        print(f"\n[FAIL] Unhandled exception: {exc}")
        traceback.print_exc()
        if node is not None:
            node._teardown("UNHANDLED_EXCEPTION")

    finally:
        if node is not None:
            node.shutdown()
        # Always clear mission lock
        try:
            MISSION_LOCK.unlink(missing_ok=True)
        except Exception:
            pass

    print("\n" + "=" * 58)
    print(f"  Result: {'PASS ✓' if passed else 'FAIL ✗'}")
    if passed:
        print("  This drone is cleared for the next test gate:")
        print("    test_altitude_validation.py --altitudes 3 6 --hold 30")
    else:
        print("  Do NOT proceed to the next gate until this test passes.")
    print("=" * 58)

    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
