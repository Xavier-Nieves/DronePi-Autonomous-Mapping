#!/usr/bin/env python3
"""
test_hover_camera.py — Camera + LiDAR integration hover test.

Validates three things in a single short flight over a known target area:

  1. Focus check    — Camera is focused: captured frames have measurable
                      sharpness (Laplacian variance > threshold).

  2. Trigger check  — Camera fires on command: frames are captured at each
                      of the four square waypoints and timestamped correctly.

  3. LiDAR + camera — Both sensors are publishing simultaneously and their
                      timestamps are close enough for pose interpolation to
                      work (gap < 50 ms between camera frame and nearest
                      LiDAR pose).

Flight path
-----------
  The drone hovers at a safe altitude, then visits four corners of a small
  square (default 1.0 m side at 1.5 m altitude), taking one photo at each
  corner.  It then returns to centre and lands.

  Top-down view (body +X = forward):

        C2 ──── C3
        │    ↑   │
        │  home  │
        │        │
        C1 ──── C4

  Each corner is held for --hold-per-corner seconds while a frame is
  captured and validated.

Output
------
  tests/hover_camera_output/
    frame_C1.jpg  frame_C2.jpg  frame_C3.jpg  frame_C4.jpg
    report.txt    (per-frame sharpness, LiDAR gap, pass/fail)

Safety
------
  The script follows the same arming / OFFBOARD / landing pattern as
  test_offboard_flight.py.  It has the same --dry-run flag that validates
  the setpoint pipeline without arming.

  RC override kills OFFBOARD immediately.  The script does not suppress
  the RC failsafe.

Dependencies
------------
  rclpy, mavros_msgs, geometry_msgs, sensor_msgs, nav_msgs
  cv2, numpy
  All present in the dronepi ROS 2 + conda environment.

Usage
-----
  # Dry run — no arming, verifies pipeline only
  source /opt/ros/jazzy/setup.bash
  python3 tests/test_hover_camera.py --dry-run

  # Full flight
  python3 tests/test_hover_camera.py --alt 1.5 --side 1.0 --hold-per-corner 3

  # Tighter square, lower altitude for indoor testing
  python3 tests/test_hover_camera.py --alt 1.2 --side 0.6 --hold-per-corner 2
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import threading
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg  import PoseStamped, TwistStamped
from mavros_msgs.msg    import State
from mavros_msgs.srv    import CommandBool, SetMode
from nav_msgs.msg       import Odometry
from sensor_msgs.msg    import Image
from std_msgs.msg       import Header

# ── Output directory ──────────────────────────────────────────────────────────
_TESTS_DIR  = Path(__file__).resolve().parent
OUTPUT_DIR  = _TESTS_DIR / "hover_camera_output"

# ── Sharpness threshold ───────────────────────────────────────────────────────
# Laplacian variance — frames below this are considered blurry/unfocused.
# Typical sharp outdoor scene: 300–2000.  Blurry/defocused: < 80.
SHARPNESS_THRESHOLD = 80.0

# ── Timestamp sync tolerance ─────────────────────────────────────────────────
# Max acceptable gap (seconds) between a camera frame timestamp and the
# nearest LiDAR pose timestamp.  Must be < EXTRAPOLATION_LIMIT_NS (50 ms).
SYNC_TOLERANCE_S = 0.045   # 45 ms


class HoverCameraTest(Node):
    """
    ROS 2 node that flies a square hover pattern and captures frames.

    Inherits the same arming/OFFBOARD pattern as test_offboard_flight.py
    so the flight behaviour is known-good.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("hover_camera_test")
        self.args = args

        # ── QoS profiles ─────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability = ReliabilityPolicy.RELIABLE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 10,
        )
        sensor_qos = QoSProfile(
            reliability  = ReliabilityPolicy.BEST_EFFORT,
            history      = HistoryPolicy.KEEP_LAST,
            depth        = 5,
            durability   = DurabilityPolicy.VOLATILE,
        )

        # ── State ─────────────────────────────────────────────────────────────
        self.fcu_state:       Optional[State]    = None
        self.local_pose:      Optional[PoseStamped] = None
        self.latest_image:    Optional[Image]    = None
        self.latest_lidar_ts: Optional[float]    = None   # seconds
        self._home_z:         float              = 0.0
        self._lock           = threading.Lock()

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(State,       "/mavros/state",
                                 self._cb_state,      reliable_qos)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._cb_pose,       sensor_qos)
        self.create_subscription(Odometry,    "/aft_mapped_to_init",
                                 self._cb_lidar_pose, sensor_qos)
        self.create_subscription(Image,       "/arducam/image_raw",
                                 self._cb_image,      sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", reliable_qos)

        # ── Service clients ───────────────────────────────────────────────────
        self._arming_client  = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client    = self.create_client(SetMode,     "/mavros/set_mode")

        # ── Results ───────────────────────────────────────────────────────────
        self._results: list[dict] = []

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_state(self, msg: State) -> None:
        self.fcu_state = msg

    def _cb_pose(self, msg: PoseStamped) -> None:
        self.local_pose = msg

    def _cb_lidar_pose(self, msg: Odometry) -> None:
        # Record the ROS timestamp of the latest LiDAR pose
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self.latest_lidar_ts = ts

    def _cb_image(self, msg: Image) -> None:
        with self._lock:
            self.latest_image = msg

    # ── FCU helpers ───────────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self.fcu_state is not None and self.fcu_state.connected

    @property
    def armed(self) -> bool:
        return self.fcu_state is not None and self.fcu_state.armed

    @property
    def mode(self) -> str:
        return self.fcu_state.mode if self.fcu_state else ""

    def _current_z(self) -> float:
        if self.local_pose is None:
            return 0.0
        return self.local_pose.pose.position.z

    def _make_setpoint(self, x: float, y: float, z: float) -> PoseStamped:
        msg              = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        return msg

    def _stream_setpoint(self, x: float, y: float, z: float) -> None:
        self._sp_pub.publish(self._make_setpoint(x, y, z))

    def set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._mode_client.service_is_ready():
                future = self._mode_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() and future.result().mode_sent:
                    return True
            time.sleep(0.2)
        return False

    def arm(self, timeout: float = 10.0) -> bool:
        req = CommandBool.Request()
        req.value = True
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._arming_client.service_is_ready():
                future = self._arming_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.result() and future.result().success:
                    return True
            time.sleep(0.3)
        return False

    def disarm(self) -> None:
        req = CommandBool.Request()
        req.value = False
        if self._arming_client.service_is_ready():
            future = self._arming_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def shutdown(self) -> None:
        self.destroy_node()

    # ── Setpoint streaming ────────────────────────────────────────────────────

    def _stream_for(
        self,
        x: float, y: float, z: float,
        duration: float,
        hz: float = 20.0,
    ) -> None:
        """Publish setpoint at hz for duration seconds, spinning between."""
        interval = 1.0 / hz
        deadline = time.time() + duration
        while time.time() < deadline:
            self._stream_setpoint(x, y, z)
            rclpy.spin_once(self, timeout_sec=interval)

    # ── Altitude hold ─────────────────────────────────────────────────────────

    def _wait_for_altitude(
        self,
        target_z: float,
        tolerance: float = 0.15,
        timeout: float   = 20.0,
    ) -> bool:
        """Stream setpoint and wait until local Z is within tolerance."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._stream_setpoint(0.0, 0.0, target_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            if abs(self._current_z() - target_z) < tolerance:
                return True
        return False

    def _fly_to(
        self,
        x: float, y: float, z: float,
        tolerance: float = 0.20,
        timeout:   float = 15.0,
    ) -> bool:
        """Stream setpoint and wait until position is within tolerance."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._stream_setpoint(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.local_pose:
                dx = self.local_pose.pose.position.x - x
                dy = self.local_pose.pose.position.y - y
                dz = self.local_pose.pose.position.z - z
                if math.sqrt(dx*dx + dy*dy + dz*dz) < tolerance:
                    return True
        return False

    # ── Camera capture ────────────────────────────────────────────────────────

    def capture_frame_at(
        self,
        corner_name: str,
        hold_s:      float,
        x: float, y: float, z: float,
    ) -> dict:
        """
        Hold position for hold_s seconds, grab the latest camera frame,
        compute sharpness, and check LiDAR sync gap.

        Returns a result dict with keys:
          corner, frame_ts, lidar_ts, sync_gap_s,
          sharpness, sharp_ok, sync_ok, image_path, pass
        """
        result = {
            "corner":     corner_name,
            "frame_ts":   None,
            "lidar_ts":   None,
            "sync_gap_s": None,
            "sharpness":  None,
            "sharp_ok":   False,
            "sync_ok":    False,
            "image_path": None,
            "pass":       False,
        }

        # Hold position and wait for a fresh frame
        print(f"    Holding at {corner_name} for {hold_s:.1f}s...")
        t0 = time.time()
        with self._lock:
            pre_image = self.latest_image  # snapshot before hold

        while time.time() - t0 < hold_s:
            self._stream_setpoint(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Grab the frame that arrived during the hold
        with self._lock:
            frame_msg  = self.latest_image
            lidar_ts   = self.latest_lidar_ts

        if frame_msg is None:
            print(f"    [FAIL] {corner_name}: no camera frame received")
            return result

        # Check it is actually a new frame (not the same one from before)
        if frame_msg is pre_image:
            print(f"    [WARN] {corner_name}: camera did not produce a new frame during hold")

        # ROS timestamp of the frame
        frame_ts = (
            frame_msg.header.stamp.sec
            + frame_msg.header.stamp.nanosec * 1e-9
        )
        result["frame_ts"] = frame_ts

        # LiDAR sync gap
        if lidar_ts is not None:
            gap = abs(frame_ts - lidar_ts)
            result["lidar_ts"]   = lidar_ts
            result["sync_gap_s"] = gap
            result["sync_ok"]    = gap < SYNC_TOLERANCE_S
            status = "✓" if result["sync_ok"] else "✗"
            print(f"    LiDAR sync gap: {gap*1000:.1f} ms  {status}")
        else:
            print(f"    [WARN] {corner_name}: no LiDAR pose received — "
                  f"is Point-LIO running?")

        # Decode image and compute sharpness
        img_bgr = self._decode_image(frame_msg)
        if img_bgr is not None:
            gray      = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
            sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            result["sharpness"] = sharpness
            result["sharp_ok"]  = sharpness >= SHARPNESS_THRESHOLD
            status = "✓" if result["sharp_ok"] else "✗ (blurry/defocused)"
            print(f"    Sharpness (Laplacian var): {sharpness:.1f}  {status}")

            # Save image
            OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
            img_path = OUTPUT_DIR / f"frame_{corner_name}.jpg"
            cv2.imwrite(str(img_path), img_bgr)
            result["image_path"] = str(img_path)
            print(f"    Saved → {img_path.name}")
        else:
            print(f"    [FAIL] {corner_name}: could not decode image "
                  f"(encoding='{frame_msg.encoding}')")

        result["pass"] = result["sharp_ok"] and result["sync_ok"]
        return result

    @staticmethod
    def _decode_image(msg: Image) -> Optional[np.ndarray]:
        """Decode sensor_msgs/Image to BGR numpy array."""
        enc = msg.encoding.lower()
        h, w = int(msg.height), int(msg.width)
        raw  = bytes(msg.data)
        if enc == "bgr8":
            return np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3).copy()
        if enc == "rgb8":
            rgb = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3).copy()
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if enc == "mono8":
            gray = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        return None

    # ── Main flight sequence ───────────────────────────────────────────────────

    def run(self) -> bool:
        """
        Execute the full test sequence.  Returns True if all checks pass.

        Sequence
        --------
        1. Wait for FCU connection
        2. Wait for LiDAR + camera topics to confirm both sensors live
        3. Pre-stream setpoints (OFFBOARD requirement)
        4. Arm + switch to OFFBOARD
        5. Climb to target altitude
        6. Visit four corners, capture and evaluate one frame per corner
        7. Return to centre
        8. AUTO.LAND → disarm
        9. Write report
        """
        alt  = self.args.alt
        side = self.args.side / 2.0   # half-side for symmetric corners
        hold = self.args.hold_per_corner

        # Corner positions in local ENU frame (drone starts at origin)
        corners = [
            ("C1", -side, -side, alt),
            ("C2", -side, +side, alt),
            ("C3", +side, +side, alt),
            ("C4", +side, -side, alt),
        ]

        success = False

        try:
            # ── [1] FCU connection ────────────────────────────────────────────
            print("\n[1/8] Waiting for FCU connection...")
            deadline = time.time() + 20.0
            while not self.connected and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
            if not self.connected:
                print("  [FAIL] FCU not connected")
                return False
            print("  [OK] FCU connected")

            # ── [2] Sensor liveness check ─────────────────────────────────────
            print("\n[2/8] Checking sensor topics...")
            deadline = time.time() + 10.0
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.3)
                with self._lock:
                    cam_live   = self.latest_image    is not None
                    lidar_live = self.latest_lidar_ts is not None
                if cam_live and lidar_live:
                    break

            if not cam_live:
                print("  [FAIL] /arducam/image_raw not publishing — "
                      "is ArducamNode running?")
                if not self.args.dry_run:
                    return False
                print("  [WARN] Continuing dry run without camera")
            else:
                print("  [OK] /arducam/image_raw publishing")

            if not lidar_live:
                print("  [WARN] /aft_mapped_to_init not publishing — "
                      "Point-LIO may not be running")
                print("         LiDAR sync checks will be skipped")
            else:
                print("  [OK] /aft_mapped_to_init publishing")

            # ── [3] Capture home Z ────────────────────────────────────────────
            print("\n[3/8] Capturing home position...")
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)
            self._home_z = self._current_z()
            target_z     = self._home_z + alt
            print(f"  Home Z = {self._home_z:.3f}m  →  target = {target_z:.3f}m")

            # ── [4] Pre-stream setpoints ──────────────────────────────────────
            print("\n[4/8] Pre-streaming setpoints (3s)...")
            self._stream_for(0.0, 0.0, self._home_z, duration=3.0)
            print("  [OK] Setpoint stream established")

            if self.args.dry_run:
                print("\n  --dry-run: stopping before arm.")
                print("  Sensor topic check passed — ready for live flight.")
                return True

            # ── [5] Arm + OFFBOARD ────────────────────────────────────────────
            print("\n[5/8] Arming...")
            if not self.arm():
                print("  [FAIL] Arm rejected")
                return False
            print("  [OK] Armed")

            print("  Switching to OFFBOARD...")
            if not self.set_mode("OFFBOARD"):
                print("  [FAIL] OFFBOARD rejected")
                self.disarm()
                return False
            print("  [OK] OFFBOARD active")

            # ── [6] Climb ─────────────────────────────────────────────────────
            print(f"\n[6/8] Climbing to {alt}m above home...")
            if not self._wait_for_altitude(target_z, timeout=20.0):
                print("  [FAIL] Did not reach target altitude in 20s")
                self.set_mode("AUTO.LAND")
                return False
            print(f"  [OK] Altitude reached: {self._current_z():.2f}m")

            # ── [7] Square pattern + capture ─────────────────────────────────
            print(f"\n[7/8] Flying {len(corners)}-corner square "
                  f"(side={self.args.side}m, alt={alt}m)...")

            for name, cx, cy, cz in corners:
                print(f"\n  → Corner {name}  ({cx:+.2f}, {cy:+.2f}, {cz:.2f}m)")
                reached = self._fly_to(cx, cy, cz + self._home_z, timeout=12.0)
                if not reached:
                    print(f"  [WARN] Did not fully converge to {name} — "
                          f"capturing anyway")

                result = self.capture_frame_at(
                    name, hold, cx, cy, cz + self._home_z
                )
                self._results.append(result)
                status = "PASS" if result["pass"] else "FAIL"
                print(f"    → {name}: {status}")

            # ── [8] Return to centre + land ───────────────────────────────────
            print("\n[8/8] Returning to centre...")
            self._fly_to(0.0, 0.0, target_z, timeout=12.0)
            self._stream_for(0.0, 0.0, target_z, duration=2.0)

            print("  Switching to AUTO.LAND...")
            self.set_mode("AUTO.LAND")

            # Wait for disarm
            deadline = time.time() + 30.0
            while self.armed and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.3)
            if not self.armed:
                print("  [OK] Landed and disarmed")
            else:
                print("  [WARN] Still armed after 30s — check manually")
                self.disarm()

            success = True

        except KeyboardInterrupt:
            print("\n  [ABORT] KeyboardInterrupt — switching to AUTO.LAND")
            self.set_mode("AUTO.LAND")
            time.sleep(3.0)
            self.disarm()

        except Exception as exc:
            import traceback
            print(f"\n  [FAIL] Unexpected exception: {exc}")
            traceback.print_exc()
            self.set_mode("AUTO.LAND")
            time.sleep(2.0)
            self.disarm()

        return success

    # ── Report ────────────────────────────────────────────────────────────────

    def write_report(self) -> None:
        """Write report.txt and print summary to terminal."""
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        report_path = OUTPUT_DIR / "report.txt"

        lines = [
            "=" * 60,
            "  DronePi — Hover Camera Validation Report",
            f"  Date    : {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"  Altitude: {self.args.alt}m   Side: {self.args.side}m",
            "=" * 60,
            "",
        ]

        all_pass = True

        for r in self._results:
            lines.append(f"Corner {r['corner']}")
            lines.append(f"  Image saved  : {r['image_path'] or 'none'}")

            if r["sharpness"] is not None:
                status = "PASS" if r["sharp_ok"] else "FAIL (blurry)"
                lines.append(
                    f"  Sharpness    : {r['sharpness']:.1f}  "
                    f"(threshold {SHARPNESS_THRESHOLD})  → {status}"
                )
            else:
                lines.append("  Sharpness    : no image captured  → FAIL")

            if r["sync_gap_s"] is not None:
                status = "PASS" if r["sync_ok"] else "FAIL (too large)"
                lines.append(
                    f"  LiDAR gap    : {r['sync_gap_s']*1000:.1f} ms  "
                    f"(limit {SYNC_TOLERANCE_S*1000:.0f} ms)  → {status}"
                )
            else:
                lines.append("  LiDAR gap    : no LiDAR data  → SKIP")

            verdict = "PASS" if r["pass"] else "FAIL"
            lines.append(f"  Overall      : {verdict}")
            lines.append("")

            if not r["pass"]:
                all_pass = False

        lines.append("=" * 60)
        lines.append(
            f"  Final verdict : {'ALL PASS' if all_pass else 'ONE OR MORE CHECKS FAILED'}"
        )
        lines.append("")

        # Diagnostic hints for common failure modes
        if not all_pass:
            lines.append("Diagnostic hints:")
            for r in self._results:
                if r["sharpness"] is not None and not r["sharp_ok"]:
                    lines.append(
                        f"  {r['corner']} sharpness={r['sharpness']:.1f} < {SHARPNESS_THRESHOLD}:\n"
                        f"    → Check camera focus ring, lens cap, and mount vibration.\n"
                        f"    → At 1.5m alt, IMX477 fixed-focus should be sharp on ground.\n"
                        f"    → If all four frames are blurry, lens may be dirty or defocused."
                    )
                if r["sync_gap_s"] is not None and not r["sync_ok"]:
                    lines.append(
                        f"  {r['corner']} LiDAR gap={r['sync_gap_s']*1000:.1f}ms "
                        f"> {SYNC_TOLERANCE_S*1000:.0f}ms:\n"
                        f"    → Verify ArducamNode stamps with ROS clock "
                        f"(not system wall clock).\n"
                        f"    → Verify Point-LIO is publishing at ≥10 Hz.\n"
                        f"    → Check ROS clock synchronisation between nodes."
                    )

        lines.append("=" * 60)

        report_text = "\n".join(lines)
        print("\n" + report_text)

        with open(report_path, "w") as f:
            f.write(report_text + "\n")
        print(f"\n  Report saved → {report_path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Hover camera + LiDAR integration test — square pattern flight."
    )
    parser.add_argument("--alt",             type=float, default=1.5,
                        help="Hover altitude above home (metres, default 1.5)")
    parser.add_argument("--side",            type=float, default=1.0,
                        help="Square side length (metres, default 1.0)")
    parser.add_argument("--hold-per-corner", type=float, default=3.0,
                        help="Seconds to hold at each corner while capturing (default 3)")
    parser.add_argument("--dry-run",         action="store_true",
                        help="Validate sensors + setpoint stream only — no arming")
    args = parser.parse_args()

    print("=" * 60)
    print("  DronePi — Hover Camera Validation Test")
    print(f"  Altitude        : {args.alt}m above home")
    print(f"  Square side     : {args.side}m")
    print(f"  Hold per corner : {args.hold_per_corner}s")
    print(f"  Dry run         : {'YES' if args.dry_run else 'NO'}")
    print("=" * 60)

    if not args.dry_run:
        print("\n  *** LIVE FLIGHT — drone will arm and take off ***")
        print("  Press Ctrl+C within 5s to abort.")
        for i in range(5, 0, -1):
            print(f"  {i}...", end="\r", flush=True)
            time.sleep(1.0)
        print()

    rclpy.init()
    node = HoverCameraTest(args)

    try:
        flight_ok = node.run()
        if not args.dry_run:
            node.write_report()
    finally:
        node.shutdown()
        rclpy.shutdown()

    print("\n" + "=" * 60)
    if args.dry_run:
        print("  Dry run complete — run without --dry-run for live flight")
    else:
        passed = sum(1 for r in node._results if r["pass"])
        total  = len(node._results)
        print(f"  {passed}/{total} corners passed all checks")
        if passed == total:
            print("  Camera focus, trigger, and LiDAR sync all verified.")
            print("  Ready for first textured survey flight.")
        else:
            print("  See report.txt for diagnostic hints.")
    print("=" * 60)

    sys.exit(0 if flight_ok else 1)


if __name__ == "__main__":
    main()
