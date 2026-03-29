#!/usr/bin/env python3
"""test_ekf2_latency.py — Measure Pi→PX4 vision latency and recommend EKF2_EV_DELAY.

Subscribes to /mavros/vision_pose/pose (published by _slam_bridge.py) and
measures the delay between the message's header.stamp (Point-LIO estimation
time) and the local ROS clock at receipt.  This captures total latency from
SLAM output to MAVROS queue, which is the dominant component of EKF2_EV_DELAY.

A fixed serial offset is then added to account for MAVLink transmission at
57600 baud (Pixhawk 6X default).

Measurement chain
-----------------
  LiDAR scan → Point-LIO SLAM → header.stamp  ← measured from here
      → ROS DDS publish → MAVROS subscriber receives  ← to here
      → MAVROS serialises → serial 57600 baud → PX4 EKF2  ← +SERIAL_OFFSET_MS

Requirements
------------
  - Point-LIO running and publishing /aft_mapped_to_init
  - _slam_bridge.py running and publishing /mavros/vision_pose/pose
  - System time sync between Pi and Pixhawk is NOT required (we only use Pi-side clocks)

Usage
-----
  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash

  python3 test_ekf2_latency.py                     # default: 200 samples
  python3 test_ekf2_latency.py --samples 500        # more samples, higher confidence
  python3 test_ekf2_latency.py --timeout 60         # wait up to 60s for first message

Output
------
  Prints a table of latency statistics and a clear instruction:
    ╔══════════════════════════════════════╗
    ║  Set EKF2_EV_DELAY = XX ms in QGC   ║
    ╚══════════════════════════════════════╝

Safety
------
  Read-only — subscribes only, publishes nothing.  Safe to run on a live system.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from geometry_msgs.msg import PoseStamped

# ── Constants ─────────────────────────────────────────────────────────────────

# Extra latency to add on top of the measured ROS-side delay.
# Accounts for:
#   - MAVROS serialisation and MAVLink framing   ~3 ms
#   - Serial TX at 57600 baud (PoseStamped ≈ 72 B MAVLink frame ≈ 10 ms)
#   - PX4 EKF scheduling jitter                 ~2 ms
SERIAL_OFFSET_MS = 15

# Safety margin on top of p95 so EKF2 never misses a measurement
SAFETY_MARGIN_MS = 5

VISION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class LatencyProbe(Node):
    """Collects timing samples from /mavros/vision_pose/pose."""

    def __init__(self, n_samples: int):
        super().__init__("ekf2_latency_probe")
        self.n_samples  = n_samples
        self.delays_ms  = []

        self._sub = self.create_subscription(
            PoseStamped,
            "/mavros/vision_pose/pose",
            self._callback,
            VISION_QOS,
        )
        self.get_logger().info(
            f"Listening on /mavros/vision_pose/pose  "
            f"(collecting {n_samples} samples) ..."
        )

    def _callback(self, msg: PoseStamped) -> None:
        now_ns   = self.get_clock().now().nanoseconds
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )
        delay_ms = (now_ns - stamp_ns) / 1e6

        # Ignore negative or implausibly large values (clock glitch / stale msg)
        if 0.0 < delay_ms < 2_000.0:
            self.delays_ms.append(delay_ms)
            n = len(self.delays_ms)
            if n % 50 == 0 or n == self.n_samples:
                self.get_logger().info(
                    f"  Collected {n}/{self.n_samples} samples  "
                    f"(latest={delay_ms:.1f} ms)"
                )

    @property
    def done(self) -> bool:
        return len(self.delays_ms) >= self.n_samples


def _print_results(delays_ms: list) -> int:
    """Print statistics and return the recommended EKF2_EV_DELAY in ms."""
    arr = np.array(delays_ms)

    mean_ms   = float(np.mean(arr))
    median_ms = float(np.median(arr))
    std_ms    = float(np.std(arr))
    p75_ms    = float(np.percentile(arr, 75))
    p90_ms    = float(np.percentile(arr, 90))
    p95_ms    = float(np.percentile(arr, 95))
    p99_ms    = float(np.percentile(arr, 99))
    max_ms    = float(np.max(arr))

    raw_rec   = p95_ms + SERIAL_OFFSET_MS + SAFETY_MARGIN_MS
    # Round up to nearest 5 ms for a clean QGC value
    recommended = int(math.ceil(raw_rec / 5.0) * 5)

    print()
    print("=" * 52)
    print("  EKF2_EV_DELAY Latency Report")
    print(f"  Samples : {len(arr)}")
    print("=" * 52)
    print(f"  Mean      : {mean_ms:6.1f} ms")
    print(f"  Median    : {median_ms:6.1f} ms")
    print(f"  Std dev   : {std_ms:6.1f} ms")
    print(f"  p75       : {p75_ms:6.1f} ms")
    print(f"  p90       : {p90_ms:6.1f} ms")
    print(f"  p95       : {p95_ms:6.1f} ms   ← basis for recommendation")
    print(f"  p99       : {p99_ms:6.1f} ms")
    print(f"  Max       : {max_ms:6.1f} ms")
    print("-" * 52)
    print(f"  p95 ({p95_ms:.1f}) + serial offset ({SERIAL_OFFSET_MS} ms)"
          f" + safety ({SAFETY_MARGIN_MS} ms) = {raw_rec:.1f} ms")
    print(f"  Rounded up to nearest 5 ms = {recommended} ms")
    print("=" * 52)
    print()
    print("  ╔══════════════════════════════════════════════╗")
    print(f"  ║  Set  EKF2_EV_DELAY = {recommended} ms  in QGC         ║")
    print("  ╚══════════════════════════════════════════════╝")
    print()
    print("  Steps:")
    print("    1. Open QGroundControl → Vehicle Setup → Parameters")
    print(f"    2. Search for EKF2_EV_DELAY")
    print(f"    3. Set value to {recommended}")
    print("    4. Reboot the Pixhawk (power cycle or MAVLink REBOOT_AUTOPILOT)")
    print()

    if p95_ms > 100:
        print("  [WARN] p95 > 100 ms — high latency detected.")
        print("         Check Point-LIO CPU load and ROS DDS settings.")
    elif p95_ms < 5:
        print("  [WARN] p95 < 5 ms — suspiciously low. Verify slam_bridge is running.")

    return recommended


def main():
    parser = argparse.ArgumentParser(
        description="Measure SLAM→MAVROS latency and recommend EKF2_EV_DELAY.")
    parser.add_argument("--samples",  type=int, default=200,
                        help="Number of pose samples to collect (default: 200)")
    parser.add_argument("--timeout",  type=float, default=120.0,
                        help="Max seconds to wait for enough samples (default: 120)")
    args = parser.parse_args()

    rclpy.init()
    node = LatencyProbe(n_samples=args.samples)

    t_start = time.time()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.time() - t_start
            if elapsed > args.timeout:
                print(f"\n[WARN] Timeout after {args.timeout:.0f}s — "
                      f"only {len(node.delays_ms)} samples collected.")
                break
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if len(node.delays_ms) < 20:
        print(f"\n[FAIL] Not enough samples ({len(node.delays_ms)}).")
        print("       Is _slam_bridge.py running?  Is Point-LIO publishing?")
        print("       Check:  ros2 topic hz /mavros/vision_pose/pose")
        sys.exit(1)

    _print_results(node.delays_ms)


if __name__ == "__main__":
    main()
