#!/usr/bin/env python3
"""test_ekf2_latency.py — Measure Pi→PX4 vision latency and recommend EKF2_EV_DELAY.

Subscribes to /mavros/vision_pose/pose (published by _slam_bridge.py) and
measures the delay between the message's header.stamp (Point-LIO estimation
time) and the local ROS clock at receipt.

Measurement chain
-----------------
  LiDAR scan → Point-LIO SLAM → header.stamp  ← measured from here
      → ROS DDS publish → MAVROS subscriber receives  ← to here
      → MAVROS serialises → serial 57600 baud → PX4 EKF2  ← +SERIAL_OFFSET_MS

Watchdog isolation
------------------
Stop drone-watchdog.service manually before running this test, then
restart it when done. The watchdog must not be running during the test
because it competes with the test's own Point-LIO and SLAM bridge
subprocesses for the same ROS topics and serial port.

  sudo systemctl stop drone-watchdog.service
  python3 test_ekf2_latency.py
  sudo systemctl start drone-watchdog.service

Architecture note
-----------------
This script owns a single rclpy context (one rclpy.init() call).
Point-LIO is launched as an OS subprocess — it runs in a separate
process and has no connection to this script's rclpy context.
The SLAM bridge is also launched as a subprocess.

The _wait_for_cloud() readiness check subscribes through the same
single rclpy node (passed in as a parameter) rather than calling
rclpy.init() again, which would conflict.

Requirements
------------
  - sudo systemctl stop drone-watchdog.service  (before running)
  - Unitree L1 LiDAR connected on /dev/ttyUSB0
  - pointlio-standby.service active and publishing /aft_mapped_to_init
  - SLAM bridge (_slam_bridge.py) available
  - MAVROS running (mavros.service)
  - System time sync between Pi and Pixhawk is NOT required

Usage
-----
  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash

  python3 test_ekf2_latency.py                  # default: 200 samples
  python3 test_ekf2_latency.py --samples 500
  python3 test_ekf2_latency.py --timeout 120

  sudo systemctl start drone-watchdog.service   (after test completes)

Output
------
  Prints latency statistics and a clear recommendation:
    ╔══════════════════════════════════════╗
    ║  Set EKF2_EV_DELAY = XX ms in QGC   ║
    ╚══════════════════════════════════════╝
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# ── Paths ─────────────────────────────────────────────────────────────────────

ROS_SETUP   = "/opt/ros/jazzy/setup.bash"
WS_SETUP    = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
BRIDGE_SCRIPT = Path(__file__).parent.parent / "flight" / "_slam_bridge.py"
MISSION_LOCK  = Path("/tmp/dronepi_mission.lock")
LIDAR_PORT    = "/dev/ttyUSB0"

POINTLIO_TIMEOUT_S  = 45
GRACEFUL_KILL_S     = 5

# ── Latency constants ─────────────────────────────────────────────────────────

# Added on top of measured ROS-side p95 latency:
#   MAVROS serialisation + MAVLink framing  ~3 ms
#   Serial TX at 57600 baud (~72B frame)    ~10 ms
#   PX4 EKF scheduling jitter               ~2 ms
SERIAL_OFFSET_MS = 15
SAFETY_MARGIN_MS = 5


# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)




# ── Lock file helpers ─────────────────────────────────────────────────────────

def write_lock():
    MISSION_LOCK.write_text(
        json.dumps({"mode": "bench_scan",
                    "started_at": datetime.now().isoformat()}))
    log("Watchdog lock written (bench_scan) — secondary process guard active")


def clear_lock():
    if MISSION_LOCK.exists():
        try:
            data = json.loads(MISSION_LOCK.read_text())
            if data.get("mode") == "bench_scan":
                MISSION_LOCK.unlink()
                log("Watchdog lock cleared")
        except Exception:
            pass


# ── Process helpers ───────────────────────────────────────────────────────────

def start_proc(name: str, cmd: str) -> subprocess.Popen:
    log(f"Starting {name}...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log(f"{name} started (PID {proc.pid})")
    return proc


def stop_proc(name: str, proc: subprocess.Popen):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly")
    except subprocess.TimeoutExpired:
        log(f"{name} did not exit — sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    except ProcessLookupError:
        pass


# ── ROS QoS profiles ──────────────────────────────────────────────────────────

def _make_reliable_qos():
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


# ── Latency probe node ────────────────────────────────────────────────────────

class LatencyProbeNode:
    """Single rclpy node that handles both /cloud_registered readiness
    and /mavros/vision_pose/pose latency measurement.

    A single rclpy.init() is called here. No other component in this
    script may call rclpy.init() — subprocesses are used for Point-LIO
    and the SLAM bridge instead.
    """

    def __init__(self, n_samples: int):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import PointCloud2

        self._rclpy     = rclpy
        self._lock      = threading.Lock()
        self.delays_ms  = []
        self.n_samples  = n_samples
        self._cloud_evt = threading.Event()

        rclpy.init()
        self._node = Node("ekf2_latency_probe")

        # Subscription 1: readiness check for /cloud_registered
        cloud_qos = _make_reliable_qos()
        self._node.create_subscription(
            PointCloud2, "/cloud_registered",
            self._cloud_cb, cloud_qos)

        # Subscription 2: latency measurement for /mavros/vision_pose/pose
        vision_qos = _make_reliable_qos()
        self._node.create_subscription(
            PoseStamped, "/mavros/vision_pose/pose",
            self._vision_cb, vision_qos)

        # Single spin thread for the single node
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

        self._node.get_logger().info(
            "Latency probe started — waiting for /cloud_registered "
            "then /mavros/vision_pose/pose"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg) -> None:
        """Fire once to confirm Point-LIO is publishing."""
        self._cloud_evt.set()

    def _vision_cb(self, msg) -> None:
        """Measure per-message latency from header.stamp to receipt."""
        now_ns   = self._node.get_clock().now().nanoseconds
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)
        delay_ms = (now_ns - stamp_ns) / 1e6

        # Ignore negative values and implausibly large ones (clock glitch)
        if 0.0 < delay_ms < 2_000.0:
            with self._lock:
                self.delays_ms.append(delay_ms)
                n = len(self.delays_ms)
            if n % 50 == 0 or n == self.n_samples:
                log(f"  Collected {n}/{self.n_samples} samples  "
                    f"(latest={delay_ms:.1f} ms)")

    # ── Wait helpers ───────────────────────────────────────────────────────────

    def wait_for_cloud(self, timeout: float) -> bool:
        """Block until /cloud_registered publishes or timeout.

        Uses the same node and spin thread — no second rclpy.init() needed.
        """
        t0       = time.time()
        last_log = -1
        while not self._cloud_evt.is_set():
            elapsed = time.time() - t0
            if elapsed >= timeout:
                return False
            tick = int(elapsed) // 5
            if tick != last_log and int(elapsed) > 0:
                log(f"Waiting for /cloud_registered... "
                    f"({elapsed:.0f}s / {timeout:.0f}s)")
                last_log = tick
            time.sleep(0.5)
        log(f"/cloud_registered confirmed ({time.time() - t0:.1f}s after launch)")
        return True

    @property
    def done(self) -> bool:
        with self._lock:
            return len(self.delays_ms) >= self.n_samples

    def shutdown(self):
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ── Results printer ───────────────────────────────────────────────────────────

def print_results(delays_ms: list) -> int:
    """Print latency statistics and return recommended EKF2_EV_DELAY in ms."""
    arr = np.array(delays_ms)

    mean_ms   = float(np.mean(arr))
    median_ms = float(np.median(arr))
    std_ms    = float(np.std(arr))
    p75_ms    = float(np.percentile(arr, 75))
    p90_ms    = float(np.percentile(arr, 90))
    p95_ms    = float(np.percentile(arr, 95))
    p99_ms    = float(np.percentile(arr, 99))
    max_ms    = float(np.max(arr))

    raw_rec     = p95_ms + SERIAL_OFFSET_MS + SAFETY_MARGIN_MS
    recommended = int(math.ceil(raw_rec / 5.0) * 5)   # round up to nearest 5ms

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
    print(f"  p95 ({p95_ms:.1f}) + serial ({SERIAL_OFFSET_MS} ms)"
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
    print("    4. Reboot the Pixhawk (power cycle)")
    print()

    if p95_ms > 100:
        print("  [WARN] p95 > 100 ms — high latency. "
              "Check Point-LIO CPU load and ROS DDS settings.")
    elif p95_ms < 5:
        print("  [WARN] p95 < 5 ms — suspiciously low. "
              "Verify _slam_bridge.py is running.")

    return recommended




# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Measure SLAM→MAVROS latency and recommend EKF2_EV_DELAY.")
    parser.add_argument("--samples", type=int, default=200,
                        help="Pose samples to collect (default: 200)")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="Max seconds to wait for samples (default: 120)")
    args = parser.parse_args()

    print("=" * 55)
    print("  EKF2_EV_DELAY Latency Test")
    print("=" * 55)

    pointlio_proc = None
    bridge_proc   = None
    probe         = None
    def _cleanup():
        """
        Tear down the test stack in the correct order.
        Called on both normal exit and Ctrl+C.
        """
        log("Cleaning up test stack...")
        if bridge_proc:
            stop_proc("slam_bridge", bridge_proc)
        if pointlio_proc:
            stop_proc("Point-LIO", pointlio_proc)
        if probe:
            try:
                probe.shutdown()
            except Exception:
                pass
        clear_lock()

    def _sighandler(signum, frame):
        print()
        log(f"Signal {signum} received — stopping test cleanly")
        _cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _sighandler)
    signal.signal(signal.SIGTERM, _sighandler)

    write_lock()

    # ── Single rclpy node owns all subscriptions ─────────────────────────────
    # LatencyProbeNode calls rclpy.init() exactly once. Point-LIO and the
    # SLAM bridge are OS subprocesses — they do not share this context.
    probe = LatencyProbeNode(n_samples=args.samples)

    # ── Step 3: Launch Point-LIO subprocess ───────────────────────────────────
    pointlio_cmd = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:={LIDAR_PORT}"
    )
    pointlio_proc = start_proc("Point-LIO", pointlio_cmd)

    # ── Step 4: Wait for SLAM output ──────────────────────────────────────────
    # Reuses the probe node — no second rclpy.init()
    cloud_ok = probe.wait_for_cloud(timeout=POINTLIO_TIMEOUT_S)
    if not cloud_ok:
        log(f"[WARN] /cloud_registered not seen after {POINTLIO_TIMEOUT_S}s")
        log("       Continuing — SLAM bridge may still produce vision poses")

    # ── Step 5: Launch SLAM bridge subprocess ─────────────────────────────────
    if BRIDGE_SCRIPT.exists():
        bridge_cmd = (
            f"source {ROS_SETUP} && source {WS_SETUP} && "
            f"python3 {BRIDGE_SCRIPT}"
        )
        bridge_proc = start_proc("slam_bridge", bridge_cmd)
        log("Allowing slam_bridge 5s to stabilise...")
        time.sleep(5.0)
    else:
        log(f"[FAIL] _slam_bridge.py not found at {BRIDGE_SCRIPT}")
        _cleanup()
        sys.exit(1)

    # ── Step 6: Collect samples ───────────────────────────────────────────────
    log(f"Subscribed to /mavros/vision_pose/pose — collecting {args.samples} samples")
    t_start = time.time()

    try:
        while not probe.done:
            elapsed = time.time() - t_start
            if elapsed > args.timeout:
                log(f"[WARN] Timeout after {args.timeout:.0f}s — "
                    f"only {len(probe.delays_ms)} samples collected")
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        log("Interrupted by user")

    # ── Step 7: Capture samples before tearing down the probe ────────────────
    # Save the delay list now — probe.shutdown() destroys the rclpy node but
    # does not clear delays_ms. We null the probe reference afterward so the
    # _cleanup() signal handler does not attempt a double-shutdown.
    final_samples = list(probe.delays_ms)

    # ── Stop test stack ───────────────────────────────────────────────────────
    stop_proc("slam_bridge", bridge_proc)
    bridge_proc = None
    stop_proc("Point-LIO", pointlio_proc)
    pointlio_proc = None
    probe.shutdown()
    probe = None
    clear_lock()

    # ── Print results ─────────────────────────────────────────────────────────
    if len(final_samples) < 20:
        print(f"\n[FAIL] Not enough samples ({len(final_samples)}).")
        print("       Is _slam_bridge.py publishing /mavros/vision_pose/pose?")
        print("       Check:  ros2 topic hz /mavros/vision_pose/pose")
        sys.exit(1)

    print_results(final_samples)


if __name__ == "__main__":
    main()
