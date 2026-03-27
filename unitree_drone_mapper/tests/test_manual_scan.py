#!/usr/bin/env python3
"""
test_manual_scan.py — Manual flight with on-demand LiDAR scanning.

Designed for situations where GPS quality is too poor for OFFBOARD
autonomous flight but you still want to collect a LiDAR scan while
flying manually in STABILIZED, ALTCTL, or POSCTL mode.

What this script does
---------------------
1. Verifies LiDAR is connected and hardware is healthy
2. Waits for your command to start scanning
3. Starts Point-LIO and waits for /cloud_registered to publish
4. Starts bag recording only once SLAM output is confirmed
5. You fly manually via RC in any flight mode
6. Stop scanning on your command (or on disarm)
7. Runs postflight processing automatically
8. Clears the mission lock so watchdog resumes normal operation

Lock File Protocol
------------------
  bench_scan  — Written by this script when --no-disarm-stop is set.
                Watchdog yields entirely (same as autonomous mode).
                Use for indoor bench testing without a real flight.

  manual_scan — Written by this script during real flights.
                Watchdog monitors for disarm and triggers post-flight.
                Do NOT use --no-disarm-stop in this mode.

This script does NOT require OFFBOARD mode or GPS lock.
You fly entirely via RC -- the script only manages the scanning stack.

Usage
-----
  # Normal outdoor flight scan
  python3 test_manual_scan.py

  # Skip LiDAR health check (LiDAR already confirmed working)
  python3 test_manual_scan.py --skip-lidar-check

  # Skip postflight processing (process manually later)
  python3 test_manual_scan.py --no-postflight

  # Indoor bench test — watchdog yields, disarm auto-stop disabled
  python3 test_manual_scan.py --no-disarm-stop --skip-lidar-check --no-postflight

SAFETY
------
  - Fly in STABILIZED or ALTCTL for best manual control
  - This script never arms, never changes flight mode
  - RC transmitter has full control at all times
  - Ctrl+C stops scanning cleanly and saves the bag
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────

PROJECT_ROOT  = Path(__file__).parent.parent
FLIGHT_DIR    = PROJECT_ROOT / "flight"
UTILS_DIR     = PROJECT_ROOT / "utils"

ROS_SETUP     = "/opt/ros/jazzy/setup.bash"
WS_SETUP      = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE   = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
POSTFLIGHT_SCRIPT = UTILS_DIR / "run_postflight.py"
MISSION_LOCK      = Path("/tmp/dronepi_mission.lock")
LIDAR_PORT        = Path("/dev/ttyUSB0")
ROSBAG_DIR        = Path("/mnt/ssd/rosbags")

GRACEFUL_KILL_S    = 5
POINTLIO_TIMEOUT_S = 30   # Max seconds to wait for /cloud_registered


# ── Helpers ───────────────────────────────────────────────────────────────────

def log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def write_lock(mode: str):
    MISSION_LOCK.write_text(
        json.dumps({"mode": mode,
                    "started_at": datetime.now().isoformat()}))
    log(f"Watchdog lock written ({mode})")


def clear_lock():
    if MISSION_LOCK.exists():
        MISSION_LOCK.unlink()
        log("Watchdog lock cleared -- watchdog resuming normal operation")


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
        log(f"{name} did not exit -- sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    except ProcessLookupError:
        pass


# ── LiDAR Health Check ────────────────────────────────────────────────────────

def check_lidar() -> bool:
    """Three-outcome LiDAR check.

    1. /dev/ttyUSB0 absent          → LiDAR off or unplugged
    2. Device present, no data 8s   → Point-LIO not yet running (non-fatal)
    3. Device present, data flowing → healthy

    Note: /unilidar/cloud only publishes after the combined launch starts.
    A timeout warning at this stage is expected and non-fatal — the
    device-present result alone is sufficient to proceed.
    """
    if not LIDAR_PORT.exists():
        log("[FAIL] LiDAR not detected at /dev/ttyUSB0")
        log("       Power on the Unitree L1 and plug in USB")
        return False

    log(f"LiDAR device found at {LIDAR_PORT}")
    log("Checking if /unilidar/cloud is already publishing (8s timeout)...")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2

        received = threading.Event()

        rclpy.init()
        node = Node("lidar_health_check")
        qos  = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        node.create_subscription(
            PointCloud2, "/unilidar/cloud",
            lambda msg: received.set(), qos)

        spin_thread = threading.Thread(
            target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        result = received.wait(timeout=8.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        if result:
            log("[OK] LiDAR healthy -- /unilidar/cloud publishing")
        else:
            log("[WARN] LiDAR device found but no data within 8s")
            log("       Point-LIO not yet running -- will start with scan")
        return True   # device present in both cases; proceed with launch

    except Exception as e:
        log(f"[WARN] Could not verify LiDAR topic: {e}")
        log("       Proceeding since device is present")
        return True


# ── MAVROS State Monitor ──────────────────────────────────────────────────────

class StateMonitor:
    """Lightweight MAVROS state subscriber.

    Owns the single rclpy context for this process. Other components
    that need to subscribe to ROS topics must reuse this node via
    add_subscription() rather than calling rclpy.init() themselves.
    """

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from mavros_msgs.msg import State

        self._lock  = threading.Lock()
        self._armed = False
        self._mode  = ""
        self._rclpy = rclpy

        rclpy.init()
        self._node = Node("manual_scan_monitor")
        self._node.create_subscription(
            State, "/mavros/state", self._cb, 10)

        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()

    def add_subscription(self, msg_type, topic: str, callback, qos):
        """Add a subscription to this node from any thread.

        Use this instead of creating a separate rclpy node, since only
        one rclpy context is permitted per process.
        """
        self._node.create_subscription(msg_type, topic, callback, qos)

    def _cb(self, msg):
        with self._lock:
            self._armed = msg.armed
            self._mode  = msg.mode

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    def shutdown(self):
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ── Point-LIO Readiness Check ─────────────────────────────────────────────────

def wait_for_cloud_registered(
        monitor: StateMonitor,
        timeout: float = POINTLIO_TIMEOUT_S) -> bool:
    """Block until /cloud_registered publishes or timeout expires.

    Reuses the StateMonitor's existing rclpy node to avoid creating a
    second rclpy context, which is not permitted within a single process.

    Returns True if data arrived within timeout, False otherwise.
    The bag recorder must only be started after this function returns,
    ensuring bags always contain /cloud_registered point cloud data.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2

    received = threading.Event()

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    monitor.add_subscription(
        PointCloud2, "/cloud_registered",
        lambda msg: received.set(), qos
    )

    t0       = time.time()
    last_log = -1

    while not received.is_set():
        elapsed = time.time() - t0

        if elapsed >= timeout:
            log(f"[WARN] /cloud_registered not seen after {timeout:.0f}s")
            log("       Point-LIO may still be initialising -- starting bag anyway")
            return False

        tick = int(elapsed) // 5
        if tick != last_log and int(elapsed) > 0:
            log(f"Waiting for /cloud_registered... ({elapsed:.0f}s / {timeout:.0f}s)")
            last_log = tick

        time.sleep(0.5)

    log(f"/cloud_registered confirmed ({time.time() - t0:.1f}s after launch)")
    return True


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Manual flight with on-demand LiDAR scanning.")
    parser.add_argument("--skip-lidar-check", action="store_true",
                        help="Skip LiDAR health check")
    parser.add_argument("--no-disarm-stop",   action="store_true",
                        help=(
                            "Disable auto-stop on disarm. "
                            "Writes bench_scan lock so watchdog yields entirely. "
                            "Use for indoor bench testing only."
                        ))
    parser.add_argument("--no-postflight",    action="store_true",
                        help="Skip postflight processing")
    args = parser.parse_args()

    # Lock mode depends on whether we are bench testing or flying for real.
    # bench_scan → watchdog yields entirely (same behaviour as autonomous).
    # manual_scan → watchdog monitors for disarm and triggers post-flight.
    lock_mode = "bench_scan" if args.no_disarm_stop else "manual_scan"

    print("\n" + "=" * 55)
    print("  DronePi Manual Scan")
    print("  Fly manually via RC -- scan on your command")
    print("=" * 55)
    print("  No OFFBOARD mode required.")
    print("  No GPS lock required.")
    print("  Fly in STABILIZED / ALTCTL / POSCTL.")
    if args.no_disarm_stop:
        print("  [BENCH MODE] Disarm auto-stop disabled.")
        print("  [BENCH MODE] Watchdog will yield (bench_scan lock).")
    print()

    # ── Check LiDAR ──────────────────────────────────────────────────────────
    if not args.skip_lidar_check:
        if not check_lidar():
            log("LiDAR check failed -- fix and retry")
            sys.exit(1)
    else:
        if not LIDAR_PORT.exists():
            log("[FAIL] LiDAR not detected -- connect and power on first")
            sys.exit(1)
        log("LiDAR check skipped (--skip-lidar-check)")

    # ── Check SSD ────────────────────────────────────────────────────────────
    if not ROSBAG_DIR.exists():
        log("[FAIL] /mnt/ssd/rosbags not found -- SSD not mounted")
        log("       Run: sudo mount -a")
        sys.exit(1)

    # ── Write mission lock ────────────────────────────────────────────────────
    # bench_scan → watchdog yields (no stack conflict)
    # manual_scan → watchdog monitors disarm + post-flight for real flights
    write_lock(lock_mode)

    pointlio_proc = None
    bag_proc      = None
    monitor       = None

    def _cleanup():
        log("Stopping scan stack...")
        stop_proc("Bag recorder", bag_proc)
        stop_proc("Point-LIO",    pointlio_proc)
        clear_lock()
        if monitor:
            monitor.shutdown()

    def _sighandler(signum, frame):
        print()
        log("Ctrl+C -- stopping scan cleanly...")
        _cleanup()
        if not args.no_postflight and bag_proc is not None:
            log("Running postflight processing...")
            subprocess.run([
                sys.executable, str(POSTFLIGHT_SCRIPT),
                "--auto", "--skip-wait"
            ])
        sys.exit(0)

    signal.signal(signal.SIGINT,  _sighandler)
    signal.signal(signal.SIGTERM, _sighandler)

    # ── Start MAVROS state monitor (owns the rclpy context) ──────────────────
    try:
        monitor = StateMonitor()
        log(f"MAVROS monitor active -- mode={monitor.mode}  armed={monitor.armed}")
    except Exception as e:
        log(f"[WARN] Could not start MAVROS monitor: {e}")
        log("       Proceeding without arm/mode monitoring")
        monitor = None

    # ── Wait for operator to start scan ──────────────────────────────────────
    print()
    print("  Ready to scan.")
    print("  Press Enter to START Point-LIO and bag recording.")
    print("  Ctrl+C at any time to STOP and save.")
    print()
    try:
        input("  > Press Enter to start scan: ")
    except KeyboardInterrupt:
        log("Cancelled.")
        clear_lock()
        sys.exit(0)

    # ── Launch Point-LIO ──────────────────────────────────────────────────────
    scan_ts      = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir      = str(ROSBAG_DIR / f"scan_{scan_ts}")
    pointlio_cmd = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0"
    )
    bag_cmd = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 bag record -o {bag_dir} "
        f"/cloud_registered /aft_mapped_to_init /unilidar/imu "
        f"/mavros/state /mavros/local_position/pose "
        f"/mavros/global_position/global"
    )

    pointlio_proc = start_proc("Point-LIO", pointlio_cmd)

    # Wait for SLAM output before starting the bag recorder.
    # Reuses the StateMonitor node to avoid a second rclpy.init() call.
    log("Waiting for Point-LIO to publish /cloud_registered before starting bag...")
    if monitor:
        wait_for_cloud_registered(monitor)
    else:
        log("[WARN] No monitor node available -- falling back to 15s fixed wait")
        time.sleep(15.0)

    bag_proc = start_proc("Bag recorder", bag_cmd)
    log(f"Bag recording to: {bag_dir}")

    print()
    print("  ╔══════════════════════════════════════════╗")
    print("  ║  SCANNING ACTIVE -- Fly now via RC       ║")
    print("  ║  Press Enter to STOP and save            ║")
    print("  ║  Ctrl+C also stops cleanly               ║")
    print("  ╚══════════════════════════════════════════╝")
    print()

    # ── Monitor while scanning ────────────────────────────────────────────────
    stop_event = threading.Event()

    def _status_thread():
        while not stop_event.is_set():
            if monitor:
                armed_str = "ARMED" if monitor.armed else "disarmed"
                print(f"\r  [{datetime.now().strftime('%H:%M:%S')}] "
                      f"Scanning  mode={monitor.mode}  {armed_str}   ",
                      end="", flush=True)
            stop_event.wait(2.0)

    status_t = threading.Thread(target=_status_thread, daemon=True)
    status_t.start()

    # Wait for Enter or auto-stop on disarm.
    # Disarm detection is disabled in bench mode (--no-disarm-stop).
    stop_reason = "operator"
    try:
        entered = threading.Event()

        def _wait_input():
            try:
                input()
                entered.set()
            except Exception:
                entered.set()

        input_t = threading.Thread(target=_wait_input, daemon=True)
        input_t.start()

        while not entered.is_set():
            if (not args.no_disarm_stop
                    and monitor
                    and not monitor.armed
                    and monitor.mode != ""):
                # Give 3s grace period in case of brief disarm
                time.sleep(3.0)
                if not monitor.armed:
                    stop_reason = "disarm"
                    log("Drone disarmed -- stopping scan automatically")
                    break
            time.sleep(0.5)

    except KeyboardInterrupt:
        stop_reason = "ctrl+c"

    stop_event.set()
    print()

    # ── Stop stack ────────────────────────────────────────────────────────────
    log(f"Stopping scan ({stop_reason})...")
    stop_proc("Bag recorder", bag_proc)
    bag_proc = None
    stop_proc("Point-LIO",    pointlio_proc)
    pointlio_proc = None
    clear_lock()

    if monitor:
        monitor.shutdown()

    print()
    print("=" * 55)
    print("  Scan complete")
    print(f"  Bag saved: {bag_dir}")
    print("=" * 55)

    # ── Postflight ────────────────────────────────────────────────────────────
    if not args.no_postflight:
        print()
        log("Running postflight processing...")
        log("(This may take several minutes depending on point count)")
        result = subprocess.run([
            sys.executable, str(POSTFLIGHT_SCRIPT),
            "--bag", bag_dir, "--auto", "--skip-wait"
        ])
        if result.returncode == 0:
            log("Postflight complete -- check browser:")
            log("  http://10.42.0.1:8080/meshview.html")
        else:
            log("[WARN] Postflight processing failed")
            log(f"  Run manually: python3 {POSTFLIGHT_SCRIPT} --bag {bag_dir}")
    else:
        log("Postflight skipped (--no-postflight)")
        log(f"  Run manually: python3 {POSTFLIGHT_SCRIPT} --bag {bag_dir}")


if __name__ == "__main__":
    main()
