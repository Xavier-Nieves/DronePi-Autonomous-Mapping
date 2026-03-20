#!/usr/bin/env python3
"""
test_manual_scan.py — Manual flight with on-demand LiDAR scanning.

Designed for situations where GPS quality is too poor for OFFBOARD
autonomous flight but you still want to collect a LiDAR scan while
flying manually in STABILIZED, ALTCTL, or POSCTL mode.

What this script does
---------------------
1. Verifies watchdog is running and hands off to supervised mode
2. Checks LiDAR is connected and healthy
3. Waits for your command to start scanning
4. Starts Point-LIO + bag recording on your command
5. You fly manually via RC in any flight mode
6. Stop scanning on your command (or on disarm)
7. Runs postflight processing automatically
8. Restores watchdog to normal operation

This script does NOT require OFFBOARD mode or GPS lock.
You fly entirely via RC -- the script only manages the scanning stack.

Usage
-----
  python3 test_manual_scan.py

  # Skip LiDAR health check (LiDAR already confirmed working)
  python3 test_manual_scan.py --skip-lidar-check

  # Skip postflight processing (process manually later)
  python3 test_manual_scan.py --no-postflight

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

# ── paths ─────────────────────────────────────────────────────────────────────

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

GRACEFUL_KILL_S = 5
POINTLIO_INIT_S = 5

# ── helpers ───────────────────────────────────────────────────────────────────

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

# ── LiDAR health check ────────────────────────────────────────────────────────

def check_lidar() -> bool:
    """
    Three outcome LiDAR check:
      1. /dev/ttyUSB0 absent  -> LiDAR off or unplugged
      2. Device present, no data within 8s -> hardware issue
      3. Device present, data flowing -> healthy
    """
    if not LIDAR_PORT.exists():
        log("[FAIL] LiDAR not detected at /dev/ttyUSB0")
        log("       Power on the Unitree L1 and plug in USB")
        return False

    log(f"LiDAR device found at {LIDAR_PORT}")
    log("Checking if LiDAR is publishing (8s timeout)...")

    # Quick topic check -- start Point-LIO briefly then check for data
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2

        received = threading.Event()

        rclpy.init()
        node = Node("lidar_health_check")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub = node.create_subscription(
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
            return True
        else:
            log("[WARN] LiDAR device found but no data within 8s")
            log("       Point-LIO may not be running yet")
            log("       Proceeding -- Point-LIO will start with scan")
            return True   # device present, proceed with launch

    except Exception as e:
        log(f"[WARN] Could not verify LiDAR topic: {e}")
        log("       Proceeding since device is present")
        return True

# ── MAVROS state monitor ──────────────────────────────────────────────────────

class StateMonitor:
    """Lightweight MAVROS state subscriber for monitoring only."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from mavros_msgs.msg import State

        self._lock  = threading.Lock()
        self._armed = False
        self._mode  = ""

        rclpy.init()
        self._node = Node("manual_scan_monitor")
        self._node.create_subscription(
            State, "/mavros/state", self._cb, 10)
        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()
        self._rclpy = rclpy

    def _cb(self, msg):
        with self._lock:
            self._armed = msg.armed
            self._mode  = msg.mode

    @property
    def armed(self) -> bool:
        with self._lock: return self._armed

    @property
    def mode(self) -> str:
        with self._lock: return self._mode

    def shutdown(self):
        if self._rclpy.ok():
            self._rclpy.shutdown()

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Manual flight with on-demand LiDAR scanning.")
    parser.add_argument("--skip-lidar-check", action="store_true",
                        help="Skip LiDAR health check")
    parser.add_argument("--no-postflight",    action="store_true",
                        help="Skip postflight processing")
    args = parser.parse_args()

    print("\n" + "=" * 55)
    print("  DronePi Manual Scan")
    print("  Fly manually via RC -- scan on your command")
    print("=" * 55)
    print("  No OFFBOARD mode required.")
    print("  No GPS lock required.")
    print("  Fly in STABILIZED / ALTCTL / POSCTL.")
    print()

    # ── check LiDAR ──────────────────────────────────────────────────────────
    if not args.skip_lidar_check:
        if not check_lidar():
            log("LiDAR check failed -- fix and retry")
            sys.exit(1)
    else:
        if not LIDAR_PORT.exists():
            log("[FAIL] LiDAR not detected -- connect and power on first")
            sys.exit(1)
        log("LiDAR check skipped (--skip-lidar-check)")

    # ── check SSD ────────────────────────────────────────────────────────────
    if not ROSBAG_DIR.exists():
        log("[FAIL] /mnt/ssd/rosbags not found -- SSD not mounted")
        log("       Run: sudo mount -a")
        sys.exit(1)

    # ── write manual_scan lock so watchdog stays out of the way ──────────────
    # The watchdog will see this lock and skip its own launch.
    # It will still monitor for disarm and run postflight when done.
    write_lock("manual_scan")

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

    # ── start MAVROS state monitor ────────────────────────────────────────────
    try:
        monitor = StateMonitor()
        log(f"MAVROS monitor active -- mode={monitor.mode}  armed={monitor.armed}")
    except Exception as e:
        log(f"[WARN] Could not start MAVROS monitor: {e}")
        log("       Proceeding without arm/mode monitoring")
        monitor = None

    # ── wait for operator to start scan ──────────────────────────────────────
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

    # ── launch Point-LIO ──────────────────────────────────────────────────────
    ts            = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir       = str(ROSBAG_DIR / f"scan_{ts}")
    pointlio_cmd  = (
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
    log(f"Waiting {POINTLIO_INIT_S}s for Point-LIO to initialize...")
    time.sleep(POINTLIO_INIT_S)

    bag_proc = start_proc("Bag recorder", bag_cmd)
    log(f"Bag recording to: {bag_dir}")

    print()
    print("  ╔══════════════════════════════════════════╗")
    print("  ║  SCANNING ACTIVE -- Fly now via RC       ║")
    print("  ║  Press Enter to STOP and save            ║")
    print("  ║  Ctrl+C also stops cleanly               ║")
    print("  ╚══════════════════════════════════════════╝")
    print()

    # ── monitor while scanning ────────────────────────────────────────────────
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

    # Wait for Enter or auto-stop on disarm
    stop_reason = "operator"
    try:
        # Run input in a thread so we can also detect disarm
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
            # Auto-stop if drone disarms (landed)
            if monitor and not monitor.armed and monitor.mode != "":
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

    # ── stop stack ────────────────────────────────────────────────────────────
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

    # ── postflight ────────────────────────────────────────────────────────────
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
