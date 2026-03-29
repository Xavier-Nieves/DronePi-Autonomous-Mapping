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
7. Runs postflight processing with Pixhawk buzzer feedback
8. Clears the mission lock so watchdog resumes normal operation

Buzzer Feedback (identical tunes to drone_watchdog.py)
-------------------------------------------------------
  Slow mid pulse every 5s   — postflight mesh pipeline running
  Two-note up (C-E)         — postflight complete successfully
  Staccato triple (C-C-C)   — postflight failed

  BuzzerHelper reuses the StateMonitor node — no second rclpy
  context. Silently disabled if MAVROS is not available.

Lock File Protocol
------------------
  bench_scan  — --no-disarm-stop flag. Watchdog yields entirely.
  manual_scan — Normal flights. Watchdog monitors for disarm.

Disarm Auto-Stop
----------------
  Only fires if the drone armed during this session then disarmed.
  A drone already disarmed at scan start is NOT auto-stopped.

Usage
-----
  python3 test_manual_scan.py
  python3 test_manual_scan.py --skip-lidar-check
  python3 test_manual_scan.py --no-postflight
  python3 test_manual_scan.py --no-disarm-stop --skip-lidar-check
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
UTILS_DIR     = PROJECT_ROOT / "utils"

ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
POSTFLIGHT_SCRIPT = UTILS_DIR / "run_postflight.py"
MISSION_LOCK      = Path("/tmp/dronepi_mission.lock")
LIDAR_PORT        = Path("/dev/ttyUSB0")
ROSBAG_DIR        = Path("/mnt/ssd/rosbags")

GRACEFUL_KILL_S    = 5
POINTLIO_TIMEOUT_S = 30

# ── Buzzer tunes — identical to watchdog_core/buzzer.py ──────────────────────
_TUNE_FORMAT     = 1               # QBASIC1_1 — confirmed working on PX4
_TUNE_PROCESSING = "T200L32O4G"    # slow mid pulse — postflight running
_TUNE_DONE       = "T240L16O5CE"   # two-note up    — complete
_TUNE_ERROR      = "T200L16O5CPCPC" # staccato x3   — failed
_BEEP_INTERVAL_S = 5.0


# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)


# ── Lock file ─────────────────────────────────────────────────────────────────

def write_lock(mode: str):
    MISSION_LOCK.write_text(
        json.dumps({"mode": mode, "started_at": datetime.now().isoformat()}))
    log(f"Watchdog lock written ({mode})")


def clear_lock():
    if MISSION_LOCK.exists():
        MISSION_LOCK.unlink()
        log("Watchdog lock cleared -- watchdog resuming normal operation")


# ── Process management ────────────────────────────────────────────────────────

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


# ── Buzzer Helper ─────────────────────────────────────────────────────────────

class BuzzerHelper:
    """Pixhawk buzzer publisher using the StateMonitor's existing rclpy node.

    Mirrors PostflightMonitor + PostflightBeeper from drone_watchdog.py.
    No second rclpy context is created — the publisher is added onto
    the StateMonitor's node which is already spinning on a daemon thread.

    Silently no-ops if MAVROS is unavailable or import fails, so the
    scan and postflight always complete regardless of buzzer status.

    Parameters
    ----------
    monitor : StateMonitor or None
        Active monitor whose node hosts the PlayTuneV2 publisher.
    """

    def __init__(self, monitor):
        self._pub          = None
        self._PlayTuneV2   = None
        self._active       = False
        self._beep_thread  = None

        if monitor is None:
            return
        try:
            from mavros_msgs.msg import PlayTuneV2
            self._PlayTuneV2 = PlayTuneV2
            self._pub = monitor._node.create_publisher(
                PlayTuneV2, "/mavros/play_tune", 10)
        except Exception as exc:
            log(f"[Buzzer] Publisher unavailable: {exc} — audio disabled")

    def play(self, tune: str) -> None:
        """Publish one tune string immediately. Non-blocking."""
        if self._pub is None:
            return
        try:
            msg        = self._PlayTuneV2()
            msg.format = _TUNE_FORMAT
            msg.tune   = tune
            self._pub.publish(msg)
        except Exception:
            pass

    def start_processing(self) -> None:
        """Begin playing the processing pulse on a daemon thread."""
        if self._pub is None:
            return
        self._active      = True
        self._beep_thread = threading.Thread(target=self._loop, daemon=True)
        self._beep_thread.start()

    def stop_processing(self) -> None:
        """Stop the processing pulse thread."""
        self._active = False
        if self._beep_thread:
            self._beep_thread.join(timeout=_BEEP_INTERVAL_S + 1)
            self._beep_thread = None

    def _loop(self) -> None:
        while self._active:
            self.play(_TUNE_PROCESSING)
            for _ in range(int(_BEEP_INTERVAL_S * 10)):
                if not self._active:
                    break
                time.sleep(0.1)


# ── LiDAR health check ────────────────────────────────────────────────────────

def check_lidar() -> bool:
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
        qos  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        node.create_subscription(PointCloud2, "/unilidar/cloud",
                                 lambda msg: received.set(), qos)
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        result = received.wait(timeout=8.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        if result:
            log("[OK] LiDAR healthy -- /unilidar/cloud publishing")
        else:
            log("[WARN] LiDAR device found but no data within 8s")
            log("       Point-LIO not yet running -- will start with scan")
        return True
    except Exception as e:
        log(f"[WARN] Could not verify LiDAR topic: {e}")
        return True


# ── MAVROS state monitor ──────────────────────────────────────────────────────

class StateMonitor:
    """Owns the single rclpy context. BuzzerHelper and cloud readiness check
    must use add_subscription() rather than calling rclpy.init() again."""

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
        self._node.create_subscription(State, "/mavros/state", self._cb, 10)
        threading.Thread(target=rclpy.spin, args=(self._node,),
                         daemon=True).start()

    def add_subscription(self, msg_type, topic, callback, qos):
        self._node.create_subscription(msg_type, topic, callback, qos)

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
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ── Point-LIO readiness check ─────────────────────────────────────────────────

def wait_for_cloud_registered(monitor: StateMonitor,
                               timeout: float = POINTLIO_TIMEOUT_S) -> bool:
    """Block until /cloud_registered publishes. Reuses monitor's node."""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2

    received = threading.Event()
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    monitor.add_subscription(PointCloud2, "/cloud_registered",
                              lambda msg: received.set(), qos)

    t0 = time.time()
    last_log = -1
    while not received.is_set():
        elapsed = time.time() - t0
        if elapsed >= timeout:
            log(f"[WARN] /cloud_registered not seen after {timeout:.0f}s")
            log("       Starting bag anyway")
            return False
        tick = int(elapsed) // 5
        if tick != last_log and int(elapsed) > 0:
            log(f"Waiting for /cloud_registered... ({elapsed:.0f}s / {timeout:.0f}s)")
            last_log = tick
        time.sleep(0.5)

    log(f"/cloud_registered confirmed ({time.time() - t0:.1f}s after launch)")
    return True


# ── Postflight with buzzer ────────────────────────────────────────────────────

def run_postflight_with_beeps(bag_dir: str, buzzer: BuzzerHelper) -> int:
    """Run run_postflight.py subprocess with Pixhawk audio feedback.

    Mirrors PostflightMonitor.trigger() from drone_watchdog.py:
      beep_processing() on start → pulse every 5s while running →
      beep_done() on success or beep_error() on failure.

    Parameters
    ----------
    bag_dir : str  — bag directory passed to run_postflight.py
    buzzer  : BuzzerHelper — silently no-ops if MAVROS unavailable

    Returns exit code (0 = success).
    """
    log("Running postflight processing...")
    log("(This may take several minutes depending on point count)")

    # First pulse immediately so the operator hears confirmation
    buzzer.play(_TUNE_PROCESSING)
    buzzer.start_processing()

    try:
        result = subprocess.run([
            sys.executable, str(POSTFLIGHT_SCRIPT),
            "--bag", bag_dir, "--auto", "--skip-wait"
        ])
        rc = result.returncode
    except Exception as exc:
        log(f"[WARN] Postflight subprocess error: {exc}")
        rc = 1
    finally:
        buzzer.stop_processing()

    if rc == 0:
        buzzer.play(_TUNE_DONE)
        log("Postflight complete -- check browser:")
        log("  http://10.42.0.1:8080/meshview.html")
    else:
        buzzer.play(_TUNE_ERROR)
        log("[WARN] Postflight processing failed")
        log(f"  Run manually: python3 {POSTFLIGHT_SCRIPT} --bag {bag_dir}")

    return rc


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Manual flight with on-demand LiDAR scanning.")
    parser.add_argument("--skip-lidar-check", action="store_true")
    parser.add_argument("--no-disarm-stop",   action="store_true",
                        help="Disable auto-stop on disarm (bench testing only)")
    parser.add_argument("--no-postflight",    action="store_true")
    args = parser.parse_args()

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
    print()

    # ── Preflight checks ──────────────────────────────────────────────────────
    if not args.skip_lidar_check:
        if not check_lidar():
            log("LiDAR check failed -- fix and retry")
            sys.exit(1)
    else:
        if not LIDAR_PORT.exists():
            log("[FAIL] LiDAR not detected -- connect and power on first")
            sys.exit(1)
        log("LiDAR check skipped (--skip-lidar-check)")

    if not ROSBAG_DIR.exists():
        log("[FAIL] /mnt/ssd/rosbags not found -- SSD not mounted")
        log("       Run: sudo mount -a")
        sys.exit(1)

    write_lock(lock_mode)

    pointlio_proc = None
    bag_proc      = None
    monitor       = None
    buzzer        = BuzzerHelper(None)   # safe no-op until monitor is ready

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
        sys.exit(0)

    signal.signal(signal.SIGINT,  _sighandler)
    signal.signal(signal.SIGTERM, _sighandler)

    # ── Start MAVROS state monitor ────────────────────────────────────────────
    try:
        monitor = StateMonitor()
        buzzer  = BuzzerHelper(monitor)   # upgrade to real buzzer now node exists
        log(f"MAVROS monitor active -- mode={monitor.mode}  armed={monitor.armed}")
    except Exception as e:
        log(f"[WARN] Could not start MAVROS monitor: {e}")
        log("       Proceeding without arm/mode monitoring or buzzer")

    # ── Wait for operator ─────────────────────────────────────────────────────
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

    # ── Launch Point-LIO and bag recorder ─────────────────────────────────────
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

    log("Waiting for Point-LIO to publish /cloud_registered before starting bag...")
    if monitor:
        wait_for_cloud_registered(monitor)
    else:
        log("[WARN] No monitor -- falling back to 15s fixed wait")
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

    # ── Status thread ─────────────────────────────────────────────────────────
    stop_event = threading.Event()

    def _status_thread():
        while not stop_event.is_set():
            if monitor:
                armed_str = "ARMED" if monitor.armed else "disarmed"
                print(f"\r  [{datetime.now().strftime('%H:%M:%S')}] "
                      f"Scanning  mode={monitor.mode}  {armed_str}   ",
                      end="", flush=True)
            stop_event.wait(2.0)

    threading.Thread(target=_status_thread, daemon=True).start()

    # ── Monitor loop ──────────────────────────────────────────────────────────
    # was_armed_at_any_point: latch that prevents false auto-stop when the
    # drone starts disarmed (ground scans, bench tests, pre-arm scanning).
    stop_reason            = "operator"
    was_armed_at_any_point = False

    try:
        entered = threading.Event()

        def _wait_input():
            try:
                input()
                entered.set()
            except Exception:
                entered.set()

        threading.Thread(target=_wait_input, daemon=True).start()

        while not entered.is_set():
            if monitor and monitor.armed:
                was_armed_at_any_point = True

            if (not args.no_disarm_stop
                    and monitor
                    and was_armed_at_any_point
                    and not monitor.armed
                    and monitor.mode != ""):
                time.sleep(3.0)   # brief grace period for transient disarms
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

    # ── Postflight with buzzer feedback ───────────────────────────────────────
    if not args.no_postflight:
        print()
        run_postflight_with_beeps(bag_dir, buzzer)
    else:
        log("Postflight skipped (--no-postflight)")
        log(f"  Run manually: python3 {POSTFLIGHT_SCRIPT} --bag {bag_dir}")


if __name__ == "__main__":
    main()
