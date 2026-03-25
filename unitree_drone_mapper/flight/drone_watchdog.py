#!/usr/bin/env python3
"""drone_watchdog.py — Flight stack supervisor with RC toggle and buzzer feedback.

Runs as the drone-watchdog systemd service. Works in coordination with
main.py (dronepi-main service) via a shared lock file at
/tmp/dronepi_mission.lock.

Lock File Behaviour
-------------------
  absent        — watchdog operates normally (arm + OFFBOARD detection)
  manual_scan   — main.py has committed MODE 2; watchdog starts the flight
                  stack (Point-LIO + SLAM bridge + bag recorder) and handles
                  disarm + post-flight trigger
  autonomous    — main.py owns the full stack for MODE 3; watchdog skips
                  launch entirely but still handles disarm + post-flight

RC Toggle (Manual Scan Override)
---------------------------------
  Momentary button on CH6 (index 5) starts or stops the flight stack when
  no lock file is present. Has no effect once a mode is locked.

Buzzer Feedback (QBASIC Format = 1)
------------------------------------
  Rising tone   — stack starting
  Falling tone  — stack stopping
  Single beep   — button acknowledged / FCU connected

PX4 RC Channel Setup
---------------------
  Assign a momentary button (e.g., Taranis SH) to CH6 in QGC Radio tab.
  Confirm RC_TOGGLE_CHANNEL matches the 0-indexed channel number below.

Deployment
----------
  sudo systemctl restart drone-watchdog
  sudo journalctl -u drone-watchdog -f
"""

import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State, RCIn, PlayTuneV2

# ── RC Configuration ──────────────────────────────────────────────────────────

RC_TOGGLE_CHANNEL = 5      # 0-indexed: CH6 = index 5
RC_HIGH_THRESHOLD = 1700   # PWM value above which button is considered pressed
RC_LOW_THRESHOLD  = 1300   # PWM value below which button is considered released
RC_DEBOUNCE_MS    = 200    # Minimum ms between toggle events

# ── Buzzer Tunes (QBASIC Format = 1) ─────────────────────────────────────────

TUNE_FORMAT = 1                    # QBASIC1_1 — confirmed working on this firmware
TUNE_START  = "T240L16O5CEGC"     # Rising scale: stack starting
TUNE_STOP   = "T240L16O5CG<EGC"   # Falling scale: stack stopping
TUNE_ACK    = "T200L32O6C"        # Single beep: button acknowledged
TUNE_ERROR  = "T200L16O5CPCPC"    # Three staccato beeps: error

# ── Paths and Configuration ───────────────────────────────────────────────────

ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
ROSBAG_DIR = Path("/mnt/ssd/rosbags")

SCRIPT_DIR         = Path(__file__).parent
SLAM_BRIDGE_SCRIPT = SCRIPT_DIR / "_slam_bridge.py"
POSTFLIGHT_SCRIPT  = SCRIPT_DIR.parent / "utils" / "run_postflight.py"
LAUNCH_FILE        = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
MISSION_LOCK = Path("/tmp/dronepi_mission.lock")

BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
    "/mavros/vision_pose/pose",
]

ENABLE_RC_TOGGLE = True
ENABLE_BUZZER    = True
POLL_HZ          = 10
MONITOR_HZ       = 10
GRACEFUL_KILL_S  = 5
MAVROS_WAIT_S    = 15

# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


# ── Lock File Reader ──────────────────────────────────────────────────────────

def read_lock_mode() -> str:
    """Return the lock mode string, or empty string if no lock is present."""
    if not MISSION_LOCK.exists():
        return ""
    try:
        data = json.loads(MISSION_LOCK.read_text())
        return data.get("mode", "")
    except Exception:
        return ""


# ── Process Management ────────────────────────────────────────────────────────

def start_process(name: str, cmd: str) -> subprocess.Popen:
    log(f"Starting {name}...")
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log(f"{name} started (PID {proc.pid})")
    return proc


def stop_process(name: str, proc: subprocess.Popen) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    log(f"Stopping {name}...")
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly.")
    except subprocess.TimeoutExpired:
        log(f"{name} — sending SIGKILL.")
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


# ── Command Builders ──────────────────────────────────────────────────────────

def _pointlio_cmd() -> str:
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0"
    )


def _slam_bridge_cmd() -> str:
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"python3 {SLAM_BRIDGE_SCRIPT}"
    )


def _bag_record_cmd() -> str:
    ts     = datetime.now().strftime("%Y%m%d_%H%M%S")
    out    = ROSBAG_DIR / f"scan_{ts}"
    topics = " ".join(BAG_TOPICS)
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 bag record -o {out} {topics}"
    )


# ── MAVROS Interface ──────────────────────────────────────────────────────────

class MavrosReader:
    """ROS 2 interface for state monitoring, RC input, and buzzer output."""

    def __init__(self):
        self._armed        = False
        self._mode         = ""
        self._connected    = False
        self._rc_channels  = []
        self._lock         = threading.Lock()

        # RC toggle edge detection
        self._button_was_high  = False
        self._last_toggle_time = 0.0
        self._toggle_pending   = False

        rclpy.init()
        self._node = Node("drone_watchdog")

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(
            State, "/mavros/state", self._state_cb, state_qos)
        self._node.create_subscription(
            RCIn, "/mavros/rc/in", self._rc_cb, sensor_qos)

        self._tune_pub = self._node.create_publisher(
            PlayTuneV2, "/mavros/play_tune", 10)

        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()

        log("MAVROS interface started")
        log(f"  RC toggle: CH{RC_TOGGLE_CHANNEL + 1} (index {RC_TOGGLE_CHANNEL})")
        log(f"  Buzzer: {'ENABLED (QBASIC)' if ENABLE_BUZZER else 'DISABLED'}")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        with self._lock:
            self._armed     = msg.armed
            self._mode      = msg.mode
            self._connected = msg.connected

    def _rc_cb(self, msg: RCIn) -> None:
        with self._lock:
            self._rc_channels = list(msg.channels)
            if len(self._rc_channels) <= RC_TOGGLE_CHANNEL:
                return

            ch_value       = self._rc_channels[RC_TOGGLE_CHANNEL]
            now            = time.time()
            button_is_high = ch_value > RC_HIGH_THRESHOLD
            button_is_low  = ch_value < RC_LOW_THRESHOLD

            # Rising-edge detection with debounce
            if button_is_high and not self._button_was_high:
                if (now - self._last_toggle_time) > (RC_DEBOUNCE_MS / 1000.0):
                    self._toggle_pending   = True
                    self._last_toggle_time = now

            if button_is_low:
                self._button_was_high = False
            elif button_is_high:
                self._button_was_high = True

    # ── Buzzer ────────────────────────────────────────────────────────────────

    def play_tune(self, tune: str) -> None:
        if not ENABLE_BUZZER:
            return
        msg        = PlayTuneV2()
        msg.format = TUNE_FORMAT
        msg.tune   = tune
        self._tune_pub.publish(msg)

    def beep_start(self):   self.play_tune(TUNE_START)
    def beep_stop(self):    self.play_tune(TUNE_STOP)
    def beep_ack(self):     self.play_tune(TUNE_ACK)
    def beep_error(self):   self.play_tune(TUNE_ERROR)

    # ── State Properties ──────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        with self._lock: return self._armed

    @property
    def mode(self) -> str:
        with self._lock: return self._mode

    @property
    def connected(self) -> bool:
        with self._lock: return self._connected

    def get_rc_channel(self, channel: int) -> int:
        with self._lock:
            return self._rc_channels[channel] if channel < len(self._rc_channels) else 0

    def check_toggle_pressed(self) -> bool:
        with self._lock:
            if self._toggle_pending:
                self._toggle_pending = False
                return True
            return False

    def shutdown(self) -> None:
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ── Flight Stack Manager ──────────────────────────────────────────────────────

class FlightStack:
    """Manages Point-LIO, SLAM bridge, and bag recorder subprocesses."""

    def __init__(self, reader: MavrosReader):
        self._reader          = reader
        self._pointlio_proc   = None
        self._bridge_proc     = None
        self._bag_proc        = None
        self.is_running       = False
        self.session_count    = 0

    def start(self, mode: str) -> None:
        if self.is_running:
            log("[WARN] Stack already running — ignoring start request")
            return

        self.session_count += 1
        log(f"[ACTIVATING] Session #{self.session_count} ({mode})")
        self._reader.beep_start()

        self._pointlio_proc = start_process("Point-LIO", _pointlio_cmd())
        time.sleep(3.0)

        if SLAM_BRIDGE_SCRIPT.exists():
            self._bridge_proc = start_process("SLAM bridge", _slam_bridge_cmd())
            time.sleep(1.0)
        else:
            log("[WARN] SLAM bridge script not found — vision fusion disabled")

        self._bag_proc  = start_process("Bag recorder", _bag_record_cmd())
        self.is_running = True
        log(f"[ACTIVE] Session #{self.session_count} — scanning")

    def stop(self) -> None:
        if not self.is_running:
            return

        log(f"[DEACTIVATING] Session #{self.session_count}")
        self._reader.beep_stop()

        # Stop in reverse launch order to ensure clean bag finalisation
        stop_process("Bag recorder", self._bag_proc)
        self._bag_proc = None

        stop_process("SLAM bridge", self._bridge_proc)
        self._bridge_proc = None

        stop_process("Point-LIO", self._pointlio_proc)
        self._pointlio_proc = None

        self.is_running = False
        log(f"[WAITING] Session #{self.session_count} complete")

        _trigger_postflight()

    def check_health(self) -> None:
        """Log warnings if any managed process has exited unexpectedly."""
        if self._pointlio_proc and self._pointlio_proc.poll() is not None:
            log("[WARN] Point-LIO exited unexpectedly")
            self._pointlio_proc = None
        if self._bridge_proc and self._bridge_proc.poll() is not None:
            log("[WARN] SLAM bridge exited unexpectedly")
            self._bridge_proc = None
        if self._bag_proc and self._bag_proc.poll() is not None:
            log("[WARN] Bag recorder exited unexpectedly")
            self._bag_proc = None


# ── Post-Flight Trigger ───────────────────────────────────────────────────────

def _trigger_postflight() -> None:
    """Launch run_postflight.py in the background after each session."""
    if not POSTFLIGHT_SCRIPT.exists():
        log(f"[WARN] Post-flight script not found: {POSTFLIGHT_SCRIPT}")
        return
    log("Launching post-flight processing...")
    try:
        subprocess.Popen(
            [sys.executable, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as exc:
        log(f"[WARN] Post-flight launch failed: {exc}")


# ── Startup Helper ────────────────────────────────────────────────────────────

def _wait_for_mavros(reader: MavrosReader) -> None:
    log(f"Waiting for MAVROS connection (up to {MAVROS_WAIT_S}s)...")
    deadline = time.time() + MAVROS_WAIT_S
    while time.time() < deadline:
        if reader.connected:
            log(f"FCU connected.  Mode: {reader.mode}  Armed: {reader.armed}")
            time.sleep(0.5)
            reader.beep_ack()
            return
        time.sleep(1.0)
    log("[WARN] FCU not connected — continuing anyway")


# ── Main Loop ─────────────────────────────────────────────────────────────────

def main() -> None:
    log("=" * 60)
    log("DronePi Flight Stack Watchdog")
    log(f"  RC Toggle: CH{RC_TOGGLE_CHANNEL + 1}")
    log(f"  Buzzer:    {'ENABLED (QBASIC)' if ENABLE_BUZZER else 'DISABLED'}")
    log("=" * 60)

    ROSBAG_DIR.mkdir(parents=True, exist_ok=True)

    try:
        reader = MavrosReader()
    except Exception as exc:
        log(f"[FAIL] ROS 2 init failed: {exc}")
        sys.exit(1)

    _wait_for_mavros(reader)

    stack = FlightStack(reader)

    try:
        while True:
            lock_mode = read_lock_mode()

            # ── MODE 3 (autonomous) — main.py owns the stack ──────────────
            # Watchdog steps aside entirely; main.py manages all processes.
            if lock_mode == "autonomous":
                if stack.is_running:
                    log("Lock mode switched to autonomous — stopping watchdog stack")
                    stack.stop()
                log("[WATCHDOG] autonomous lock active — yielding to main.py")
                time.sleep(1.0 / POLL_HZ)
                continue

            # ── MODE 2 (manual_scan) — watchdog owns the stack ────────────
            if lock_mode == "manual_scan":
                if not stack.is_running:
                    log("[WATCHDOG] manual_scan lock detected — starting stack")
                    stack.start("MANUAL_LOCK")

                stack.check_health()

                # Wait for disarm then trigger post-flight and clear lock
                if not reader.armed:
                    log("[WATCHDOG] Disarmed — stopping stack")
                    stack.stop()
                    if MISSION_LOCK.exists():
                        MISSION_LOCK.unlink()
                        log("Lock cleared")

                time.sleep(1.0 / MONITOR_HZ)
                continue

            # ── No lock — standard RC toggle operation ────────────────────
            if not stack.is_running:
                if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                    log("RC button pressed — starting stack")
                    reader.beep_ack()
                    stack.start("MANUAL_RC")
                else:
                    ch_val = reader.get_rc_channel(RC_TOGGLE_CHANNEL)
                    log(f"[WAITING] armed={reader.armed}  "
                        f"mode={reader.mode or '?'}  "
                        f"CH{RC_TOGGLE_CHANNEL + 1}={ch_val}")
                    time.sleep(1.0 / POLL_HZ)
            else:
                stack.check_health()
                if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                    log("RC button pressed — stopping stack")
                    reader.beep_ack()
                    stack.stop()
                else:
                    time.sleep(1.0 / MONITOR_HZ)

    except KeyboardInterrupt:
        log("Interrupted — shutting down")
    finally:
        if stack.is_running:
            stack.stop()
        reader.shutdown()
        log("Watchdog stopped.")


if __name__ == "__main__":
    main()
