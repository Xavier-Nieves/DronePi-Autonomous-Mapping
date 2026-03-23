#!/usr/bin/env python3
"""drone_watchdog.py — Flight stack supervisor with RC toggle and buzzer feedback.

FEATURES
--------
  1. AUTO MODE: armed + OFFBOARD → auto-start stack
  2. MANUAL MODE: RC momentary button → toggle stack on/off
  3. BUZZER FEEDBACK: Pixhawk beeps on start/stop (no external hardware needed)

BUZZER TUNES
------------
  - Stack START: Rising tone (low → high) ♪♪♪
  - Stack STOP:  Falling tone (high → low) ♪♪♪
  - Button ACK:  Single short beep ♪

The buzzer is the one built into your Pixhawk 6X. Tunes are sent via
MAVROS using the PLAY_TUNE_V2 MAVLink message with MML (Music Macro Language).

MML QUICK REFERENCE
-------------------
  T<tempo>  - Set tempo (T120 = 120 BPM)
  L<len>    - Default note length (L8 = eighth note)
  O<oct>    - Octave (O4 = middle, O5 = high, O3 = low)
  C D E F G A B - Notes
  R         - Rest
  < >       - Octave down/up

RC CHANNEL CONFIGURATION
------------------------
  1. Taranis: Assign momentary button (e.g., SH) to channel (e.g., CH7)
  2. Script: Set RC_TOGGLE_CHANNEL to match (CH7 = index 6)
  3. Verify in QGC Radio tab

DEPLOYMENT
----------
  sudo systemctl restart drone-watchdog
  sudo journalctl -u drone-watchdog -f
"""

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

# ── RC Toggle Configuration ───────────────────────────────────────────────────

RC_TOGGLE_CHANNEL = 6      # 0-indexed: CH7=6, CH8=7
RC_HIGH_THRESHOLD = 1700   # Button pressed above this
RC_LOW_THRESHOLD = 1300    # Button released below this
RC_DEBOUNCE_MS = 200       # Debounce time

# ── Buzzer Tunes (MML Format) ─────────────────────────────────────────────────

# MML_MODERN format (value 2) for PX4
TUNE_FORMAT = 2  # MML_MODERN

# Rising tone: Stack starting (excited, ascending)
TUNE_START = "T180 L16 O4 C E G > C"

# Falling tone: Stack stopping (calming, descending) 
TUNE_STOP = "T180 L16 O5 C < G E C"

# Single beep: Button acknowledged
TUNE_ACK = "T200 L32 O5 C"

# Error beep: Something went wrong
TUNE_ERROR = "T200 L16 O4 C R C R C"

# ── General Configuration ─────────────────────────────────────────────────────

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
ROSBAG_DIR = "/mnt/ssd/rosbags"

SCRIPT_DIR = Path(__file__).parent
SLAM_BRIDGE_SCRIPT = SCRIPT_DIR / "_slam_bridge.py"
POSTFLIGHT_SCRIPT = SCRIPT_DIR / "run_postflight.py"

LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)

REQUIRE_OFFBOARD = True
ENABLE_AUTO_MODE = True
ENABLE_RC_TOGGLE = True
ENABLE_BUZZER = True       # Set False to disable buzzer feedback

POLL_HZ = 10
MONITOR_HZ = 10
GRACEFUL_KILL_S = 5
MAVROS_WAIT_S = 15

BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
    "/mavros/vision_pose/pose",
]

# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def log_state(state: str, detail: str = ""):
    sep = f"  {detail}" if detail else ""
    log(f"[{state}]{sep}")


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


def stop_process(name: str, proc: subprocess.Popen):
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

def pointlio_cmd() -> str:
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0"
    )


def slam_bridge_cmd() -> str:
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"python3 {SLAM_BRIDGE_SCRIPT}"
    )


def bag_record_cmd() -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = f"{ROSBAG_DIR}/scan_{ts}"
    topics = " ".join(BAG_TOPICS)
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 bag record -o {out} {topics}"
    )


# ── MAVROS Reader with Buzzer ─────────────────────────────────────────────────

class MavrosReader:
    """ROS 2 interface for state, RC input, and buzzer output."""

    def __init__(self):
        self._armed = False
        self._mode = ""
        self._connected = False
        self._rc_channels = []
        self._lock = threading.Lock()

        # RC toggle state
        self._button_was_high = False
        self._last_toggle_time = 0.0
        self._toggle_pending = False

        rclpy.init()
        self._node = Node("drone_watchdog")

        # Subscriptions
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

        # Buzzer publisher
        self._tune_pub = self._node.create_publisher(
            PlayTuneV2, "/mavros/play_tune", 10)

        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()

        log("MAVROS interface started")
        log(f"  RC toggle: CH{RC_TOGGLE_CHANNEL + 1}")
        log(f"  Buzzer feedback: {'ENABLED' if ENABLE_BUZZER else 'DISABLED'}")

    def _state_cb(self, msg: State):
        with self._lock:
            self._armed = msg.armed
            self._mode = msg.mode
            self._connected = msg.connected

    def _rc_cb(self, msg: RCIn):
        with self._lock:
            self._rc_channels = list(msg.channels)

            if len(self._rc_channels) > RC_TOGGLE_CHANNEL:
                ch_value = self._rc_channels[RC_TOGGLE_CHANNEL]
                now = time.time()

                button_is_high = ch_value > RC_HIGH_THRESHOLD
                button_is_low = ch_value < RC_LOW_THRESHOLD

                # Rising edge detection
                if button_is_high and not self._button_was_high:
                    if (now - self._last_toggle_time) > (RC_DEBOUNCE_MS / 1000.0):
                        self._toggle_pending = True
                        self._last_toggle_time = now

                if button_is_high:
                    self._button_was_high = True
                elif button_is_low:
                    self._button_was_high = False

    # ── Buzzer Control ────────────────────────────────────────────────────────

    def play_tune(self, tune: str):
        """Play a tune on the Pixhawk buzzer using MML format."""
        if not ENABLE_BUZZER:
            return

        msg = PlayTuneV2()
        msg.format = TUNE_FORMAT  # MML_MODERN = 2
        msg.tune = tune

        self._tune_pub.publish(msg)
        log(f"  ♪ Buzzer: {tune[:20]}...")

    def beep_start(self):
        """Rising tone indicating stack is starting."""
        self.play_tune(TUNE_START)

    def beep_stop(self):
        """Falling tone indicating stack is stopping."""
        self.play_tune(TUNE_STOP)

    def beep_ack(self):
        """Short beep acknowledging button press."""
        self.play_tune(TUNE_ACK)

    def beep_error(self):
        """Error beep pattern."""
        self.play_tune(TUNE_ERROR)

    # ── State Properties ──────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def get_rc_channel(self, channel: int) -> int:
        with self._lock:
            if channel < len(self._rc_channels):
                return self._rc_channels[channel]
            return 0

    def check_toggle_pressed(self) -> bool:
        with self._lock:
            if self._toggle_pending:
                self._toggle_pending = False
                return True
            return False

    def auto_conditions_met(self) -> bool:
        if not ENABLE_AUTO_MODE:
            return False
        with self._lock:
            if REQUIRE_OFFBOARD:
                return self._armed and self._mode == "OFFBOARD"
            else:
                return self._armed

    def auto_conditions_lost(self) -> bool:
        with self._lock:
            if REQUIRE_OFFBOARD:
                return not self._armed or self._mode != "OFFBOARD"
            else:
                return not self._armed

    def shutdown(self):
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ── Flight Stack Manager ──────────────────────────────────────────────────────

class FlightStack:
    """Manages Point-LIO, SLAM bridge, and bag recorder."""

    def __init__(self, reader: MavrosReader):
        self.reader = reader
        self.pointlio_proc = None
        self.bridge_proc = None
        self.bag_proc = None
        self.is_running = False
        self.session_count = 0
        self.activation_mode = None

    def start(self, mode: str):
        if self.is_running:
            log("[WARN] Stack already running")
            return

        self.session_count += 1
        self.activation_mode = mode
        log_state("ACTIVATING", f"Session #{self.session_count} ({mode})")

        # Buzzer: rising tone
        self.reader.beep_start()

        # Start Point-LIO
        self.pointlio_proc = start_process("Point-LIO", pointlio_cmd())
        time.sleep(3.0)

        # Start SLAM bridge
        if SLAM_BRIDGE_SCRIPT.exists():
            self.bridge_proc = start_process("SLAM bridge", slam_bridge_cmd())
            time.sleep(1.0)

        # Start bag recorder
        self.bag_proc = start_process("Bag recorder", bag_record_cmd())

        self.is_running = True
        log_state("ACTIVE", f"Session #{self.session_count} — scanning")

    def stop(self):
        if not self.is_running:
            return

        log_state("DEACTIVATING", f"Session #{self.session_count}")

        # Buzzer: falling tone
        self.reader.beep_stop()

        # Stop in reverse order
        stop_process("Bag recorder", self.bag_proc)
        self.bag_proc = None

        stop_process("SLAM bridge", self.bridge_proc)
        self.bridge_proc = None

        stop_process("Point-LIO", self.pointlio_proc)
        self.pointlio_proc = None

        self.is_running = False
        self.activation_mode = None

        log_state("WAITING", f"Session #{self.session_count} complete")

        # Trigger post-flight
        trigger_postflight()

    def check_health(self):
        if self.pointlio_proc and self.pointlio_proc.poll() is not None:
            log("[WARN] Point-LIO exited unexpectedly")
            self.pointlio_proc = None

        if self.bridge_proc and self.bridge_proc.poll() is not None:
            log("[WARN] SLAM bridge exited")
            self.bridge_proc = None

        if self.bag_proc and self.bag_proc.poll() is not None:
            log("[WARN] Bag recorder exited")
            self.bag_proc = None


# ── Post-flight ───────────────────────────────────────────────────────────────

def trigger_postflight() -> subprocess.Popen | None:
    if not POSTFLIGHT_SCRIPT.exists():
        return None

    log("Launching post-flight processing...")
    try:
        return subprocess.Popen(
            [sys.executable, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        log(f"[WARN] Post-flight failed: {e}")
        return None


# ── Main Loop ─────────────────────────────────────────────────────────────────

def wait_for_mavros(reader: MavrosReader):
    log(f"Waiting for MAVROS connection (up to {MAVROS_WAIT_S}s)...")
    deadline = time.time() + MAVROS_WAIT_S
    while time.time() < deadline:
        if reader.connected:
            log(f"FCU connected. Mode: {reader.mode}  Armed: {reader.armed}")
            # Welcome beep
            reader.beep_ack()
            return
        time.sleep(1.0)
    log("[WARN] FCU not connected — continuing anyway")


def main():
    log("=" * 55)
    log("DronePi Flight Stack Watchdog")
    log("  RC Toggle + Buzzer Feedback")
    log("=" * 55)

    if ENABLE_AUTO_MODE:
        mode_str = "armed + OFFBOARD" if REQUIRE_OFFBOARD else "armed"
        log(f"  AUTO: {mode_str} → start")

    if ENABLE_RC_TOGGLE:
        log(f"  MANUAL: CH{RC_TOGGLE_CHANNEL + 1} button → toggle")

    if ENABLE_BUZZER:
        log("  BUZZER: Pixhawk beeps on start/stop")

    log("=" * 55)

    Path(ROSBAG_DIR).mkdir(parents=True, exist_ok=True)

    try:
        reader = MavrosReader()
    except Exception as e:
        log(f"[FAIL] ROS 2 init failed: {e}")
        sys.exit(1)

    wait_for_mavros(reader)

    stack = FlightStack(reader)
    postflight_proc = None

    try:
        while True:
            # Block during post-flight processing
            if postflight_proc is not None:
                rc = postflight_proc.poll()
                if rc is None:
                    time.sleep(1.0 / POLL_HZ)
                    continue
                postflight_proc = None

            # ── WAITING ───────────────────────────────────────────────
            if not stack.is_running:
                # Check RC toggle
                if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                    log("RC button pressed — starting")
                    reader.beep_ack()
                    stack.start("MANUAL")
                    continue

                # Check AUTO
                if ENABLE_AUTO_MODE and reader.auto_conditions_met():
                    stack.start("AUTO")
                    continue

                # Status display
                ch_val = reader.get_rc_channel(RC_TOGGLE_CHANNEL) if ENABLE_RC_TOGGLE else 0
                log_state("WAITING",
                          f"armed={reader.armed}  mode={reader.mode or '?'}  "
                          f"CH{RC_TOGGLE_CHANNEL+1}={ch_val}")
                time.sleep(1.0 / POLL_HZ)
                continue

            # ── ACTIVE ────────────────────────────────────────────────
            stack.check_health()

            # RC toggle to stop
            if ENABLE_RC_TOGGLE and reader.check_toggle_pressed():
                log("RC button pressed — stopping")
                reader.beep_ack()
                stack.stop()
                continue

            # AUTO deactivation
            if stack.activation_mode == "AUTO" and reader.auto_conditions_lost():
                log(f"AUTO conditions lost (armed={reader.armed}, mode={reader.mode})")
                stack.stop()
                continue

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
