#!/usr/bin/env python3
"""drone_watchdog.py — Flight stack supervisor for DronePi.

Monitors /mavros/state and manages the Point-LIO SLAM node, SLAM bridge,
and ROS 2 bag recorder based on the drone's armed and flight mode state.

STATE MACHINE
-------------
  WAITING
      Polls /mavros/state every second.
      Condition to advance: armed=True AND mode=OFFBOARD.
      When condition met -> launches Point-LIO + SLAM bridge + bag -> ACTIVE.

  ACTIVE
      Monitors /mavros/state continuously.
      Condition to revert: armed=False OR mode != OFFBOARD.
      When condition lost -> stops all processes -> WAITING.
      A new session starts automatically if the pilot re-arms in OFFBOARD.

SAFETY DESIGN
-------------
  - This script NEVER arms the drone or switches flight modes.
    Arming and mode selection are exclusively the pilot's responsibility
    via the RC transmitter. The watchdog is observe-only on those topics.
  - Point-LIO is killed with SIGINT (clean shutdown) first, then SIGKILL
    after a 5-second grace period if it does not exit.
  - The bag recorder is stopped with SIGINT so the MCAP file is closed
    correctly. An unclean bag close corrupts the recording.
  - All state transitions are logged to stdout (captured by journalctl).

SLAM BRIDGE
-----------
  The SLAM bridge (_slam_bridge.py) is launched alongside Point-LIO.
  It forwards Point-LIO poses to /mavros/vision_pose/pose for EKF fusion.
  Requires EKF2_EV_CTRL=15 in PX4 parameters.

DEPENDENCIES
------------
  rclpy, mavros_msgs (ROS 2 Jazzy)
  flight_controller module (for clean state subscription)

DEPLOYMENT
----------
  Managed by drone-watchdog.service (systemd).
  See setup_flight_services.sh for installation.

  Manual run (for testing):
      source /opt/ros/jazzy/setup.bash
      source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
      python3 drone_watchdog.py

  View logs:
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
from mavros_msgs.msg import State

# ── Config ────────────────────────────────────────────────────────────────────

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
ROSBAG_DIR = "/mnt/ssd/rosbags"

# Path to scripts (same directory as this file)
SCRIPT_DIR = Path(__file__).parent
SLAM_BRIDGE_SCRIPT = SCRIPT_DIR / "_slam_bridge.py"
POSTFLIGHT_SCRIPT = SCRIPT_DIR / "run_postflight.py"

# Point-LIO launch
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)

# Activation mode
# REQUIRE_OFFBOARD = True  -- field use: armed AND mode=OFFBOARD
# REQUIRE_OFFBOARD = False -- bench use: armed only
REQUIRE_OFFBOARD = True

POLL_HZ = 2           # state poll rate while WAITING
MONITOR_HZ = 5        # state monitor rate while ACTIVE
GRACEFUL_KILL_S = 5   # seconds before SIGKILL after SIGINT
MAVROS_WAIT_S = 15    # seconds to wait for MAVROS on boot

# Topics recorded in every bag session
BAG_TOPICS = [
    "/cloud_registered",
    "/aft_mapped_to_init",
    "/unilidar/imu",
    "/mavros/state",
    "/mavros/local_position/pose",
    "/mavros/global_position/global",
    "/mavros/vision_pose/pose",  # SLAM bridge output for debugging
]

# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str):
    """Timestamped log line — captured by journalctl."""
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def log_state(state: str, detail: str = ""):
    sep = f"  {detail}" if detail else ""
    log(f"[{state}]{sep}")


# ── Process Management ────────────────────────────────────────────────────────

def start_process(name: str, cmd: str) -> subprocess.Popen:
    """Launch a shell command in a new process group."""
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
    """Stop a process cleanly with SIGINT, then SIGKILL if needed."""
    if proc is None or proc.poll() is not None:
        return

    pgid = None
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return

    log(f"Stopping {name} (PID {proc.pid}, PGID {pgid})...")

    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return

    try:
        proc.wait(timeout=GRACEFUL_KILL_S)
        log(f"{name} stopped cleanly.")
    except subprocess.TimeoutExpired:
        log(f"{name} did not exit in {GRACEFUL_KILL_S}s — sending SIGKILL.")
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()
        log(f"{name} killed.")


# ── Command Builders ──────────────────────────────────────────────────────────

def pointlio_cmd() -> str:
    """Build Point-LIO launch command."""
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:=false port:=/dev/ttyUSB0"
    )


def slam_bridge_cmd() -> str:
    """Build SLAM bridge launch command."""
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"python3 {SLAM_BRIDGE_SCRIPT}"
    )


def bag_record_cmd() -> str:
    """Build ros2 bag record command for current session."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = f"{ROSBAG_DIR}/scan_{ts}"
    topics = " ".join(BAG_TOPICS)
    return (
        f"source {ROS_SETUP} && "
        f"source {WS_SETUP} && "
        f"ros2 bag record -o {out} {topics}"
    )


# ── MAVROS State Reader ───────────────────────────────────────────────────────

class MavrosStateReader:
    """Lightweight ROS 2 subscriber for /mavros/state."""

    def __init__(self):
        self._armed = False
        self._mode = ""
        self._connected = False
        self._lock = threading.Lock()

        rclpy.init()
        self._node = Node("drone_watchdog")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._node.create_subscription(
            State, "/mavros/state", self._cb, qos)

        self._thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()
        log("MAVROS state subscriber started.")

    def _cb(self, msg):
        with self._lock:
            self._armed = msg.armed
            self._mode = msg.mode
            self._connected = msg.connected

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

    def flight_conditions_met(self) -> bool:
        """Return True when activation conditions are met."""
        with self._lock:
            if REQUIRE_OFFBOARD:
                return self._armed and self._mode == "OFFBOARD"
            else:
                return self._armed

    def shutdown(self):
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ── Post-flight Trigger ───────────────────────────────────────────────────────

def trigger_postflight() -> subprocess.Popen | None:
    """Launch run_postflight.py in background after session ends."""
    if not POSTFLIGHT_SCRIPT.exists():
        log(f"[WARN] run_postflight.py not found at {POSTFLIGHT_SCRIPT}")
        return None

    log("Launching post-flight pipeline in background...")
    try:
        proc = subprocess.Popen(
            [sys.executable, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        log(f"Post-flight pipeline started (PID {proc.pid})")
        return proc
    except Exception as e:
        log(f"[WARN] Could not launch post-flight: {e}")
        return None


# ── Main Watchdog Loop ────────────────────────────────────────────────────────

def wait_for_mavros(reader: MavrosStateReader):
    """Block until MAVROS reports connected FCU."""
    log(f"Waiting for MAVROS FCU connection (up to {MAVROS_WAIT_S}s)...")
    deadline = time.time() + MAVROS_WAIT_S
    while time.time() < deadline:
        if reader.connected:
            log(f"FCU connected. Mode: {reader.mode}  Armed: {reader.armed}")
            return
        time.sleep(1.0)
    log("[WARN] FCU not connected after wait — continuing anyway.")


def main():
    log("=" * 50)
    log("DronePi Flight Stack Watchdog")
    if REQUIRE_OFFBOARD:
        log("Activation: armed=True AND mode=OFFBOARD (field mode)")
    else:
        log("Activation: armed=True only (bench mode)")
    log("This script never arms or changes flight mode.")
    log("=" * 50)

    # Ensure rosbag directory exists
    Path(ROSBAG_DIR).mkdir(parents=True, exist_ok=True)

    # Check SLAM bridge exists
    if not SLAM_BRIDGE_SCRIPT.exists():
        log(f"[WARN] SLAM bridge not found: {SLAM_BRIDGE_SCRIPT}")
        log("       Vision pose fusion will not be available.")

    try:
        reader = MavrosStateReader()
    except Exception as e:
        log(f"[FAIL] Could not start ROS 2 node: {e}")
        sys.exit(1)

    wait_for_mavros(reader)

    pointlio_proc = None
    bridge_proc = None
    bag_proc = None
    postflight_proc = None
    session_count = 0

    try:
        while True:
            # Block new session if post-flight still running
            if postflight_proc is not None:
                rc = postflight_proc.poll()
                if rc is None:
                    log_state("WAITING",
                              f"armed={reader.armed}  mode={reader.mode or '?'}  "
                              f"[POST-PROCESSING]")
                    time.sleep(1.0 / POLL_HZ)
                    continue
                else:
                    if rc == 0:
                        log("Post-flight complete. New session allowed.")
                    else:
                        log(f"[WARN] Post-flight exited with code {rc}")
                    postflight_proc = None

            # WAITING state
            if not reader.flight_conditions_met():
                log_state("WAITING",
                          f"armed={reader.armed}  mode={reader.mode or '?'}")
                time.sleep(1.0 / POLL_HZ)
                continue

            # Conditions met — launch flight stack
            session_count += 1
            log_state("ACTIVATING",
                      f"Session #{session_count}  "
                      f"armed={reader.armed}  mode={reader.mode}")

            # Start Point-LIO
            pointlio_proc = start_process("Point-LIO", pointlio_cmd())
            time.sleep(3.0)  # Wait for Point-LIO to initialize

            # Start SLAM bridge (if available)
            if SLAM_BRIDGE_SCRIPT.exists():
                bridge_proc = start_process("SLAM bridge", slam_bridge_cmd())
                time.sleep(1.0)
            else:
                bridge_proc = None

            # Start bag recorder
            bag_proc = start_process("Bag recorder", bag_record_cmd())

            log_state("ACTIVE",
                      f"Session #{session_count} running — "
                      f"monitoring for disarm or mode change")

            # ACTIVE state — monitor continuously
            while True:
                time.sleep(1.0 / MONITOR_HZ)

                # Check subprocess health
                if pointlio_proc and pointlio_proc.poll() is not None:
                    log("[WARN] Point-LIO exited unexpectedly.")
                    pointlio_proc = None

                if bridge_proc and bridge_proc.poll() is not None:
                    log("[WARN] SLAM bridge exited unexpectedly.")
                    bridge_proc = None

                if bag_proc and bag_proc.poll() is not None:
                    log("[WARN] Bag recorder exited unexpectedly.")
                    bag_proc = None

                # Check flight conditions
                if not reader.flight_conditions_met():
                    log_state("DEACTIVATING",
                              f"armed={reader.armed}  mode={reader.mode}")
                    break

            # Conditions lost — stop flight stack (order matters for clean bag close)
            stop_process("Bag recorder", bag_proc)
            bag_proc = None
            stop_process("SLAM bridge", bridge_proc)
            bridge_proc = None
            stop_process("Point-LIO", pointlio_proc)
            pointlio_proc = None

            # Trigger post-flight
            postflight_proc = trigger_postflight()

            log_state("WAITING",
                      f"Session #{session_count} complete. "
                      f"Re-arm in OFFBOARD to start new session.")

    except KeyboardInterrupt:
        log("Watchdog interrupted — shutting down.")
    finally:
        stop_process("Bag recorder", bag_proc)
        stop_process("SLAM bridge", bridge_proc)
        stop_process("Point-LIO", pointlio_proc)
        reader.shutdown()
        log("Watchdog stopped.")


if __name__ == "__main__":
    main()
