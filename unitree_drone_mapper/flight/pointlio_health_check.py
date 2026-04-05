#!/usr/bin/env python3
"""
pointlio_health_check.py
------------------------
Wrapper around Point-LIO that monitors /aft_mapped_to_init publication
rate and exits non-zero if SLAM output goes silent.

PURPOSE
-------
Point-LIO does not self-exit when the LiDAR serial port opens but produces
no sensor data. It starts, initialises its internal state, and then waits
indefinitely for the first scan. This means systemd has no signal that the
stack is unhealthy — from systemd's perspective the process is running fine.

This wrapper launches Point-LIO as a subprocess and runs a parallel ROS 2
subscriber that verifies /aft_mapped_to_init is publishing at an acceptable
rate. If the topic is absent or silent beyond the configured timeouts, the
wrapper terminates Point-LIO and exits with code 1, which triggers systemd's
Restart=on-failure to relaunch the whole stack.

TIMEOUT LOGIC
-------------
FIRST_SCAN_TIMEOUT_S (90s):
    How long to wait for the very first /aft_mapped_to_init message after
    Point-LIO starts. IMU initialisation takes ~2s, map building takes a
    few more seconds after the first scan arrives. 90 seconds is generous
    enough to cover slow IMU convergence without false-alarming on a healthy
    startup.

SILENCE_TIMEOUT_S (15s):
    Once publishing has started, how long a gap in messages triggers a
    restart. 15 seconds covers transient LiDAR communication glitches
    without being so short that normal processing pauses cause false alarms.

EXIT CODES
----------
    0   Point-LIO exited cleanly (SIGINT from systemd stop)
    1   SLAM topic timeout — systemd will restart via Restart=on-failure
    2   Point-LIO subprocess failed to launch

ARCHITECTURE
------------
This script is a standalone executable, not a production module.
It is the ExecStart target in pointlio-standby.service. It does not
import anything from the flight stack — it is intentionally isolated
so it can be tested and debugged independently.

USAGE (manual test)
-------------------
    source /opt/ros/jazzy/setup.bash
    source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
    python3 pointlio_health_check.py

    # Override timeouts for bench testing:
    POINTLIO_FIRST_SCAN_TIMEOUT=30 python3 pointlio_health_check.py
"""

import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── Configuration ─────────────────────────────────────────────────────────────
# All values are overridable via environment variables for bench testing.

ROS_SETUP   = "/opt/ros/jazzy/setup.bash"
WS_SETUP    = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
LIDAR_PORT = "/dev/ttyUSB0"

FIRST_SCAN_TIMEOUT_S = float(os.environ.get("POINTLIO_FIRST_SCAN_TIMEOUT", "90"))
SILENCE_TIMEOUT_S    = float(os.environ.get("POINTLIO_SILENCE_TIMEOUT",    "15"))
CHECK_INTERVAL_S     = 2.0    # How often the health loop evaluates topic state
SLAM_TOPIC           = "/aft_mapped_to_init"

# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] [pointlio_health] {msg}", flush=True)


# ── Topic Monitor ─────────────────────────────────────────────────────────────

class SlamTopicMonitor:
    """
    Subscribes to /aft_mapped_to_init and tracks the timestamp of the last
    received message. Thread-safe. Does not spin its own executor — the
    caller provides a rclpy node and spin thread.

    Attributes
    ----------
    first_message_time : float or None
        monotonic time of the first received message, None if not yet received.
    last_message_time : float or None
        monotonic time of the most recent received message.
    message_count : int
        Total messages received since construction.
    """

    def __init__(self, node) -> None:
        self._lock             = threading.Lock()
        self.first_message_time = None
        self.last_message_time  = None
        self.message_count      = 0

        from nav_msgs.msg import Odometry
        from rclpy.qos import (
            QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        node.create_subscription(Odometry, SLAM_TOPIC, self._callback, qos)

    def _callback(self, msg) -> None:
        now = time.monotonic()
        with self._lock:
            if self.first_message_time is None:
                self.first_message_time = now
            self.last_message_time = now
            self.message_count += 1

    def seconds_since_last(self) -> float:
        """Seconds since the last message. Returns inf if never received."""
        with self._lock:
            if self.last_message_time is None:
                return float("inf")
            return time.monotonic() - self.last_message_time

    def has_started(self) -> bool:
        """True if at least one message has been received."""
        with self._lock:
            return self.first_message_time is not None


# ── Point-LIO Process Manager ─────────────────────────────────────────────────

class PointLIOProcess:
    """
    Launches and manages the Point-LIO ROS 2 launch subprocess.

    Uses os.setsid() so the entire process group can be killed cleanly
    with SIGINT to allow Point-LIO to flush its map before exiting.
    """

    def __init__(self) -> None:
        self._proc: subprocess.Popen | None = None

    def start(self) -> bool:
        """
        Launch Point-LIO. Returns True if the subprocess started successfully.
        Returns False if the launch file is missing or subprocess.Popen fails.
        """
        launch = Path(os.path.expanduser(LAUNCH_FILE))
        if not launch.exists():
            log(f"[FAIL] Launch file not found: {LAUNCH_FILE}")
            return False

        cmd = (
            f"source {ROS_SETUP} && "
            f"source {WS_SETUP} && "
            f"ros2 launch {LAUNCH_FILE} rviz:=false port:={LIDAR_PORT}"
        )
        log(f"Launching Point-LIO: port={LIDAR_PORT}")
        try:
            self._proc = subprocess.Popen(
                ["bash", "-c", cmd],
                preexec_fn=os.setsid,
                # stdout/stderr flow to journald via systemd service capture
            )
            log(f"Point-LIO started (PID {self._proc.pid})")
            return True
        except Exception as exc:
            log(f"[FAIL] subprocess.Popen failed: {exc}")
            return False

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def stop(self, label: str = "stop") -> None:
        """
        Send SIGINT to the process group for a clean shutdown.
        Escalates to SIGKILL after 10 seconds if the process does not exit.
        Point-LIO flushes its map on SIGINT — do not skip straight to SIGKILL.
        """
        if self._proc is None or self._proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(self._proc.pid)
            log(f"Sending SIGINT to Point-LIO process group (reason: {label})")
            os.killpg(pgid, signal.SIGINT)
            try:
                self._proc.wait(timeout=10)
                log("Point-LIO exited cleanly")
            except subprocess.TimeoutExpired:
                log("Point-LIO did not exit within 10s — sending SIGKILL")
                os.killpg(pgid, signal.SIGKILL)
                self._proc.wait(timeout=3)
        except ProcessLookupError:
            pass
        except Exception as exc:
            log(f"[WARN] Error stopping Point-LIO: {exc}")

    def wait(self) -> int:
        """Block until the subprocess exits. Returns the exit code."""
        if self._proc is None:
            return 0
        return self._proc.wait()


# ── Health Check Loop ─────────────────────────────────────────────────────────

def run_health_loop(
    monitor: SlamTopicMonitor,
    pointlio: PointLIOProcess,
    shutdown_event: threading.Event,
) -> int:
    """
    Monitor /aft_mapped_to_init and terminate Point-LIO if the topic is
    absent or goes silent beyond configured timeouts.

    Returns an exit code:
        0  Clean shutdown (systemd stop signal received)
        1  Topic timeout — restart required
    """
    t_start = time.monotonic()
    logged_waiting = False

    while not shutdown_event.is_set():
        time.sleep(CHECK_INTERVAL_S)

        # If Point-LIO exited on its own, propagate its exit code
        if not pointlio.is_running():
            log("Point-LIO process exited unexpectedly")
            return 1

        elapsed = time.monotonic() - t_start

        if not monitor.has_started():
            # Waiting for first scan
            if not logged_waiting:
                log(
                    f"Waiting for first {SLAM_TOPIC} message "
                    f"(timeout: {FIRST_SCAN_TIMEOUT_S:.0f}s)..."
                )
                logged_waiting = True

            if elapsed > FIRST_SCAN_TIMEOUT_S:
                log(
                    f"[TIMEOUT] No {SLAM_TOPIC} message received after "
                    f"{FIRST_SCAN_TIMEOUT_S:.0f}s — "
                    f"LiDAR may not be producing data. Restarting stack."
                )
                pointlio.stop("first_scan_timeout")
                return 1

            # Periodic progress log every 15 seconds
            if int(elapsed) % 15 == 0 and int(elapsed) > 0:
                log(
                    f"Still waiting for first scan... "
                    f"({elapsed:.0f}s / {FIRST_SCAN_TIMEOUT_S:.0f}s)"
                )

        else:
            # Publishing has started — monitor for silence
            silence = monitor.seconds_since_last()
            if silence > SILENCE_TIMEOUT_S:
                log(
                    f"[TIMEOUT] {SLAM_TOPIC} silent for {silence:.1f}s "
                    f"(threshold: {SILENCE_TIMEOUT_S:.0f}s) — "
                    f"LiDAR connection may have dropped. Restarting stack."
                )
                pointlio.stop("silence_timeout")
                return 1

            # Periodic health log every 30 seconds
            if int(elapsed) % 30 == 0 and int(elapsed) > 0:
                log(
                    f"SLAM healthy — {monitor.message_count} messages, "
                    f"last: {silence:.1f}s ago"
                )

    # Shutdown event was set (systemd SIGINT/SIGTERM)
    return 0


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> int:
    log("=" * 55)
    log("Point-LIO Health Check Wrapper")
    log(f"  First scan timeout : {FIRST_SCAN_TIMEOUT_S:.0f}s")
    log(f"  Silence timeout    : {SILENCE_TIMEOUT_S:.0f}s")
    log(f"  SLAM topic         : {SLAM_TOPIC}")
    log("=" * 55)

    # ── Validate device ───────────────────────────────────────────────────────
    if not Path(LIDAR_PORT).exists():
        log(f"[WARN] {LIDAR_PORT} not present at startup — "
            f"Point-LIO will launch and wait. "
            f"First-scan timeout will trigger restart if LiDAR never connects.")

    # ── Launch Point-LIO ──────────────────────────────────────────────────────
    pointlio = PointLIOProcess()
    if not pointlio.start():
        return 2

    # ── ROS 2 monitoring node ─────────────────────────────────────────────────
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as exc:
        log(f"[FAIL] rclpy not importable — is ROS 2 sourced? {exc}")
        pointlio.stop("ros_import_failure")
        return 2

    # Allow Point-LIO a few seconds to register its publishers in the ROS graph
    # before we start the subscriber, to avoid a missed first message.
    time.sleep(4.0)

    rclpy.init()
    node = Node("pointlio_health_check")

    monitor       = SlamTopicMonitor(node)
    shutdown_event = threading.Event()

    # ROS spin thread — processes incoming /aft_mapped_to_init callbacks
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    # ── Signal handling ───────────────────────────────────────────────────────
    # systemd sends SIGINT (KillSignal=SIGINT in service file) on stop.
    # We forward it to Point-LIO and exit cleanly with code 0.
    def _handle_signal(signum, frame) -> None:
        log(f"Signal {signum} received — shutting down cleanly")
        shutdown_event.set()

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # ── Health check loop ─────────────────────────────────────────────────────
    try:
        exit_code = run_health_loop(monitor, pointlio, shutdown_event)
    finally:
        # Always stop Point-LIO and clean up ROS before exiting
        pointlio.stop("wrapper_exit")
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    log(f"Exiting with code {exit_code}")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
