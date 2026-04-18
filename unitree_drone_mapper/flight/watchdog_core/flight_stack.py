"""watchdog_core/flight_stack.py — manage scan-session processes and health checks.

This updated version removes old reader.beep_* helper usage so sound ownership
stays in drone_watchdog.py and postflight.py.

Responsibilities
----------------
- Start the scan stack processes
- Stop the scan stack processes
- Trigger postflight after a completed scan session
- Expose health checks used by the watchdog

Sound ownership
---------------
- Start / stop / ready / active / finished tones are handled by the watchdog
- Postflight tones are handled by postflight.py
- This file should not directly decide buzzer meaning
"""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Callable, Optional

from .logging_utils import log

# These paths/commands may need to match your project exactly.
# They are kept generic here to preserve the structure of the original file.
PROJECT_ROOT = Path(__file__).resolve().parents[3]
ROSBAG_DIR   = Path("/mnt/ssd/rosbags")

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP = str(PROJECT_ROOT / "RPI5/ros2_ws/install/setup.bash")

SLAM_MIN_HZ = 5.0
BRIDGE_MIN_HZ = 8.0
SLAM_TOPIC_TIMEOUT_S = 30.0
BRIDGE_TOPIC_TIMEOUT_S = 15.0

# hailo_flight_node runs in its own venv (hailo_inference_env) because
# hailo_platform is not installed in the dronepi conda env. The node is
# launched via the explicit interpreter path so the active environment
# (dronepi conda) is bypassed entirely — identical to how arducam_node
# uses rpicam-vid as a subprocess rather than a Python import.
HAILO_PYTHON      = Path.home() / "hailo_inference_env/bin/python3"
HAILO_NODE_SCRIPT = PROJECT_ROOT / "unitree_drone_mapper/hailo/hailo_flight_node.py"
# Matches main.py: PROJECT_ROOT/unitree_drone_mapper/hailo/hailo_flight_node.py
# main.py's PROJECT_ROOT = ~/unitree_lidar_project/unitree_drone_mapper, so its
# path is equivalent: PROJECT_ROOT/"hailo"/"hailo_flight_node.py".


class FlightStack:
    """Owns the runtime scan stack subprocesses and basic health supervision."""

    def __init__(self, reader, postflight_fn: Optional[Callable[[], None]] = None) -> None:
        self._reader = reader
        self._postflight_fn = postflight_fn

        self._running = False
        self._session_reason = ""
        self._started_at = 0.0

        self._bag_proc: Optional[subprocess.Popen] = None
        self._slam_proc: Optional[subprocess.Popen] = None
        self._bridge_proc: Optional[subprocess.Popen] = None
        # hailo_flight_node — optional, skipped if HAILO_PYTHON or script absent
        self._hailo_proc: Optional[subprocess.Popen] = None

    # ── Public state ─────────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._running

    # ── Start / Stop ─────────────────────────────────────────────────────────

    def start(self, reason: str = "UNKNOWN") -> bool:
        """Start the scan stack.

        Returns True on success, False on startup failure.
        """
        if self._running:
            log("[STACK] Start requested, but stack is already running")
            return True

        self._session_reason = reason
        self._started_at = time.time()

        try:
            ROSBAG_DIR.mkdir(parents=True, exist_ok=True)

            # NOTE:
            # Replace these placeholder commands with your exact original ones if needed.
            # They are intentionally simple so the generated file is easy to merge.
            self._bag_proc = self._spawn(
                f"source {ROS_SETUP} && source {WS_SETUP} && ros2 bag record -a -o {ROSBAG_DIR / 'latest_scan'}"
            )
            self._slam_proc = self._spawn(
                f"source {ROS_SETUP} && source {WS_SETUP} && ros2 launch point_lio mapping.launch.py"
            )
            self._bridge_proc = self._spawn(
                f"source {ROS_SETUP} && source {WS_SETUP} && ros2 run unitree_lidar_ros2 bridge_node"
            )

            # hailo_flight_node — launched after SLAM bridge so that
            # /mavros/local_position/pose (used for AGL) is already available.
            # Skipped gracefully if the interpreter or script is not present
            # so the stack still starts on a system without Hailo configured.
            self._hailo_proc = self._spawn_hailo()

            self._running = True
            log(f"[STACK] Started (reason={reason})")
            return True

        except Exception as exc:
            log(f"[STACK] Startup failed: {exc}")
            self._cleanup_partial_start()
            self._running = False
            return False

    def stop(self) -> None:
        """Stop the scan stack and optionally trigger postflight."""
        if not self._running:
            log("[STACK] Stop requested, but stack is not running")
            return

        log("[STACK] Stopping stack...")
        # Stop order (reverse of start): bag → hailo → bridge → slam
        # Bag stops first so it captures any final Hailo flow messages.
        # Hailo stops before the bridge because it publishes to ROS topics
        # that the bridge may be consuming.
        self._terminate_proc(self._bag_proc, "rosbag")
        self._terminate_proc(self._hailo_proc, "hailo_flight_node")
        self._terminate_proc(self._bridge_proc, "bridge")
        self._terminate_proc(self._slam_proc, "slam")

        self._bag_proc = None
        self._hailo_proc = None
        self._bridge_proc = None
        self._slam_proc = None
        self._running = False

        duration_s = time.time() - self._started_at if self._started_at else 0.0
        log(f"[STACK] Stopped after {duration_s:.1f}s")

        if self._postflight_fn is not None:
            try:
                self._postflight_fn()
            except Exception as exc:
                log(f"[STACK] Postflight trigger failed: {exc}")

    # ── Health ───────────────────────────────────────────────────────────────

    def check_health(self) -> None:
        """Log process-health issues.

        This version does not directly raise alarms. The watchdog can inspect
        process state or extend this class later to return structured faults.
        """
        if not self._running:
            return

        bad = False

        if self._bag_proc is not None and self._bag_proc.poll() is not None:
            log(f"[STACK][HEALTH] rosbag exited unexpectedly with code {self._bag_proc.returncode}")
            bad = True

        if self._slam_proc is not None and self._slam_proc.poll() is not None:
            log(f"[STACK][HEALTH] slam exited unexpectedly with code {self._slam_proc.returncode}")
            bad = True

        if self._bridge_proc is not None and self._bridge_proc.poll() is not None:
            log(f"[STACK][HEALTH] bridge exited unexpectedly with code {self._bridge_proc.returncode}")
            bad = True

        # Hailo is optional — log exit but do not set bad=True so the
        # watchdog does not abort the scan session over a non-critical process.
        if self._hailo_proc is not None and self._hailo_proc.poll() is not None:
            log(f"[STACK][HEALTH] hailo_flight_node exited (code {self._hailo_proc.returncode}) "
                f"— flow augmentation inactive for this session")
            self._hailo_proc = None

        if bad:
            log("[STACK][HEALTH] One or more stack processes died")

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _spawn(self, command: str) -> subprocess.Popen:
        log(f"[STACK] Launching: {command}")
        return subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

    def _terminate_proc(self, proc: Optional[subprocess.Popen], label: str) -> None:
        if proc is None:
            return

        if proc.poll() is not None:
            log(f"[STACK] {label} already exited with code {proc.returncode}")
            return

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=5.0)
                log(f"[STACK] {label} terminated cleanly")
            except subprocess.TimeoutExpired:
                log(f"[STACK] {label} did not terminate in time; killing")
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
        except Exception as exc:
            log(f"[STACK] Failed to stop {label}: {exc}")

    def _spawn_hailo(self) -> Optional[subprocess.Popen]:
        """Launch hailo_flight_node.py in hailo_inference_env.

        Returns the Popen handle on success, or None if the interpreter or
        script is absent. None return is treated as graceful skip — the scan
        session continues without Hailo flow augmentation.

        hailo_flight_node publishes /hailo/optical_flow and /hailo/ground_class
        over ROS 2 DDS. FlowBridge (in the main conda env) consumes the flow
        topic and forwards it to /mavros/odometry/out for EKF2 fusion.

        The explicit interpreter path (hailo_inference_env/bin/python3) is used
        rather than relying on the active environment because hailo_platform is
        only installed in hailo_inference_env — not in the dronepi conda env
        that the watchdog systemd service activates.

        ROS 2 setup files are sourced inside the bash command so the node can
        publish and subscribe to ROS 2 topics despite running outside the
        dronepi conda env.
        """
        if not HAILO_PYTHON.exists():
            log("[STACK] hailo_inference_env not found — Hailo node skipped")
            return None
        if not HAILO_NODE_SCRIPT.exists():
            log(f"[STACK] hailo_flight_node.py not found at {HAILO_NODE_SCRIPT} — skipped")
            return None

        cmd = (
            f"source {ROS_SETUP} && source {WS_SETUP} && "
            f"{HAILO_PYTHON} {HAILO_NODE_SCRIPT}"
        )
        proc = self._spawn(cmd)
        log(f"[STACK] hailo_flight_node started (pid={proc.pid})")
        return proc

    def _cleanup_partial_start(self) -> None:
        self._terminate_proc(self._bag_proc, "rosbag")
        self._terminate_proc(self._hailo_proc, "hailo_flight_node")
        self._terminate_proc(self._bridge_proc, "bridge")
        self._terminate_proc(self._slam_proc, "slam")
        self._bag_proc = None
        self._hailo_proc = None
        self._bridge_proc = None
        self._slam_proc = None
