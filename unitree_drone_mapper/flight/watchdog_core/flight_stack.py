#!/usr/bin/env python3
"""
watchdog_core/flight_stack.py — Flight stack subprocess manager.

Owns Point-LIO, SLAM bridge, and bag recorder for manual scan sessions.
Called by drone_watchdog.py. drone_watchdog.py itself is NOT modified.

LOGGING INTEGRATION
-------------------
flight_logger.open_session() is called inside start() immediately after
self._running = True. flight_logger.close_session() is called at the
top of stop() before any subprocess is terminated, so the duration is
accurate and the record is written even if a subprocess fails to stop.

Both calls are non-fatal — any exception is caught and logged. A failure
in flight_logger never affects the stop sequence or the postflight trigger.

FLIGHT TYPE
-----------
All sessions started by FlightStack are FLIGHT_TYPE_MANUAL.
The script_name field is always "drone_watchdog" to distinguish these
from SafeFlightMixin-managed sessions (test scripts and main.py).

SESSION ID
----------
Uses self._bag_path.name (e.g. "scan_20260422_143215") as the session_id,
which matches the bag directory name that postprocess_mesh.py receives.
This allows flight_logger.log_flight() (called by postprocess_mesh.py)
to find the existing session record and append postprocess results to it
rather than creating a duplicate entry.

STOP REASON MAPPING
--------------------
  "manual_scan_disarm"       reader.armed went False — normal scan end
  "rc_toggle_stop"           operator pressed CH6 to stop mid-scan
  "watchdog_yielded_bench"   bench_scan lock appeared while stack was running
  "watchdog_yielded_auto"    autonomous lock appeared while stack was running
  "watchdog_shutdown"        SIGTERM/SIGINT received, finally block running

BACKWARD COMPATIBILITY
----------------------
All existing call signatures are unchanged. drone_watchdog.py does not
need modification.
"""

from __future__ import annotations

import importlib.util
import logging
import os
import signal
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

from watchdog_core.logging_utils import log

# ── Paths ──────────────────────────────────────────────────────────────────────

PROJECT_ROOT = Path(__file__).resolve().parents[1]

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP  = str(PROJECT_ROOT / "RPI5" / "ros2_ws" / "install" / "setup.bash")

ROSBAG_DIR = Path("/mnt/ssd/rosbags")

# ── Command builders ───────────────────────────────────────────────────────────

def _ros_source() -> str:
    return f"source {ROS_SETUP} && source {WS_SETUP} && "


def _pointlio_cmd() -> str:
    launch = (PROJECT_ROOT /
              "RPI5/ros2_ws/src/point_lio_ros2/launch/combined_lidar_mapping.launch.py")
    return _ros_source() + f"ros2 launch {launch} rviz:=false port:=/dev/ttyUSB0"


def _bridge_cmd() -> str:
    bridge = PROJECT_ROOT / "unitree_drone_mapper/flight/_slam_bridge.py"
    return _ros_source() + f"python3 {bridge}"


def _bag_cmd(bag_path: Path) -> str:
    topics = " ".join([
        "/cloud_registered",
        "/aft_mapped_to_init",
        "/mavros/local_position/pose",
        "/mavros/state",
        "/mavros/global_position/global",
        "/dronepi/collision_zone",
        "/mavros/distance_sensor/lidar_down_sub",
    ])
    return _ros_source() + f"ros2 bag record {topics} -o {bag_path} --storage mcap"


# ── Health check thresholds ────────────────────────────────────────────────────

SLAM_MIN_HZ          = 5.0
BRIDGE_MIN_HZ        = 8.0
SLAM_TOPIC_TIMEOUT_S = 30.0


# ══════════════════════════════════════════════════════════════════════════════
# FLIGHT LOGGER LOADER
# ══════════════════════════════════════════════════════════════════════════════

def _load_flight_logger():
    """
    Load flight_logger.py by resolving its path relative to this file.
    flight_logger.py: <project_root>/utils/flight_logger.py
    flight_stack.py:  <project_root>/watchdog_core/flight_stack.py
    → parent is watchdog_core/, parent.parent is project root.

    Returns the module, or None if unavailable. Never raises.
    """
    try:
        logger_path = PROJECT_ROOT / "utils" / "flight_logger.py"
        if not logger_path.exists():
            log(f"[STACK] flight_logger not found at {logger_path}")
            return None
        spec = importlib.util.spec_from_file_location("flight_logger", logger_path)
        mod  = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    except Exception as exc:
        log(f"[STACK] Could not load flight_logger (non-fatal): {exc}")
        return None


# ══════════════════════════════════════════════════════════════════════════════
# SUBPROCESS HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _spawn(label: str, cmd: str) -> subprocess.Popen:
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    log(f"[STACK] {label} started (pid={proc.pid})")
    return proc


def _kill(label: str, proc: Optional[subprocess.Popen]) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=8)
        log(f"[STACK] {label} stopped cleanly")
    except subprocess.TimeoutExpired:
        log(f"[STACK] {label} SIGINT timeout — sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    except Exception as exc:
        log(f"[STACK] {label} stop error: {exc}")


# ══════════════════════════════════════════════════════════════════════════════
# FLIGHT STACK
# ══════════════════════════════════════════════════════════════════════════════

class FlightStack:
    """
    Owns Point-LIO, SLAM bridge, and bag recorder for manual scan sessions.

    Args:
        reader:        MavrosReader — for FCU state queries (armed/mode).
        postflight_fn: callable() — called after stop() to trigger postflight.
                       Decoupled so FlightStack has no import dependency on
                       postflight.py.
    """

    def __init__(
        self,
        reader,
        postflight_fn: Optional[Callable[[], None]] = None,
    ) -> None:
        self._reader        = reader
        self._postflight_fn = postflight_fn

        self._running        = False
        self._session_reason = ""
        self._started_at     = 0.0
        self._bag_path:  Optional[Path] = None

        self._pointlio_proc: Optional[subprocess.Popen] = None
        self._bridge_proc:   Optional[subprocess.Popen] = None
        self._bag_proc:      Optional[subprocess.Popen] = None

        # Flight logger state — set in start(), used in stop()
        self._flight_logger_mod  = None
        self._flight_number: int = 0

    # ── Public state ──────────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._running

    # ── Start ─────────────────────────────────────────────────────────────────

    def start(self, reason: str = "UNKNOWN") -> bool:
        """
        Start the scan stack. Returns True on success.

        Launch order:
          1. Point-LIO  — must be first; SLAM bridge subscribes to its output
          2. SLAM bridge — 3s after Point-LIO to allow SLAM initialisation
          3. Bag recorder — 1s after bridge; records immediately once both up

        Source: watchdog session docs, FlightStack architecture notes.
        """
        if self._running:
            log("[STACK] Start requested — stack already running")
            return True

        self._session_reason = reason
        self._started_at     = time.time()

        try:
            ROSBAG_DIR.mkdir(parents=True, exist_ok=True)

            # 1. Point-LIO
            log("[STACK] Starting Point-LIO...")
            self._pointlio_proc = _spawn("Point-LIO", _pointlio_cmd())
            time.sleep(3.0)

            # 2. SLAM bridge
            bridge_script = PROJECT_ROOT / "unitree_drone_mapper/flight/_slam_bridge.py"
            if bridge_script.exists():
                log("[STACK] Starting SLAM bridge...")
                self._bridge_proc = _spawn("SLAM bridge", _bridge_cmd())
                time.sleep(1.0)
            else:
                log("[STACK] WARN: _slam_bridge.py not found — vision fusion disabled")

            # 3. Bag recorder
            ts               = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._bag_path   = ROSBAG_DIR / f"scan_{ts}"
            log(f"[STACK] Starting bag recorder → {self._bag_path.name}")
            self._bag_proc   = _spawn("bag recorder", _bag_cmd(self._bag_path))

            self._running = True
            log(f"[STACK] Stack started  reason={reason}  bag={self._bag_path.name}")

            # ── Open flight log session ───────────────────────────────────────
            # Called after self._running = True so bag_path is set.
            # Non-fatal: exception never prevents the scan from continuing.
            self._logger_open(reason)

            return True

        except Exception as exc:
            log(f"[STACK] Startup failed: {exc}")
            self._cleanup_partial()
            self._running = False
            return False

    # ── Stop ──────────────────────────────────────────────────────────────────

    def stop(self, reason: str = "manual_scan_disarm") -> None:
        """
        Stop the scan stack and trigger postflight.

        Stop order is the reverse of launch order — critical for clean MCAP
        finalisation. Bag recorder must finish writing before Point-LIO stops
        publishing, otherwise the final messages are lost.

          1. Bag recorder  — MCAP file finalised
          2. SLAM bridge   — no longer needed once bag is closed
          3. Point-LIO     — kept alive until both dependents are done

        The flight log session is closed BEFORE subprocess teardown so that
        duration reflects actual flight time, not cleanup time.
        """
        if not self._running:
            return

        log(f"[STACK] Stopping stack  reason={reason}")

        # ── Close flight log session ──────────────────────────────────────────
        # Done first so duration is flight time, not teardown time.
        # Non-fatal: exception never affects the stop sequence.
        self._logger_close(reason)

        # ── Subprocess teardown (reverse launch order) ────────────────────────
        _kill("bag recorder", self._bag_proc)
        self._bag_proc = None

        _kill("SLAM bridge", self._bridge_proc)
        self._bridge_proc = None

        _kill("Point-LIO", self._pointlio_proc)
        self._pointlio_proc = None

        self._running = False
        log(f"[STACK] Stack stopped.")

        # ── Postflight ────────────────────────────────────────────────────────
        if self._postflight_fn is not None:
            try:
                self._postflight_fn()
            except Exception as exc:
                log(f"[STACK] Postflight callback failed: {exc}")

    def check_health(self) -> None:
        """Warn if any managed subprocess has exited unexpectedly."""
        if self._pointlio_proc and self._pointlio_proc.poll() is not None:
            log("[STACK] WARN: Point-LIO exited unexpectedly")
            self._pointlio_proc = None
        if self._bridge_proc and self._bridge_proc.poll() is not None:
            log("[STACK] WARN: SLAM bridge exited unexpectedly")
            self._bridge_proc = None
        if self._bag_proc and self._bag_proc.poll() is not None:
            log("[STACK] WARN: Bag recorder exited unexpectedly")
            self._bag_proc = None

    # ── Partial cleanup on failed start ───────────────────────────────────────

    def _cleanup_partial(self) -> None:
        _kill("bag recorder (partial)", self._bag_proc)
        _kill("SLAM bridge (partial)",  self._bridge_proc)
        _kill("Point-LIO (partial)",    self._pointlio_proc)
        self._bag_proc       = None
        self._bridge_proc    = None
        self._pointlio_proc  = None

    # ══════════════════════════════════════════════════════════════════════════
    # FLIGHT LOGGER INTEGRATION
    # ══════════════════════════════════════════════════════════════════════════

    def _logger_open(self, reason: str) -> None:
        """
        Call flight_logger.open_session() after stack start.
        Maps the start reason to a descriptive script_name field so the
        flight_history.log line clearly identifies the trigger.

        Reason → script_name mapping:
          "MANUAL_LOCK" → "drone_watchdog (manual_scan_lock)"
          "MANUAL_RC"   → "drone_watchdog (rc_toggle)"
          anything else → "drone_watchdog (<reason>)"
        """
        try:
            mod = _load_flight_logger()
            if mod is None:
                return

            self._flight_logger_mod = mod

            script_label = f"drone_watchdog ({reason.lower()})"
            session_id   = self._bag_path.name if self._bag_path else f"scan_unknown_{int(self._started_at)}"

            self._flight_number = mod.open_session(
                session_id  = session_id,
                script_name = script_label,
                flight_type = mod.FLIGHT_TYPE_MANUAL,
                bag_path    = str(self._bag_path) if self._bag_path else None,
                context     = "manual",
            )
            log(f"[STACK] Flight {self._flight_number:03d} opened in flight_logger  "
                f"session={session_id}")
        except Exception as exc:
            log(f"[STACK] flight_logger open_session failed (non-fatal): {exc}")

    def _logger_close(self, reason: str) -> None:
        """
        Call flight_logger.close_session() at the start of stop().
        Duration is computed from self._started_at (set in start()).
        """
        try:
            mod = self._flight_logger_mod
            if mod is None:
                return

            duration_s = (time.time() - self._started_at
                          if self._started_at else None)
            session_id = (self._bag_path.name if self._bag_path
                          else f"scan_unknown_{int(self._started_at or 0)}")

            mod.close_session(
                session_id         = session_id,
                end_reason         = reason,
                duration_s         = duration_s,
                bag_closed_cleanly = True,   # bag stop follows immediately after
                bag_path           = str(self._bag_path) if self._bag_path else None,
            )
            log(f"[STACK] Flight {self._flight_number:03d} closed in flight_logger  "
                f"reason={reason}")
        except Exception as exc:
            log(f"[STACK] flight_logger close_session failed (non-fatal): {exc}")
