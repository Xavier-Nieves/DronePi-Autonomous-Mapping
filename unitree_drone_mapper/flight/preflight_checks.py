#!/usr/bin/env python3
"""flight/preflight_checks.py — Pre-arm preflight gate module.

PURPOSE
-------
Runs a sequence of hardware and software readiness checks before
SafeFlightMixin arms the vehicle. All checks are independent and reported
with structured results so the caller (main.py) can decide whether to
abort or log a warning and continue.

CHECKS IMPLEMENTED
------------------
1. Stale mission lock PID
   If /tmp/dronepi_mission.lock exists from a previous session (e.g. warm
   restart after power loss mid-flight), checks whether the recorded PID is
   still alive. If dead, clears the stale lock. If alive, reports a conflict.

2. SSD mount + free space
   Verifies /mnt/ssd is an active mount point with at least MIN_FREE_SSD_GB
   free. A missing mount means the bag recorder writes to the root filesystem.

3. SLAM convergence
   Waits for a minimum number of /aft_mapped_to_init messages from Point-LIO
   before declaring the map initialised. Arming before SLAM has a stable map
   risks EKF2 accepting a frozen or poorly-initialised pose.

ARCHITECTURE
------------
PreflightChecker is an importable class with no main(). It is instantiated
by main.py's run() between MAVROS-ready and warmup setpoints — before any
arming call. Each check returns a PreflightResult dataclass, and also calls
log_event_fn (SafeFlightMixin._log_event) so every result appears in the
session flight_events.json alongside mixin safety events.

ENVIRONMENT OVERRIDES
---------------------
    MIN_FREE_SSD_GB      Minimum free space on SSD in GB (default: 2.0)
    SSD_MOUNT_PATH       Mount point to check (default: /mnt/ssd)
    SLAM_MIN_MSGS        Min Point-LIO messages for convergence (default: 50)
    SLAM_TIMEOUT_S       Seconds to wait for SLAM convergence (default: 30.0)

SOURCES
-------
- PX4 EKF2: EKF2_EV_CTRL vision fusion bits
  https://docs.px4.io/main/en/advanced_config/parameter_reference.html
- Point-LIO: /aft_mapped_to_init topic, nav_msgs/Odometry
  https://github.com/hku-mars/Point-LIO
- SafeFlightMixin: MISSION_LOCK = /tmp/dronepi_mission.lock (session context)
- POSIX signal 0 for PID liveness: man 2 kill
"""

from __future__ import annotations

import json
import logging
import os
import shutil
import time
from dataclasses import dataclass, field
from pathlib import Path

logger = logging.getLogger(__name__)

# ── Environment-configurable constants ────────────────────────────────────────

SSD_MOUNT_PATH  = os.environ.get("SSD_MOUNT_PATH",  "/mnt/ssd")
MIN_FREE_SSD_GB = float(os.environ.get("MIN_FREE_SSD_GB",  "2.0"))
SLAM_MIN_MSGS   = int(os.environ.get("SLAM_MIN_MSGS",       "50"))
SLAM_TIMEOUT_S  = float(os.environ.get("SLAM_TIMEOUT_S",   "30.0"))

MISSION_LOCK    = Path("/tmp/dronepi_mission.lock")


# ── Result type ───────────────────────────────────────────────────────────────

@dataclass
class PreflightResult:
    """Structured result for a single preflight check.

    Attributes
    ----------
    check  : Short identifier (e.g. "ssd_mount", "slam_convergence").
    passed : True if the check passed or was safely skipped.
    fatal  : True if a failure should abort the mission.
    reason : Machine-readable outcome tag (e.g. "ok", "ssd_not_mounted").
    detail : Optional structured data for the event log.
    """
    check:  str
    passed: bool
    fatal:  bool
    reason: str
    detail: dict = field(default_factory=dict)


# ── Checker ───────────────────────────────────────────────────────────────────

class PreflightChecker:
    """Runs all preflight gates and returns structured results.

    Parameters
    ----------
    node : rclpy.node.Node
        The active ROS 2 node (MainNode). Required for the SLAM convergence
        check which creates a temporary subscription.
    spin_fn : callable
        Advances the ROS event loop once:
            lambda: rclpy.spin_once(node, timeout_sec=0.5)
    log_event_fn : callable or None
        Reference to SafeFlightMixin._log_event(event_type, extra).
        When provided, every check result is written to flight_events.json.
        When None, results go to logging.getLogger() only (journald/stdout).
    """

    def __init__(self, node, spin_fn, log_event_fn=None) -> None:
        self._node      = node
        self._spin      = spin_fn
        self._log_event = log_event_fn if log_event_fn is not None else (
            lambda *a, **kw: None
        )

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _emit(self, result: PreflightResult) -> PreflightResult:
        """Write result to flight_events.json and return it unchanged."""
        self._log_event("PREFLIGHT_CHECK", {
            "check":  result.check,
            "passed": result.passed,
            "fatal":  result.fatal,
            "reason": result.reason,
            "detail": result.detail,
        })
        return result

    # ── Public API ────────────────────────────────────────────────────────────

    def run_all(self) -> list[PreflightResult]:
        """Run all preflight checks in sequence.

        Returns a list of PreflightResult in execution order.
        The caller should abort on the first result where
        ``not result.passed and result.fatal``.

        Execution order:
          1. stale_lock       — clear dead-PID lock before SSD/SLAM checks
          2. ssd_mount        — fast I/O check, no ROS dependency
          3. slam_convergence — slowest (up to SLAM_TIMEOUT_S), runs last
        """
        results: list[PreflightResult] = []
        results.append(self.check_stale_lock())
        results.append(self.check_ssd())
        results.append(self.check_slam_convergence())
        return results

    # ── Check 1: stale mission lock ───────────────────────────────────────────

    def check_stale_lock(self) -> PreflightResult:
        """Detect and clear stale mission lock files from previous sessions.

        SafeFlightMixin writes MISSION_LOCK with the current PID on
        start_safety_monitors() and removes it on _teardown(). A warm
        restart after power loss mid-flight leaves a stale lock that blocks
        the next mission with a false "another mission running" error.

        Outcomes:
          No lock file       → pass
          Lock, PID alive    → fatal conflict, do not clear
          Lock, PID dead     → stale, cleared, pass with warning
          Lock, unreadable   → cleared, pass with warning

        Source: POSIX kill(2) — signal 0 tests PID existence without
                delivering a signal. Raises ProcessLookupError if PID absent.
        """
        if not MISSION_LOCK.exists():
            return self._emit(PreflightResult(
                check="stale_lock", passed=True, fatal=False,
                reason="no_lock_file",
            ))

        try:
            data      = json.loads(MISSION_LOCK.read_text())
            stale_pid = data.get("pid")

            if stale_pid is None:
                raise ValueError("no PID field in lock")

            try:
                os.kill(stale_pid, 0)
                # PID is alive — genuine conflict
                logger.error(
                    f"[PreflightChecker] Mission lock held by live PID "
                    f"{stale_pid}. Another mission may be running."
                )
                return self._emit(PreflightResult(
                    check="stale_lock", passed=False, fatal=True,
                    reason="lock_held_by_live_pid",
                    detail={"pid": stale_pid},
                ))
            except ProcessLookupError:
                MISSION_LOCK.unlink(missing_ok=True)
                logger.warning(
                    f"[PreflightChecker] Stale lock from dead PID "
                    f"{stale_pid} cleared."
                )
                return self._emit(PreflightResult(
                    check="stale_lock", passed=True, fatal=False,
                    reason="stale_lock_cleared",
                    detail={"cleared_pid": stale_pid},
                ))

        except Exception as exc:
            MISSION_LOCK.unlink(missing_ok=True)
            logger.warning(
                f"[PreflightChecker] Unreadable lock cleared: {exc}"
            )
            return self._emit(PreflightResult(
                check="stale_lock", passed=True, fatal=False,
                reason="corrupt_lock_cleared",
                detail={"error": str(exc)},
            ))

    # ── Check 2: SSD mount and free space ─────────────────────────────────────

    def check_ssd(self) -> PreflightResult:
        """Verify the SSD is mounted and has sufficient free space.

        The bag recorder writes to /mnt/ssd/rosbags/. If the SSD is not
        mounted at the time main.py runs the recorder either fails silently
        or writes to the root filesystem tmpfs, exhausting it rapidly.

        drone-watchdog.service declares After=mnt-ssd.mount, covering the
        boot-time dependency. This check guards against USB disconnect between
        boot and mission arm — a realistic field scenario on the Pi.

        Free space floor: MIN_FREE_SSD_GB (default 2.0 GB).
        At ~150–200 MB/min for compressed MCAP (LiDAR + camera), 2 GB
        provides ~10 minutes of recording margin.
        """
        if not os.path.ismount(SSD_MOUNT_PATH):
            logger.error(
                f"[PreflightChecker] SSD not mounted at {SSD_MOUNT_PATH}."
            )
            return self._emit(PreflightResult(
                check="ssd_mount", passed=False, fatal=True,
                reason="ssd_not_mounted",
                detail={"path": SSD_MOUNT_PATH},
            ))

        try:
            usage    = shutil.disk_usage(SSD_MOUNT_PATH)
            free_gb  = usage.free  / 1e9
            total_gb = usage.total / 1e9
        except OSError as exc:
            logger.error(f"[PreflightChecker] disk_usage() failed: {exc}")
            return self._emit(PreflightResult(
                check="ssd_mount", passed=False, fatal=True,
                reason="disk_usage_error",
                detail={"error": str(exc)},
            ))

        if free_gb < MIN_FREE_SSD_GB:
            logger.error(
                f"[PreflightChecker] SSD free {free_gb:.2f} GB < "
                f"minimum {MIN_FREE_SSD_GB:.2f} GB."
            )
            return self._emit(PreflightResult(
                check="ssd_mount", passed=False, fatal=True,
                reason="ssd_insufficient_space",
                detail={
                    "free_gb":  round(free_gb,  2),
                    "total_gb": round(total_gb, 2),
                    "min_gb":   MIN_FREE_SSD_GB,
                },
            ))

        logger.info(
            f"[PreflightChecker] SSD OK: {free_gb:.1f} GB free "
            f"of {total_gb:.1f} GB at {SSD_MOUNT_PATH}."
        )
        return self._emit(PreflightResult(
            check="ssd_mount", passed=True, fatal=False,
            reason="ok",
            detail={"free_gb": round(free_gb, 2), "total_gb": round(total_gb, 2)},
        ))

    # ── Check 3: SLAM convergence ─────────────────────────────────────────────

    def check_slam_convergence(self) -> PreflightResult:
        """Wait for Point-LIO to produce a stable map before arming.

        Subscribes to /aft_mapped_to_init (nav_msgs/Odometry) and counts
        incoming messages. SLAM_MIN_MSGS messages within SLAM_TIMEOUT_S
        confirms active data flow through the full chain:
            LiDAR → Point-LIO → /aft_mapped_to_init → SLAM bridge → EKF2

        A message count gate is used rather than a time delay because a
        time delay passes silently even when Point-LIO is not publishing
        (LiDAR cable fault, USB enumeration failure, process crash).

        Threshold: 50 messages at Point-LIO's nominal 10 Hz output = ~5 s
        of continuous odometry — sufficient for EKF2 to converge on a
        stable position estimate before setpoints are published.

        Sources:
          Point-LIO output: /aft_mapped_to_init, nav_msgs/Odometry, ~10 Hz
          EKF2_EV_CTRL = 15: all four vision fusion bits active
          _slam_bridge.py covariance assignment (session context)
        """
        try:
            from nav_msgs.msg import Odometry
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        except ImportError as exc:
            logger.error(f"[PreflightChecker] ROS import failed: {exc}")
            return self._emit(PreflightResult(
                check="slam_convergence", passed=False, fatal=True,
                reason="ros_import_error",
                detail={"error": str(exc)},
            ))

        _count: list[int] = [0]

        def _cb(_msg) -> None:
            _count[0] += 1

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        sub = self._node.create_subscription(
            Odometry, "/aft_mapped_to_init", _cb, qos
        )

        logger.info(
            f"[PreflightChecker] Waiting for SLAM convergence "
            f"({SLAM_MIN_MSGS} msgs, timeout {SLAM_TIMEOUT_S:.0f}s)..."
        )

        t0 = time.monotonic()
        while (_count[0] < SLAM_MIN_MSGS
               and time.monotonic() - t0 < SLAM_TIMEOUT_S):
            self._spin()

        elapsed = time.monotonic() - t0
        self._node.destroy_subscription(sub)

        if _count[0] < SLAM_MIN_MSGS:
            logger.error(
                f"[PreflightChecker] SLAM not converged — "
                f"{_count[0]}/{SLAM_MIN_MSGS} messages in {elapsed:.1f}s."
            )
            return self._emit(PreflightResult(
                check="slam_convergence", passed=False, fatal=True,
                reason="slam_not_converged",
                detail={
                    "received":  _count[0],
                    "required":  SLAM_MIN_MSGS,
                    "elapsed_s": round(elapsed, 1),
                    "timeout_s": SLAM_TIMEOUT_S,
                },
            ))

        logger.info(
            f"[PreflightChecker] SLAM OK: {_count[0]} messages "
            f"in {elapsed:.1f}s."
        )
        return self._emit(PreflightResult(
            check="slam_convergence", passed=True, fatal=False,
            reason="ok",
            detail={"received": _count[0], "elapsed_s": round(elapsed, 1)},
        ))
