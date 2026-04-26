#!/usr/bin/env python3
"""
flight/safe_flight_mixin.py — Shared safety base class for all DronePi flight scripts.

ROOT CAUSE CONTEXT
------------------
On 2026-04-18, a Tarot 810 was lost during test_square_camera.py execution.
An SSH hotspot disconnection left the script running headlessly. The operator
armed manually via RC not knowing the script was still publishing setpoints at
20 Hz. PX4 accepted OFFBOARD, overrode RC input, climbed to 6m, flipped, crashed.

DEPLOYMENT MODEL
----------------
Scripts are started from the Pi terminal or triggered via RC — never from an
active SSH session during flight. SSH is for development only.
TTY death detection is therefore not implemented in this system.
RC CH7 is the primary manual abort in all contexts.

ARCHITECTURE CONSTRAINTS — NON-NEGOTIABLE
------------------------------------------
1. Every flight script MUST inherit SafeFlightMixin before any arming call.
2. Rosbag recorder MUST be started as the first subprocess via start_bag_recorder().
3. Every setpoint publish MUST update: self._sp_heartbeat = time.monotonic()
4. Every setpoint publish MUST update: self._alt_setpoint = z
5. RC Channel 7 monitoring MUST be active in all flight scripts.
6. Mission lock MUST contain PID.
7. No flight script MAY arm if self._pilot_override is True.

CONTEXT MODES
-------------
CONTEXT_TEST (default for test scripts):
    OFFBOARD lost → immediate teardown + AUTO.LAND. No recovery window.
    Rationale: a test is a controlled procedure. Any mode change means
    the operator took deliberate control — abort cleanly.

CONTEXT_MISSION (for main.py autonomous missions):
    OFFBOARD lost → enter a pause window (default 30s).
    During the pause, CH1-CH4 (roll/pitch/throttle/yaw) are sampled every
    0.5s. If any stick moves more than RC_STICK_DEADBAND PWM from its
    position at the moment OFFBOARD was lost, the pilot is marked active.

    On OFFBOARD restored AND pilot static → resume mission from current waypoint.
    On OFFBOARD restored AND pilot active → AUTO.RTL.
    On timeout (30s) regardless of activity → AUTO.RTL.

    The pause window signals are exposed via two public booleans that the
    waypoint loop in main.py should poll:
        self._offboard_pause_active  — True while window is open
        self._offboard_aborted       — True after AUTO.RTL was commanded

    This REPLACES main.py's existing while-loop for OFFBOARD loss.
    Do not run both simultaneously.

MONITORS (started via start_safety_monitors())
----------------------------------------------
  RC Override Monitor     — /mavros/state: handles OFFBOARD loss per context
  Setpoint Watchdog       — heartbeat stale > 2.0s → AUTO.LAND
  RC CH7 Kill Switch      — CH7 PWM > RC_KILL_THRESHOLD → teardown
                            Also caches all RC channels for activity detection
  Erratic Motion Detector — angular velocity > threshold sustained 0.5s → AUTO.LAND
                            altitude deviation from setpoint > threshold → AUTO.LAND
  PID Mission Lock        — prevents two scripts from arming simultaneously
"""

from __future__ import annotations

import importlib.util
import json
import logging
import os
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

log = logging.getLogger(__name__)

# ── Context constants ─────────────────────────────────────────────────────────

CONTEXT_TEST    = "test"
CONTEXT_MISSION = "mission"

# ── Environment-configurable thresholds ───────────────────────────────────────

RC_KILL_CHANNEL   = int(os.environ.get("RC_KILL_CHANNEL",   "7"))    # 1-indexed
RC_KILL_THRESHOLD = int(os.environ.get("RC_KILL_THRESHOLD", "1700"))

# CH1-CH4 are the stick axes (roll/pitch/throttle/yaw), 1-indexed
RC_STICK_CHANNELS = [1, 2, 3, 4]
# Single-sample movement on any stick beyond this PWM = pilot is actively flying
RC_STICK_DEADBAND = int(os.environ.get("RC_STICK_DEADBAND", "30"))

MOTION_ANGVEL_THRESHOLD = float(os.environ.get("MOTION_ANGVEL_THRESHOLD", "2.0"))  # rad/s
MOTION_ALT_DEVIATION    = float(os.environ.get("MOTION_ALT_DEVIATION",    "1.5"))  # metres

SETPOINT_WATCHDOG_TIMEOUT  = 2.0   # seconds stale before AUTO.LAND
SETPOINT_WATCHDOG_INTERVAL = 0.5   # seconds between watchdog checks
RC_PAUSE_POLL_INTERVAL     = 0.5   # seconds between samples during pause window

MISSION_LOCK = Path("/tmp/dronepi_mission.lock")

BAG_TOPICS_REQUIRED = [
    "/mavros/state",
    "/mavros/setpoint_position/local",
    "/mavros/local_position/pose",
    "/mavros/imu/data",
    "/mavros/rc/in",
    "/mavros/vision_pose/pose",
    "/hailo/optical_flow",
    "/hailo/ground_class",
    "/arducam/image_raw",
    "/aft_mapped_to_init",
    "/rpi/health",
]

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
WS_SETUP  = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
ROSBAG_DIR = Path("/mnt/ssd/rosbags")


class SafeFlightMixin:
    """
    Safety base class. Inherit alongside rclpy.node.Node in MRO order:

        class FlightNode(SafeFlightMixin, Node):
            def __init__(self):
                Node.__init__(self, "node_name")
                SafeFlightMixin.__init__(self, script_name=__file__)

    Parameters
    ----------
    script_name : str
        __file__ of the calling script. Written into the mission lock.
    context : str
        CONTEXT_TEST or CONTEXT_MISSION. Controls RC override behavior.
    offboard_resume_s : float
        CONTEXT_MISSION only. Seconds before AUTO.RTL if OFFBOARD not
        restored. Default 30.
    session_output_dir : Path or None
        Where to write event log and bag. Defaults to ROSBAG_DIR/<session>.
    """

    def __init__(
        self,
        script_name:        str,
        context:            str = CONTEXT_TEST,
        offboard_resume_s:  float = 30.0,
        session_output_dir: Optional[Path] = None,
    ) -> None:
        self._script_name       = Path(script_name).name
        self._context           = context
        self._offboard_resume_s = offboard_resume_s
        self._pid               = os.getpid()

        # ── Public state — read by subclasses ─────────────────────────────────
        # True when any monitor triggers → subclass MUST stop publishing setpoints
        self._pilot_override: bool = False

        # Updated by every setpoint publish → feeds the watchdog
        self._sp_heartbeat: float = 0.0

        # CONTEXT_MISSION pause window signals (poll these in the waypoint loop)
        self._offboard_pause_active: bool = False
        self._offboard_aborted:      bool = False

        # ── Internal ──────────────────────────────────────────────────────────
        self._monitors_started = False
        self._teardown_called  = False
        self._teardown_lock    = threading.Lock()

        # Updated by subclass on every setpoint publish
        self._alt_setpoint: float = 0.0

        # Shared state written by ROS callbacks, read by monitor threads
        self._state_lock = threading.Lock()
        self._last_mode:         str        = ""
        self._last_armed:        bool       = False
        self._last_pose_z:       float      = 0.0
        self._last_angvel_mag:   float      = 0.0
        self._last_rc_channels:  list[int]  = []

        self._angvel_above_since: Optional[float] = None

        # Emergency land client — independent of the main ROS executor
        self._emergency_node   = None
        self._emergency_client = None

        # Flight logger integration
        self._flight_number:     int   = 0
        self._flight_logger_mod        = None   # cached module reference

        # Bag recorder process handle
        self._bag_proc: Optional[subprocess.Popen] = None

        # Monitor thread handles
        self._monitor_threads: list[threading.Thread] = []

        # Session / event log setup
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_tag = (
            f"{self._script_name.replace('.py','').replace(' ','_')}"
            f"_{ts}_pid{self._pid}"
        )
        self._session_dir = (
            Path(session_output_dir) if session_output_dir
            else ROSBAG_DIR / self._session_tag
        )
        self._session_dir.mkdir(parents=True, exist_ok=True)
        self._event_log_path = self._session_dir / "flight_events.json"
        self._event_log: list[dict] = []
        self._event_lock = threading.Lock()

        self._log_event("STARTUP", {"context": self._context})

    # ══════════════════════════════════════════════════════════════════════════
    # PUBLIC API
    # ══════════════════════════════════════════════════════════════════════════

    def start_safety_monitors(self) -> bool:
        """
        Write the mission lock and start all safety monitors.
        MUST be called before any arming attempt.

        Returns False if a live process already owns the lock.
        """
        if self._monitors_started:
            return True

        # Open a flight log session immediately — before lock acquisition —
        # so even a lock-refused attempt is recorded.
        self._flight_number = self._logger_open_session()

        if not self._acquire_mission_lock():
            return False

        self.start_bag_recorder()          # bag recorder is always first
        self._init_emergency_client()

        self._start_thread("rc_override_monitor",    self._rc_override_monitor_loop)
        self._start_thread("setpoint_watchdog",      self._setpoint_watchdog_loop)
        self._start_thread("rc_kill_monitor",        self._rc_kill_monitor_loop)
        self._start_thread("erratic_motion_monitor", self._erratic_motion_monitor_loop)

        # Start 2 Hz telemetry sampler (setpoint vs actual, RC, IMU)
        self._data_sampler = _FlightDataSampler(self)
        self._data_sampler.start()

        self._monitors_started = True
        self._log_event("MONITORS_STARTED", {"context": self._context,
                                              "flight_number": self._flight_number})
        log.info(
            f"[SafeFlightMixin] Monitors running  "
            f"context={self._context}  pid={self._pid}  "
            f"flight={self._flight_number}"
        )
        return True

    def start_bag_recorder(self) -> bool:
        """
        Launch ros2 bag record as the FIRST subprocess of the flight.
        Failure is non-fatal — the mission continues without recording.
        """
        if self._bag_proc is not None:
            return True

        bag_path = self._session_dir / self._session_tag
        topics   = " ".join(BAG_TOPICS_REQUIRED)
        cmd = (
            f"source {ROS_SETUP} && source {WS_SETUP} && "
            f"ros2 bag record {topics} -o {bag_path}"
        )
        try:
            self._bag_proc = subprocess.Popen(
                ["bash", "-c", cmd],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            log.info(
                f"[SafeFlightMixin] Bag recorder started  "
                f"pid={self._bag_proc.pid}  path={bag_path}"
            )
            return True
        except Exception as exc:
            log.warning(f"[SafeFlightMixin] Bag recorder failed to start: {exc}")
            return False

    def monitors_alive(self) -> bool:
        """Return True if all monitor threads are still alive."""
        return all(t.is_alive() for t in self._monitor_threads)

    def _teardown(self, reason: str = "unspecified") -> None:
        """
        Stop setpoint publishing, command AUTO.LAND, close bag, clear lock.
        Idempotent — safe to call from multiple threads simultaneously.
        """
        with self._teardown_lock:
            if self._teardown_called:
                return
            self._teardown_called = True

        self._pilot_override = True
        log.critical(f"[SafeFlightMixin] TEARDOWN — {reason}")
        self._log_event("TEARDOWN", {"reason": reason})

        # Record disarm event before commanding land so duration is accurate
        self._logger_close_session(reason)

        self._command_mode("AUTO.LAND", reason)
        self._stop_bag()
        self._release_mission_lock()
        self._flush_event_log()

    # ══════════════════════════════════════════════════════════════════════════
    # MISSION LOCK
    # ══════════════════════════════════════════════════════════════════════════

    def _acquire_mission_lock(self) -> bool:
        """
        Write a PID-stamped mission lock.
        Refuses to proceed if a live process already owns the lock.

        Lock format is backward-compatible with drone_watchdog.py's
        read_lock_mode() which calls data.get("mode", "").
        """
        if MISSION_LOCK.exists():
            try:
                lines     = MISSION_LOCK.read_text().strip().splitlines()
                owner_pid = int(lines[0]) if lines else -1
                if owner_pid > 0 and Path(f"/proc/{owner_pid}").exists():
                    log.error(
                        f"[SafeFlightMixin] Lock owned by live PID {owner_pid} "
                        f"({lines[1] if len(lines) > 1 else '?'}). "
                        "Refusing to arm."
                    )
                    return False
                log.warning(
                    f"[SafeFlightMixin] Stale lock (PID {owner_pid} dead). Clearing."
                )
            except Exception:
                pass  # unreadable lock → overwrite it

        ts   = datetime.now().isoformat()
        mode = "autonomous" if self._context == CONTEXT_MISSION else "bench_scan"
        # First line is PID for this class's stale-lock detection.
        # JSON on subsequent lines is for watchdog compatibility.
        MISSION_LOCK.write_text(
            f"{self._pid}\n{self._script_name}\n{ts}\n"
            f'{{"mode": "{mode}", "pid": {self._pid}, "started_at": "{ts}"}}\n'
        )
        log.info(f"[SafeFlightMixin] Lock written  pid={self._pid}  mode={mode}")
        return True

    def _release_mission_lock(self) -> None:
        """Clear the lock only if this process wrote it."""
        try:
            if MISSION_LOCK.exists():
                lines = MISSION_LOCK.read_text().strip().splitlines()
                owner = int(lines[0]) if lines else -1
                if owner == self._pid:
                    MISSION_LOCK.unlink()
                    log.info("[SafeFlightMixin] Lock cleared.")
        except Exception as exc:
            log.warning(f"[SafeFlightMixin] Lock release error: {exc}")

    # ══════════════════════════════════════════════════════════════════════════
    # FLIGHT MODE COMMANDS
    # ══════════════════════════════════════════════════════════════════════════

    def _init_emergency_client(self) -> None:
        """
        Create a ROS service client on a separate Node from the main executor.
        This ensures AUTO.LAND can be commanded even when the main spin loop
        is hung (which is exactly when the watchdog triggers).
        """
        try:
            import rclpy
            from rclpy.node      import Node
            from mavros_msgs.srv import SetMode
            if not rclpy.ok():
                return
            self._emergency_node   = Node("sfm_emergency_client")
            self._emergency_client = self._emergency_node.create_client(
                SetMode, "/mavros/set_mode"
            )
        except Exception as exc:
            log.warning(f"[SafeFlightMixin] Emergency client init failed: {exc}")

    def _command_mode(self, mode_str: str, reason: str = "") -> None:
        """
        Send a mode change request. Tries the dedicated service client first,
        falls back to a shell call so the command gets through regardless of
        ROS executor state.
        """
        if reason:
            log.critical(f"[SafeFlightMixin] Commanding {mode_str} ({reason})")
        self._log_event("MODE_COMMAND", {"mode": mode_str, "reason": reason})

        # Dedicated client path
        if self._emergency_client is not None:
            try:
                import rclpy
                from mavros_msgs.srv import SetMode
                req = SetMode.Request()
                req.custom_mode = mode_str
                if self._emergency_client.service_is_ready():
                    future = self._emergency_client.call_async(req)
                    rclpy.spin_until_future_complete(
                        self._emergency_node, future, timeout_sec=3.0
                    )
                    if future.result() and future.result().mode_sent:
                        log.info(f"[SafeFlightMixin] {mode_str} confirmed via service")
                        return
            except Exception as exc:
                log.warning(f"[SafeFlightMixin] Service client error: {exc}")

        # Shell fallback — works even if the ROS executor is hung
        try:
            subprocess.run(
                ["bash", "-c",
                 f"source {ROS_SETUP} && source {WS_SETUP} && "
                 f"ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "
                 f"'{{custom_mode: {mode_str}}}'"],
                timeout=5,
                capture_output=True,
            )
            log.info(f"[SafeFlightMixin] {mode_str} sent via shell fallback")
        except Exception as exc:
            log.error(f"[SafeFlightMixin] Shell fallback failed: {exc}")

    def _stop_bag(self) -> None:
        if self._bag_proc is not None and self._bag_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._bag_proc.pid), signal.SIGINT)
                self._bag_proc.wait(timeout=8)
            except Exception:
                pass

    def _start_thread(self, name: str, target) -> None:
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        self._monitor_threads.append(t)

    # ══════════════════════════════════════════════════════════════════════════
    # MONITOR: RC OVERRIDE — context-split behavior
    # ══════════════════════════════════════════════════════════════════════════

    def _rc_override_monitor_loop(self) -> None:
        """
        Subscribes to /mavros/state at ~10 Hz.

        Waits until OFFBOARD is first confirmed active (latch), then watches
        for any exit from OFFBOARD.

        CONTEXT_TEST:
            Mode change → _teardown("PILOT_OVERRIDE") immediately.

        CONTEXT_MISSION:
            Mode change → launch _mission_offboard_pause() in a thread.
            That thread manages the 30s window and resume/abort logic.
            The waypoint loop polls _offboard_pause_active and
            _offboard_aborted to know what to do.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from mavros_msgs.msg import State

            node = Node("sfm_rc_override_monitor")
            qos  = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

            ever_in_offboard = [False]

            def _state_cb(msg: State) -> None:
                with self._state_lock:
                    self._last_mode  = msg.mode
                    self._last_armed = msg.armed

                if self._pilot_override or self._teardown_called:
                    return

                if msg.mode == "OFFBOARD":
                    ever_in_offboard[0] = True
                    return

                # Haven't entered OFFBOARD yet — don't trigger on pre-arm mode
                if not ever_in_offboard[0]:
                    return

                # ── OFFBOARD was active, now it isn't ─────────────────────────
                if self._context == CONTEXT_TEST:
                    log.critical(
                        f"[RC_OVERRIDE] OFFBOARD → {msg.mode}. "
                        "Test context: immediate teardown."
                    )
                    self._log_event("PILOT_OVERRIDE", {
                        "new_mode": msg.mode,
                        "armed":    msg.armed,
                    })
                    threading.Thread(
                        target=self._teardown,
                        args=("PILOT_OVERRIDE",),
                        daemon=True,
                    ).start()

                elif self._context == CONTEXT_MISSION:
                    # Only start one pause window at a time
                    if not self._offboard_pause_active:
                        log.warning(
                            f"[RC_OVERRIDE] OFFBOARD → {msg.mode}. "
                            f"Mission context: entering "
                            f"{self._offboard_resume_s:.0f}s pause window."
                        )
                        self._log_event("OFFBOARD_EXIT", {
                            "new_mode": msg.mode,
                            "armed":    msg.armed,
                        })
                        threading.Thread(
                            target=self._mission_offboard_pause,
                            daemon=True,
                            name="offboard_pause_window",
                        ).start()

            node.create_subscription(State, "/mavros/state", _state_cb, qos)
            rclpy.spin(node)

        except Exception as exc:
            log.error(f"[RC_OVERRIDE_MONITOR] Crashed: {exc}")

    def _mission_offboard_pause(self) -> None:
        """
        CONTEXT_MISSION pause window.  Runs in its own thread.

        Samples RC stick channels (CH1-CH4) every RC_PAUSE_POLL_INTERVAL
        seconds to determine whether the pilot is actively flying.

        Detection logic:
          - Record baseline stick values at the moment of OFFBOARD loss.
          - If any single sample shows any CH1-CH4 moving more than
            RC_STICK_DEADBAND PWM from baseline → pilot_active = True (latches).
          - RC noise is typically ±5-10 PWM; a deliberate stick input
            moves 50-200 PWM. Deadband of 30 is well above noise floor
            and below any intentional input.

        Outcomes:
          OFFBOARD restored + pilot static   → clear pause, waypoint loop resumes
          OFFBOARD restored + pilot active   → AUTO.RTL, _offboard_aborted = True
          Timeout (30s) regardless           → AUTO.RTL, _offboard_aborted = True
        """
        self._offboard_pause_active = True
        pilot_active = False
        t_start      = time.monotonic()
        deadline     = t_start + self._offboard_resume_s

        # Capture stick baseline at the moment of OFFBOARD loss
        with self._state_lock:
            baseline = list(self._last_rc_channels)

        log.info(
            f"[OFFBOARD_PAUSE] Window open for {self._offboard_resume_s:.0f}s. "
            f"Watching CH{RC_STICK_CHANNELS} deadband={RC_STICK_DEADBAND} PWM."
        )

        while time.monotonic() < deadline:
            if self._teardown_called:
                self._offboard_pause_active = False
                return

            time.sleep(RC_PAUSE_POLL_INTERVAL)

            with self._state_lock:
                current_mode     = self._last_mode
                current_channels = list(self._last_rc_channels)

            # ── Stick activity check ──────────────────────────────────────────
            if not pilot_active and len(current_channels) >= 4 and len(baseline) >= 4:
                for ch_1indexed in RC_STICK_CHANNELS:
                    idx = ch_1indexed - 1
                    if idx >= len(current_channels) or idx >= len(baseline):
                        continue
                    delta = abs(current_channels[idx] - baseline[idx])
                    if delta > RC_STICK_DEADBAND:
                        pilot_active = True
                        elapsed = time.monotonic() - t_start
                        log.warning(
                            f"[OFFBOARD_PAUSE] Pilot activity on CH{ch_1indexed}: "
                            f"delta={delta} PWM (>{RC_STICK_DEADBAND}). "
                            f"t={elapsed:.1f}s. Will abort at timeout."
                        )
                        self._log_event("PILOT_ACTIVE_DETECTED", {
                            "channel":   ch_1indexed,
                            "delta_pwm": delta,
                            "elapsed_s": round(elapsed, 1),
                        })
                        break  # one channel is enough — latch is set

            # ── OFFBOARD restored ─────────────────────────────────────────────
            if current_mode == "OFFBOARD":
                elapsed = time.monotonic() - t_start
                if not pilot_active:
                    log.info(
                        f"[OFFBOARD_PAUSE] OFFBOARD restored after {elapsed:.1f}s. "
                        "Pilot was static — resuming mission."
                    )
                    self._log_event("OFFBOARD_RESTORE_RESUME", {
                        "elapsed_s": round(elapsed, 1),
                    })
                    self._offboard_pause_active = False
                    # _offboard_aborted stays False → waypoint loop continues
                else:
                    log.warning(
                        f"[OFFBOARD_PAUSE] OFFBOARD restored after {elapsed:.1f}s "
                        "but pilot was active. Aborting → AUTO.RTL."
                    )
                    self._log_event("OFFBOARD_RESTORE_ABORT", {
                        "reason":    "pilot_active",
                        "elapsed_s": round(elapsed, 1),
                    })
                    self._offboard_pause_active = False
                    self._offboard_aborted      = True
                    with self._state_lock:
                        still_armed = self._last_armed
                    if still_armed:
                        self._command_mode("AUTO.RTL", "pilot_active_after_restore")
                return  # exit the pause loop regardless

        # ── Timeout reached ───────────────────────────────────────────────────
        elapsed = time.monotonic() - t_start
        reason  = "pilot_active_timeout" if pilot_active else "offboard_not_restored"
        log.critical(
            f"[OFFBOARD_PAUSE] Window expired after {elapsed:.1f}s. "
            f"Reason: {reason}. Aborting → AUTO.RTL."
        )
        self._log_event("OFFBOARD_PAUSE_TIMEOUT", {
            "elapsed_s":    round(elapsed, 1),
            "pilot_active": pilot_active,
            "reason":       reason,
        })
        self._offboard_pause_active = False
        self._offboard_aborted      = True
        with self._state_lock:
            still_armed = self._last_armed
        if still_armed:
            self._command_mode("AUTO.RTL", reason)

    # ══════════════════════════════════════════════════════════════════════════
    # MONITOR: SETPOINT WATCHDOG
    # ══════════════════════════════════════════════════════════════════════════

    def _setpoint_watchdog_loop(self) -> None:
        """
        Checks every SETPOINT_WATCHDOG_INTERVAL seconds that the flight loop
        is still updating self._sp_heartbeat.

        If the heartbeat is stale for more than SETPOINT_WATCHDOG_TIMEOUT
        seconds, the main loop has hung. The emergency client is used to
        command AUTO.LAND independently of the hung executor.

        The watchdog resets its baseline during the CONTEXT_MISSION pause
        window because setpoints are intentionally not published then.
        """
        time.sleep(5.0)  # give the flight loop time to start
        self._sp_heartbeat = time.monotonic()

        while not self._teardown_called:
            time.sleep(SETPOINT_WATCHDOG_INTERVAL)

            if self._pilot_override or self._teardown_called:
                break

            # During a pause window, setpoints are intentionally stopped.
            # Reset the heartbeat so we don't trigger a false watchdog alarm.
            if self._offboard_pause_active:
                self._sp_heartbeat = time.monotonic()
                continue

            age = time.monotonic() - self._sp_heartbeat
            if age > SETPOINT_WATCHDOG_TIMEOUT:
                log.critical(
                    f"[SETPOINT_WATCHDOG] Heartbeat stale for {age:.1f}s "
                    f"(limit {SETPOINT_WATCHDOG_TIMEOUT}s). "
                    "Main loop hung — commanding AUTO.LAND."
                )
                self._log_event("WATCHDOG_TRIGGER", {"stale_s": round(age, 2)})
                self._teardown("SETPOINT_WATCHDOG_STALE")
                break

    # ══════════════════════════════════════════════════════════════════════════
    # MONITOR: RC CH7 KILL SWITCH
    # ══════════════════════════════════════════════════════════════════════════

    def _rc_kill_monitor_loop(self) -> None:
        """
        Subscribes to /mavros/rc/in.

        Two jobs:
          1. Cache all RC channel values in self._last_rc_channels so the
             pilot-activity detector in _mission_offboard_pause() can read them.
          2. Trigger teardown if CH7 exceeds RC_KILL_THRESHOLD in either context.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from mavros_msgs.msg import RCIn

            node     = Node("sfm_rc_kill_monitor")
            ch_index = RC_KILL_CHANNEL - 1  # convert to 0-based
            qos      = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )

            def _rc_cb(msg: RCIn) -> None:
                # Always cache channels — needed by pilot-activity detection
                with self._state_lock:
                    self._last_rc_channels = list(msg.channels)

                if self._pilot_override or self._teardown_called:
                    return

                channels = list(msg.channels)
                if len(channels) <= ch_index:
                    return

                pwm = channels[ch_index]
                if pwm > RC_KILL_THRESHOLD:
                    log.critical(
                        f"[RC_KILL] CH{RC_KILL_CHANNEL} PWM={pwm} "
                        f"> {RC_KILL_THRESHOLD}. Kill switch active."
                    )
                    self._log_event("RC_KILL", {
                        "channel":   RC_KILL_CHANNEL,
                        "pwm":       pwm,
                        "threshold": RC_KILL_THRESHOLD,
                    })
                    threading.Thread(
                        target=self._teardown,
                        args=("RC_KILL_SWITCH",),
                        daemon=True,
                    ).start()

            node.create_subscription(RCIn, "/mavros/rc/in", _rc_cb, qos)
            rclpy.spin(node)

        except Exception as exc:
            log.error(f"[RC_KILL_MONITOR] Crashed: {exc}")

    # ══════════════════════════════════════════════════════════════════════════
    # MONITOR: ERRATIC MOTION
    # ══════════════════════════════════════════════════════════════════════════

    def _erratic_motion_monitor_loop(self) -> None:
        """
        Subscribes to /mavros/imu/data and /mavros/local_position/pose.

        Angular velocity check:
          Any sustained angular velocity magnitude above MOTION_ANGVEL_THRESHOLD
          (default 2.0 rad/s) for 0.5s triggers AUTO.LAND. This catches a flip
          or tumble early enough to cut props before the drone hits the ground.

        Altitude deviation check:
          If current altitude deviates from self._alt_setpoint by more than
          MOTION_ALT_DEVIATION (default 1.5m), triggers AUTO.LAND.
          The check is only active when the setpoint is above 0.3m so it
          doesn't fire on the ground.
        """
        try:
            import rclpy
            from rclpy.node        import Node
            from rclpy.qos         import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from sensor_msgs.msg   import Imu
            from geometry_msgs.msg import PoseStamped

            node = Node("sfm_erratic_motion_monitor")
            qos  = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            ANGVEL_SUSTAIN_S = 0.5

            def _imu_cb(msg: Imu) -> None:
                if self._pilot_override or self._teardown_called:
                    return
                wx  = msg.angular_velocity.x
                wy  = msg.angular_velocity.y
                wz  = msg.angular_velocity.z
                mag = (wx**2 + wy**2 + wz**2) ** 0.5

                with self._state_lock:
                    self._last_angvel_mag = mag

                if mag > MOTION_ANGVEL_THRESHOLD:
                    if self._angvel_above_since is None:
                        self._angvel_above_since = time.monotonic()
                    elif (time.monotonic() - self._angvel_above_since
                          >= ANGVEL_SUSTAIN_S):
                        log.critical(
                            f"[ERRATIC_MOTION] Angular velocity {mag:.2f} rad/s "
                            f"sustained {ANGVEL_SUSTAIN_S}s. Possible tumble."
                        )
                        self._log_event("MOTION_ANOMALY", {
                            "type":      "angular_velocity",
                            "value":     round(mag, 3),
                            "threshold": MOTION_ANGVEL_THRESHOLD,
                        })
                        threading.Thread(
                            target=self._teardown,
                            args=("ERRATIC_ANGULAR_VELOCITY",),
                            daemon=True,
                        ).start()
                else:
                    self._angvel_above_since = None

            def _pose_cb(msg: PoseStamped) -> None:
                if self._pilot_override or self._teardown_called:
                    return
                z = msg.pose.position.z
                with self._state_lock:
                    self._last_pose_z = z

                if self._alt_setpoint < 0.3:
                    return  # on the ground — skip deviation check

                alt_err = abs(z - self._alt_setpoint)
                if alt_err > MOTION_ALT_DEVIATION:
                    log.critical(
                        f"[ERRATIC_MOTION] Altitude {z:.2f}m deviates "
                        f"{alt_err:.2f}m from setpoint {self._alt_setpoint:.2f}m "
                        f"(limit {MOTION_ALT_DEVIATION}m)."
                    )
                    self._log_event("MOTION_ANOMALY", {
                        "type":       "altitude_deviation",
                        "current_z":  round(z, 3),
                        "setpoint_z": round(self._alt_setpoint, 3),
                        "deviation":  round(alt_err, 3),
                        "threshold":  MOTION_ALT_DEVIATION,
                    })
                    threading.Thread(
                        target=self._teardown,
                        args=("ALTITUDE_DEVIATION",),
                        daemon=True,
                    ).start()

            node.create_subscription(Imu, "/mavros/imu/data", _imu_cb, qos)
            node.create_subscription(
                PoseStamped, "/mavros/local_position/pose", _pose_cb, qos
            )
            rclpy.spin(node)

        except Exception as exc:
            log.error(f"[ERRATIC_MOTION_MONITOR] Crashed: {exc}")

    # ══════════════════════════════════════════════════════════════════════════
    # STRUCTURED FLIGHT EVENT LOG
    # ══════════════════════════════════════════════════════════════════════════

    def _log_event(self, event_type: str, extra: dict) -> None:
        now = time.time()
        with self._state_lock:
            mode   = self._last_mode
            armed  = self._last_armed
            pose_z = self._last_pose_z
            angvel = self._last_angvel_mag

        entry = {
            "t_s":        round(now, 3),
            "ts_iso":     datetime.fromtimestamp(now).isoformat(),
            "event_type": event_type,
            "script":     self._script_name,
            "pid":        self._pid,
            "context":    self._context,
            "mode":       mode,
            "armed":      armed,
            "pose_z":     round(pose_z, 3),
            "setpoint_z": round(self._alt_setpoint, 3),
            "angvel_mag": round(angvel, 3),
            "anomaly_flags": {
                "pilot_override":        self._pilot_override,
                "teardown_called":       self._teardown_called,
                "offboard_pause_active": self._offboard_pause_active,
                "offboard_aborted":      self._offboard_aborted,
            },
        }
        entry.update(extra)
        with self._event_lock:
            self._event_log.append(entry)

    def _flush_event_log(self) -> None:
        try:
            with self._event_lock:
                data = list(self._event_log)
            self._event_log_path.write_text(json.dumps(data, indent=2))
            log.info(f"[SafeFlightMixin] Event log → {self._event_log_path}")
        except Exception as exc:
            log.warning(f"[SafeFlightMixin] Event log write failed: {exc}")

        # ── Mirror to repo logs/ for dev visibility ───────────────────────────
        # Primary copy lives on SSD at _event_log_path.
        # A second copy is written to ~/unitree_lidar_project/logs/events/
        # so developers can inspect logs without mounting the SSD.
        # Failure is non-fatal and never propagates.
        try:
            repo_logs = (
                Path(__file__).resolve().parent.parent
                / "logs" / "events"
            )
            repo_logs.mkdir(parents=True, exist_ok=True)
            mirror_path = repo_logs / f"{self._session_tag}_events.json"
            mirror_path.write_text(json.dumps(data, indent=2))
            log.info(f"[SafeFlightMixin] Event log mirrored → {mirror_path}")
        except Exception as exc:
            log.debug(f"[SafeFlightMixin] Event log mirror failed (non-fatal): {exc}")

    # ══════════════════════════════════════════════════════════════════════════
    # FLIGHT LOGGER INTEGRATION
    # ══════════════════════════════════════════════════════════════════════════

    def _load_flight_logger(self):
        """
        Resolve and load flight_logger.py by path.
        Returns the module or None. Caches on self._flight_logger_mod.
        flight_logger.py lives at: <project_root>/utils/flight_logger.py
        safe_flight_mixin.py lives at: <project_root>/flight/safe_flight_mixin.py
        So: Path(__file__).parent.parent / "utils" / "flight_logger.py"
        """
        if self._flight_logger_mod is not None:
            return self._flight_logger_mod
        try:
            logger_path = (
                Path(__file__).resolve().parent.parent
                / "utils" / "flight_logger.py"
            )
            if not logger_path.exists():
                log.debug(f"[SafeFlightMixin] flight_logger not found at {logger_path}")
                return None
            spec = importlib.util.spec_from_file_location(
                "flight_logger", logger_path
            )
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._flight_logger_mod = mod
            return mod
        except Exception as exc:
            log.warning(f"[SafeFlightMixin] Could not load flight_logger: {exc}")
            return None

    def _logger_open_session(self) -> int:
        """
        Call flight_logger.open_session() to record the arm event.
        Returns assigned flight number, or 0 if logger unavailable.
        """
        mod = self._load_flight_logger()
        if mod is None:
            return 0
        try:
            flight_type = {
                "test":    mod.FLIGHT_TYPE_TEST,
                "mission": mod.FLIGHT_TYPE_AUTONOMUS,
            }.get(self._context, mod.FLIGHT_TYPE_UNKNOWN)

            return mod.open_session(
                session_id  = self._session_tag,
                script_name = self._script_name,
                flight_type = flight_type,
                bag_path    = str(self._session_dir),
                context     = self._context,
                pid         = self._pid,
            )
        except Exception as exc:
            log.warning(f"[SafeFlightMixin] open_session failed: {exc}")
            return 0

    def _logger_close_session(self, reason: str) -> None:
        """
        Call flight_logger.close_session() at teardown.
        Computes duration from the arm_time_unix stored in the session JSON.
        """
        mod = self._load_flight_logger()
        if mod is None:
            return
        try:
            duration_s = None
            record = mod.read_session_record(self._session_tag)
            if record and record.get("arm_time_unix"):
                duration_s = time.time() - record["arm_time_unix"]

            # bag_closed_cleanly is optimistic here — the bag stop happens
            # after this call. If stop fails, the watchdog's cleanup handles it.
            mod.close_session(
                session_id         = self._session_tag,
                end_reason         = reason,
                duration_s         = duration_s,
                bag_closed_cleanly = True,
                bag_path           = str(self._session_dir),
            )
        except Exception as exc:
            log.warning(f"[SafeFlightMixin] close_session failed: {exc}")


# ══════════════════════════════════════════════════════════════════════════════
# FLIGHT DATA SAMPLER — module-level class, instantiated by SafeFlightMixin
# ══════════════════════════════════════════════════════════════════════════════

class _FlightDataSampler:
    """
    2 Hz sampler. Writes one CSV row every 0.5s to:
      <session_dir>/flight_samples.csv

    Columns:
      timestamp_iso, t_flight_s,
      cmd_x, cmd_y, cmd_z,         what the flight script commanded
      actual_x, actual_y, actual_z ENU from EKF2 local_position/pose
      pos_error_m,                 3D distance between cmd and actual
      angvel_x, angvel_y, angvel_z, angvel_mag,
      rc_ch1..ch4, rc_ch7,         stick axes + kill switch PWM
      mode, armed

    Topics subscribed (BEST_EFFORT QoS, so no extra load on the bus):
      /mavros/setpoint_position/local
      /mavros/local_position/pose
      /mavros/imu/data
      /mavros/rc/in

    The rc/in subscription here does NOT replace the kill-switch monitor in
    the mixin — that monitor has its own independent Node and is authoritative
    for safety decisions. This sampler is read-only data recording.

    Source: MAVROS ROS 2 topic reference — geometry_msgs/PoseStamped,
    sensor_msgs/Imu, mavros_msgs/RCIn.
    """

    SAMPLE_INTERVAL = 0.5   # seconds (2 Hz)

    CSV_HEADER = (
        "timestamp_iso,t_flight_s,"
        "cmd_x,cmd_y,cmd_z,"
        "actual_x,actual_y,actual_z,"
        "pos_error_m,"
        "angvel_x,angvel_y,angvel_z,angvel_mag,"
        "rc_ch1,rc_ch2,rc_ch3,rc_ch4,rc_ch7,"
        "mode,armed\n"
    )

    def __init__(self, mixin: "SafeFlightMixin") -> None:
        self._mixin    = mixin
        self._csv_path = mixin._session_dir / "flight_samples.csv"

        self._lock     = threading.Lock()
        self._cmd_x = self._cmd_y = self._cmd_z = 0.0
        self._act_x = self._act_y = self._act_z = 0.0
        self._wx = self._wy = self._wz = 0.0
        self._rc_channels: list = []
        self._started_at = 0.0

        self._thread = threading.Thread(
            target=self._loop,
            name="flight_data_sampler",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def _loop(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg   import Imu
            from mavros_msgs.msg   import RCIn

            node = Node("sfm_data_sampler")
            qos  = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )

            def _sp_cb(msg: PoseStamped) -> None:
                with self._lock:
                    self._cmd_x = msg.pose.position.x
                    self._cmd_y = msg.pose.position.y
                    self._cmd_z = msg.pose.position.z

            def _pose_cb(msg: PoseStamped) -> None:
                with self._lock:
                    self._act_x = msg.pose.position.x
                    self._act_y = msg.pose.position.y
                    self._act_z = msg.pose.position.z

            def _imu_cb(msg: Imu) -> None:
                with self._lock:
                    self._wx = msg.angular_velocity.x
                    self._wy = msg.angular_velocity.y
                    self._wz = msg.angular_velocity.z

            def _rc_cb(msg: RCIn) -> None:
                with self._lock:
                    self._rc_channels = list(msg.channels)

            node.create_subscription(
                PoseStamped, "/mavros/setpoint_position/local", _sp_cb, qos
            )
            node.create_subscription(
                PoseStamped, "/mavros/local_position/pose", _pose_cb, qos
            )
            node.create_subscription(Imu, "/mavros/imu/data", _imu_cb, qos)
            node.create_subscription(RCIn, "/mavros/rc/in", _rc_cb, qos)

            # Write CSV header
            try:
                self._csv_path.write_text(self.CSV_HEADER)
            except Exception as exc:
                log.warning(f"[DataSampler] Could not create CSV: {exc}")
                node.destroy_node()
                return

            self._started_at = time.monotonic()

            spin_thread = threading.Thread(
                target=rclpy.spin, args=(node,), daemon=True
            )
            spin_thread.start()

            while not self._mixin._teardown_called:
                time.sleep(self.SAMPLE_INTERVAL)
                self._write_row()

            # Final row at teardown
            self._write_row()
            node.destroy_node()

        except Exception as exc:
            log.error(f"[DataSampler] Crashed: {exc}")

    def _write_row(self) -> None:
        try:
            now      = datetime.now()
            t_flight = round(time.monotonic() - self._started_at, 2)

            with self._lock:
                cx, cy, cz = self._cmd_x, self._cmd_y, self._cmd_z
                ax, ay, az = self._act_x, self._act_y, self._act_z
                wx, wy, wz = self._wx,    self._wy,    self._wz
                channels   = list(self._rc_channels)

            err  = ((cx-ax)**2 + (cy-ay)**2 + (cz-az)**2) ** 0.5
            wmag = (wx**2 + wy**2 + wz**2) ** 0.5

            def ch(i: int) -> int:
                return channels[i] if i < len(channels) else 0

            with self._mixin._state_lock:
                mode  = self._mixin._last_mode
                armed = self._mixin._last_armed

            row = (
                f"{now.isoformat()},{t_flight:.2f},"
                f"{cx:.4f},{cy:.4f},{cz:.4f},"
                f"{ax:.4f},{ay:.4f},{az:.4f},"
                f"{err:.4f},"
                f"{wx:.4f},{wy:.4f},{wz:.4f},{wmag:.4f},"
                f"{ch(0)},{ch(1)},{ch(2)},{ch(3)},{ch(6)},"
                f"{mode},{armed}\n"
            )
            with open(self._csv_path, "a") as f:
                f.write(row)
        except Exception as exc:
            log.debug(f"[DataSampler] Row write error: {exc}")
