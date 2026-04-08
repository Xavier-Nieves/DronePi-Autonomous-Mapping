"""watchdog_core/buzzer.py — semantic buzzer tunes, periodic beepers, fault alarms,
and post-flight pipeline stage driver.

QBASIC Format 1 tunes for Pixhawk / MAVROS.

Tune catalogue
--------------
Event tones (one-shot, fired by drone_watchdog.py):
  TUNE_SYSTEM_START       Boot fanfare — ascending arpeggio
  TUNE_SCAN_READY         Short neutral ack — system ready for input
  TUNE_SCAN_START         Upward medium ramp — action initiated
  TUNE_SCAN_ACTIVE        Short repetitive tick — scan heartbeat (periodic)
  TUNE_SCAN_FINISHED      Descending stop tone — scan ended
  TUNE_POSTPROCESS_ACTIVE Short high tick — post-processing heartbeat (periodic)
  TUNE_POSTPROCESS_DONE   Happy completion chord
  TUNE_WARNING            One-shot caution double-beep

Post-flight pipeline stage tones (fired by PostflightBuzzerDriver):
  TUNE_PF_EXTRACT         Short chirp — bag extraction complete
  TUNE_PF_CLASSIFY        Ascending two-note — ground classification complete
  TUNE_PF_MESH_STAGE      Medium confirm — DTM or DSM build complete

Fault tones (repeating while active, via FaultAlarmManager):
  TUNE_ERROR              Repeating low unpleasant tone (armed only)
  TUNE_CRITICAL           Repeating descending unsettling (armed only)
  TUNE_SYSTEM_FAILURE     Repeating alarm (armed or startup)

Buzzer → LED state correspondence
----------------------------------
Every buzzer event maps to a matching LED state so audio and visual
feedback are always coherent. See flight/led_service.py _PATTERNS for
the visual half of each pair.

  TUNE_SYSTEM_START       ↔  LED SYSTEM_START  (sequence animation)
  TUNE_SCAN_READY         ↔  LED SCAN_READY    (green+yellow solid)
  TUNE_SCAN_START         ↔  LED SCAN_START    (green+yellow blink 2.5 Hz)
  TUNE_SCAN_ACTIVE        ↔  LED SCANNING      (green blink 1.5 Hz)
  TUNE_SCAN_FINISHED      ↔  LED SCAN_FINISHED (green+yellow blink 2.0 Hz)
  TUNE_PF_EXTRACT         ↔  LED POSTFLIGHT_RUNNING (yellow blink 1 Hz)
  TUNE_PF_CLASSIFY        ↔  LED POSTFLIGHT_RUNNING (yellow blink 1 Hz)
  TUNE_PF_MESH_STAGE      ↔  LED POSTFLIGHT_RUNNING (yellow blink 1 Hz)
  TUNE_POSTPROCESS_DONE   ↔  LED POSTFLIGHT_DONE    (green+yellow solid)
  TUNE_ERROR              ↔  LED ERROR / POSTFLIGHT_FAILED / HAILO_FAILED
  TUNE_WARNING            ↔  LED WARNING / HAILO_DEGRADED
  TUNE_CRITICAL           ↔  LED CRITICAL
  TUNE_SYSTEM_FAILURE     ↔  LED SYSTEM_FAILURE

Architecture notes
------------------
- PostflightBuzzerDriver is the only class that reads /tmp/postflight_status.json.
  It is poll-driven from drone_watchdog.py's main loop — no extra thread required.
- ScanBeeper and PostflightBeeper are PeriodicBeeper subclasses that own their
  own daemon thread. They are started/stopped by the watchdog or main.py.
- FaultAlarmManager owns one daemon thread that fires repeating fault tones.
- Nothing in this module touches GPIO or ROS — all output goes through the
  play_tune_fn callable injected at construction time.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Callable, Optional

TUNE_FORMAT = 1  # QBASIC Format 1

# ── Event tones ───────────────────────────────────────────────────────────────

TUNE_SYSTEM_START       = "T180L8O4CEGCEG>C"   # ascending fanfare
TUNE_SCAN_READY         = "T250L32O5CE"         # short neutral ack
TUNE_SCAN_START         = "T200L12O5CEG>C"      # upward ramp — action started
TUNE_SCAN_ACTIVE        = "T200L64O4GE"         # periodic scan tick
TUNE_SCAN_FINISHED      = "T180L8O5C<GEG<C"     # descending stop
TUNE_POSTPROCESS_ACTIVE = "T220L64O5C"          # periodic postflight tick
TUNE_POSTPROCESS_DONE   = "T180L12O5CEGCE"      # happy completion
TUNE_WARNING            = "T200L32O5CC"         # one-shot caution

# ── Post-flight pipeline stage tones ─────────────────────────────────────────
# These fire once each time the pipeline advances to a named stage.
# They are quieter / shorter than event tones so they do not overwhelm
# the periodic TUNE_POSTPROCESS_ACTIVE tick that runs concurrently.

TUNE_PF_EXTRACT     = "T260L32O5C"             # bag extraction done — single chirp
TUNE_PF_CLASSIFY    = "T220L24O4EG"            # ground classification done — ascending 2-note
TUNE_PF_MESH_STAGE  = "T200L16O5CE"            # DTM or DSM build done — short confirm

# ── Fault tones ───────────────────────────────────────────────────────────────

TUNE_ERROR          = "T140L4O4C<C"            # long low unpleasant
TUNE_CRITICAL       = "T180L8O6C<A<F"          # descending unsettling
TUNE_SYSTEM_FAILURE = "T220L8O6CECECE"         # repeating alarm

# ── Intervals ─────────────────────────────────────────────────────────────────

SCAN_BEEP_INTERVAL_S         = 1.0
POSTPROCESS_BEEP_INTERVAL_S  = 1.0
ERROR_REPEAT_INTERVAL_S      = 2.0
CRITICAL_REPEAT_INTERVAL_S   = 1.25
SYSTEM_FAILURE_INTERVAL_S    = 0.85

# Stages that receive a one-shot stage-transition tone from PostflightBuzzerDriver.
# Stages absent from this map produce only the background heartbeat tick.
# mls, cap, merge, publish are intentionally excluded — they are instantaneous
# or already wrapped between two named milestones that do fire a tone.
_PF_STAGE_TONES: dict[str, str] = {
    "bag_extract":     TUNE_PF_EXTRACT,
    "ground_classify": TUNE_PF_CLASSIFY,
    "dtm":             TUNE_PF_MESH_STAGE,
    "dsm":             TUNE_PF_MESH_STAGE,
}

_PF_STATUS_FILE = "/tmp/postflight_status.json"


# ── Internal helpers ──────────────────────────────────────────────────────────

def _sleep_responsive(active_fn: Callable[[], bool], seconds: float) -> None:
    """Sleep for `seconds` but wake immediately if active_fn() returns False."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        if not active_fn():
            return
        time.sleep(0.1)


# ── PeriodicBeeper ────────────────────────────────────────────────────────────

class PeriodicBeeper:
    """Generic background beeper that repeats one tune at a fixed interval."""

    def __init__(
        self,
        play_tune_fn: Callable[[str], None],
        tune: str,
        interval_s: float,
    ) -> None:
        self._play      = play_tune_fn
        self._tune      = tune
        self._interval  = interval_s
        self._active    = False
        self._thread:   Optional[threading.Thread] = None

    @property
    def active(self) -> bool:
        return self._active

    def start(self) -> None:
        if self._active:
            return
        self._active = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._active = False
        if self._thread:
            self._thread.join(timeout=self._interval + 1.0)
            self._thread = None

    def _loop(self) -> None:
        while self._active:
            try:
                self._play(self._tune)
            except Exception:
                pass
            _sleep_responsive(lambda: self._active, self._interval)


class ScanBeeper(PeriodicBeeper):
    """Repeats TUNE_SCAN_ACTIVE every second while a scan is in progress.

    Used by both drone_watchdog.py (manual scan) and main.py (autonomous scan)
    so audio feedback is consistent regardless of which process owns the stack.
    """

    def __init__(self, play_tune_fn: Callable[[str], None]) -> None:
        super().__init__(play_tune_fn, TUNE_SCAN_ACTIVE, SCAN_BEEP_INTERVAL_S)


class PostflightBeeper(PeriodicBeeper):
    """Repeats TUNE_POSTPROCESS_ACTIVE every second during post-flight processing.

    Runs as the background heartbeat tick while the pipeline is executing.
    PostflightBuzzerDriver fires additional one-shot stage tones on top of
    this background tick — the two run concurrently without conflict because
    the Pixhawk QBASIC queue accepts overlapping tune requests.
    """

    def __init__(self, play_tune_fn: Callable[[str], None]) -> None:
        super().__init__(
            play_tune_fn, TUNE_POSTPROCESS_ACTIVE, POSTPROCESS_BEEP_INTERVAL_S
        )


# ── PostflightBuzzerDriver ────────────────────────────────────────────────────

class PostflightBuzzerDriver:
    """Poll-driven post-flight pipeline stage buzzer.

    Reads /tmp/postflight_status.json on every call to poll() and fires
    a one-shot stage-transition tone whenever the pipeline advances to a
    new named stage. Also manages the PostflightBeeper background tick.

    This class is poll-driven rather than thread-driven so it integrates
    cleanly into drone_watchdog.py's existing main loop without adding
    another daemon thread.

    Usage (from drone_watchdog.py main loop):
        pf_buzzer = PostflightBuzzerDriver(
            play_tune_fn=lambda tune: _play_tune(reader, tune),
        )
        # inside the loop:
        pf_buzzer.poll()

    Buzzer → LED correspondence for post-flight pipeline:
        Stage advance  → one-shot stage tone  + LED POSTFLIGHT_RUNNING (yellow 1 Hz)
        Pipeline done  → TUNE_POSTPROCESS_DONE + LED POSTFLIGHT_DONE  (green+yellow)
        Pipeline failed→ TUNE_ERROR            + LED POSTFLIGHT_FAILED (red 3 Hz)

    The done/failed tones are played here directly from the IPC file so they
    fire whether postprocess_mesh.py was launched by the watchdog or manually
    from the terminal. _done_latched / _fail_latched prevent double-play if
    PostflightMonitor._monitor() already fired the tone via subprocess exit.
    """

    def __init__(self, play_tune_fn: Callable[[str], None]) -> None:
        self._play          = play_tune_fn
        self._beeper        = PostflightBeeper(play_tune_fn)
        self._last_stage:   str  = ""
        self._running:      bool = False
        self._done_latched: bool = False   # True once done tone fired this session
        self._fail_latched: bool = False   # True once fail tone fired this session

    def poll(self) -> None:
        """
        Read pipeline status and fire tones on state transitions.

        Called by drone_watchdog.py on every POLL_HZ cycle. Reads
        /tmp/postflight_status.json and reacts to stage changes.
        Safe to call even when no pipeline is running — returns immediately
        if the status file is absent or stale.
        """
        try:
            raw  = Path(_PF_STATUS_FILE).read_text()
            data = json.loads(raw)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            # No pipeline running or file not yet written — stop any beeper
            # that was running from a previous session.
            if self._running:
                self._beeper.stop()
                self._running    = False
                self._last_stage = ""
            return

        stage  = str (data.get("stage",  ""))
        failed = bool(data.get("failed", False))
        done   = bool(data.get("done",   False))

        # ── pipeline ended (done or failed) ───────────────────────────────────
        # Play the terminal tone here so it fires whether the pipeline was
        # launched by the watchdog (PostflightMonitor._monitor()) or manually
        # from the terminal. Latches prevent double-play if _monitor() already
        # fired the tone via subprocess exit code.
        if done or failed:
            if self._running:
                self._beeper.stop()
                self._running    = False
                self._last_stage = ""

            if done and not self._done_latched:
                self._done_latched = True
                self._fail_latched = False
                try:
                    self._play(TUNE_POSTPROCESS_DONE)
                except Exception:
                    pass
            elif failed and not self._fail_latched:
                self._fail_latched = True
                self._done_latched = False
                try:
                    self._play(TUNE_ERROR)
                except Exception:
                    pass
            return

        # ── pipeline running ───────────────────────────────────────────────────
        if not stage:
            return

        # Start the background heartbeat tick on first stage detection.
        # Reset done/fail latches so each new pipeline session gets its own
        # terminal tone regardless of what the previous run did.
        if not self._running:
            self._beeper.start()
            self._running      = True
            self._done_latched = False
            self._fail_latched = False

        # Fire a one-shot stage-transition tone when the stage name changes
        if stage != self._last_stage:
            self._last_stage = stage
            tone = _PF_STAGE_TONES.get(stage)
            if tone:
                try:
                    self._play(tone)
                except Exception:
                    pass

    def stop(self) -> None:
        """Stop all pipeline audio. Call on watchdog shutdown."""
        self._beeper.stop()
        self._running      = False
        self._last_stage   = ""
        self._done_latched = False
        self._fail_latched = False


# ── FaultAlarmManager ─────────────────────────────────────────────────────────

class FaultAlarmManager:
    """Manages one-shot warning plus repeating fault alarms.

    Repeating alarms only sound while their fault is active and the system is
    armed, except system_failure which may optionally repeat during startup.

    Priority: SYSTEM_FAILURE > CRITICAL > ERROR.
    WARNING is always one-shot (latched until cleared).
    """

    def __init__(
        self,
        play_tune_fn:            Callable[[str], None],
        is_armed_fn:             Callable[[], bool],
        startup_failure_repeats: bool = True,
    ) -> None:
        self._play                   = play_tune_fn
        self._is_armed               = is_armed_fn
        self._startup_failure_repeats = startup_failure_repeats

        self._warning_latched  = False
        self._error_active     = False
        self._critical_active  = False
        self._failure_active   = False

        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)

    def set_warning(self, active: bool = True) -> None:
        """Latch warning — plays TUNE_WARNING once and holds until cleared."""
        if active and not self._warning_latched:
            self._warning_latched = True
            try:
                self._play(TUNE_WARNING)
            except Exception:
                pass
        elif not active:
            self._warning_latched = False

    def set_error(self, active: bool) -> None:
        self._error_active = bool(active)

    def set_critical(self, active: bool) -> None:
        self._critical_active = bool(active)

    def set_system_failure(self, active: bool) -> None:
        self._failure_active = bool(active)

    def clear_all(self) -> None:
        self._warning_latched = False
        self._error_active    = False
        self._critical_active = False
        self._failure_active  = False

    def _current_alarm(self) -> tuple[Optional[str], float]:
        if self._failure_active:
            if self._is_armed() or self._startup_failure_repeats:
                return TUNE_SYSTEM_FAILURE, SYSTEM_FAILURE_INTERVAL_S
        if not self._is_armed():
            return None, 0.1
        if self._critical_active:
            return TUNE_CRITICAL, CRITICAL_REPEAT_INTERVAL_S
        if self._error_active:
            return TUNE_ERROR, ERROR_REPEAT_INTERVAL_S
        return None, 0.1

    def _loop(self) -> None:
        while self._running:
            tune, interval_s = self._current_alarm()
            if tune is None:
                time.sleep(interval_s)
                continue
            try:
                self._play(tune)
            except Exception:
                pass
            _sleep_responsive(lambda: self._running, interval_s)
