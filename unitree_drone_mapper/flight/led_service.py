#!/usr/bin/env python3
"""
flight/led_service.py — DronePi LED Observer Service.

ARCHITECTURE
------------
This is the SOLE process that owns GPIO. It observes system state by reading
status files written by drone_watchdog.py, main.py, and postprocess_mesh.py,
derives the correct LED state using a priority table, and drives the hardware
accordingly.

Neither drone_watchdog.py nor main.py tells this service what colour to show.
They report facts about the system (FCU connected, armed, lock mode, Hailo
health, pipeline stage). This service decides what those facts mean visually.

ADDING NEW STATES
-----------------
To add a new LED pattern:
  1. Add a row to _PRIORITY_RULES in _derive_state() with your condition.
  2. Add the pattern to _PATTERNS dict (mode, channels, blink_hz, frames).
  3. That is all — no changes needed in watchdog, main, or postprocess.

STATUS FILES (read by this service)
-------------------------------------
  /tmp/dronepi_mission.lock       Written by watchdog/main. JSON with "mode" key.
                                  Modes: manual_scan, autonomous, bench_scan, absent

  /tmp/watchdog_status.json       Written by drone_watchdog.py every poll cycle.
                                  Schema: {
                                    "ts":            float,   # Unix timestamp (heartbeat)
                                    "fcu":           bool,    # FCU connected
                                    "armed":         bool,    # Vehicle armed
                                    "processing":    bool,    # Post-flight processing active
                                    "stack_running": bool,    # Flight stack running
                                    "led_state":     str,     # Transient override state name
                                    "led_until":     float,   # Expiry time for override
                                    "warning":       bool,
                                    "error":         bool,
                                    "critical":      bool,
                                    "system_failure":bool,
                                  }

  /tmp/main_status.json           Written by main.py every poll cycle.
                                  Schema: {
                                    "ts":             float,  # Unix timestamp (heartbeat)
                                    "hailo_active":   bool,
                                    "hailo_degraded": bool,
                                    "hailo_failed":   bool,
                                  }

  /tmp/postflight_status.json     Written by postprocess_mesh.py during pipeline.
                                  Schema: {
                                    "ts":      float,  # Unix timestamp
                                    "stage":   str,    # pipeline stage name
                                    "failed":  bool,   # True if pipeline crashed
                                    "done":    bool,   # True if pipeline completed
                                  }

PHYSICAL PIN MAP
----------------
  GND    →  Pin 34  (common ground)
  Green  →  BCM 25  (BOARD pin 22)
  Yellow →  BCM  8  (BOARD pin 24)
  Red    →  BCM  7  (BOARD pin 26)

  BCM numbers must match watchdog_core/led_controller.py.

HEARTBEAT TIMEOUTS
------------------
  WATCHDOG_DEAD_S = 5.0   Watchdog heartbeat stale → red slow blink
  MAIN_DEAD_S     = 8.0   Main heartbeat stale during autonomous → red fast blink
                          (longer timeout because main.py has longer blocking calls)

PRIORITY TABLE (top = highest priority)
-----------------------------------------
  1.  WATCHDOG_DEAD          — watchdog process gone
  2.  MAIN_DEAD              — main.py gone during autonomous
  3.  Transient override     — led_state / led_until written by watchdog
  4.  SYSTEM_FAILURE         — system_failure flag
  5.  CRITICAL               — critical flag
  6.  ERROR                  — error flag
  7.  WARNING                — warning flag
  8.  HAILO_FAILED           — Hailo hard failure
  9.  HAILO_DEGRADED         — Hailo degraded
  10. HAILO_ACTIVE           — Hailo active during autonomous
  11. POSTFLIGHT_FAILED      — pipeline crashed
  12. POSTFLIGHT_DONE        — pipeline complete (timed)
  13. POSTFLIGHT_RUNNING     — pipeline in progress
  14. SCANNING               — stack running / autonomous active
  15. PROCESSING             — watchdog post-processing flag (legacy)
  16. WAITING_FCU            — FCU not yet connected (only after first heartbeat)
  17. OFF / IDLE             — normal idle

USAGE
-----
  Managed by systemd led.service. See flight/led.service.
  Manual run (stop led.service first):
    GPIOZERO_PIN_FACTORY=lgpio python3 flight/led_service.py

REQUIREMENTS
------------
  pip install gpiozero lgpio   (inside the dronepi conda env)
"""

import json
import logging
import os
import sys
import time
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Backend must be forced before any gpiozero import.
# ---------------------------------------------------------------------------
os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")
os.environ.setdefault("HOME", "/tmp")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LED] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

WATCHDOG_STATUS_FILE   = "/tmp/watchdog_status.json"
MAIN_STATUS_FILE       = "/tmp/main_status.json"
MISSION_LOCK_FILE      = "/tmp/dronepi_mission.lock"
POSTFLIGHT_STATUS_FILE = "/tmp/postflight_status.json"

POLL_HZ         = 10      # State derivation rate
WATCHDOG_DEAD_S = 5.0     # Watchdog heartbeat timeout (seconds)
MAIN_DEAD_S     = 8.0     # Main heartbeat timeout during autonomous (seconds)

# How long to hold POSTFLIGHT_DONE before dropping back to IDLE.
# Long enough to be clearly seen; short enough not to be annoying.
POSTFLIGHT_DONE_HOLD_S = 5.0

_BCM_PINS: dict[str, int] = {
    "green":  25,
    "yellow": 8,
    "red":    7,
}

# ---------------------------------------------------------------------------
# Pattern registry
#
# mode      : "off" | "solid" | "blink" | "sequence"
# channels  : tuple of channel names to drive
# blink_hz  : blink rate; only used when mode="blink"
# frames    : sequence frames as ((channels_tuple, duration_s), ...)
#             only used when mode="sequence"
#
# To add a new visual state:
#   1. Add an entry here.
#   2. Add a priority rule in _derive_state().
# ---------------------------------------------------------------------------

@dataclass
class _Pattern:
    mode:     str
    channels: tuple = ()
    blink_hz: Optional[float] = None
    frames:   tuple = ()

    @property
    def half_cycle(self) -> Optional[float]:
        return (1.0 / self.blink_hz / 2.0) if self.blink_hz else None


_PATTERNS: dict[str, _Pattern] = {
    # ── System states ─────────────────────────────────────────────────────────
    "OFF":            _Pattern("off"),
    "SYSTEM_START":   _Pattern(
        "sequence",
        frames=(
            (("green",),  0.22),
            (tuple(),     0.06),
            (("yellow",), 0.22),
            (tuple(),     0.06),
            (("red",),    0.22),
            (tuple(),     0.06),
        ),
    ),
    "IDLE":           _Pattern("solid",  channels=("green",)),
    "WAITING_FCU":    _Pattern("blink",  channels=("yellow",), blink_hz=1.5),
    "SCAN_READY":     _Pattern("solid",  channels=("green", "yellow")),
    "SCAN_START":     _Pattern("blink",  channels=("green", "yellow"), blink_hz=2.5),
    "SCANNING":       _Pattern("blink",  channels=("green",), blink_hz=1.5),
    "SCAN_FINISHED":  _Pattern("blink",  channels=("green", "yellow"), blink_hz=2.0),
    "PROCESSING":     _Pattern("solid",  channels=("yellow",)),
    "WARNING":        _Pattern("blink",  channels=("yellow",), blink_hz=2.5),
    "ERROR":          _Pattern("solid",  channels=("red",)),
    "CRITICAL":       _Pattern("blink",  channels=("green", "yellow", "red"), blink_hz=2.5),
    "SYSTEM_FAILURE": _Pattern("blink",  channels=("green", "yellow", "red"), blink_hz=5.0),
    # ── Hailo states ──────────────────────────────────────────────────────────
    "HAILO_ACTIVE":   _Pattern("solid",  channels=("green", "yellow")),
    "HAILO_DEGRADED": _Pattern("blink",  channels=("yellow",), blink_hz=3.0),
    "HAILO_FAILED":   _Pattern("blink",  channels=("red",),    blink_hz=3.0),
    # ── Fault / process-health states ─────────────────────────────────────────
    "WATCHDOG_DEAD":  _Pattern("blink",  channels=("red",),    blink_hz=0.8),
    "MAIN_DEAD":      _Pattern("blink",  channels=("red",),    blink_hz=3.0),
    # ── Post-flight pipeline states ───────────────────────────────────────────
    # POSTFLIGHT_RUNNING: slow yellow blink while pipeline runs
    "POSTFLIGHT_RUNNING": _Pattern("blink", channels=("yellow",), blink_hz=1.0),
    # POSTFLIGHT_DONE: green + yellow solid for POSTFLIGHT_DONE_HOLD_S seconds
    "POSTFLIGHT_DONE":    _Pattern("solid", channels=("green", "yellow")),
    # POSTFLIGHT_FAILED: fast red blink
    "POSTFLIGHT_FAILED":  _Pattern("blink", channels=("red",), blink_hz=3.0),
}


# ---------------------------------------------------------------------------
# System state snapshot — populated by reading status files each poll cycle
# ---------------------------------------------------------------------------

@dataclass
class _SystemSnapshot:
    """Decoded view of all system status files at one point in time."""
    # Watchdog fields
    watchdog_ts:           float = 0.0
    watchdog_fcu:          bool  = False
    watchdog_armed:        bool  = False
    watchdog_processing:   bool  = False
    watchdog_stack_running: bool = False
    led_state:             str   = ""
    led_until:             float = 0.0
    warning:               bool  = False
    error:                 bool  = False
    critical:              bool  = False
    system_failure:        bool  = False
    # Main fields
    main_ts:               float = 0.0
    hailo_active:          bool  = False
    hailo_degraded:        bool  = False
    hailo_failed:          bool  = False
    # Lock file
    lock_mode:             str   = ""
    # Post-flight pipeline
    pf_ts:                 float = 0.0
    pf_stage:              str   = ""
    pf_failed:             bool  = False
    pf_done:               bool  = False


def _read_json(path: str) -> dict:
    """Read a JSON file safely. Returns empty dict on any error."""
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def _read_snapshot() -> _SystemSnapshot:
    """
    Read all status files and return a unified system snapshot.

    All reads are non-blocking and fault-tolerant — a missing or malformed
    file returns zeros/defaults rather than raising an exception.
    """
    s = _SystemSnapshot()

    wd = _read_json(WATCHDOG_STATUS_FILE)
    s.watchdog_ts            = float(wd.get("ts",             0.0))
    s.watchdog_fcu           = bool (wd.get("fcu",            False))
    s.watchdog_armed         = bool (wd.get("armed",          False))
    s.watchdog_processing    = bool (wd.get("processing",     False))
    s.watchdog_stack_running = bool (wd.get("stack_running",  False))
    s.led_state              = str  (wd.get("led_state",      ""))
    s.led_until              = float(wd.get("led_until",      0.0))
    s.warning                = bool (wd.get("warning",        False))
    s.error                  = bool (wd.get("error",          False))
    s.critical               = bool (wd.get("critical",       False))
    s.system_failure         = bool (wd.get("system_failure", False))

    mn = _read_json(MAIN_STATUS_FILE)
    s.main_ts        = float(mn.get("ts",             0.0))
    s.hailo_active   = bool (mn.get("hailo_active",   False))
    s.hailo_degraded = bool (mn.get("hailo_degraded", False))
    s.hailo_failed   = bool (mn.get("hailo_failed",   False))

    lock        = _read_json(MISSION_LOCK_FILE)
    s.lock_mode = str(lock.get("mode", ""))

    pf          = _read_json(POSTFLIGHT_STATUS_FILE)
    s.pf_ts     = float(pf.get("ts",     0.0))
    s.pf_stage  = str  (pf.get("stage",  ""))
    s.pf_failed = bool (pf.get("failed", False))
    s.pf_done   = bool (pf.get("done",   False))

    return s


# ---------------------------------------------------------------------------
# State derivation — priority table
# ---------------------------------------------------------------------------

def _derive_state(snap: _SystemSnapshot, now: float) -> str:
    """
    Derive the correct LED state from the current system snapshot.

    Rules are evaluated top-to-bottom. The first matching rule wins.
    Add new conditions here and a matching pattern in _PATTERNS — no other
    file needs to change.

    Parameters
    ----------
    snap : _SystemSnapshot
        Decoded view of all status files at this poll cycle.
    now : float
        Current time.time() — all staleness checks use the same reference.

    Returns
    -------
    str
        Key into _PATTERNS.
    """
    watchdog_age = (now - snap.watchdog_ts) if snap.watchdog_ts > 0 else float("inf")
    main_age     = (now - snap.main_ts)     if snap.main_ts     > 0 else float("inf")

    # ── Priority 1: Watchdog process dead ─────────────────────────────────────
    # Only fires once at least one valid heartbeat has been seen (ts > 0),
    # preventing false alarms during the systemd startup window.
    if snap.watchdog_ts > 0 and watchdog_age > WATCHDOG_DEAD_S:
        return "WATCHDOG_DEAD"

    # ── Priority 2: Main process dead during autonomous mission ───────────────
    if snap.lock_mode == "autonomous" and snap.main_ts > 0 and main_age > MAIN_DEAD_S:
        return "MAIN_DEAD"

    # ── Priority 3: Transient override from watchdog ──────────────────────────
    # The watchdog writes led_state + led_until for boot-sequence animations
    # (SYSTEM_START, SCAN_READY, SCAN_START, SCAN_FINISHED) that are too brief
    # to be reliably derived from structural state alone.
    if snap.led_state and snap.led_state in _PATTERNS and now < snap.led_until:
        return snap.led_state

    # ── Priority 4: System failure ────────────────────────────────────────────
    if snap.system_failure:
        return "SYSTEM_FAILURE"

    # ── Priority 5: Critical ──────────────────────────────────────────────────
    if snap.critical:
        return "CRITICAL"

    # ── Priority 6: Error ─────────────────────────────────────────────────────
    if snap.error:
        return "ERROR"

    # ── Priority 7: Warning ───────────────────────────────────────────────────
    if snap.warning:
        return "WARNING"

    # ── Priority 8: Hailo hard failure ────────────────────────────────────────
    if snap.hailo_failed:
        return "HAILO_FAILED"

    # ── Priority 9: Hailo degraded ────────────────────────────────────────────
    if snap.hailo_degraded:
        return "HAILO_DEGRADED"

    # ── Priority 10: Hailo active during autonomous flight ────────────────────
    if snap.lock_mode == "autonomous" and snap.hailo_active:
        return "HAILO_ACTIVE"

    # ── Priority 11: Post-flight pipeline failed ──────────────────────────────
    # pf_ts > 0 guards against stale files from a previous boot.
    if snap.pf_ts > 0 and snap.pf_failed:
        return "POSTFLIGHT_FAILED"

    # ── Priority 12: Post-flight pipeline complete ────────────────────────────
    # Hold for POSTFLIGHT_DONE_HOLD_S after completion, then drop to IDLE.
    if snap.pf_ts > 0 and snap.pf_done:
        age = now - snap.pf_ts
        if age < POSTFLIGHT_DONE_HOLD_S:
            return "POSTFLIGHT_DONE"
        # Timed out — fall through to IDLE below

    # ── Priority 13: Post-flight pipeline running ─────────────────────────────
    if snap.pf_ts > 0 and snap.pf_stage and not snap.pf_done and not snap.pf_failed:
        return "POSTFLIGHT_RUNNING"

    # ── Priority 14: Flight stack active ──────────────────────────────────────
    if snap.watchdog_stack_running or snap.lock_mode in ("manual_scan", "autonomous"):
        return "SCANNING"

    # ── Priority 15: Legacy watchdog processing flag ──────────────────────────
    # Fires when watchdog.is_active is True but pf_status file is absent or stale.
    # Kept for backward compatibility with runs that pre-date postflight_status.json.
    if snap.watchdog_processing:
        return "PROCESSING"

    # ── Priority 16: FCU not yet connected ────────────────────────────────────
    # Guard watchdog_ts > 0 prevents false WAITING_FCU at boot before the
    # watchdog has written its first heartbeat.
    if snap.watchdog_ts > 0 and not snap.watchdog_fcu:
        return "WAITING_FCU"

    # ── Priority 17: Watchdog not yet started (boot grace window) ─────────────
    if snap.watchdog_ts == 0:
        return "OFF"

    # ── Priority 18: Default — system healthy, no active task ─────────────────
    return "IDLE"


# ---------------------------------------------------------------------------
# Pattern thread — one instance, replaced on state change
# ---------------------------------------------------------------------------

class _PatternThread(threading.Thread):
    """
    Daemon thread that drives one or more LED channels according to a pattern.

    Supports three active modes:
      blink    — toggles the named channels at a fixed rate
      sequence — cycles through a list of (channels, duration) frames

    Uses threading.Event.wait() so stop() causes an immediate wakeup rather
    than waiting for the current sleep to expire. Drives all pins LOW on exit
    so pin state is always determinate after stop() + join().
    """

    def __init__(self, leds: dict, pattern: _Pattern) -> None:
        super().__init__(daemon=True)
        self._leds       = leds
        self._pattern    = pattern
        self._stop_event = threading.Event()

    def _all_off(self) -> None:
        for led in self._leds.values():
            try:
                led.off()
            except Exception:
                pass

    def _set_channels(self, channels: tuple) -> None:
        self._all_off()
        for ch in channels:
            if ch in self._leds:
                self._leds[ch].on()

    def run(self) -> None:
        if self._pattern.mode == "blink":
            half = self._pattern.half_cycle or 0.333
            while not self._stop_event.is_set():
                self._set_channels(self._pattern.channels)
                if self._stop_event.wait(half):
                    break
                self._all_off()
                if self._stop_event.wait(half):
                    break

        elif self._pattern.mode == "sequence":
            while not self._stop_event.is_set():
                for channels, duration in self._pattern.frames:
                    self._set_channels(channels)
                    if self._stop_event.wait(duration):
                        self._all_off()
                        return

        self._all_off()

    def stop(self) -> None:
        self._stop_event.set()


# ---------------------------------------------------------------------------
# LED hardware driver
# ---------------------------------------------------------------------------

class LEDDriver:
    """
    Hardware abstraction for the three-channel LED indicator.

    Instantiated once at service startup. apply() is the only public method —
    it accepts a state name string, looks up the pattern, and drives hardware.
    All GPIO operations are internal to this class.

    Extending:
        Add a new entry to _PATTERNS at module level, then call
        driver.apply("YOUR_NEW_STATE") from _derive_state(). No changes to
        this class are required.
    """

    def __init__(self) -> None:
        from gpiozero import LED
        self._leds: dict[str, object] = {
            name: LED(bcm) for name, bcm in _BCM_PINS.items()
        }
        self._thread:  Optional[_PatternThread] = None
        self._current: str = ""
        self._all_off()
        logger.info(
            "GPIO initialised — BCM G:%d Y:%d R:%d",
            _BCM_PINS["green"], _BCM_PINS["yellow"], _BCM_PINS["red"],
        )

    def apply(self, state_name: str) -> None:
        """
        Transition to the named LED state.

        No-op if already in that state — avoids unnecessary thread restarts
        on every poll cycle. Unknown state names are logged and treated as OFF
        so the service never crashes on an unrecognised string.

        Parameters
        ----------
        state_name : str
            Must be a key in _PATTERNS. Unknown keys degrade to OFF.
        """
        if state_name == self._current:
            return

        if state_name not in _PATTERNS:
            logger.warning(
                "Unknown state '%s' — treating as OFF. Add to _PATTERNS.",
                state_name,
            )
            state_name = "OFF"

        pattern = _PATTERNS[state_name]

        self._stop_worker()
        self._all_off()

        if pattern.mode == "solid":
            for ch in pattern.channels:
                if ch in self._leds:
                    self._leds[ch].on()

        elif pattern.mode in ("blink", "sequence"):
            self._thread = _PatternThread(self._leds, pattern)
            self._thread.start()
        # mode == "off" falls through — all_off already called above

        self._current = state_name
        logger.info("State → %s", state_name)

    def cleanup(self) -> None:
        """Drive all pins LOW, stop pattern thread, release GPIO resources."""
        self._stop_worker()
        self._all_off()
        for led in self._leds.values():
            try:
                led.close()
            except Exception:
                pass
        self._leds.clear()
        logger.info("GPIO released.")

    def _stop_worker(self) -> None:
        if self._thread is None:
            return
        self._thread.stop()
        # Join with a generous timeout: worst case is one full sequence cycle.
        # The longest single frame in _PATTERNS is 0.22s; a full sequence
        # (6 frames) takes 1.56s. 2.0s covers that plus scheduling jitter.
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            logger.warning("Pattern thread did not exit within timeout.")
        self._thread = None

    def _all_off(self) -> None:
        for led in self._leds.values():
            try:
                led.off()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main service loop
# ---------------------------------------------------------------------------

def main() -> int:
    logger.info("=" * 52)
    logger.info("  DronePi LED Observer Service")
    logger.info("  Watchdog timeout : %.1fs → WATCHDOG_DEAD", WATCHDOG_DEAD_S)
    logger.info("  Main timeout     : %.1fs → MAIN_DEAD (autonomous only)", MAIN_DEAD_S)
    logger.info("  Poll rate        : %d Hz", POLL_HZ)
    logger.info("  Post-flight done hold: %.1fs → POSTFLIGHT_DONE", POSTFLIGHT_DONE_HOLD_S)
    logger.info("=" * 52)

    try:
        driver = LEDDriver()
    except Exception as exc:
        logger.error("GPIO init failed: %s", exc)
        logger.error("Ensure lgpio is installed and user is in the gpio group.")
        return 1

    poll_interval = 1.0 / POLL_HZ

    try:
        while True:
            snap  = _read_snapshot()
            state = _derive_state(snap, now=time.time())
            driver.apply(state)
            time.sleep(poll_interval)

    except KeyboardInterrupt:
        logger.info("Interrupted.")
    finally:
        driver.cleanup()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
