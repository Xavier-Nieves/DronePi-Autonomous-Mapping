#!/usr/bin/env python3
"""
flight/led_service.py — DronePi LED Observer Service.

BOOT SEQUENCE CHANGE
--------------------
A new BOOTING LED state covers the MAVROS wait window (~60 s) between
SYSTEM_START finishing and the FCU connecting. This lets the user
distinguish three separate conditions that previously all looked like
yellow blink:

  SYSTEM_START   → cycling RGB animation  (first 2.5 s, watchdog just started)
  BOOTING        → green slow blink 0.5 Hz (MAVROS wait, services initialising)
  WAITING_FCU    → yellow blink 1.5 Hz    (boot complete, Pixhawk not connected)
  IDLE           → green solid             (boot complete, FCU connected, ready)

The BOOTING state is derived from booting_complete=False in watchdog_status.json.
The watchdog writes booting_complete=False during the entire _wait_for_mavros()
window and transitions to booting_complete=True immediately after, regardless
of whether the FCU connected within the timeout.

Priority table change (inserted between SYSTEM_START transient and WAITING_FCU):
  Old priority 16: WAITING_FCU — fires whenever fcu=False and watchdog alive
  New priority 16: BOOTING     — fires when booting_complete=False
  New priority 17: WAITING_FCU — fires when booting_complete=True and fcu=False

This ensures WAITING_FCU only appears after the system has fully started,
not during the normal 60 s MAVROS initialisation window.

PHYSICAL PIN MAP
----------------
  GND    →  Pin 34  (common ground)
  Green  →  BCM 25  (BOARD pin 22)
  Yellow →  BCM  8  (BOARD pin 24)
  Red    →  BCM  7  (BOARD pin 26)
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

os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")
os.environ.setdefault("HOME", "/tmp")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LED] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)

# ── Status file paths ─────────────────────────────────────────────────────────

WATCHDOG_STATUS_FILE   = "/tmp/watchdog_status.json"
MAIN_STATUS_FILE       = "/tmp/main_status.json"
MISSION_LOCK_FILE      = "/tmp/dronepi_mission.lock"
POSTFLIGHT_STATUS_FILE = "/tmp/postflight_status.json"

POLL_HZ         = 10
WATCHDOG_DEAD_S = 5.0
MAIN_DEAD_S     = 8.0
POSTFLIGHT_DONE_HOLD_S = 5.0

_BCM_PINS: dict[str, int] = {
    "green":  25,
    "yellow": 8,
    "red":    7,
}

# ── Pattern registry ──────────────────────────────────────────────────────────

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
    # ── System states ──────────────────────────────────────────────────────────
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
    # BOOTING: slow green blink — system is alive and initialising services.
    # Distinct from WAITING_FCU (yellow) so the user knows MAVROS is still
    # starting up rather than the Pixhawk being absent.
    "BOOTING":        _Pattern("blink",  channels=("green",), blink_hz=0.5),
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
    "POSTFLIGHT_RUNNING": _Pattern("blink", channels=("yellow",), blink_hz=1.0),
    "POSTFLIGHT_DONE":    _Pattern("solid", channels=("green", "yellow")),
    "POSTFLIGHT_FAILED":  _Pattern("blink", channels=("red",), blink_hz=3.0),
}


# ── System state snapshot ─────────────────────────────────────────────────────

@dataclass
class _SystemSnapshot:
    watchdog_ts:            float = 0.0
    watchdog_fcu:           bool  = False
    watchdog_armed:         bool  = False
    watchdog_processing:    bool  = False
    watchdog_stack_running: bool  = False
    led_state:              str   = ""
    led_until:              float = 0.0
    warning:                bool  = False
    error:                  bool  = False
    critical:               bool  = False
    system_failure:         bool  = False
    booting_complete:       bool  = False   # new field
    main_ts:                float = 0.0
    hailo_active:           bool  = False
    hailo_degraded:         bool  = False
    hailo_failed:           bool  = False
    lock_mode:              str   = ""
    pf_ts:                  float = 0.0
    pf_stage:               str   = ""
    pf_failed:              bool  = False
    pf_done:                bool  = False


def _read_json(path: str) -> dict:
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}


def _read_snapshot() -> _SystemSnapshot:
    s = _SystemSnapshot()

    wd = _read_json(WATCHDOG_STATUS_FILE)
    s.watchdog_ts            = float(wd.get("ts",               0.0))
    s.watchdog_fcu           = bool (wd.get("fcu",              False))
    s.watchdog_armed         = bool (wd.get("armed",            False))
    s.watchdog_processing    = bool (wd.get("processing",       False))
    s.watchdog_stack_running = bool (wd.get("stack_running",    False))
    s.led_state              = str  (wd.get("led_state",        ""))
    s.led_until              = float(wd.get("led_until",        0.0))
    s.warning                = bool (wd.get("warning",          False))
    s.error                  = bool (wd.get("error",            False))
    s.critical               = bool (wd.get("critical",         False))
    s.system_failure         = bool (wd.get("system_failure",   False))
    s.booting_complete       = bool (wd.get("booting_complete", False))

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


# ── State derivation — priority table ─────────────────────────────────────────

def _derive_state(snap: _SystemSnapshot, now: float) -> str:
    """
    Derive the correct LED state from the current system snapshot.

    Priority table (top = highest):
      1.  WATCHDOG_DEAD       watchdog heartbeat stale
      2.  MAIN_DEAD           main.py gone during autonomous
      3.  Transient override  led_state / led_until from watchdog
      4.  SYSTEM_FAILURE
      5.  CRITICAL
      6.  ERROR
      7.  WARNING
      8.  HAILO_FAILED
      9.  HAILO_DEGRADED
      10. HAILO_ACTIVE
      11. POSTFLIGHT_FAILED
      12. POSTFLIGHT_DONE
      13. POSTFLIGHT_RUNNING
      14. SCANNING
      15. PROCESSING (legacy)
      16. BOOTING              booting_complete=False (NEW)
      17. WAITING_FCU          booting_complete=True and fcu=False
      18. OFF / IDLE
    """
    watchdog_age = (now - snap.watchdog_ts) if snap.watchdog_ts > 0 else float("inf")
    main_age     = (now - snap.main_ts)     if snap.main_ts     > 0 else float("inf")

    # 1 — Watchdog dead
    if snap.watchdog_ts > 0 and watchdog_age > WATCHDOG_DEAD_S:
        return "WATCHDOG_DEAD"

    # 2 — Main dead during autonomous
    if snap.lock_mode == "autonomous" and snap.main_ts > 0 and main_age > MAIN_DEAD_S:
        return "MAIN_DEAD"

    # 3 — Transient override (SYSTEM_START, SCAN_READY, SCAN_START, SCAN_FINISHED)
    if snap.led_state and snap.led_state in _PATTERNS and now < snap.led_until:
        return snap.led_state

    # 4 — System failure
    if snap.system_failure:
        return "SYSTEM_FAILURE"

    # 5 — Critical
    if snap.critical:
        return "CRITICAL"

    # 6 — Error
    if snap.error:
        return "ERROR"

    # 7 — Warning
    if snap.warning:
        return "WARNING"

    # 8 — Hailo hard failure
    if snap.hailo_failed:
        return "HAILO_FAILED"

    # 9 — Hailo degraded
    if snap.hailo_degraded:
        return "HAILO_DEGRADED"

    # 10 — Hailo active
    if snap.lock_mode == "autonomous" and snap.hailo_active:
        return "HAILO_ACTIVE"

    # 11 — Post-flight failed
    if snap.pf_ts > 0 and snap.pf_failed:
        return "POSTFLIGHT_FAILED"

    # 12 — Post-flight done (held for POSTFLIGHT_DONE_HOLD_S)
    if snap.pf_ts > 0 and snap.pf_done:
        if (now - snap.pf_ts) < POSTFLIGHT_DONE_HOLD_S:
            return "POSTFLIGHT_DONE"

    # 13 — Post-flight running
    if snap.pf_ts > 0 and snap.pf_stage and not snap.pf_done and not snap.pf_failed:
        return "POSTFLIGHT_RUNNING"

    # 14 — Flight stack active
    if snap.watchdog_stack_running or snap.lock_mode in ("manual_scan", "autonomous"):
        return "SCANNING"

    # 15 — Legacy watchdog processing flag
    if snap.watchdog_processing:
        return "PROCESSING"

    # 16 — BOOTING: watchdog alive but still in MAVROS wait window.
    #      booting_complete=False means _wait_for_mavros() has not returned yet.
    #      Show slow green blink so user knows the system is alive and loading.
    if snap.watchdog_ts > 0 and not snap.booting_complete:
        return "BOOTING"

    # 17 — WAITING_FCU: boot complete but Pixhawk not connected.
    #      Only reached after booting_complete=True, so this no longer fires
    #      during the normal 60 s MAVROS startup window.
    if snap.watchdog_ts > 0 and not snap.watchdog_fcu:
        return "WAITING_FCU"

    # 18 — Watchdog not yet started
    if snap.watchdog_ts == 0:
        return "OFF"

    # 19 — Default: healthy idle
    return "IDLE"


# ── Pattern thread ────────────────────────────────────────────────────────────

class _PatternThread(threading.Thread):
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


# ── LED hardware driver ───────────────────────────────────────────────────────

class LEDDriver:
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
        if state_name == self._current:
            return
        if state_name not in _PATTERNS:
            logger.warning("Unknown state '%s' — treating as OFF.", state_name)
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

        self._current = state_name
        logger.info("State → %s", state_name)

    def cleanup(self) -> None:
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


# ── Main service loop ─────────────────────────────────────────────────────────

def main() -> int:
    logger.info("=" * 52)
    logger.info("  DronePi LED Observer Service")
    logger.info("  Watchdog timeout : %.1fs → WATCHDOG_DEAD", WATCHDOG_DEAD_S)
    logger.info("  Main timeout     : %.1fs → MAIN_DEAD (autonomous only)", MAIN_DEAD_S)
    logger.info("  Poll rate        : %d Hz", POLL_HZ)
    logger.info("  Post-flight done hold: %.1fs → POSTFLIGHT_DONE", POSTFLIGHT_DONE_HOLD_S)
    logger.info("=" * 52)
    logger.info("Boot state sequence:")
    logger.info("  OFF          → watchdog not yet started")
    logger.info("  SYSTEM_START → cycling animation (first 2.5s)")
    logger.info("  BOOTING      → slow green blink (MAVROS wait, up to 60s)")
    logger.info("  WAITING_FCU  → yellow blink (boot done, Pixhawk absent)")
    logger.info("  IDLE         → green solid (ready)")
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
