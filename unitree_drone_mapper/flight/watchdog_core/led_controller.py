#!/usr/bin/env python3
"""
watchdog_core/led_controller.py — Canonical LED state vocabulary for DronePi.

This module defines the authoritative list of LED state names and their
physical pin assignments. It is imported by:
  - drone_watchdog.py (LED_STATES imported for reference / type checking)
  - Any test script that needs the BCM pin map

The actual pattern definitions and hardware driver live in flight/led_service.py.
"""

BCM_GREEN:  int = 25   # BOARD pin 22
BCM_YELLOW: int = 8    # BOARD pin 24
BCM_RED:    int = 7    # BOARD pin 26

LED_STATES: tuple[str, ...] = (
    # ── Core system ───────────────────────────────────────────────────────────
    "OFF",
    "SYSTEM_START",
    "IDLE",
    "WAITING_FCU",
    "SCAN_READY",
    "SCAN_START",
    "SCANNING",
    "SCAN_FINISHED",
    "PROCESSING",
    # ── Fault ladder ──────────────────────────────────────────────────────────
    "WARNING",
    "ERROR",
    "CRITICAL",
    "SYSTEM_FAILURE",
    # ── Hailo / process-health ────────────────────────────────────────────────
    "HAILO_ACTIVE",
    "HAILO_DEGRADED",
    "HAILO_FAILED",
    "WATCHDOG_DEAD",
    "MAIN_DEAD",
    # ── Post-flight pipeline ──────────────────────────────────────────────────
    "POSTFLIGHT_RUNNING",   # pipeline executing — yellow 1 Hz blink
    "POSTFLIGHT_DONE",      # pipeline complete  — green + yellow solid (held 5 s)
    "POSTFLIGHT_FAILED",    # pipeline crashed   — red 3 Hz blink
)

LED_STATE_DESCRIPTIONS: dict[str, str] = {
    "OFF":               "All LEDs off",
    "SYSTEM_START":      "Boot sequence: green → yellow → red cycling",
    "IDLE":              "Green solid — system healthy, awaiting input",
    "WAITING_FCU":       "Yellow blink 1.5 Hz — waiting for Pixhawk connection",
    "SCAN_READY":        "Green + Yellow solid — system ready, stack not running",
    "SCAN_START":        "Green + Yellow blink 2.5 Hz — stack launch initiated",
    "SCANNING":          "Green blink 1.5 Hz — LiDAR scan in progress",
    "SCAN_FINISHED":     "Green + Yellow blink 2.0 Hz — scan session ended",
    "PROCESSING":        "Yellow solid — legacy post-processing flag",
    "WARNING":           "Yellow blink 2.5 Hz — non-fatal warning",
    "ERROR":             "Red solid — recoverable error",
    "CRITICAL":          "All three LEDs blink 2.5 Hz — in-flight critical fault",
    "SYSTEM_FAILURE":    "All three LEDs blink 5 Hz — system failure alarm",
    "HAILO_ACTIVE":      "Green + Yellow solid — Hailo inference augmenting EKF2",
    "HAILO_DEGRADED":    "Yellow blink 3 Hz — Hailo flow bridge rejections exceeded threshold",
    "HAILO_FAILED":      "Red blink 3 Hz — Hailo node crashed during flight",
    "WATCHDOG_DEAD":     "Red blink 0.8 Hz — drone_watchdog.py process gone",
    "MAIN_DEAD":         "Red blink 3 Hz — main.py gone during autonomous mission",
    "POSTFLIGHT_RUNNING":"Yellow blink 1 Hz — post-flight pipeline executing",
    "POSTFLIGHT_DONE":   "Green + Yellow solid (5 s) — post-flight pipeline complete",
    "POSTFLIGHT_FAILED": "Red blink 3 Hz — post-flight pipeline crashed",
}
