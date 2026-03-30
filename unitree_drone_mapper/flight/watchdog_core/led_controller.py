#!/usr/bin/env python3
"""
watchdog_core/led_controller.py — RGB status LED controller.

HARDWARE CONFIGURATION
----------------------
This module targets a three-channel LED indicator wired directly to the
Raspberry Pi GPIO header.  Pin numbers below are PHYSICAL (board) numbers,
not BCM GPIO numbers.  Both numbering schemes are documented for clarity.

  Physical pin 34  →  GND   (common ground for all LED channels)
  Physical pin 36  →  Green  LED  (BCM GPIO 16)
  Physical pin 38  →  Yellow LED  (BCM GPIO 20)
  Physical pin 40  →  Red    LED  (BCM GPIO 21)

STATUS MAPPING
--------------
  GREEN   solid       System idle / healthy, no active scan
  GREEN   blink       Scan session active — LiDAR recording in progress
  YELLOW  solid       Post-flight processing running
  YELLOW  blink       Waiting for MAVROS FCU connection
  RED     solid       Fatal error — watchdog entered error state
  RED     blink       Non-fatal warning (e.g. low disk space)
  ALL OFF             System initialising or powered down

ENABLE / DISABLE
----------------
Set LED_ENABLED = False to disable all GPIO output without removing call sites.
All public methods become no-ops when disabled, so the rest of the watchdog
needs no conditional logic around LED calls.

When the HDMI extension cables and GPIO wiring are complete:
  1. Set LED_ENABLED = True below
  2. Confirm pin numbers against your physical wiring
  3. Run tests/test_led_controller.py to validate each channel

DESIGN NOTES
------------
- Uses RPi.GPIO in BOARD mode so physical pin numbers match the silkscreen
  on the Pi hat — no BCM lookup required when wiring.
- BlinkThread is a daemon thread: it dies automatically when the main process
  exits, so no explicit cleanup is required for blink states.
- .is_available() checks both LED_ENABLED and whether RPi.GPIO imported
  successfully, enabling graceful degradation on dev laptops.
- All public methods are safe to call before GPIO.setup() completes — the
  guard at the top of each method prevents partial-init crashes.
"""

import threading
import time
import logging
from enum import Enum, auto

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration — edit these when wiring is complete
# ---------------------------------------------------------------------------

LED_ENABLED: bool = False
"""
Set to True once GPIO wiring is physically connected.
When False all LED methods are silent no-ops.
"""

# Physical (BOARD) pin numbers — match Pi 40-pin header silkscreen directly
_PIN_GREEN:  int = 36   # BCM GPIO 16
_PIN_YELLOW: int = 38   # BCM GPIO 20
_PIN_RED:    int = 40   # BCM GPIO 21
_PIN_GND:    int = 34   # Reference only — not configured via GPIO library

_BLINK_HZ:   float = 1.5    # Blink rate for all channels (cycles per second)
_BLINK_ON:   float = 1.0 / _BLINK_HZ * 0.5   # 50% duty cycle


# ---------------------------------------------------------------------------
# Public state enum
# ---------------------------------------------------------------------------

class LEDState(Enum):
    """Named LED states consumed by the watchdog state machine."""
    OFF           = auto()   # All LEDs off
    IDLE          = auto()   # Green solid
    SCANNING      = auto()   # Green blink
    PROCESSING    = auto()   # Yellow solid
    WAITING_FCU   = auto()   # Yellow blink
    ERROR         = auto()   # Red solid
    WARNING       = auto()   # Red blink


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

class _BlinkThread(threading.Thread):
    """Daemon thread that toggles a GPIO pin at _BLINK_HZ until stopped."""

    def __init__(self, pin: int, gpio_module) -> None:
        super().__init__(daemon=True)
        self._pin  = pin
        self._gpio = gpio_module
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            self._gpio.output(self._pin, self._gpio.HIGH)
            time.sleep(_BLINK_ON)
            if self._stop_event.is_set():
                break
            self._gpio.output(self._pin, self._gpio.LOW)
            time.sleep(_BLINK_ON)

    def stop(self) -> None:
        self._stop_event.set()


# ---------------------------------------------------------------------------
# Main controller class
# ---------------------------------------------------------------------------

class LEDController:
    """
    Three-channel GPIO LED controller for DronePi status indication.

    Instantiate once at watchdog startup.  Call set_state() to transition
    between named states.  Call cleanup() on shutdown.

    Example
    -------
    >>> leds = LEDController()
    >>> if leds.is_available():
    ...     leds.set_state(LEDState.SCANNING)
    ...     # ... later ...
    ...     leds.cleanup()
    """

    def __init__(self) -> None:
        self._gpio        = None
        self._initialised = False
        self._current_state: LEDState = LEDState.OFF
        self._blink_thread: _BlinkThread | None = None

        if not LED_ENABLED:
            logger.info(
                "[LEDController] LED_ENABLED=False — GPIO output disabled. "
                "Set LED_ENABLED=True in led_controller.py once wiring is complete."
            )
            return

        try:
            import RPi.GPIO as GPIO   # type: ignore[import]
            self._gpio = GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(_PIN_GREEN,  GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(_PIN_YELLOW, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(_PIN_RED,    GPIO.OUT, initial=GPIO.LOW)
            self._initialised = True
            logger.info(
                "[LEDController] GPIO initialised. Pins — G:%d Y:%d R:%d (BOARD mode)",
                _PIN_GREEN, _PIN_YELLOW, _PIN_RED,
            )
        except ImportError:
            logger.warning(
                "[LEDController] RPi.GPIO not available (running off-device?). "
                "LED output will be suppressed."
            )
        except Exception as exc:
            logger.error("[LEDController] GPIO setup failed: %s", exc)

    # ── Public API ────────────────────────────────────────────────────────────

    def is_available(self) -> bool:
        """Return True if GPIO is initialised and LED output is active."""
        return self._initialised

    def set_state(self, state: LEDState) -> None:
        """
        Transition to a named LED state.

        Stops any active blink thread, turns all channels off, then applies
        the new state.  Safe to call from any thread.

        Parameters
        ----------
        state : LEDState
            Target state.  See module docstring for the visual meaning of
            each state.
        """
        if not self._initialised:
            return

        if state == self._current_state:
            return

        self._stop_blink()
        self._all_off()

        if   state == LEDState.OFF:          pass                                        # already off
        elif state == LEDState.IDLE:         self._solid(_PIN_GREEN)
        elif state == LEDState.SCANNING:     self._blink(_PIN_GREEN)
        elif state == LEDState.PROCESSING:   self._solid(_PIN_YELLOW)
        elif state == LEDState.WAITING_FCU:  self._blink(_PIN_YELLOW)
        elif state == LEDState.ERROR:        self._solid(_PIN_RED)
        elif state == LEDState.WARNING:      self._blink(_PIN_RED)
        else:
            logger.warning("[LEDController] Unknown state: %s", state)
            return

        self._current_state = state
        logger.debug("[LEDController] State → %s", state.name)

    def cleanup(self) -> None:
        """
        Turn off all LEDs, stop blink thread, and release GPIO resources.

        Call this from the watchdog shutdown handler.
        """
        if not self._initialised:
            return
        self._stop_blink()
        self._all_off()
        try:
            self._gpio.cleanup([_PIN_GREEN, _PIN_YELLOW, _PIN_RED])
        except Exception as exc:
            logger.warning("[LEDController] GPIO cleanup warning: %s", exc)
        self._initialised = False
        logger.info("[LEDController] GPIO released.")

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _solid(self, pin: int) -> None:
        self._gpio.output(pin, self._gpio.HIGH)

    def _blink(self, pin: int) -> None:
        self._blink_thread = _BlinkThread(pin, self._gpio)
        self._blink_thread.start()

    def _all_off(self) -> None:
        for pin in (_PIN_GREEN, _PIN_YELLOW, _PIN_RED):
            try:
                self._gpio.output(pin, self._gpio.LOW)
            except Exception:
                pass

    def _stop_blink(self) -> None:
        if self._blink_thread is not None and self._blink_thread.is_alive():
            self._blink_thread.stop()
            self._blink_thread.join(timeout=0.5)
        self._blink_thread = None
