#!/usr/bin/env python3
"""
tests/test_led_controller.py — Standalone GPIO LED channel verification test.

PURPOSE
-------
Validates physical wiring of the three-channel LED indicator before enabling
it in production.  Run this directly on the Pi after GPIO wiring is complete
and before setting LED_ENABLED = True in led_controller.py.

This is a STANDALONE TEST SCRIPT — it reimplements a minimal version of the
GPIO control logic so it can be run without importing any project modules.
This isolation means a bug in led_controller.py does not mask a wiring fault.

PHYSICAL PIN MAP (verify against your wiring before running)
------------------------------------------------------------
  Pin 34  GND    (common ground)
  Pin 36  Green  LED
  Pin 38  Yellow LED
  Pin 40  Red    LED

USAGE
-----
  # On the Pi
  python3 tests/test_led_controller.py

  # Run only a specific channel (useful for tracing a single wire)
  python3 tests/test_led_controller.py --channel green
  python3 tests/test_led_controller.py --channel yellow
  python3 tests/test_led_controller.py --channel red

  # Run all states from the production LEDState enum in sequence
  python3 tests/test_led_controller.py --states

REQUIREMENTS
------------
  RPi.GPIO  (pre-installed on Raspberry Pi OS)
  No conda env or ROS 2 sourcing required.
"""

import argparse
import sys
import time

# ---------------------------------------------------------------------------
# Pin map — must match led_controller.py
# ---------------------------------------------------------------------------
_PINS: dict[str, int] = {
    "green":  36,
    "yellow": 38,
    "red":    40,
}
_BLINK_ON  = 0.33   # seconds HIGH per blink cycle
_BLINK_OFF = 0.33   # seconds LOW per blink cycle


def _require_gpio():
    """Import RPi.GPIO or exit with a clear message."""
    try:
        import RPi.GPIO as GPIO   # type: ignore[import]
        return GPIO
    except ImportError:
        print("[ERROR] RPi.GPIO not found.")
        print("        This test must run on the Raspberry Pi.")
        print("        It cannot run on a dev laptop.")
        sys.exit(1)


def _setup(GPIO):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    for pin in _PINS.values():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)


def _all_off(GPIO):
    for pin in _PINS.values():
        GPIO.output(pin, GPIO.LOW)


def _blink(GPIO, pin: int, cycles: int = 3) -> None:
    for _ in range(cycles):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(_BLINK_ON)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(_BLINK_OFF)


def _solid(GPIO, pin: int, duration: float = 1.5) -> None:
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(pin, GPIO.LOW)


# ---------------------------------------------------------------------------
# Test routines
# ---------------------------------------------------------------------------

def test_single_channel(GPIO, channel: str) -> None:
    """Blink and hold a single channel for visual wire tracing."""
    pin = _PINS.get(channel)
    if pin is None:
        print(f"[ERROR] Unknown channel '{channel}'. Choose: {list(_PINS)}")
        sys.exit(1)

    print(f"\n── Single channel test: {channel.upper()} (pin {pin}) ──")
    print(f"   Blink × 3, then solid 2 s, then off.")

    _setup(GPIO)
    _blink(GPIO, pin, cycles=3)
    time.sleep(0.2)
    _solid(GPIO, pin, duration=2.0)
    _all_off(GPIO)
    GPIO.cleanup(list(_PINS.values()))
    print("   Done.")


def test_all_channels(GPIO) -> None:
    """Step through each channel individually to verify wiring."""
    _setup(GPIO)

    tests = [
        ("green",  "Idle / scanning"),
        ("yellow", "Processing / waiting FCU"),
        ("red",    "Error / warning"),
    ]

    print("\n── Individual channel test ──")
    for channel, meaning in tests:
        pin = _PINS[channel]
        print(f"\n  [{channel.upper():6s}]  Pin {pin}  —  {meaning}")
        print(f"           Solid 1 s...")
        _solid(GPIO, pin, duration=1.0)
        time.sleep(0.2)
        print(f"           Blink × 3...")
        _blink(GPIO, pin, cycles=3)
        time.sleep(0.3)

    _all_off(GPIO)


def test_named_states(GPIO) -> None:
    """
    Demonstrate every LEDState visual pattern without importing the module.
    Confirms the full state table matches physical output.
    """
    _setup(GPIO)

    states = [
        ("OFF",          None,            False,  0.8,  "All off"),
        ("IDLE",         "green",         False,  1.5,  "Green solid"),
        ("SCANNING",     "green",         True,   2.5,  "Green blink"),
        ("PROCESSING",   "yellow",        False,  1.5,  "Yellow solid"),
        ("WAITING_FCU",  "yellow",        True,   2.5,  "Yellow blink"),
        ("ERROR",        "red",           False,  1.5,  "Red solid"),
        ("WARNING",      "red",           True,   2.5,  "Red blink"),
    ]

    print("\n── Named state sequence test ──")
    print("   Reproduces every state defined in LEDState enum.\n")

    for state_name, channel, do_blink, duration, desc in states:
        print(f"  {state_name:<14}  {desc}")
        _all_off(GPIO)

        if channel is None:
            time.sleep(duration)
            continue

        pin = _PINS[channel]
        if do_blink:
            # Approximate blink for duration
            cycles = max(1, int(duration / (_BLINK_ON + _BLINK_OFF)))
            _blink(GPIO, pin, cycles=cycles)
        else:
            _solid(GPIO, pin, duration=duration)

        time.sleep(0.2)

    _all_off(GPIO)


def test_sequence_all(GPIO) -> None:
    """Run all channel and state tests in sequence for full validation."""
    test_all_channels(GPIO)
    time.sleep(0.5)
    test_named_states(GPIO)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone GPIO LED wiring verification for DronePi status LEDs."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--channel",
        choices=["green", "yellow", "red"],
        help="Test a single LED channel only.",
    )
    group.add_argument(
        "--states",
        action="store_true",
        help="Step through every LEDState pattern instead of the channel test.",
    )
    args = parser.parse_args()

    GPIO = _require_gpio()

    print("=" * 52)
    print("  DronePi LED Controller — Wiring Verification")
    print(f"  Pins: GND=34  G=36  Y=38  R=40  (BOARD mode)")
    print("=" * 52)

    try:
        if args.channel:
            test_single_channel(GPIO, args.channel)
        elif args.states:
            test_named_states(GPIO)
            GPIO.cleanup(list(_PINS.values()))
            print("\n  All states complete.")
        else:
            test_sequence_all(GPIO)
            GPIO.cleanup(list(_PINS.values()))
            print("\n  All tests complete. If all LEDs lit as described,")
            print("  set LED_ENABLED = True in watchdog_core/led_controller.py")
    except KeyboardInterrupt:
        print("\n  Interrupted.")
        _all_off(GPIO)
        GPIO.cleanup(list(_PINS.values()))
        sys.exit(0)


if __name__ == "__main__":
    main()
