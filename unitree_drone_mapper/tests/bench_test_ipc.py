#!/usr/bin/env python3
"""
bench_test_ipc.py — DronePi LED + Buzzer bench test utility.

Simulates every system state by writing IPC files directly, without
requiring MAVROS, ROS 2, or a connected drone. Run this on the Pi
with led.service running to confirm every LED pattern and buzzer tone
fires correctly before a flight test.

Buzzer note
-----------
Buzzer tones require MAVROS to be running (it bridges PlayTuneV2 to
the Pixhawk). If MAVROS is not running, buzzer sections are skipped
gracefully — LED tests still work independently.

Usage
-----
  # LED tests only (no ROS needed)
  python3 bench_test_ipc.py

  # LED + buzzer (requires: source /opt/ros/jazzy/setup.bash first)
  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
  python3 bench_test_ipc.py --buzzer

  # Run a specific state directly (useful for scripting)
  python3 bench_test_ipc.py --state SCANNING
  python3 bench_test_ipc.py --state POSTFLIGHT_RUNNING
  python3 bench_test_ipc.py --state WATCHDOG_DEAD

  # Run the full automated sequence (all states, 3s each)
  python3 bench_test_ipc.py --auto

  # Clean up all IPC files when done
  python3 bench_test_ipc.py --cleanup

Requirements
------------
  - led.service must be running:  sudo systemctl status led
  - Run as dronepi user (has write access to /tmp)
  - For buzzer: ROS 2 Jazzy + MAVROS sourced + Pixhawk connected
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

# ── IPC file paths ────────────────────────────────────────────────────────────

WATCHDOG_STATUS   = "/tmp/watchdog_status.json"
MAIN_STATUS       = "/tmp/main_status.json"
MISSION_LOCK      = "/tmp/dronepi_mission.lock"
POSTFLIGHT_STATUS = "/tmp/postflight_status.json"

# ── Colours for terminal output ───────────────────────────────────────────────

_G  = "\033[32m"    # green
_Y  = "\033[33m"    # yellow
_R  = "\033[31m"    # red
_C  = "\033[36m"    # cyan
_W  = "\033[37m"    # white
_DIM = "\033[2m"    # dim
_B  = "\033[1m"     # bold
_RST = "\033[0m"    # reset


def _ts() -> float:
    return time.time()


def _log(msg: str, colour: str = _W) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"{_DIM}[{ts}]{_RST} {colour}{msg}{_RST}", flush=True)


# ── IPC writers ───────────────────────────────────────────────────────────────

def write_watchdog(
    fcu:            bool  = True,
    armed:          bool  = False,
    processing:     bool  = False,
    stack_running:  bool  = False,
    led_state:      str   = "",
    led_until:      float = 0.0,
    warning:        bool  = False,
    error:          bool  = False,
    critical:       bool  = False,
    system_failure: bool  = False,
) -> None:
    payload = json.dumps({
        "ts":             _ts(),
        "fcu":            fcu,
        "armed":          armed,
        "processing":     processing,
        "stack_running":  stack_running,
        "led_state":      led_state,
        "led_until":      led_until,
        "warning":        warning,
        "error":          error,
        "critical":       critical,
        "system_failure": system_failure,
    })
    Path(WATCHDOG_STATUS).write_text(payload)


def write_main(
    hailo_active:   bool = False,
    hailo_degraded: bool = False,
    hailo_failed:   bool = False,
) -> None:
    payload = json.dumps({
        "ts":             _ts(),
        "hailo_active":   hailo_active,
        "hailo_degraded": hailo_degraded,
        "hailo_failed":   hailo_failed,
    })
    Path(MAIN_STATUS).write_text(payload)


def write_lock(mode: str) -> None:
    payload = json.dumps({"mode": mode, "started_at": datetime.now().isoformat()})
    Path(MISSION_LOCK).write_text(payload)


def write_postflight(stage: str, failed: bool = False, done: bool = False) -> None:
    payload = json.dumps({
        "ts":     _ts(),
        "stage":  stage,
        "failed": failed,
        "done":   done,
    })
    Path(POSTFLIGHT_STATUS).write_text(payload)


def clear_all() -> None:
    for f in [WATCHDOG_STATUS, MAIN_STATUS, MISSION_LOCK, POSTFLIGHT_STATUS]:
        Path(f).unlink(missing_ok=True)
    _log("All IPC files cleared", _G)


# ── State definitions ─────────────────────────────────────────────────────────
# Each entry: (description, led_colour_hint, setup_fn, hold_s)

def _clear_side_files() -> None:
    """Clear every IPC file that is NOT being explicitly set by the current
    state. Called at the top of every setup function so stale files from a
    previous state (or from a real pipeline run) never bleed into the test."""
    Path(POSTFLIGHT_STATUS).unlink(missing_ok=True)
    Path(MISSION_LOCK).unlink(missing_ok=True)
    Path(MAIN_STATUS).unlink(missing_ok=True)


def _state_off():
    clear_all()

def _state_booting():
    _clear_side_files()
    write_watchdog(
        fcu=False, led_state="SYSTEM_START",
        led_until=_ts() + 5.0
    )

def _state_waiting_fcu():
    _clear_side_files()
    write_watchdog(fcu=False)

def _state_idle():
    _clear_side_files()
    write_watchdog(fcu=True)

def _state_scan_ready():
    _clear_side_files()
    write_watchdog(fcu=True, led_state="SCAN_READY", led_until=_ts() + 5.0)

def _state_scan_start():
    _clear_side_files()
    write_watchdog(fcu=True, led_state="SCAN_START", led_until=_ts() + 5.0)

def _state_scanning_manual():
    _clear_side_files()
    write_watchdog(fcu=True, armed=True, stack_running=True)
    write_lock("manual_scan")

def _state_scanning_auto():
    _clear_side_files()
    write_watchdog(fcu=True, armed=True, stack_running=False)
    write_lock("autonomous")
    write_main()

def _state_scan_finished():
    _clear_side_files()
    write_watchdog(fcu=True, led_state="SCAN_FINISHED", led_until=_ts() + 5.0)

def _state_processing():
    _clear_side_files()
    write_watchdog(fcu=True, processing=True)

def _state_pf_running_extract():
    _clear_side_files()
    write_watchdog(fcu=True, processing=True)
    write_postflight("bag_extract")

def _state_pf_running_classify():
    _clear_side_files()
    write_watchdog(fcu=True, processing=True)
    write_postflight("ground_classify")

def _state_pf_running_dtm():
    _clear_side_files()
    write_watchdog(fcu=True, processing=True)
    write_postflight("dtm")

def _state_pf_running_dsm():
    _clear_side_files()
    write_watchdog(fcu=True, processing=True)
    write_postflight("dsm")

def _state_pf_done():
    _clear_side_files()
    write_watchdog(fcu=True)
    write_postflight("publish", done=True)

def _state_pf_failed():
    _clear_side_files()
    write_watchdog(fcu=True)
    write_postflight("dsm", failed=True)

def _state_hailo_active():
    _clear_side_files()
    write_watchdog(fcu=True, armed=True, stack_running=False)
    write_lock("autonomous")
    write_main(hailo_active=True)

def _state_hailo_degraded():
    _clear_side_files()
    write_watchdog(fcu=True, armed=True, stack_running=False)
    write_lock("autonomous")
    write_main(hailo_active=True, hailo_degraded=True)

def _state_hailo_failed():
    _clear_side_files()
    write_watchdog(fcu=True)
    write_main(hailo_failed=True)

def _state_warning():
    _clear_side_files()
    write_watchdog(fcu=True, warning=True)

def _state_error():
    _clear_side_files()
    write_watchdog(fcu=True, error=True)

def _state_critical():
    _clear_side_files()
    write_watchdog(fcu=True, armed=True, critical=True)

def _state_system_failure():
    _clear_side_files()
    write_watchdog(fcu=True, system_failure=True)

def _state_watchdog_dead():
    _clear_side_files()
    payload = json.dumps({
        "ts": _ts() - 30.0,   # 30s stale — well past WATCHDOG_DEAD_S=5
        "fcu": True, "armed": False, "processing": False,
        "stack_running": False, "led_state": "", "led_until": 0,
        "warning": False, "error": False, "critical": False,
        "system_failure": False,
    })
    Path(WATCHDOG_STATUS).write_text(payload)

def _state_main_dead():
    _clear_side_files()
    write_watchdog(fcu=True)
    write_lock("autonomous")
    payload = json.dumps({
        "ts": _ts() - 30.0,   # 30s stale — well past MAIN_DEAD_S=8
        "hailo_active": False, "hailo_degraded": False, "hailo_failed": False,
    })
    Path(MAIN_STATUS).write_text(payload)


# ── State catalogue ───────────────────────────────────────────────────────────

STATES: dict[str, tuple] = {
    # name: (description, colour_hint, setup_fn)
    "OFF":                  ("All LEDs off — no IPC files",            _W,   _state_off),
    "SYSTEM_START":         ("Boot animation: G→Y→R sequence",         _G,   _state_booting),
    "WAITING_FCU":          ("Yellow blink 1.5Hz — FCU not connected",  _Y,   _state_waiting_fcu),
    "IDLE":                 ("Green solid — healthy, nothing active",   _G,   _state_idle),
    "SCAN_READY":           ("Green+Yellow solid — ready to scan",      _G,   _state_scan_ready),
    "SCAN_START":           ("Green+Yellow blink 2.5Hz — starting",     _G,   _state_scan_start),
    "SCANNING_MANUAL":      ("Green blink 1.5Hz — manual scan active",  _G,   _state_scanning_manual),
    "SCANNING_AUTO":        ("Green blink 1.5Hz — autonomous active",   _G,   _state_scanning_auto),
    "SCAN_FINISHED":        ("Green+Yellow blink 2Hz — scan ended",     _G,   _state_scan_finished),
    "PROCESSING":           ("Yellow solid — post-processing (legacy)", _Y,   _state_processing),
    "POSTFLIGHT_RUNNING_EXTRACT": ("Yellow blink 1Hz — extracting bag", _Y,   _state_pf_running_extract),
    "POSTFLIGHT_RUNNING_CLASSIFY":("Yellow blink 1Hz — classifying",    _Y,   _state_pf_running_classify),
    "POSTFLIGHT_RUNNING_DTM":     ("Yellow blink 1Hz — building DTM",   _Y,   _state_pf_running_dtm),
    "POSTFLIGHT_RUNNING_DSM":     ("Yellow blink 1Hz — building DSM",   _Y,   _state_pf_running_dsm),
    "POSTFLIGHT_DONE":      ("Green+Yellow solid 5s — pipeline done",   _G,   _state_pf_done),
    "POSTFLIGHT_FAILED":    ("Red blink 3Hz — pipeline crashed",        _R,   _state_pf_failed),
    "HAILO_ACTIVE":         ("Green+Yellow solid — Hailo augmenting",   _G,   _state_hailo_active),
    "HAILO_DEGRADED":       ("Yellow blink 3Hz — Hailo degraded",       _Y,   _state_hailo_degraded),
    "HAILO_FAILED":         ("Red blink 3Hz — Hailo crashed",           _R,   _state_hailo_failed),
    "WARNING":              ("Yellow blink 2.5Hz — non-fatal warning",  _Y,   _state_warning),
    "ERROR":                ("Red solid — recoverable error",           _R,   _state_error),
    "CRITICAL":             ("All LEDs blink 2.5Hz — critical fault",   _R,   _state_critical),
    "SYSTEM_FAILURE":       ("All LEDs blink 5Hz — system failure",     _R,   _state_system_failure),
    "WATCHDOG_DEAD":        ("Red blink 0.8Hz — watchdog gone",         _R,   _state_watchdog_dead),
    "MAIN_DEAD":            ("Red blink 3Hz — main.py gone",            _R,   _state_main_dead),
}

# ── Buzzer tones ──────────────────────────────────────────────────────────────

BUZZER_TONES: dict[str, tuple] = {
    # name: (description, qbasic_string)
    "SYSTEM_START":       ("Boot fanfare — ascending arpeggio",          "T180L8O4CEGCEG>C"),
    "SCAN_READY":         ("Short neutral ack — system ready",           "T250L32O5CE"),
    "SCAN_START":         ("Upward ramp — action initiated",             "T200L12O5CEG>C"),
    "SCAN_ACTIVE":        ("Periodic tick — scan heartbeat",             "T200L64O4GE"),
    "SCAN_FINISHED":      ("Descending stop — scan ended",               "T180L8O5C<GEG<C"),
    "PF_EXTRACT":         ("Chirp — bag extraction done",                "T260L32O5C"),
    "PF_CLASSIFY":        ("Ascending 2-note — classification done",     "T220L24O4EG"),
    "PF_MESH_STAGE":      ("Short confirm — DTM or DSM built",           "T200L16O5CE"),
    "POSTPROCESS_ACTIVE": ("High tick — postflight heartbeat",           "T220L64O5C"),
    "POSTPROCESS_DONE":   ("Happy chord — pipeline complete",            "T180L12O5CEGCE"),
    "WARNING":            ("One-shot caution",                           "T200L32O5CC"),
    "ERROR":              ("Low unpleasant — fault",                     "T140L4O4C<C"),
    "CRITICAL":           ("Descending unsettling",                      "T180L8O6C<A<F"),
    "SYSTEM_FAILURE":     ("Repeating alarm",                            "T220L8O6CECECE"),
}


# ── Buzzer publisher (requires ROS 2) ─────────────────────────────────────────

class BuzzerPublisher:
    """Minimal ROS 2 publisher for PlayTuneV2. Used only when --buzzer is set."""

    def __init__(self) -> None:
        self._available = False
        try:
            import rclpy
            from rclpy.node import Node
            from mavros_msgs.msg import PlayTuneV2
            rclpy.init()
            self._node      = Node("bench_test_buzzer")
            self._pub       = self._node.create_publisher(PlayTuneV2, "/mavros/play_tune", 10)
            self._rclpy     = rclpy
            self._TuneMsg   = PlayTuneV2
            self._available = True
            # Brief spin to allow publisher discovery
            time.sleep(0.5)
            _log("Buzzer publisher ready on /mavros/play_tune", _G)
        except Exception as exc:
            _log(f"Buzzer unavailable (ROS not sourced or MAVROS not running): {exc}", _Y)

    def play(self, tune: str, label: str = "") -> None:
        if not self._available:
            _log(f"  [BUZZER SKIPPED] {label or tune}", _DIM)
            return
        try:
            msg        = self._TuneMsg()
            msg.format = 1
            msg.tune   = tune
            self._pub.publish(msg)
            _log(f"  [BUZZER] {label or tune}", _C)
        except Exception as exc:
            _log(f"  [BUZZER ERROR] {exc}", _R)

    def shutdown(self) -> None:
        if self._available:
            try:
                self._node.destroy_node()
                self._rclpy.shutdown()
            except Exception:
                pass


# ── Interactive menu ──────────────────────────────────────────────────────────

def _print_header() -> None:
    print(f"\n{_B}{_C}{'─'*60}")
    print("  DronePi Bench Test — LED + Buzzer Simulator")
    print(f"{'─'*60}{_RST}")
    print(f"  {_DIM}IPC files: /tmp/{{watchdog,main,postflight}}_status.json")
    print(f"  led.service reads these every 100ms{_RST}\n")


def _print_states(buzzer_available: bool) -> None:
    print(f"{_B}  LED STATES{_RST}")
    print(f"  {'─'*56}")
    for i, (name, (desc, colour, _)) in enumerate(STATES.items(), 1):
        print(f"  {_DIM}{i:2d}.{_RST} {colour}{name:<32}{_RST}  {_DIM}{desc}{_RST}")

    if buzzer_available:
        print(f"\n{_B}  BUZZER TONES{_RST}")
        print(f"  {'─'*56}")
        for i, (name, (desc, tune)) in enumerate(BUZZER_TONES.items(), 1):
            idx = len(STATES) + i
            print(f"  {_DIM}{idx:2d}.{_RST} {_C}{name:<28}{_RST}  {_DIM}{desc}{_RST}")

    print(f"\n  {_DIM}A{_RST}  = Run full automated sequence")
    print(f"  {_DIM}C{_RST}  = Clear all IPC files")
    print(f"  {_DIM}S{_RST}  = Show current IPC file contents")
    print(f"  {_DIM}Q{_RST}  = Quit and clear IPC files\n")


def _show_current() -> None:
    print(f"\n{_B}  Current IPC state:{_RST}")
    for path in [WATCHDOG_STATUS, MAIN_STATUS, MISSION_LOCK, POSTFLIGHT_STATUS]:
        p = Path(path)
        if p.exists():
            try:
                data = json.loads(p.read_text())
                print(f"\n  {_C}{p.name}{_RST}")
                for k, v in data.items():
                    if k == "ts":
                        age = time.time() - float(v)
                        print(f"    {_DIM}{k:<18}{_RST} {age:.1f}s ago")
                    else:
                        print(f"    {_DIM}{k:<18}{_RST} {v}")
            except Exception:
                print(f"  {_Y}{p.name}: (unreadable){_RST}")
        else:
            print(f"  {_DIM}{p.name}: absent{_RST}")
    print()


def _activate_state(name: str, buzzer: BuzzerPublisher | None, hold_s: float = 3.0) -> None:
    if name not in STATES:
        _log(f"Unknown state: {name}", _R)
        return

    desc, colour, setup_fn = STATES[name]
    _log(f"→ {name}  ({desc})", colour)
    setup_fn()

    # Play matching buzzer tone for states that have one
    if buzzer is not None:
        tone_map = {
            "SYSTEM_START":   "SYSTEM_START",
            "SCAN_READY":     "SCAN_READY",
            "SCAN_START":     "SCAN_START",
            "SCAN_FINISHED":  "SCAN_FINISHED",
            "POSTFLIGHT_RUNNING_EXTRACT":  "PF_EXTRACT",
            "POSTFLIGHT_RUNNING_CLASSIFY": "PF_CLASSIFY",
            "POSTFLIGHT_RUNNING_DTM":      "PF_MESH_STAGE",
            "POSTFLIGHT_RUNNING_DSM":      "PF_MESH_STAGE",
            "POSTFLIGHT_DONE":    "POSTPROCESS_DONE",
            "POSTFLIGHT_FAILED":  "ERROR",
            "HAILO_DEGRADED":     "WARNING",
            "HAILO_FAILED":       "ERROR",
            "WARNING":            "WARNING",
            "ERROR":              "ERROR",
            "CRITICAL":           "CRITICAL",
            "SYSTEM_FAILURE":     "SYSTEM_FAILURE",
        }
        if name in tone_map:
            tname = tone_map[name]
            _, tune = BUZZER_TONES[tname]
            buzzer.play(tune, tname)

    if hold_s > 0:
        _log(f"  Holding {hold_s:.0f}s — watch LED ...", _DIM)
        # Refresh IPC files every 80ms so the live watchdog (10Hz writer)
        # cannot overwrite the test state during the hold window.
        deadline = time.time() + hold_s
        while time.time() < deadline:
            setup_fn()
            time.sleep(0.08)


def _play_tone(tone_name: str, buzzer: BuzzerPublisher) -> None:
    if tone_name not in BUZZER_TONES:
        _log(f"Unknown tone: {tone_name}", _R)
        return
    desc, tune = BUZZER_TONES[tone_name]
    _log(f"→ Playing {tone_name}  ({desc})", _C)
    buzzer.play(tune, tone_name)
    time.sleep(1.5)


def _run_auto_sequence(buzzer: BuzzerPublisher | None) -> None:
    _log("Starting full automated sequence (3s per state) ...", _B)
    _log("Press Ctrl+C to stop early\n", _DIM)
    try:
        for name in STATES:
            _activate_state(name, buzzer, hold_s=3.0)
        _log("Sequence complete", _G)
    except KeyboardInterrupt:
        _log("\nSequence interrupted", _Y)
    finally:
        clear_all()


def _interactive_loop(buzzer: BuzzerPublisher | None) -> None:
    state_list  = list(STATES.keys())
    tone_list   = list(BUZZER_TONES.keys()) if buzzer else []
    n_states    = len(state_list)

    while True:
        _print_header()
        _print_states(buzzer is not None)

        try:
            raw = input(f"  {_B}Select [{1}-{n_states + len(tone_list)}] / A / C / S / Q >{_RST} ").strip().upper()
        except (EOFError, KeyboardInterrupt):
            break

        if raw == "Q":
            break
        elif raw == "A":
            _run_auto_sequence(buzzer)
        elif raw == "C":
            clear_all()
        elif raw == "S":
            _show_current()
            input(f"  {_DIM}Press Enter to continue ...{_RST}")
        elif raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < n_states:
                _activate_state(state_list[idx], buzzer, hold_s=0)
            elif buzzer and n_states <= idx < n_states + len(tone_list):
                _play_tone(tone_list[idx - n_states], buzzer)
            else:
                _log("Invalid selection", _R)
        else:
            _log("Invalid input — enter a number, A, C, S, or Q", _Y)

    clear_all()
    _log("IPC files cleared. Exiting.", _G)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="DronePi LED + Buzzer bench test — simulates IPC states",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 bench_test_ipc.py                    # interactive, LED only
  python3 bench_test_ipc.py --buzzer           # interactive, LED + buzzer
  python3 bench_test_ipc.py --state SCANNING_MANUAL
  python3 bench_test_ipc.py --state POSTFLIGHT_FAILED
  python3 bench_test_ipc.py --tone SCAN_START
  python3 bench_test_ipc.py --auto             # full automated sequence
  python3 bench_test_ipc.py --cleanup          # clear IPC files and exit
        """,
    )
    parser.add_argument("--buzzer",  action="store_true",
                        help="Enable buzzer (requires ROS 2 + MAVROS)")
    parser.add_argument("--state",   default=None,
                        help=f"Activate one state directly. Options: {', '.join(STATES)}")
    parser.add_argument("--tone",    default=None,
                        help=f"Play one buzzer tone directly. Options: {', '.join(BUZZER_TONES)}")
    parser.add_argument("--hold",    type=float, default=3.0,
                        help="Seconds to hold state when using --state (default: 3)")
    parser.add_argument("--auto",    action="store_true",
                        help="Run full automated sequence (3s per state)")
    parser.add_argument("--cleanup", action="store_true",
                        help="Clear all IPC files and exit")
    args = parser.parse_args()

    if args.cleanup:
        clear_all()
        return

    buzzer = BuzzerPublisher() if args.buzzer else None

    try:
        if args.state:
            state = args.state.upper()
            _activate_state(state, buzzer, hold_s=args.hold)

        elif args.tone:
            if buzzer is None:
                _log("--tone requires --buzzer flag", _R)
                sys.exit(1)
            _play_tone(args.tone.upper(), buzzer)

        elif args.auto:
            _run_auto_sequence(buzzer)

        else:
            _interactive_loop(buzzer)

    finally:
        if buzzer:
            buzzer.shutdown()


if __name__ == "__main__":
    main()
