#!/usr/bin/env python3
"""
tests/test_buzzer_tunes.py — Standalone interactive buzzer tune tester.

PURPOSE
-------
Audition QBASIC Format 1 tune strings on the Pixhawk buzzer before committing
them to buzzer.py.  Run this script interactively on the Pi with MAVROS active.
It is a STANDALONE TEST UTILITY — it does not import any project modules and
has no dependency on the production watchdog_core package.

QBASIC FORMAT 1 QUICK REFERENCE
---------------------------------
T<n>   Tempo in BPM             T200 = 200 BPM
L<n>   Default note length      L8=eighth, L16=sixteenth, L32=thirty-second
O<n>   Octave (0–7)             O4=middle, O5=upper-mid, O6=high
>      Shift octave up one
<      Shift octave down one
C D E F G A B   Notes (natural)
C# D# F# G# A#  Sharps (append #)
R      Rest
P      Pause (same as R in practice)

USAGE
-----
  # Terminal 1 — ensure MAVROS is running
  ros2 launch mavros px4.launch fcu_url:=/dev/ttyAMA0:921600

  # Terminal 2 — run the tester
  source /opt/ros/jazzy/setup.bash
  conda activate dronepi
  python3 tests/test_buzzer_tunes.py

  # To test a single tune non-interactively and exit:
  python3 tests/test_buzzer_tunes.py --tune "T200L16O5CEGC" --label "my_tune"

REQUIREMENTS
------------
  rclpy, mavros_msgs  (already present on the Pi ROS 2 workspace)
"""

import argparse
import sys
import time

# ---------------------------------------------------------------------------
# Candidate tune library
# ---------------------------------------------------------------------------
# Each entry: (label, qbasic_string, description)
# All strings use QBASIC Format 1 (format=1 in PlayTuneV2).
# Modify, add, or remove entries freely — this file is never imported by
# production code.

CANDIDATE_TUNES: list[tuple[str, str, str]] = [
    # ── Stack START ──────────────────────────────────────────────────────────
    # Goal: clearly ascending, pleasant, audible at ~50m, not too fast
    ("start_v1_current",    "T240L16O5CEGC",        "Current — fast four-note rise"),
    ("start_v2_slower",     "T200L12O5CEG>C",        "Slower tempo, longer notes, firmer feel"),
    ("start_v3_fanfare",    "T180L8O4CEGCEG>C",      "Two-step rise, fanfare-like"),
    ("start_v4_shortrise",  "T220L16O5CE>CE",        "Two octave jump, punchy"),
    ("start_v5_smooth",     "T160L8O4EG>CE",         "Slow smooth ascent, easy on the ears"),

    # ── Stack STOP ───────────────────────────────────────────────────────────
    # Goal: clearly descending, clearly distinct from start
    ("stop_v1_current",     "T240L16O5CG<EGC",       "Current — fast descending"),
    ("stop_v2_slower",      "T160L12O5C<GEC",        "Slower fall, more deliberate"),
    ("stop_v3_twostep",     "T180L8O5C<GEG<C",       "Two-step fall, mirror of fanfare start"),
    ("stop_v4_shortfall",   "T220L16O5C<EC<E",       "Punchy two-octave drop"),
    ("stop_v5_smooth",      "T160L8O5GE<GE",         "Smooth descent, relaxed feel"),

    # ── Button ACK ───────────────────────────────────────────────────────────
    # Goal: brief, neutral confirmation — not start, not stop, not error
    ("ack_v1_current",      "T200L32O6C",            "Current — single high click"),
    ("ack_v2_twonote",      "T200L16O5GC",           "Two-note up-down confirm"),
    ("ack_v3_ping",         "T300L32O5G",            "Fast single mid-note ping"),
    ("ack_v4_chirp",        "T250L32O5CE",           "Quick two-note chirp"),
    ("ack_v5_double",       "T200L32O5CC",           "Double tap same note"),

    # ── Error ────────────────────────────────────────────────────────────────
    # Goal: alarming, clearly NOT a confirmation or start/stop
    ("error_v1_current",    "T200L16O5CPCPC",        "Current — staccato triple beep"),
    ("error_v2_descend",    "T180L8O6C<A<F",         "Falling tritone — unsettling"),
    ("error_v3_urgent",     "T260L16O6CPCP",         "High fast double — urgent"),
    ("error_v4_buzz",       "T300L32O6CDCDCD",       "Rapid semitone buzz"),
    ("error_v5_alarm",      "T220L8O6CECECE",        "Alternating alarm pattern"),

    # ── Processing heartbeat (plays every 5 s during post-flight) ────────────
    # Goal: subtle tick — present enough to confirm work is ongoing, not annoying
    ("proc_v1_current",     "T200L32O4G",            "Current — single mid tick"),
    ("proc_v2_quieter",     "T180L64O4G",            "Very short tick — barely audible"),
    ("proc_v3_softtick",    "T160L32O4E",            "Lower pitch tick"),
    ("proc_v4_twotick",     "T200L64O4GE",           "Two-note subtle tick"),
    ("proc_v5_hightick",    "T220L64O5C",            "Short high tick"),

    # ── Done (post-flight complete) ───────────────────────────────────────────
    # Goal: satisfying resolution — longer than ACK, clearly positive
    ("done_v1_current",     "T240L16O5CE",           "Current — simple two-note"),
    ("done_v2_chord",       "T180L12O5CEGCE",        "Arpeggio rise and return"),
    ("done_v3_triumphant",  "T200L8O4CEGCEG>C",      "Full fanfare — mirrors start_v3"),
    ("done_v4_clean",       "T180L12O5CEG>C<GEC",    "Rise and fall — complete arc"),
    ("done_v5_simple",      "T200L16O5GCEG",         "Gentle four-note resolve"),
]


# ---------------------------------------------------------------------------
# ROS 2 publisher
# ---------------------------------------------------------------------------

def _make_publisher():
    """Initialise rclpy and return (node, publisher, PlayTuneV2 class)."""
    try:
        import rclpy
        from rclpy.node import Node
        from mavros_msgs.msg import PlayTuneV2
    except ImportError as exc:
        print(f"[ERROR] ROS 2 / mavros_msgs not available: {exc}")
        print("        Source your ROS 2 workspace and activate the dronepi conda env.")
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node("buzzer_tune_tester")
    pub = node.create_publisher(PlayTuneV2, "/mavros/play_tune", 10)

    # Give the publisher time to connect to MAVROS
    time.sleep(0.5)
    return node, pub, PlayTuneV2


def play_tune(pub, PlayTuneV2, tune_string: str) -> None:
    """Publish a single QBASIC tune string (format=1) to the Pixhawk buzzer."""
    msg = PlayTuneV2()
    msg.format = 1          # QBASIC Format 1 — confirmed working on this firmware
    msg.tune = tune_string
    pub.publish(msg)


# ---------------------------------------------------------------------------
# Interactive menu
# ---------------------------------------------------------------------------

def _group_by_prefix(tunes: list) -> dict[str, list]:
    """Group candidate tunes by their event prefix for display."""
    groups: dict[str, list] = {}
    for label, tune_str, desc in tunes:
        prefix = label.rsplit("_v", 1)[0]  # e.g. "start_v2_slower" → "start"
        groups.setdefault(prefix, []).append((label, tune_str, desc))
    return groups


def run_interactive(node, pub, PlayTuneV2) -> None:
    """Present a grouped menu and let the user audition tunes one at a time."""
    groups = _group_by_prefix(CANDIDATE_TUNES)

    print("\n" + "=" * 62)
    print("  DronePi Buzzer Tune Tester")
    print("  QBASIC Format 1 — Pixhawk 6X")
    print("=" * 62)
    print("  Commands:")
    print("    <number>   Play the tune at that index")
    print("    a          Play ALL tunes in sequence (0.8 s gap)")
    print("    q          Quit")
    print("=" * 62 + "\n")

    # Build flat indexed list and print grouped
    indexed: list[tuple[str, str, str]] = []
    for group_name, entries in groups.items():
        print(f"  ── {group_name.upper().replace('_', ' ')} ──")
        for label, tune_str, desc in entries:
            idx = len(indexed)
            print(f"    [{idx:2d}]  {label:<28}  {tune_str:<26}  {desc}")
            indexed.append((label, tune_str, desc))
        print()

    while True:
        try:
            raw = input("Select tune > ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if raw == "q":
            print("Exiting.")
            break

        if raw == "a":
            print(f"\nPlaying all {len(indexed)} tunes with 0.8 s gap...\n")
            for i, (label, tune_str, desc) in enumerate(indexed):
                print(f"  [{i:2d}] {label}  →  {tune_str}")
                play_tune(pub, PlayTuneV2, tune_str)
                time.sleep(0.8)
            print("\nDone.\n")
            continue

        try:
            idx = int(raw)
        except ValueError:
            print("  Invalid input. Enter a number, 'a', or 'q'.")
            continue

        if idx < 0 or idx >= len(indexed):
            print(f"  Index out of range (0–{len(indexed) - 1}).")
            continue

        label, tune_str, desc = indexed[idx]
        print(f"\n  ▶  [{idx:2d}] {label}")
        print(f"     Tune   : {tune_str}")
        print(f"     Desc   : {desc}\n")
        play_tune(pub, PlayTuneV2, tune_str)

    node.destroy_node()
    try:
        import rclpy
        rclpy.shutdown()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Non-interactive single-shot mode
# ---------------------------------------------------------------------------

def run_single(node, pub, PlayTuneV2, tune_str: str, label: str) -> None:
    """Play one tune string and exit — useful for scripted testing."""
    print(f"Playing: {label}  →  {tune_str}")
    play_tune(pub, PlayTuneV2, tune_str)
    time.sleep(0.5)   # allow publish to flush before node teardown
    node.destroy_node()
    try:
        import rclpy
        rclpy.shutdown()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone Pixhawk buzzer tune tester — audition QBASIC tunes interactively."
    )
    parser.add_argument(
        "--tune",
        type=str,
        default=None,
        help="Play a single tune string directly and exit (e.g. 'T200L16O5CEGC').",
    )
    parser.add_argument(
        "--label",
        type=str,
        default="custom",
        help="Label shown when playing a single tune with --tune.",
    )
    args = parser.parse_args()

    node, pub, PlayTuneV2 = _make_publisher()

    if args.tune:
        run_single(node, pub, PlayTuneV2, args.tune, args.label)
    else:
        run_interactive(node, pub, PlayTuneV2)


if __name__ == "__main__":
    main()
