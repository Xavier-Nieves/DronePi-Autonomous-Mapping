#!/usr/bin/env python3
"""
tests/test_ulog_parser.py — Standalone test for ULogParser.

Strategy
--------
pyulog has no public API for writing .ulg files programmatically, and a real
DronePi .ulg file is not available (see handoff §12, open question 1).

This test validates ULogParser against two conditions:
  1. File-not-found → ULogParseError is raised.
  2. A minimal synthetic .ulg created using pyulog's own ULogWriter (if
     available in the installed version) OR a pre-built binary stub.
  3. Missing-message-type resilience: parser returns defaults without raising
     when message types are absent.

Because pyulog's ULogWriter API varies across versions, the test also
validates the parser's schema contract: every required output key is present
and has the correct Python type regardless of data source.

Run with:
    python tests/test_ulog_parser.py

Exits 0 on all pass, 1 on any failure.
"""

import json
import logging
import struct
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from ground_station.ulog_parser import ULogParser, ULogParseError

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

results: list[tuple[str, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    status = "PASS" if condition else "FAIL"
    results.append((name, status))
    marker = "  [PASS]" if condition else "  [FAIL]"
    log.info(f"{marker}  {name}" + (f" — {detail}" if detail else ""))


# ── Minimal valid .ulg binary stub ────────────────────────────────────────────
# A .ulg file begins with a fixed magic header. pyulog will raise on read
# if the magic is wrong; a file with correct magic but no message types will
# parse but produce empty datasets — exercising the "missing message type"
# fallback paths in ULogParser.
#
# ULog magic: b'\x55\x4C\x6F\x67\x01\x12\x35' (7 bytes) + version byte (0x01)
# followed by uint64 timestamp (8 bytes).
# Reference: https://docs.px4.io/main/en/dev_log/ulog_file_format.html

ULOG_MAGIC = b'\x55\x4C\x6F\x67\x01\x12\x35'  # 7 bytes
ULOG_VERSION = b'\x01'                           # 1 byte
ULOG_TIMESTAMP = struct.pack('<Q', 1_714_000_000_000_000)  # 8 bytes µs


def _write_minimal_ulog(path: Path) -> None:
    """Write a minimal valid ULog file with header only and no message data."""
    with open(path, "wb") as f:
        f.write(ULOG_MAGIC)
        f.write(ULOG_VERSION)
        f.write(ULOG_TIMESTAMP)
    # No further message data — pyulog will parse header, find no datasets.


def _write_corrupt_ulog(path: Path) -> None:
    """Write a file with wrong magic bytes — pyulog should reject it."""
    with open(path, "wb") as f:
        f.write(b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF')


# ── Expected output schema ────────────────────────────────────────────────────

REQUIRED_KEYS = {
    "flight_id":           str,
    "start_time":          str,
    "duration_s":          float,
    "modes":               list,
    "ekf2_innovations":    dict,
    "gps":                 dict,
    "battery":             dict,
    "rc_channel_7_events": list,
    "anomalies":           list,
}

EKF2_KEYS  = {"max", "mean", "spikes"}
GPS_KEYS   = {"hdop_max", "hdop_mean", "fix_loss_events"}
BAT_KEYS   = {"start_v", "end_v", "min_v"}


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_file_not_found() -> None:
    """ULogParseError raised for non-existent file."""
    parser = ULogParser("/nonexistent/path/flight.ulg")
    try:
        parser.parse()
        check("file_not_found_raises", False, "should have raised ULogParseError")
    except ULogParseError as exc:
        check("file_not_found_raises", True, str(exc)[:60])


def test_corrupt_file() -> None:
    """ULogParseError raised for corrupt file (bad magic bytes)."""
    with tempfile.NamedTemporaryFile(suffix=".ulg", delete=False) as f:
        tmp = Path(f.name)
    _write_corrupt_ulog(tmp)
    try:
        ULogParser(tmp).parse()
        check("corrupt_file_raises", False, "should have raised ULogParseError")
    except ULogParseError as exc:
        check("corrupt_file_raises", True, str(exc)[:60])
    finally:
        tmp.unlink(missing_ok=True)


def test_minimal_ulog_schema() -> None:
    """
    Minimal ULog (header only, no message types) returns all required keys
    with correct types and does not raise.
    """
    with tempfile.NamedTemporaryFile(suffix=".ulg", delete=False) as f:
        tmp = Path(f.name)
    _write_minimal_ulog(tmp)

    try:
        result = ULogParser(tmp).parse()
    except ULogParseError as exc:
        check("minimal_ulog_no_raise", False, str(exc)[:80])
        tmp.unlink(missing_ok=True)
        return

    check("minimal_ulog_no_raise", True)

    # Verify all top-level keys present
    for key, expected_type in REQUIRED_KEYS.items():
        present = key in result
        correct_type = isinstance(result.get(key), expected_type)
        check(f"key_{key}_present_correct_type", present and correct_type,
              f"type={type(result.get(key)).__name__} expected={expected_type.__name__}")

    # Verify nested dict keys
    for k in EKF2_KEYS:
        check(f"ekf2_{k}_present", k in result["ekf2_innovations"])
    for k in GPS_KEYS:
        check(f"gps_{k}_present", k in result["gps"])
    for k in BAT_KEYS:
        check(f"battery_{k}_present", k in result["battery"])

    # flight_id must equal file stem
    check("flight_id_is_stem", result["flight_id"] == tmp.stem,
          f"got={result['flight_id']} expected={tmp.stem}")

    tmp.unlink(missing_ok=True)


def test_defaults_are_safe() -> None:
    """
    All numeric defaults in an empty-dataset parse are >= 0 or None.
    Specifically: duration_s=0.0, ekf2 max=0.0, gps hdop=0.0, battery voltages=0.0.
    """
    with tempfile.NamedTemporaryFile(suffix=".ulg", delete=False) as f:
        tmp = Path(f.name)
    _write_minimal_ulog(tmp)

    try:
        result = ULogParser(tmp).parse()
    except ULogParseError:
        check("defaults_are_safe", False, "parse raised unexpectedly")
        tmp.unlink(missing_ok=True)
        return

    check("default_duration_s", result["duration_s"] >= 0.0,
          str(result["duration_s"]))
    check("default_ekf2_max", result["ekf2_innovations"]["max"] >= 0.0)
    check("default_gps_hdop_max", result["gps"]["hdop_max"] >= 0.0)
    check("default_battery_start_v", result["battery"]["start_v"] >= 0.0)
    check("default_modes_is_list", isinstance(result["modes"], list))
    check("default_anomalies_is_list", isinstance(result["anomalies"], list))

    tmp.unlink(missing_ok=True)


def test_output_is_json_serialisable() -> None:
    """parse() output must be JSON-serialisable (required by ReportGenerator)."""
    with tempfile.NamedTemporaryFile(suffix=".ulg", delete=False) as f:
        tmp = Path(f.name)
    _write_minimal_ulog(tmp)

    try:
        result = ULogParser(tmp).parse()
        serialised = json.dumps(result)
        check("json_serialisable", True, f"len={len(serialised)}")
    except (ULogParseError, TypeError, ValueError) as exc:
        check("json_serialisable", False, str(exc)[:80])
    finally:
        tmp.unlink(missing_ok=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    log.info("=" * 60)
    log.info("ULogParser test suite")
    log.info("=" * 60)

    test_file_not_found()
    test_corrupt_file()
    test_minimal_ulog_schema()
    test_defaults_are_safe()
    test_output_is_json_serialisable()

    log.info("=" * 60)
    passed = sum(1 for _, s in results if s == "PASS")
    failed = sum(1 for _, s in results if s == "FAIL")
    log.info(f"Results: {passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
