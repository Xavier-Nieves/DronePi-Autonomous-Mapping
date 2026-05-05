#!/usr/bin/env python3
"""
tests/test_bag_summary_parser.py — Standalone test for BagSummaryParser.

Tests
-----
1. File not found → returns defaults, does not raise.
2. Well-formed CSV → all fields parsed with correct types.
3. CSV with missing columns → absent fields fall back to defaults.
4. CSV with corrupt values (non-numeric) → affected fields return None.
5. Empty CSV (header only) → returns defaults.
6. Output is JSON-serialisable.

Run with:
    python tests/test_bag_summary_parser.py
"""

import json
import logging
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from ground_station.bag_summary_parser import BagSummaryParser

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


# ── Stub CSV data ──────────────────────────────────────────────────────────────

FULL_CSV = """\
point_count_raw,point_count_final,drift_estimate_m,loop_closures,mls_iterations,sor_outliers_removed,processing_duration_s,bag_path
2450000,1200000,0.18,3,5,18450,47.3,/mnt/ssd/rosbags/scan_20260422_143215
"""

PARTIAL_CSV = """\
point_count_final,drift_estimate_m
980000,0.24
"""

CORRUPT_CSV = """\
point_count_raw,point_count_final,drift_estimate_m,loop_closures
not_a_number,1100000,not_a_float,2
"""

EMPTY_CSV = """\
point_count_raw,point_count_final,drift_estimate_m,loop_closures
"""


def _write(content: str) -> Path:
    f = tempfile.NamedTemporaryFile(
        mode="w", suffix=".csv", delete=False, encoding="utf-8"
    )
    f.write(content)
    f.close()
    return Path(f.name)


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_file_not_found() -> None:
    """Returns defaults when file is absent — does not raise."""
    result = BagSummaryParser("/nonexistent/bag_summary.csv").parse()
    check("file_not_found_no_raise", True)
    check("file_not_found_defaults", result["point_count_final"] == 0,
          str(result["point_count_final"]))


def test_full_csv() -> None:
    """All fields parsed correctly from a well-formed CSV."""
    tmp = _write(FULL_CSV)
    try:
        result = BagSummaryParser(tmp).parse()
        check("full_point_count_raw",      result["point_count_raw"] == 2_450_000,
              str(result["point_count_raw"]))
        check("full_point_count_final",    result["point_count_final"] == 1_200_000,
              str(result["point_count_final"]))
        check("full_drift_estimate",       abs(result["drift_estimate_m"] - 0.18) < 0.001,
              str(result["drift_estimate_m"]))
        check("full_loop_closures",        result["loop_closures"] == 3,
              str(result["loop_closures"]))
        check("full_mls_iterations",       result["mls_iterations"] == 5,
              str(result["mls_iterations"]))
        check("full_sor_outliers",         result["sor_outliers_removed"] == 18_450,
              str(result["sor_outliers_removed"]))
        check("full_processing_duration",  abs(result["processing_duration_s"] - 47.3) < 0.01,
              str(result["processing_duration_s"]))
        check("full_bag_path_nonempty",    bool(result["bag_path"]),
              str(result["bag_path"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_partial_csv() -> None:
    """Missing columns fall back to defaults without raising."""
    tmp = _write(PARTIAL_CSV)
    try:
        result = BagSummaryParser(tmp).parse()
        check("partial_present_field",   result["point_count_final"] == 980_000,
              str(result["point_count_final"]))
        check("partial_absent_raw",      result["point_count_raw"] == 0,
              str(result["point_count_raw"]))
        check("partial_absent_loops",    result["loop_closures"] == 0,
              str(result["loop_closures"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_corrupt_values() -> None:
    """Non-numeric values return None for affected fields; others parse normally."""
    tmp = _write(CORRUPT_CSV)
    try:
        result = BagSummaryParser(tmp).parse()
        check("corrupt_int_returns_none",   result["point_count_raw"] is None,
              str(result["point_count_raw"]))
        check("corrupt_float_returns_none", result["drift_estimate_m"] is None,
              str(result["drift_estimate_m"]))
        check("corrupt_valid_int_parses",   result["point_count_final"] == 1_100_000,
              str(result["point_count_final"]))
        check("corrupt_valid_loops_parses", result["loop_closures"] == 2,
              str(result["loop_closures"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_empty_csv() -> None:
    """Header-only CSV returns defaults without raising."""
    tmp = _write(EMPTY_CSV)
    try:
        result = BagSummaryParser(tmp).parse()
        check("empty_csv_no_raise", True)
        check("empty_csv_defaults", result["point_count_final"] == 0,
              str(result["point_count_final"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_json_serialisable() -> None:
    """Output from a full CSV is JSON-serialisable."""
    tmp = _write(FULL_CSV)
    try:
        result = BagSummaryParser(tmp).parse()
        serialised = json.dumps(result)
        check("json_serialisable", True, f"len={len(serialised)}")
    except (TypeError, ValueError) as exc:
        check("json_serialisable", False, str(exc))
    finally:
        tmp.unlink(missing_ok=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    log.info("=" * 60)
    log.info("BagSummaryParser test suite")
    log.info("=" * 60)

    test_file_not_found()
    test_full_csv()
    test_partial_csv()
    test_corrupt_values()
    test_empty_csv()
    test_json_serialisable()

    log.info("=" * 60)
    passed = sum(1 for _, s in results if s == "PASS")
    failed = sum(1 for _, s in results if s == "FAIL")
    log.info(f"Results: {passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
