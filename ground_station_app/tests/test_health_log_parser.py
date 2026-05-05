#!/usr/bin/env python3
"""
tests/test_health_log_parser.py — Standalone test for HealthLogParser.

Tests
-----
1. File not found → returns defaults, does not raise.
2. Full CSV with all fields → correct aggregation (max, mean, throttle count).
3. CSV with throttling events → throttling_events count is correct.
4. CSV with throttle_bits → unique non-zero values collected.
5. CSV with missing columns → falls back gracefully.
6. Empty CSV → defaults returned.
7. Output is JSON-serialisable.

Run with:
    python tests/test_health_log_parser.py
"""

import json
import logging
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from ground_station.health_log_parser import HealthLogParser

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
# Column names match rpi_health_node.py collect_metrics() output keys.

FULL_CSV = """\
timestamp,cpu_percent,cpu_temp,cpu_freq_mhz,mem_percent,mem_used_mb,mem_total_mb,disk_percent,throttled,throttle_bits,load_avg_1m,load_avg_5m
1714000000.0,42.1,63.5,1800.0,52.3,8372.0,16000.0,41.2,False,0x0,1.2,1.1
1714000002.0,55.3,65.2,1800.0,53.1,8500.0,16000.0,41.2,False,0x0,1.4,1.2
1714000004.0,88.7,71.4,1800.0,64.0,10240.0,16000.0,41.3,True,0x50004,2.9,1.8
1714000006.0,72.0,68.9,1800.0,60.2,9632.0,16000.0,41.3,False,0x0,2.1,1.6
1714000008.0,61.5,66.1,1800.0,58.8,9408.0,16000.0,41.4,False,0x0,1.8,1.5
"""

THROTTLED_CSV = """\
timestamp,cpu_percent,cpu_temp,mem_percent,throttled,throttle_bits
1714000000.0,70.0,72.0,55.0,True,0x50004
1714000002.0,80.0,75.0,58.0,True,0x50000
1714000004.0,40.0,60.0,50.0,False,0x0
"""

PARTIAL_CSV = """\
timestamp,cpu_temp
1714000000.0,63.5
1714000002.0,71.2
1714000004.0,69.8
"""

EMPTY_CSV = """\
timestamp,cpu_percent,cpu_temp,mem_percent,throttled
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
    result = HealthLogParser("/nonexistent/health_log.csv").parse()
    check("file_not_found_no_raise", True)
    check("file_not_found_sample_count", result["sample_count"] == 0,
          str(result["sample_count"]))
    check("file_not_found_temp_max", result["cpu_temp_max_c"] == 0.0,
          str(result["cpu_temp_max_c"]))


def test_full_csv() -> None:
    """All fields parsed and aggregated correctly."""
    tmp = _write(FULL_CSV)
    try:
        result = HealthLogParser(tmp).parse()

        check("full_sample_count",    result["sample_count"] == 5,
              str(result["sample_count"]))
        # Max temp = 71.4
        check("full_cpu_temp_max",    abs(result["cpu_temp_max_c"] - 71.4) < 0.2,
              str(result["cpu_temp_max_c"]))
        # Mean of [63.5, 65.2, 71.4, 68.9, 66.1] = 67.02
        check("full_cpu_temp_mean",   63.0 < result["cpu_temp_mean_c"] < 72.0,
              str(result["cpu_temp_mean_c"]))
        # 1 throttling event
        check("full_throttling_events", result["throttling_events"] == 1,
              str(result["throttling_events"]))
        # Max mem = 64.0
        check("full_memory_peak",     abs(result["memory_peak_pct"] - 64.0) < 0.2,
              str(result["memory_peak_pct"]))
        # Max cpu = 88.7
        check("full_cpu_percent_max", abs(result["cpu_percent_max"] - 88.7) < 0.2,
              str(result["cpu_percent_max"]))
        # throttle_bits_seen should contain 0x50004
        check("full_throttle_bits_seen",
              "0x50004" in result["throttle_bits_seen"],
              str(result["throttle_bits_seen"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_throttled_csv() -> None:
    """Throttling count and unique bits are correct for multiple events."""
    tmp = _write(THROTTLED_CSV)
    try:
        result = HealthLogParser(tmp).parse()
        check("throttled_count",   result["throttling_events"] == 2,
              str(result["throttling_events"]))
        check("throttled_bits_two", len(result["throttle_bits_seen"]) == 2,
              str(result["throttle_bits_seen"]))
        check("zero_bits_excluded",
              "0x0" not in result["throttle_bits_seen"],
              str(result["throttle_bits_seen"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_partial_csv() -> None:
    """Missing columns fall back to defaults; present columns still parse."""
    tmp = _write(PARTIAL_CSV)
    try:
        result = HealthLogParser(tmp).parse()
        check("partial_temp_max",    result["cpu_temp_max_c"] > 0.0,
              str(result["cpu_temp_max_c"]))
        check("partial_no_throttle", result["throttling_events"] == 0,
              str(result["throttling_events"]))
        check("partial_no_mem",      result["memory_peak_pct"] == 0.0,
              str(result["memory_peak_pct"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_empty_csv() -> None:
    """Header-only CSV returns defaults without raising."""
    tmp = _write(EMPTY_CSV)
    try:
        result = HealthLogParser(tmp).parse()
        check("empty_no_raise", True)
        check("empty_sample_count", result["sample_count"] == 0,
              str(result["sample_count"]))
    finally:
        tmp.unlink(missing_ok=True)


def test_json_serialisable() -> None:
    """Output from full CSV is JSON-serialisable."""
    tmp = _write(FULL_CSV)
    try:
        result = HealthLogParser(tmp).parse()
        serialised = json.dumps(result)
        check("json_serialisable", True, f"len={len(serialised)}")
    except (TypeError, ValueError) as exc:
        check("json_serialisable", False, str(exc))
    finally:
        tmp.unlink(missing_ok=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    log.info("=" * 60)
    log.info("HealthLogParser test suite")
    log.info("=" * 60)

    test_file_not_found()
    test_full_csv()
    test_throttled_csv()
    test_partial_csv()
    test_empty_csv()
    test_json_serialisable()

    log.info("=" * 60)
    passed = sum(1 for _, s in results if s == "PASS")
    failed = sum(1 for _, s in results if s == "FAIL")
    log.info(f"Results: {passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
