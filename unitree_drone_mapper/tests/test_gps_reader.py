#!/usr/bin/env python3
"""tests/test_gps_reader.py — Standalone GPS reader test with structured log output.

PURPOSE
-------
Validates the GpsReader module in isolation and produces a structured log
file recording HDOP, satellite count, fix quality, ENU positions, and the
simulated PX4 acceptance decision for every fix observed during the live
test window.

The log file is written to the same directory as this script:

    tests/gps_reader_test_<YYYYMMDD_HHMMSS>.log
    tests/gps_reader_test_<YYYYMMDD_HHMMSS>_fixes.csv

LOG FILE SECTIONS
-----------------
  [HEADER]      Test parameters, thresholds, system info, start time.
  [UNIT TESTS]  One line per assertion — PASS / FAIL / SKIP.
  [LIVE GPS]    One row per fix: elapsed, fix_type, sats, HDOP, GDOP_est,
                lat, lon, alt, ENU delta, reliable, px4_accept, drift_m.
  [SUMMARY]     Fix statistics, HDOP histogram, pass/fail verdict.

PX4 ACCEPTANCE SIMULATION
--------------------------
For each fix the log records whether PX4 EKF2 would have accepted it if
EKF2_GPS_CTRL were enabled, using the same three criteria:
    ① fix_type >= 1    (valid fix)
    ② HDOP     <= 2.5  (EKF2_GPS_CHECK bit 1 — default threshold)
    ③ sats     >= 6    (EKF2_GPS_CHECK bit 0 — default threshold)
Source: PX4 parameter reference (EKF2_GPS_CHECK).

GDOP ESTIMATION
---------------
True GDOP requires per-satellite elevation data. Without it we estimate:
    GDOP_est = HDOP x 1.5
Valid for a healthy balanced constellation. Labelled "est" throughout.
Source: Langley (1999), GPS World, "Dilution of Precision".

USAGE
-----
    python3 tests/test_gps_reader.py                  # live + offline
    python3 tests/test_gps_reader.py --synthetic       # offline only
    python3 tests/test_gps_reader.py --duration 120    # 2-min live window
    python3 tests/test_gps_reader.py --hdop 3.0        # custom threshold
    python3 tests/test_gps_reader.py --verbose         # echo all to stdout
    python3 tests/test_gps_reader.py --duration 0      # indefinite (Ctrl-C)
"""

from __future__ import annotations

import argparse
import csv
import io
import logging
import math
import os
import platform
import sys
import time
from datetime import datetime
from pathlib import Path

# ── Path setup ────────────────────────────────────────────────────────────────
_SCRIPT_DIR   = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent
sys.path.insert(0, str(_PROJECT_ROOT))

from flight.gps_reader import (
    GpsReader, GpsFix,
    _wgs84_to_enu, _nmea_to_decimal, _build_fix,
    HDOP_THRESHOLD, SAT_COUNT_MIN, DRIFT_THRESHOLD_M,
)

# ── PX4 EKF2 GPS check thresholds ────────────────────────────────────────────
# Source: https://docs.px4.io/main/en/advanced_config/parameter_reference.html
PX4_HDOP_MAX         = 2.5
PX4_SAT_MIN          = 6
PX4_FIX_MIN          = 1
GDOP_EST_MULTIPLIER  = 1.5   # Langley (1999)

# ── Fixtures ──────────────────────────────────────────────────────────────────
HOME_LAT =  18.2085
HOME_LON = -67.1398
HOME_ALT =  10.0

FIXTURE_FIXES = {
    "home":       GpsFix(lat=HOME_LAT,            lon=HOME_LON,           alt=HOME_ALT, hdop=1.2, satellites=10, fix_type=1, reliable=True,  source="fixture"),
    "10m_east":   GpsFix(lat=HOME_LAT,            lon=HOME_LON+0.0001,    alt=HOME_ALT, hdop=1.4, satellites=9,  fix_type=1, reliable=True,  source="fixture"),
    "10m_north":  GpsFix(lat=HOME_LAT+0.0001,     lon=HOME_LON,           alt=HOME_ALT, hdop=1.5, satellites=8,  fix_type=1, reliable=True,  source="fixture"),
    "bad_hdop":   GpsFix(lat=HOME_LAT,            lon=HOME_LON,           alt=HOME_ALT, hdop=4.8, satellites=7,  fix_type=1, reliable=False, source="fixture"),
    "bad_sats":   GpsFix(lat=HOME_LAT,            lon=HOME_LON,           alt=HOME_ALT, hdop=1.1, satellites=4,  fix_type=1, reliable=False, source="fixture"),
    "no_fix":     GpsFix(lat=0.0,                 lon=0.0,                alt=0.0,      hdop=99., satellites=0,  fix_type=0, reliable=False, source="fixture"),
    "drifted_8m": GpsFix(lat=HOME_LAT+0.000072,   lon=HOME_LON,           alt=HOME_ALT, hdop=1.3, satellites=9,  fix_type=1, reliable=True,  source="fixture"),
    "marginal":   GpsFix(lat=HOME_LAT,            lon=HOME_LON,           alt=HOME_ALT, hdop=2.4, satellites=6,  fix_type=1, reliable=True,  source="fixture"),
    "px4_reject": GpsFix(lat=HOME_LAT,            lon=HOME_LON,           alt=HOME_ALT, hdop=2.6, satellites=7,  fix_type=1, reliable=False, source="fixture"),
}

# ── Log state ─────────────────────────────────────────────────────────────────
_session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
_LOG_PATH   = _SCRIPT_DIR / f"gps_reader_test_{_session_ts}.log"
_CSV_PATH   = _SCRIPT_DIR / f"gps_reader_test_{_session_ts}_fixes.csv"
_buf        = io.StringIO()
_verbose    = False
_passed = _failed = _skipped = 0

_CSV_HEADERS = [
    "elapsed_s","timestamp_iso","fix_type","satellites",
    "hdop","gdop_est","lat","lon","alt_m",
    "enu_east_m","enu_north_m","enu_up_m",
    "reliable","px4_would_accept","drift_vs_slam_m","source",
]


def _log(msg: str, level: str = "INFO") -> None:
    ts   = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}] [{level:8s}] {msg}"
    _buf.write(line + "\n")
    if _verbose or level in ("FAIL","ERROR","CRITICAL"):
        print(line)


def _section(title: str) -> None:
    sep = "─" * 60
    _buf.write(f"\n{sep}\n  {title}\n{sep}\n")
    if _verbose:
        print(f"\n── {title}")


def _flush_log() -> None:
    with open(_LOG_PATH, "w", encoding="utf-8") as f:
        f.write(_buf.getvalue())
    print(f"\n  Log  → {_LOG_PATH}")


def _pass(label: str, detail: str = "") -> None:
    global _passed; _passed += 1
    _log(f"PASS  {label}" + (f"  ({detail})" if detail else ""), "PASS")
    if not _verbose:
        print(f"  [PASS] {label}" + (f"  — {detail}" if detail else ""))


def _fail(label: str, reason: str = "") -> None:
    global _failed; _failed += 1
    _log(f"FAIL  {label}" + (f"  — {reason}" if reason else ""), "FAIL")
    if not _verbose:
        print(f"  [FAIL] {label}" + (f"  — {reason}" if reason else ""))


def _skip(label: str, reason: str = "") -> None:
    global _skipped; _skipped += 1
    _log(f"SKIP  {label}" + (f"  — {reason}" if reason else ""), "SKIP")
    if not _verbose:
        print(f"  [SKIP] {label}" + (f"  — {reason}" if reason else ""))


# ── Helpers ───────────────────────────────────────────────────────────────────

def px4_would_accept(fix: GpsFix) -> bool:
    return (fix.fix_type >= PX4_FIX_MIN
            and fix.hdop     <= PX4_HDOP_MAX
            and fix.satellites >= PX4_SAT_MIN)


def gdop_est(hdop: float) -> float:
    return round(hdop * GDOP_EST_MULTIPLIER, 2)


# ── Unit tests ────────────────────────────────────────────────────────────────

def test_nmea_parser() -> None:
    _section("UNIT TEST — NMEA coordinate parser")
    cases = [
        ("4807.0380", "N",  48.117300, 0.0001, "48°07.038'N"),
        ("01131.0000","E",  11.516667, 0.0001, "11°31.000'E"),
        ("4807.0380", "S", -48.117300, 0.0001, "48°07.038'S"),
        ("00130.0000","W",  -1.500000, 0.0001, "01°30.000'W"),
        ("1812.5100", "N",  18.208500, 0.0001, "18°22.510'N local"),
        ("6709.3880", "W", -67.156467, 0.0001, "67°09.388'W local"),
    ]
    for raw, direction, expected, tol, label in cases:
        try:
            result = _nmea_to_decimal(raw, direction)
            if abs(result - expected) <= tol:
                _pass(f"NMEA {label}", f"{result:.6f}°")
            else:
                _fail(f"NMEA {label}", f"got {result:.6f}°  expected {expected:.6f}°")
        except Exception as exc:
            _fail(f"NMEA {label}", str(exc))


def test_wgs84_enu() -> None:
    _section("UNIT TEST — WGS84 → ENU conversion")
    cases = [
        (0.0,     0.0001,  10.56,  0.0,  0.5, "0.0001° east"),
        (0.0001,  0.0,      0.0,  11.12, 0.5, "0.0001° north"),
        (-0.0001, 0.0,      0.0, -11.12, 0.5, "0.0001° south"),
        (0.0,    -0.0001, -10.56,  0.0,  0.5, "0.0001° west"),
        (0.0,     0.0,      0.0,   0.0, 0.01, "zero delta"),
    ]
    for dlat, dlon, exp_e, exp_n, tol, label in cases:
        e, n = _wgs84_to_enu(HOME_LAT+dlat, HOME_LON+dlon, HOME_LAT, HOME_LON)
        if abs(e-exp_e) <= tol and abs(n-exp_n) <= tol:
            _pass(f"ENU {label}", f"E={e:.2f}m N={n:.2f}m")
        else:
            _fail(f"ENU {label}", f"got E={e:.2f} N={n:.2f}, expected E≈{exp_e:.2f} N≈{exp_n:.2f}")


def test_quality_gate(hdop_threshold: float) -> None:
    _section(f"UNIT TEST — Quality gate (HDOP_max={hdop_threshold}  sat_min={SAT_COUNT_MIN})")
    reader = GpsReader(hdop_threshold=hdop_threshold)
    cases = [
        ("home",       True,  "Clean fix: HDOP=1.2  sats=10"),
        ("10m_east",   True,  "Offset fix: HDOP=1.4  sats=9"),
        ("marginal",   True,  f"Marginal: HDOP=2.4 (< {hdop_threshold})"),
        ("px4_reject", False, f"PX4 reject: HDOP=2.6 (> {hdop_threshold})"),
        ("bad_hdop",   False, "Bad HDOP=4.8  sats=7"),
        ("bad_sats",   False, "Bad sats=4  HDOP=1.1"),
        ("no_fix",     False, "No fix: fix_type=0"),
    ]
    for name, expected, label in cases:
        fix    = FIXTURE_FIXES[name]
        result = reader._evaluate_quality(fix)
        detail = f"HDOP={fix.hdop}  GDOP_est={gdop_est(fix.hdop)}  sats={fix.satellites}"
        if result == expected:
            _pass(label, detail)
        else:
            _fail(label, f"got reliable={result}, expected {expected}  |  {detail}")


def test_px4_simulation() -> None:
    _section("UNIT TEST — PX4 EKF2 GPS acceptance simulation")
    cases = [
        ("home",       True,  "Clean fix: EKF2 ACCEPT"),
        ("marginal",   True,  "HDOP=2.4: EKF2 ACCEPT (threshold=2.5)"),
        ("px4_reject", False, "HDOP=2.6: EKF2 REJECT"),
        ("bad_hdop",   False, "HDOP=4.8: EKF2 REJECT"),
        ("bad_sats",   False, "4 sats: EKF2 REJECT (minimum 6)"),
        ("no_fix",     False, "fix_type=0: EKF2 REJECT"),
    ]
    for name, expected, label in cases:
        fix    = FIXTURE_FIXES[name]
        result = px4_would_accept(fix)
        detail = f"HDOP={fix.hdop}  GDOP_est={gdop_est(fix.hdop)}  sats={fix.satellites}"
        if result == expected:
            _pass(label, detail)
        else:
            _fail(label, f"PX4 decision={result}, expected {expected}  |  {detail}")


def test_drift_check() -> None:
    _section("UNIT TEST — GPS↔SLAM drift detection")
    reader = GpsReader()
    home   = FIXTURE_FIXES["home"]
    home.reliable = True
    reader.set_home(home)
    cases = [
        ("drifted_8m", 0.0, 0.0, 5.0,  True,  "8m drift  threshold=5m  → alarm"),
        ("drifted_8m", 0.0, 0.0, 10.0, False, "8m drift  threshold=10m → silent"),
        ("home",       0.0, 0.0, 5.0,  False, "Stationary → no drift"),
        ("bad_hdop",   0.0, 0.0, 5.0,  False, "Unreliable fix → alarm suppressed"),
    ]
    for fix_name, sx, sy, thresh, expect, label in cases:
        reader._latest_fix = FIXTURE_FIXES[fix_name]
        result = reader.check_drift(slam_enu_x=sx, slam_enu_y=sy, threshold_m=thresh)
        if result == expect:
            _pass(label)
        else:
            _fail(label, f"got {result}, expected {expect}")


def test_home_datum() -> None:
    _section("UNIT TEST — Home datum and ENU delta")
    reader = GpsReader()

    ok = reader.set_home()
    (_pass if not ok else _fail)("set_home() no fix → False")

    home = FIXTURE_FIXES["home"]
    home.reliable = True
    ok = reader.set_home(home)
    (_pass if ok else _fail)("set_home() valid fix → True")

    reader._latest_fix = home
    delta = reader.get_enu_delta()
    if delta is not None:
        e, n, u = delta
        if abs(e) < 0.01 and abs(n) < 0.01:
            _pass(f"ENU at home ≈ (0,0,0)", f"({e:.3f},{n:.3f},{u:.3f})m")
        else:
            _fail("ENU at home should be ~(0,0,0)", f"got ({e:.3f},{n:.3f},{u:.3f})m")
    else:
        _fail("get_enu_delta() at home returned None")

    reader._latest_fix = FIXTURE_FIXES["10m_east"]
    reader._latest_fix.reliable = True
    delta = reader.get_enu_delta()
    if delta is not None:
        e, n, u = delta
        if 9.0 < e < 12.0 and abs(n) < 0.5:
            _pass("ENU 10m east correct", f"E={e:.2f}m N={n:.2f}m")
        else:
            _fail("ENU 10m east wrong", f"({e:.2f},{n:.2f},{u:.2f})m")
    else:
        _fail("get_enu_delta() 10m_east returned None")


# ── Live GPS test ─────────────────────────────────────────────────────────────

def test_live_gps(duration_s: int, hdop_threshold: float) -> None:
    _section(f"LIVE GPS TEST  duration={duration_s}s  HDOP_max={hdop_threshold}")

    reader = GpsReader(hdop_threshold=hdop_threshold)
    if not reader.start():
        _skip("Live GPS backend",
              "No GPS hardware (gpsd not running or pyserial missing). "
              "Run: sudo systemctl start gpsd")
        return

    _log(f"Backend: {reader.backend}")

    csv_file   = open(_CSV_PATH, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(_CSV_HEADERS)

    home_set    = False
    t_start     = time.monotonic()
    fix_records = []

    # Simulated SLAM at origin — represents a stationary drone on the ground.
    # drift_m shows what check_drift() would compute in a real mission.
    SIM_SLAM_X = 0.0
    SIM_SLAM_Y = 0.0

    print(f"\n  Live window: {duration_s}s  "
          f"{'(Ctrl-C to stop)' if duration_s == 0 else ''}")
    print(f"  Backend : {reader.backend}")
    print(f"\n  {'Elapsed':>7}  {'Sats':>4}  {'HDOP':>6}  {'GDOP_est':>9}  "
          f"{'Reliable':>8}  {'PX4':>6}  {'Drift_m':>8}  Fix")
    print(f"  {'-'*78}")

    try:
        while True:
            elapsed = time.monotonic() - t_start
            if duration_s > 0 and elapsed > duration_s:
                break

            time.sleep(1.0)
            elapsed = time.monotonic() - t_start
            fix     = reader.get_fix()

            if fix is None:
                print(f"  {elapsed:7.1f}s  — no fix  "
                      f"rejected={reader.get_stats()['reject_count']}")
                _log(f"elapsed={elapsed:.1f}s  no_fix  "
                     f"rejected={reader.get_stats()['reject_count']}", "LIVE")
                continue

            if not home_set and fix.reliable:
                reader.set_home(fix)
                home_set = True
                _log(f"Home set: lat={fix.lat:.7f} lon={fix.lon:.7f} "
                     f"alt={fix.alt:.1f}m HDOP={fix.hdop:.2f} sats={fix.satellites}", "GPS")

            enu_e = enu_n = enu_u = None
            drift_m = 0.0
            if home_set:
                delta = reader.get_enu_delta(fix)
                if delta:
                    enu_e, enu_n, enu_u = delta
                    drift_m = math.sqrt((enu_e - SIM_SLAM_X)**2 + (enu_n - SIM_SLAM_Y)**2)

            gd         = gdop_est(fix.hdop)
            px4_ok     = px4_would_accept(fix)
            ts_iso     = datetime.now().isoformat()
            fix_str    = {0:"NO_FIX",1:"GPS",2:"DGPS",3:"3D"}.get(fix.fix_type, str(fix.fix_type))
            rel_str    = "YES" if fix.reliable else "NO "
            px4_str    = "ACCEPT" if px4_ok else "REJECT"
            drift_str  = f"{drift_m:.1f}m" if home_set else "no-home"

            print(f"  {elapsed:7.1f}s  {fix.satellites:>4}  {fix.hdop:>6.2f}  "
                  f"{gd:>9.2f}  {rel_str:>8}  {px4_str:>6}  "
                  f"{drift_str:>8}  {fix_str}")

            _log(
                f"elapsed={elapsed:.1f}s  fix={fix.fix_type}  "
                f"sats={fix.satellites}  hdop={fix.hdop:.2f}  gdop_est={gd:.2f}  "
                f"lat={fix.lat:.7f}  lon={fix.lon:.7f}  alt={fix.alt:.1f}m  "
                f"enu_e={enu_e:.2f if enu_e else 'N/A'}  "
                f"enu_n={enu_n:.2f if enu_n else 'N/A'}  "
                f"reliable={fix.reliable}  px4={px4_str}  "
                f"drift={drift_m:.1f}m  source={fix.source}",
                "LIVE"
            )

            csv_writer.writerow([
                round(elapsed, 1), ts_iso,
                fix.fix_type, fix.satellites,
                fix.hdop, gd,
                round(fix.lat, 7), round(fix.lon, 7), round(fix.alt, 1),
                round(enu_e, 2) if enu_e is not None else "",
                round(enu_n, 2) if enu_n is not None else "",
                round(enu_u, 2) if enu_u is not None else "",
                fix.reliable, px4_ok,
                round(drift_m, 2), fix.source,
            ])
            csv_file.flush()
            fix_records.append({
                "hdop": fix.hdop, "gdop": gd,
                "sats": fix.satellites, "reliable": fix.reliable,
                "px4":  px4_ok, "drift_m": drift_m,
            })

    except KeyboardInterrupt:
        print("\n  (interrupted by user)")
        _log("Live test interrupted by Ctrl-C", "INFO")
    finally:
        csv_file.close()
        reader.stop()

    # ── Summary statistics ────────────────────────────────────────────────────
    _section("LIVE GPS SUMMARY")
    stats = reader.get_stats()
    total = len(fix_records)
    _log(f"Total reads      : {total}")
    _log(f"Gate accepted    : {stats['fix_count']}")
    _log(f"Gate rejected    : {stats['reject_count']}")
    _log(f"Backend          : {stats['backend']}")
    _log(f"Home datum set   : {home_set}")

    if fix_records:
        hdops  = [r["hdop"]  for r in fix_records]
        gdops  = [r["gdop"]  for r in fix_records]
        sats   = [r["sats"]  for r in fix_records]
        drifts = [r["drift_m"] for r in fix_records]
        px4_n  = sum(1 for r in fix_records if r["px4"])
        rel_n  = sum(1 for r in fix_records if r["reliable"])

        _log(f"HDOP   : min={min(hdops):.2f}  max={max(hdops):.2f}  mean={sum(hdops)/total:.2f}")
        _log(f"GDOP_est: min={min(gdops):.2f}  max={max(gdops):.2f}  mean={sum(gdops)/total:.2f}")
        _log(f"Sats   : min={min(sats)}  max={max(sats)}  mean={sum(sats)/total:.1f}")
        _log(f"PX4 ACCEPT : {px4_n}/{total} ({100*px4_n/total:.0f}%)")
        _log(f"Reliable   : {rel_n}/{total} ({100*rel_n/total:.0f}%)")
        _log(f"Drift      : min={min(drifts):.1f}m  max={max(drifts):.1f}m  "
             f"mean={sum(drifts)/total:.1f}m  (simulated SLAM at origin)")

        _log("\nHDOP distribution:")
        brackets = [
            ("<= 1.0",  lambda h: h <= 1.0),
            ("1.0-1.5", lambda h: 1.0 < h <= 1.5),
            ("1.5-2.0", lambda h: 1.5 < h <= 2.0),
            ("2.0-2.5", lambda h: 2.0 < h <= 2.5),
            ("2.5-3.0", lambda h: 2.5 < h <= 3.0),
            ("> 3.0",   lambda h: h  > 3.0),
        ]
        for lbl, cond in brackets:
            cnt = sum(1 for h in hdops if cond(h))
            bar = "█" * cnt
            _log(f"  HDOP {lbl:>8} : {bar} ({cnt})")

        accept_pct = 100 * px4_n / total
        if accept_pct >= 80:
            _pass(f"Live GPS PX4 acceptance rate {accept_pct:.0f}% (>= 80%)",
                  f"{px4_n}/{total}  backend={stats['backend']}")
        elif accept_pct >= 50:
            _pass(f"Live GPS marginal acceptance {accept_pct:.0f}%  (check antenna/sky view)",
                  f"{px4_n}/{total}")
        else:
            _fail(f"Live GPS PX4 acceptance too low ({accept_pct:.0f}%)",
                  "Check antenna cable, sky obstruction. See HDOP histogram in log.")

        _log(f"CSV: {_CSV_PATH}")
    else:
        _fail("Live GPS: no fixes received",
              "Ensure GPS antenna has sky view and gpsd is running")


# ── Log header ────────────────────────────────────────────────────────────────

def _write_header(args: argparse.Namespace) -> None:
    _buf.write("=" * 60 + "\n")
    _buf.write("  DronePi GPS Reader Test Suite\n")
    _buf.write(f"  Session  : {_session_ts}\n")
    _buf.write(f"  Host     : {platform.node()}  {platform.machine()}\n")
    _buf.write(f"  Python   : {platform.python_version()}\n")
    _buf.write(f"  Log      : {_LOG_PATH}\n")
    _buf.write(f"  CSV      : {_CSV_PATH}\n")
    _buf.write("─" * 60 + "\n")
    _buf.write(f"  HDOP threshold   : {args.hdop}\n")
    _buf.write(f"  SAT min          : {SAT_COUNT_MIN}\n")
    _buf.write(f"  Drift threshold  : {DRIFT_THRESHOLD_M} m\n")
    _buf.write(f"  PX4 HDOP max     : {PX4_HDOP_MAX}\n")
    _buf.write(f"  PX4 SAT min      : {PX4_SAT_MIN}\n")
    _buf.write(f"  GDOP estimator   : HDOP x {GDOP_EST_MULTIPLIER}\n")
    _buf.write(f"  Mode             : {'synthetic' if args.synthetic else 'synthetic+live'}\n")
    _buf.write(f"  Live duration    : {args.duration}s\n")
    _buf.write("=" * 60 + "\n\n")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> int:
    global _verbose

    parser = argparse.ArgumentParser(description="GpsReader test suite with log output")
    parser.add_argument("--synthetic", action="store_true",
                        help="Offline fixture tests only — skip live GPS")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Echo all log events to stdout")
    parser.add_argument("--hdop", type=float, default=2.5,
                        help="HDOP threshold (default 2.5 — PX4 EKF2 default)")
    parser.add_argument("--duration", type=int, default=60,
                        help="Live window seconds (default 60, 0=indefinite)")
    args    = parser.parse_args()
    _verbose = args.verbose

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG,
                            format="%(asctime)s %(name)s %(levelname)s %(message)s",
                            datefmt="%H:%M:%S")

    print("=" * 60)
    print("  DronePi GPS Reader Test Suite")
    print(f"  HDOP threshold : {args.hdop}  (PX4 EKF2 default)")
    print(f"  Mode           : {'synthetic only' if args.synthetic else 'synthetic + live'}")
    print(f"  Log file       : {_LOG_PATH.name}")
    print("=" * 60)

    _write_header(args)

    test_nmea_parser()
    test_wgs84_enu()
    test_quality_gate(hdop_threshold=args.hdop)
    test_px4_simulation()
    test_drift_check()
    test_home_datum()

    if not args.synthetic:
        test_live_gps(duration_s=args.duration, hdop_threshold=args.hdop)
    else:
        _skip("Live GPS test", "--synthetic flag set")

    _section("FINAL RESULTS")
    total   = _passed + _failed + _skipped
    verdict = "ALL TESTS PASSED" if _failed == 0 and _passed > 0 else f"{_failed} TEST(S) FAILED"
    _log(f"PASSED  : {_passed}")
    _log(f"FAILED  : {_failed}")
    _log(f"SKIPPED : {_skipped}")
    _log(f"TOTAL   : {total}")
    _log(verdict, "SUMMARY")
    _flush_log()

    print(f"\n{'=' * 60}")
    print(f"  {_passed}/{total} passed   {_failed} failed   {_skipped} skipped")
    print(f"  {verdict}")
    if _CSV_PATH.exists():
        print(f"  CSV  → {_CSV_PATH}")
    print("=" * 60)

    return 0 if _failed == 0 and _passed > 0 else (1 if _failed > 0 else 2)


if __name__ == "__main__":
    sys.exit(main())
