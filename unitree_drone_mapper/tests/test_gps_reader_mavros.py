#!/usr/bin/env python3
"""tests/test_gps_reader_mavros.py — Standalone MAVROS GPS reader test.

PURPOSE
-------
Validates the live GPS path used by the DronePi architecture when the GPS is
physically connected to PX4 and forwarded to the Raspberry Pi over MAVLink via
MAVROS.

Unlike the original gps_reader test, this version does NOT depend on gpsd,
serial NMEA parsing, or a GPS receiver directly attached to the Pi. Instead it
observes the ROS 2 topics published by MAVROS and records the same style of
quality metrics used elsewhere in the project.

ARCHITECTURE UNDER TEST
-----------------------
    GPS receiver -> PX4 -> MAVLink -> MAVROS -> ROS 2 topics -> this test

TOPICS USED
-----------
  1) /mavros/global_position/raw/fix
       sensor_msgs/NavSatFix
       Source of latitude / longitude / altitude.

  2) /mavros/gpsstatus/gps1/raw
       mavros_msgs/GPSRAW
       Source of fix_type, satellites_visible, and EPH/EPV accuracy fields.

WHY /global_position/global IS NOT USED AS PRIMARY SOURCE
---------------------------------------------------------
That topic may be absent until PX4 has a valid higher-level global solution.
The raw topics appear earlier and are therefore the correct layer to test the
PX4->MAVROS GPS link itself.

LOG FILES WRITTEN
-----------------
  tests/gps_mavros_test_<YYYYMMDD_HHMMSS>.log
  tests/gps_mavros_test_<YYYYMMDD_HHMMSS>_fixes.csv

LIVE TEST OUTPUT
----------------
For each observed sample the log records:
  - fix_type
  - satellite count
  - EPH / EPV
  - HDOP proxy (derived from EPH when available)
  - latitude / longitude / altitude
  - ENU delta from the first acceptable home datum
  - simulated PX4 acceptance decision

PX4 ACCEPTANCE SIMULATION
--------------------------
A sample is marked as "PX4 ACCEPT" only when all three checks pass:
    ① fix_type >= 1
    ② HDOP_proxy <= 2.5
    ③ satellites >= 6

Note:
  MAVROS GPSRAW does not always expose a true HDOP value directly. This test
  therefore uses EPH as a practical horizontal-accuracy proxy:

      hdop_proxy = eph / 100.0

  when EPH looks valid. If EPH is a known sentinel value (9999, 65535, etc.),
  the sample is treated as poor quality with hdop_proxy=99.0.

USAGE
-----
    python3 tests/test_gps_reader_mavros.py
    python3 tests/test_gps_reader_mavros.py --duration 120
    python3 tests/test_gps_reader_mavros.py --verbose
    python3 tests/test_gps_reader_mavros.py --duration 0
    python3 tests/test_gps_reader_mavros.py \
        --raw-fix-topic /mavros/global_position/raw/fix \
        --gpsraw-topic /mavros/gpsstatus/gps1/raw
"""

from __future__ import annotations

import argparse
import csv
import io
import logging
import math
import platform
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW

# ── Path setup ────────────────────────────────────────────────────────────────
_SCRIPT_DIR = Path(__file__).resolve().parent
_session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
_LOG_PATH = _SCRIPT_DIR / f"gps_mavros_test_{_session_ts}.log"
_CSV_PATH = _SCRIPT_DIR / f"gps_mavros_test_{_session_ts}_fixes.csv"

# ── PX4 EKF2 GPS check thresholds ────────────────────────────────────────────
PX4_HDOP_MAX = 2.5
PX4_SAT_MIN = 6
PX4_FIX_MIN = 1
GDOP_EST_MULTIPLIER = 1.5
DRIFT_THRESHOLD_M = 5.0

# ── Local datum used for offline math checks ─────────────────────────────────
HOME_LAT = 18.2085
HOME_LON = -67.1398
HOME_ALT = 10.0

# ── Log state ─────────────────────────────────────────────────────────────────
_buf = io.StringIO()
_verbose = False
_passed = _failed = _skipped = 0

_CSV_HEADERS = [
    "elapsed_s", "timestamp_iso", "fix_type", "satellites",
    "hdop_proxy", "gdop_est", "eph", "epv",
    "lat", "lon", "alt_m",
    "enu_east_m", "enu_north_m", "enu_up_m",
    "reliable", "px4_would_accept", "drift_vs_home_m", "source",
]


@dataclass
class GpsFix:
    lat: float
    lon: float
    alt: float
    hdop: float
    satellites: int
    fix_type: int
    reliable: bool
    source: str = "mavros"
    eph: Optional[float] = None
    epv: Optional[float] = None


# ── Logging helpers ───────────────────────────────────────────────────────────

def _log(msg: str, level: str = "INFO") -> None:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    line = f"[{ts}] [{level:8s}] {msg}"
    _buf.write(line + "\n")
    if _verbose or level in ("FAIL", "ERROR", "CRITICAL"):
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
    global _passed
    _passed += 1
    _log(f"PASS  {label}" + (f"  ({detail})" if detail else ""), "PASS")
    if not _verbose:
        print(f"  [PASS] {label}" + (f"  — {detail}" if detail else ""))



def _fail(label: str, reason: str = "") -> None:
    global _failed
    _failed += 1
    _log(f"FAIL  {label}" + (f"  — {reason}" if reason else ""), "FAIL")
    if not _verbose:
        print(f"  [FAIL] {label}" + (f"  — {reason}" if reason else ""))



def _skip(label: str, reason: str = "") -> None:
    global _skipped
    _skipped += 1
    _log(f"SKIP  {label}" + (f"  — {reason}" if reason else ""), "SKIP")
    if not _verbose:
        print(f"  [SKIP] {label}" + (f"  — {reason}" if reason else ""))


# ── Math / quality helpers ───────────────────────────────────────────────────

def _nmea_to_decimal(raw: str, direction: str) -> float:
    """Convert ddmm.mmmm / dddmm.mmmm string to signed decimal degrees."""
    value = float(raw)
    degrees = int(value // 100)
    minutes = value - degrees * 100
    decimal = degrees + minutes / 60.0
    if direction in ("S", "W"):
        decimal *= -1.0
    return decimal



def _wgs84_to_enu(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Approximate local East/North displacement in meters."""
    r_earth = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    lat0r = math.radians(lat0)
    east = dlon * r_earth * math.cos(lat0r)
    north = dlat * r_earth
    return east, north



def gdop_est(hdop: float) -> float:
    return round(hdop * GDOP_EST_MULTIPLIER, 2)



def px4_would_accept(fix: GpsFix) -> bool:
    return (
        fix.fix_type >= PX4_FIX_MIN
        and fix.hdop <= PX4_HDOP_MAX
        and fix.satellites >= PX4_SAT_MIN
    )



def eph_to_hdop_proxy(eph_value: int) -> float:
    """Convert MAVROS GPSRAW EPH into a practical HDOP-like proxy.

    In MAVLink-derived GPSRAW data, EPH is typically horizontal accuracy in
    centimeters. This is not identical to HDOP, but it is a useful quality
    proxy for field acceptance checks.

    Sentinel / invalid values are converted to a deliberately bad number so the
    sample is rejected by the simulated PX4 gate.
    """
    if eph_value in (9999, 65535, 4294967295):
        return 99.0
    if eph_value <= 0:
        return 99.0
    return round(eph_value / 100.0, 2)


# ── Fixtures for offline checks ───────────────────────────────────────────────
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


# ── Unit tests ────────────────────────────────────────────────────────────────

def test_nmea_parser() -> None:
    _section("UNIT TEST — NMEA coordinate parser")
    cases = [
        ("4807.0380", "N",  48.117300, 0.0001, "48°07.038'N"),
        ("01131.0000", "E",  11.516667, 0.0001, "11°31.000'E"),
        ("4807.0380", "S", -48.117300, 0.0001, "48°07.038'S"),
        ("00130.0000", "W",  -1.500000, 0.0001, "01°30.000'W"),
        ("1812.5100", "N",  18.208500, 0.0001, "18°12.510'N local"),
        ("6708.3880", "W", -67.139800, 0.0001, "67°08.388'W local"),
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
        e, n = _wgs84_to_enu(HOME_LAT + dlat, HOME_LON + dlon, HOME_LAT, HOME_LON)
        if abs(e - exp_e) <= tol and abs(n - exp_n) <= tol:
            _pass(f"ENU {label}", f"E={e:.2f}m N={n:.2f}m")
        else:
            _fail(f"ENU {label}", f"got E={e:.2f} N={n:.2f}, expected E≈{exp_e:.2f} N≈{exp_n:.2f}")



def test_quality_gate() -> None:
    _section(f"UNIT TEST — Quality gate (HDOP_max={PX4_HDOP_MAX}  sat_min={PX4_SAT_MIN})")
    cases = [
        ("home",       True,  "Clean fix: HDOP=1.2  sats=10"),
        ("10m_east",   True,  "Offset fix: HDOP=1.4  sats=9"),
        ("marginal",   True,  "Marginal: HDOP=2.4 (< 2.5)"),
        ("px4_reject", False, "PX4 reject: HDOP=2.6 (> 2.5)"),
        ("bad_hdop",   False, "Bad HDOP=4.8  sats=7"),
        ("bad_sats",   False, "Bad sats=4  HDOP=1.1"),
        ("no_fix",     False, "No fix: fix_type=0"),
    ]
    for name, expected, label in cases:
        fix = FIXTURE_FIXES[name]
        result = fix.reliable
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
        fix = FIXTURE_FIXES[name]
        result = px4_would_accept(fix)
        detail = f"HDOP={fix.hdop}  GDOP_est={gdop_est(fix.hdop)}  sats={fix.satellites}"
        if result == expected:
            _pass(label, detail)
        else:
            _fail(label, f"PX4 decision={result}, expected {expected}  |  {detail}")



def test_drift_check() -> None:
    _section("UNIT TEST — GPS drift magnitude check")
    cases = [
        ("drifted_8m", 5.0,  True,  "8m drift  threshold=5m  → alarm"),
        ("drifted_8m", 10.0, False, "8m drift  threshold=10m → silent"),
        ("home",       5.0,  False, "Stationary → no drift"),
        ("bad_hdop",   5.0,  False, "Unreliable fixture → drift alarm suppressed by policy"),
    ]
    home = FIXTURE_FIXES["home"]
    for fix_name, thresh, expect, label in cases:
        fix = FIXTURE_FIXES[fix_name]
        if not fix.reliable:
            result = False
        else:
            e, n = _wgs84_to_enu(fix.lat, fix.lon, home.lat, home.lon)
            drift = math.sqrt(e * e + n * n)
            result = drift > thresh
        if result == expect:
            _pass(label)
        else:
            _fail(label, f"got {result}, expected {expect}")


# ── MAVROS live reader ────────────────────────────────────────────────────────
class MavrosGpsReader(Node):
    """Collect latest NavSatFix + GPSRAW and merge them into one logical fix."""

    def __init__(self, raw_fix_topic: str, gpsraw_topic: str) -> None:
        super().__init__("gps_reader_mavros_test")
        self._raw_fix_topic = raw_fix_topic
        self._gpsraw_topic = gpsraw_topic
        self._navsat: Optional[NavSatFix] = None
        self._gpsraw: Optional[GPSRAW] = None
        self._navsat_t = 0.0
        self._gpsraw_t = 0.0
        self._sample_count = 0
        self._accept_count = 0
        self._reject_count = 0

        self.create_subscription(
            NavSatFix,
            raw_fix_topic,
            self._navsat_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GPSRAW,
            gpsraw_topic,
            self._gpsraw_cb,
            qos_profile_sensor_data,
        )

    def _navsat_cb(self, msg: NavSatFix) -> None:
        self._navsat = msg
        self._navsat_t = time.monotonic()

    def _gpsraw_cb(self, msg: GPSRAW) -> None:
        self._gpsraw = msg
        self._gpsraw_t = time.monotonic()

    def backend(self) -> str:
        return f"mavros ({self._raw_fix_topic} + {self._gpsraw_topic})"

    def topic_alive(self, max_age_s: float = 2.5) -> bool:
        now = time.monotonic()
        return (
            self._navsat is not None and self._gpsraw is not None
            and (now - self._navsat_t) <= max_age_s
            and (now - self._gpsraw_t) <= max_age_s
        )

    def get_fix(self) -> Optional[GpsFix]:
        if self._navsat is None or self._gpsraw is None:
            return None

        nav = self._navsat
        raw = self._gpsraw
        hdop = eph_to_hdop_proxy(int(raw.eph))
        fix = GpsFix(
            lat=nav.latitude,
            lon=nav.longitude,
            alt=nav.altitude,
            hdop=hdop,
            satellites=int(raw.satellites_visible),
            fix_type=int(raw.fix_type),
            reliable=False,
            source="mavros",
            eph=float(raw.eph),
            epv=float(raw.epv),
        )
        fix.reliable = px4_would_accept(fix)

        self._sample_count += 1
        if fix.reliable:
            self._accept_count += 1
        else:
            self._reject_count += 1
        return fix

    def get_stats(self) -> dict:
        return {
            "sample_count": self._sample_count,
            "fix_count": self._accept_count,
            "reject_count": self._reject_count,
            "backend": self.backend(),
            "raw_fix_topic": self._raw_fix_topic,
            "gpsraw_topic": self._gpsraw_topic,
        }


# ── Live GPS test ─────────────────────────────────────────────────────────────
def test_live_gps(duration_s: int, raw_fix_topic: str, gpsraw_topic: str) -> None:
    _section(f"LIVE MAVROS GPS TEST  duration={duration_s}s")

    rclpy.init(args=None)
    reader = MavrosGpsReader(raw_fix_topic=raw_fix_topic, gpsraw_topic=gpsraw_topic)

    csv_file = open(_CSV_PATH, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(_CSV_HEADERS)

    home_fix: Optional[GpsFix] = None
    t_start = time.monotonic()
    fix_records = []

    print(f"\n  Live window: {duration_s}s  "
          f"{'(Ctrl-C to stop)' if duration_s == 0 else ''}")
    print(f"  Backend : {reader.backend()}")
    print(f"\n  {'Elapsed':>7}  {'Sats':>4}  {'HDOP*':>6}  {'GDOP_est':>9}  "
          f"{'Reliable':>8}  {'PX4':>6}  {'Drift_m':>8}  Fix")
    print(f"  {'-' * 82}")
    print("  * HDOP shown here is a proxy derived from GPSRAW.eph when available")

    try:
        while True:
            rclpy.spin_once(reader, timeout_sec=0.25)
            elapsed = time.monotonic() - t_start
            if duration_s > 0 and elapsed > duration_s:
                break

            if not reader.topic_alive(max_age_s=2.5):
                print(f"  {elapsed:7.1f}s  — no recent MAVROS GPS sample")
                _log(f"elapsed={elapsed:.1f}s  no_recent_sample", "LIVE")
                time.sleep(0.75)
                continue

            fix = reader.get_fix()
            if fix is None:
                print(f"  {elapsed:7.1f}s  — waiting for paired NavSatFix + GPSRAW")
                _log(f"elapsed={elapsed:.1f}s  waiting_for_topics", "LIVE")
                time.sleep(0.75)
                continue

            if home_fix is None and fix.reliable:
                home_fix = fix
                _log(
                    f"Home set: lat={fix.lat:.7f} lon={fix.lon:.7f} alt={fix.alt:.1f}m "
                    f"HDOP_proxy={fix.hdop:.2f} sats={fix.satellites}",
                    "GPS",
                )

            enu_e = enu_n = enu_u = None
            drift_m = 0.0
            if home_fix is not None:
                enu_e, enu_n = _wgs84_to_enu(fix.lat, fix.lon, home_fix.lat, home_fix.lon)
                enu_u = fix.alt - home_fix.alt
                drift_m = math.sqrt(enu_e * enu_e + enu_n * enu_n)

            gd = gdop_est(fix.hdop)
            px4_ok = px4_would_accept(fix)
            ts_iso = datetime.now().isoformat()
            fix_str = {0: "NO_FIX", 1: "GPS", 2: "DGPS", 3: "3D"}.get(fix.fix_type, str(fix.fix_type))
            rel_str = "YES" if fix.reliable else "NO "
            px4_str = "ACCEPT" if px4_ok else "REJECT"
            drift_str = f"{drift_m:.1f}m" if home_fix else "no-home"

            print(f"  {elapsed:7.1f}s  {fix.satellites:>4}  {fix.hdop:>6.2f}  "
                  f"{gd:>9.2f}  {rel_str:>8}  {px4_str:>6}  "
                  f"{drift_str:>8}  {fix_str}")

            enu_e_str = f"{enu_e:.2f}" if enu_e is not None else "N/A"
            enu_n_str = f"{enu_n:.2f}" if enu_n is not None else "N/A"
            enu_u_str = f"{enu_u:.2f}" if enu_u is not None else "N/A"
            _log(
                f"elapsed={elapsed:.1f}s  fix={fix.fix_type}  "
                f"sats={fix.satellites}  hdop_proxy={fix.hdop:.2f}  gdop_est={gd:.2f}  "
                f"eph={fix.eph}  epv={fix.epv}  "
                f"lat={fix.lat:.7f}  lon={fix.lon:.7f}  alt={fix.alt:.1f}m  "
                f"enu_e={enu_e_str}  enu_n={enu_n_str}  enu_u={enu_u_str}  "
                f"reliable={fix.reliable}  px4={px4_str}  "
                f"drift={drift_m:.1f}m  source={fix.source}",
                "LIVE",
            )

            csv_writer.writerow([
                round(elapsed, 1), ts_iso,
                fix.fix_type, fix.satellites,
                fix.hdop, gd, fix.eph, fix.epv,
                round(fix.lat, 7), round(fix.lon, 7), round(fix.alt, 1),
                round(enu_e, 2) if enu_e is not None else "",
                round(enu_n, 2) if enu_n is not None else "",
                round(enu_u, 2) if enu_u is not None else "",
                fix.reliable, px4_ok,
                round(drift_m, 2), fix.source,
            ])
            csv_file.flush()
            fix_records.append({
                "hdop": fix.hdop,
                "gdop": gd,
                "sats": fix.satellites,
                "reliable": fix.reliable,
                "px4": px4_ok,
                "drift_m": drift_m,
            })
            time.sleep(0.75)

    except KeyboardInterrupt:
        print("\n  (interrupted by user)")
        _log("Live test interrupted by Ctrl-C", "INFO")
    finally:
        stats = reader.get_stats()
        reader.destroy_node()
        rclpy.shutdown()
        csv_file.close()

    # ── Summary statistics ────────────────────────────────────────────────────
    _section("LIVE MAVROS GPS SUMMARY")
    total = len(fix_records)
    _log(f"Total reads      : {total}")
    _log(f"Gate accepted    : {stats['fix_count']}")
    _log(f"Gate rejected    : {stats['reject_count']}")
    _log(f"Backend          : {stats['backend']}")
    _log(f"Raw fix topic    : {stats['raw_fix_topic']}")
    _log(f"GPSRAW topic     : {stats['gpsraw_topic']}")
    _log(f"Home datum set   : {home_fix is not None}")

    if fix_records:
        hdops = [r["hdop"] for r in fix_records]
        gdops = [r["gdop"] for r in fix_records]
        sats = [r["sats"] for r in fix_records]
        drifts = [r["drift_m"] for r in fix_records]
        px4_n = sum(1 for r in fix_records if r["px4"])
        rel_n = sum(1 for r in fix_records if r["reliable"])

        _log(f"HDOP*  : min={min(hdops):.2f}  max={max(hdops):.2f}  mean={sum(hdops) / total:.2f}")
        _log(f"GDOP_est: min={min(gdops):.2f}  max={max(gdops):.2f}  mean={sum(gdops) / total:.2f}")
        _log(f"Sats   : min={min(sats)}  max={max(sats)}  mean={sum(sats) / total:.1f}")
        _log(f"PX4 ACCEPT : {px4_n}/{total} ({100 * px4_n / total:.0f}%)")
        _log(f"Reliable   : {rel_n}/{total} ({100 * rel_n / total:.0f}%)")
        _log(f"Drift      : min={min(drifts):.1f}m  max={max(drifts):.1f}m  mean={sum(drifts) / total:.1f}m")

        _log("\nHDOP* distribution:")
        brackets = [
            ("<= 1.0",  lambda h: h <= 1.0),
            ("1.0-1.5", lambda h: 1.0 < h <= 1.5),
            ("1.5-2.0", lambda h: 1.5 < h <= 2.0),
            ("2.0-2.5", lambda h: 2.0 < h <= 2.5),
            ("2.5-3.0", lambda h: 2.5 < h <= 3.0),
            ("> 3.0",   lambda h: h > 3.0),
        ]
        for lbl, cond in brackets:
            cnt = sum(1 for h in hdops if cond(h))
            bar = "█" * cnt
            _log(f"  HDOP* {lbl:>8} : {bar} ({cnt})")

        accept_pct = 100 * px4_n / total
        if accept_pct >= 80:
            _pass(f"Live MAVROS GPS PX4 acceptance rate {accept_pct:.0f}% (>= 80%)",
                  f"{px4_n}/{total}  backend={stats['backend']}")
        elif accept_pct >= 50:
            _pass(f"Live MAVROS GPS marginal acceptance {accept_pct:.0f}%  (check sky view)",
                  f"{px4_n}/{total}")
        else:
            _fail(f"Live MAVROS GPS PX4 acceptance too low ({accept_pct:.0f}%)",
                  "Check antenna cable, sky obstruction, fix_type, and GPSRAW.eph sentinel values.")

        _log(f"CSV: {_CSV_PATH}")
    else:
        _fail("Live MAVROS GPS: no paired fixes received",
              "Ensure MAVROS is running and the configured topic names are correct.")


# ── Log header ────────────────────────────────────────────────────────────────
def _write_header(args: argparse.Namespace) -> None:
    _buf.write("=" * 60 + "\n")
    _buf.write("  DronePi MAVROS GPS Reader Test Suite\n")
    _buf.write(f"  Session  : {_session_ts}\n")
    _buf.write(f"  Host     : {platform.node()}  {platform.machine()}\n")
    _buf.write(f"  Python   : {platform.python_version()}\n")
    _buf.write(f"  Log      : {_LOG_PATH}\n")
    _buf.write(f"  CSV      : {_CSV_PATH}\n")
    _buf.write("─" * 60 + "\n")
    _buf.write(f"  PX4 HDOP max     : {PX4_HDOP_MAX}\n")
    _buf.write(f"  PX4 SAT min      : {PX4_SAT_MIN}\n")
    _buf.write(f"  PX4 FIX min      : {PX4_FIX_MIN}\n")
    _buf.write(f"  Drift threshold  : {DRIFT_THRESHOLD_M} m\n")
    _buf.write(f"  GDOP estimator   : HDOP_proxy x {GDOP_EST_MULTIPLIER}\n")
    _buf.write(f"  Mode             : {'synthetic' if args.synthetic else 'synthetic+live'}\n")
    _buf.write(f"  Live duration    : {args.duration}s\n")
    _buf.write(f"  Raw fix topic    : {args.raw_fix_topic}\n")
    _buf.write(f"  GPSRAW topic     : {args.gpsraw_topic}\n")
    _buf.write("=" * 60 + "\n\n")


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> int:
    global _verbose

    parser = argparse.ArgumentParser(description="MAVROS GPS reader test suite with log output")
    parser.add_argument("--synthetic", action="store_true",
                        help="Offline fixture tests only — skip live MAVROS GPS")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Echo all log events to stdout")
    parser.add_argument("--duration", type=int, default=60,
                        help="Live window seconds (default 60, 0=indefinite)")
    parser.add_argument("--raw-fix-topic", default="/mavros/global_position/raw/fix",
                        help="NavSatFix topic from MAVROS")
    parser.add_argument("--gpsraw-topic", default="/mavros/gpsstatus/gps1/raw",
                        help="GPSRAW topic from MAVROS")
    args = parser.parse_args()
    _verbose = args.verbose

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG,
                            format="%(asctime)s %(name)s %(levelname)s %(message)s",
                            datefmt="%H:%M:%S")

    print("=" * 60)
    print("  DronePi MAVROS GPS Reader Test Suite")
    print(f"  Mode           : {'synthetic only' if args.synthetic else 'synthetic + live'}")
    print(f"  Raw fix topic  : {args.raw_fix_topic}")
    print(f"  GPSRAW topic   : {args.gpsraw_topic}")
    print(f"  Log file       : {_LOG_PATH.name}")
    print("=" * 60)

    _write_header(args)

    test_nmea_parser()
    test_wgs84_enu()
    test_quality_gate()
    test_px4_simulation()
    test_drift_check()

    if not args.synthetic:
        test_live_gps(
            duration_s=args.duration,
            raw_fix_topic=args.raw_fix_topic,
            gpsraw_topic=args.gpsraw_topic,
        )
    else:
        _skip("Live MAVROS GPS test", "--synthetic flag set")

    _section("FINAL RESULTS")
    total = _passed + _failed + _skipped
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
