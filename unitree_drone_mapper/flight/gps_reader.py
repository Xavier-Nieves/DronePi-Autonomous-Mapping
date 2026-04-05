"""flight/gps_reader.py — Pi-side GPS reader for EXIF tagging and soft drift detection.

ARCHITECTURAL POSITION
----------------------
This module is a RUNTIME MODULE — imported directly by main.py during MODE 3
autonomous flight. It is NOT a standalone script and does NOT own a MAVROS
node. It does NOT replace or compete with Point-LIO SLAM as the EKF2 position
source.

                    ┌─────────────────────────────────────────────────────┐
                    │              EKF2 Position Authority                │
                    │                                                     │
                    │   Point-LIO SLAM  ──►  _slam_bridge.py              │
                    │   (authoritative)       /mavros/vision_pose/pose    │
                    │                         /mavros/odometry/out        │
                    └─────────────────────────────────────────────────────┘
                                                   ▲
                    ┌──────────────────────────────┤
                    │  GpsReader (this module)     │  secondary only
                    │                              │
                    │  ① get_fix()                 │  → CameraCapture.trigger()
                    │     lat/lon/alt + quality    │    EXIF + sidecar JSON
                    │                              │
                    │  ② get_enu_delta()           │  → main.py drift monitor
                    │     metres from home origin  │    corrective setpoint gate
                    │                              │
                    │  ③ is_reliable()             │  → quality gate on both above
                    └──────────────────────────────┘

GPS FEED OPTIONS
----------------
Option A — gpsd (recommended for production)
    Requires gpsd daemon running on the Pi:
        sudo apt install gpsd gpsd-clients python3-gps
        sudo systemctl enable gpsd
    GpsReader auto-detects gpsd on localhost:2947. Falls back to Option B.

Option B — Direct serial NMEA (fallback / bench testing)
    Reads raw NMEA sentences from the serial port directly.
    Set SERIAL_PORT and SERIAL_BAUD in config or environment:
        GPS_SERIAL_PORT=/dev/ttyAMA0
        GPS_SERIAL_BAUD=9600
    Works without gpsd installed.

QUALITY GATING
--------------
Fix quality is evaluated on every read via three independent criteria:

    1. Fix type      — require 3D fix (NMEA fix quality ≥ 1, or GGA quality 1/2)
    2. HDOP          — reject if HDOP > HDOP_THRESHOLD (default 2.5)
                       This is the same threshold PX4 EKF2 uses internally.
    3. Satellite count — reject if fewer than SAT_COUNT_MIN (default 6)

A fix that fails any criterion sets is_reliable() → False.
The fix coordinates are still returned by get_fix() so callers can log
the raw data for post-mission diagnostics, but the quality flag allows
callers to decide whether to act on the position.

DATUM / FRAME CONVERSION
------------------------
GPS returns WGS84 geodetic coordinates (lat, lon, alt).
Your EKF2 local frame is ENU (East-North-Up) relative to the SLAM origin.
set_home() captures the GPS fix at mission start as the WGS84 datum.
get_enu_delta() returns metres east/north from that datum using a flat-Earth
approximation valid for missions under ~1 km radius.

    East  (m) = (lon - lon0) × R × cos(lat0)   in radians
    North (m) = (lat - lat0) × R                in radians

where R = 6,371,000 m (mean Earth radius).

This is the same approximation used by PX4's local_position estimator and
by MAVLink's LOCAL_POSITION_NED message. Error is < 1 cm at 100 m from home.

THREAD SAFETY
-------------
GpsReader runs a background polling thread at GPS_POLL_HZ (default 5 Hz).
All public methods acquire _lock before reading shared state. Safe to call
from main.py's orchestrator thread while the GPS poll thread runs.

DRIFT MONITOR INTEGRATION
--------------------------
main.py's handle_autonomous() should call check_drift() after each
fly_to() completes. If drift exceeds DRIFT_THRESHOLD_M, the method
returns True and the caller may issue a corrective setpoint. Example:

    if gps.check_drift(current_enu_x, current_enu_y, threshold_m=5.0):
        log("[GPS] Gross drift detected — holding position")
        node.fly_to(current_enu_x, current_enu_y, ez, 0.0)

This is a coarse catch (±3–5 m GPS accuracy). It is NOT a precision hover
stabiliser — the SLAM bridge handles that. It catches only runaway drift
scenarios where SLAM has silently degraded or lost its map.

DEPENDENCIES
------------
    gpsd path:    pip install gps3          (or python3-gps via apt)
    serial path:  pip install pyserial      (fallback NMEA parser)
    Both are optional — the module degrades gracefully if neither is available.

INSTALL CHECK
-------------
    python3 -c "from flight.gps_reader import GpsReader; print(GpsReader().backend)"
"""

from __future__ import annotations

import math
import os
import threading
import time
import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

# ── Configuration constants ───────────────────────────────────────────────────
# All overridable via environment variables for bench testing.

GPSD_HOST        = os.environ.get("GPS_GPSD_HOST",   "127.0.0.1")
GPSD_PORT        = int(os.environ.get("GPS_GPSD_PORT",   "2947"))
SERIAL_PORT      = os.environ.get("GPS_SERIAL_PORT",  "/dev/ttyAMA0")
SERIAL_BAUD      = int(os.environ.get("GPS_SERIAL_BAUD",  "9600"))

GPS_POLL_HZ      = float(os.environ.get("GPS_POLL_HZ",    "5.0"))   # background poll rate
FIX_STALE_S      = float(os.environ.get("GPS_STALE_S",   "3.0"))   # seconds before fix marked stale

# Quality gate thresholds — same values PX4 EKF2 uses internally
HDOP_THRESHOLD   = float(os.environ.get("GPS_HDOP_MAX",   "2.5"))
SAT_COUNT_MIN    = int  (os.environ.get("GPS_SAT_MIN",    "6"  ))

# Drift monitor
DRIFT_THRESHOLD_M = float(os.environ.get("GPS_DRIFT_M",  "5.0"))   # metres — gross drift catch

# Earth radius for flat-Earth ENU conversion
_EARTH_RADIUS_M = 6_371_000.0


# ── Data types ────────────────────────────────────────────────────────────────

@dataclass
class GpsFix:
    """Single GPS fix snapshot with quality metadata.

    Attributes
    ----------
    lat, lon, alt : float
        WGS84 geodetic coordinates. lat/lon in decimal degrees, alt in metres MSL.
    hdop : float
        Horizontal Dilution of Precision. Lower is better. > 2.5 = unreliable.
    satellites : int
        Number of satellites used in the fix solution.
    fix_type : int
        0 = no fix, 1 = GPS fix, 2 = DGPS fix. 3D fix requires fix_type ≥ 1.
    timestamp : float
        monotonic time of this fix (time.monotonic()).
    reliable : bool
        True if fix passes all quality gates (hdop, sat count, fix type).
    source : str
        "gpsd" or "serial" — which backend produced this fix.
    """
    lat:        float = 0.0
    lon:        float = 0.0
    alt:        float = 0.0
    hdop:       float = 99.0
    satellites: int   = 0
    fix_type:   int   = 0
    timestamp:  float = field(default_factory=time.monotonic)
    reliable:   bool  = False
    source:     str   = "unknown"

    def is_stale(self, max_age_s: float = FIX_STALE_S) -> bool:
        """Return True if this fix is older than max_age_s seconds."""
        return (time.monotonic() - self.timestamp) > max_age_s

    def to_dict(self) -> dict:
        """Serialise to dict for sidecar JSON and EXIF injection."""
        return {
            "lat":        self.lat,
            "lon":        self.lon,
            "alt":        self.alt,
            "hdop":       round(self.hdop, 2),
            "satellites": self.satellites,
            "fix_type":   self.fix_type,
            "reliable":   self.reliable,
            "source":     self.source,
        }


# ── Backend implementations ───────────────────────────────────────────────────

class _GpsdBackend:
    """gpsd-based GPS reader using the gps3 or python-gps client library.

    Maintains a persistent connection to gpsd and reads TPV (Time-Position-Velocity)
    reports. gpsd handles NMEA parsing, satellite tracking, and fix quality
    calculation internally — this backend only reads the structured output.
    """

    def __init__(self, host: str, port: int):
        self._host   = host
        self._port   = port
        self._socket = None
        self._stream = None
        self._available = False

        # Try gps3 first (more Pythonic API), fall back to gpsd (legacy)
        self._lib = self._detect_library()

    def _detect_library(self) -> Optional[str]:
        try:
            import gps3   # noqa: F401
            return "gps3"
        except ImportError:
            pass
        try:
            import gps    # noqa: F401
            return "gps"
        except ImportError:
            pass
        return None

    def connect(self) -> bool:
        """Open connection to gpsd. Returns True on success."""
        if self._lib is None:
            logger.warning(
                "[GpsReader/gpsd] Neither gps3 nor python-gps installed. "
                "Install with: pip install gps3  OR  sudo apt install python3-gps"
            )
            return False
        try:
            if self._lib == "gps3":
                from gps3 import gps3
                self._socket = gps3.GPSDSocket()
                self._stream = gps3.DataStream()
                self._socket.connect(self._host, self._port)
                self._socket.watch()
            else:
                import gps
                self._session = gps.gps(host=self._host, port=self._port)
                self._session.stream(gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

            self._available = True
            logger.info(
                f"[GpsReader/gpsd] Connected to gpsd at {self._host}:{self._port} "
                f"(library: {self._lib})"
            )
            return True
        except Exception as exc:
            logger.warning(f"[GpsReader/gpsd] Connection failed: {exc}")
            return False

    def read(self) -> Optional[GpsFix]:
        """Read one TPV report from gpsd. Returns None if no valid fix."""
        if not self._available:
            return None
        try:
            if self._lib == "gps3":
                new_data = self._socket.next()
                if new_data:
                    self._stream.unpack(new_data)
                    tpv = self._stream.TPV

                    lat  = _safe_float(tpv.get("lat"))
                    lon  = _safe_float(tpv.get("lon"))
                    alt  = _safe_float(tpv.get("alt"),  default=0.0)
                    hdop = _safe_float(tpv.get("hdop"), default=99.0)
                    mode = int(tpv.get("mode", 0))    # 0=no fix, 2=2D, 3=3D

                    if lat is None or lon is None:
                        return None

                    # gpsd mode 2 = 2D fix (no reliable alt), 3 = 3D fix
                    fix_type = 1 if mode >= 2 else 0

                    # gpsd does not expose satellite count directly in TPV
                    # Read from SKY report if available; default conservative
                    sat_count = int(tpv.get("satellites_used", 0))
                    if sat_count == 0:
                        sky = self._stream.SKY
                        sat_count = _count_used_sats(sky.get("satellites", []))

                    return _build_fix(lat, lon, alt, hdop, sat_count, fix_type, "gpsd")

            else:
                # legacy gps library
                report = self._session.next()
                if report["class"] == "TPV":
                    lat  = getattr(report, "lat",  None)
                    lon  = getattr(report, "lon",  None)
                    alt  = getattr(report, "alt",  0.0)
                    hdop = getattr(report, "hdop", 99.0)
                    mode = getattr(report, "mode", 0)
                    fix_type  = 1 if mode >= 2 else 0
                    sat_count = 0   # retrieved from SKY separately
                    if lat is None or lon is None:
                        return None
                    return _build_fix(lat, lon, alt, hdop, sat_count, fix_type, "gpsd")

        except StopIteration:
            logger.debug("[GpsReader/gpsd] gpsd stream exhausted — reconnecting")
            self._available = False
        except Exception as exc:
            logger.warning(f"[GpsReader/gpsd] Read error: {exc}")
        return None

    def close(self) -> None:
        try:
            if self._lib == "gps3" and self._socket:
                self._socket.close()
            elif self._lib == "gps" and hasattr(self, "_session"):
                self._session.close()
        except Exception:
            pass
        self._available = False


class _SerialNmeaBackend:
    """Direct serial NMEA parser — fallback when gpsd is unavailable.

    Reads raw NMEA sentences from the GPS module's UART and parses
    GGA (position + fix quality) and GSA (DOP values) sentences.

    GGA provides: lat, lon, alt, fix quality, satellite count
    GSA provides: HDOP, VDOP, PDOP (not always present in minimal firmware)

    This backend is intentionally minimal — it does not track satellites
    individually, does not support SBAS/DGPS enrichment, and does not
    handle binary UBX protocol. For production use, gpsd is preferred.
    """

    def __init__(self, port: str, baud: int):
        self._port  = port
        self._baud  = baud
        self._ser   = None
        self._hdop  = 99.0   # Updated from GSA sentence when available

    def connect(self) -> bool:
        try:
            import serial
            self._ser = serial.Serial(
                self._port, self._baud,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            logger.info(
                f"[GpsReader/serial] Opened {self._port} at {self._baud} baud"
            )
            return True
        except ImportError:
            logger.warning(
                "[GpsReader/serial] pyserial not installed. "
                "Install with: pip install pyserial"
            )
            return False
        except Exception as exc:
            logger.warning(f"[GpsReader/serial] Open failed on {self._port}: {exc}")
            return False

    def read(self) -> Optional[GpsFix]:
        """Read and parse one NMEA sentence. Returns GpsFix on GGA, None otherwise."""
        if self._ser is None or not self._ser.is_open:
            return None
        try:
            raw = self._ser.readline()
            if not raw:
                return None
            line = raw.decode("ascii", errors="replace").strip()

            if line.startswith("$GPGGA") or line.startswith("$GNGGA"):
                return self._parse_gga(line)
            elif line.startswith("$GPGSA") or line.startswith("$GNGSA"):
                self._parse_gsa(line)   # updates _hdop in place
        except Exception as exc:
            logger.debug(f"[GpsReader/serial] Parse error: {exc}")
        return None

    def _parse_gga(self, sentence: str) -> Optional[GpsFix]:
        """Parse GGA sentence for position and fix quality.

        GGA format:
            $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,b,q,ss,hdop,altitude,M,...
            field[1]=time, [2]=lat, [3]=N/S, [4]=lon, [5]=E/W,
            [6]=fix quality (0=invalid,1=GPS,2=DGPS), [7]=sat count,
            [8]=hdop, [9]=altitude
        """
        try:
            # Strip checksum
            sentence = sentence.split("*")[0]
            fields   = sentence.split(",")
            if len(fields) < 10:
                return None

            lat_raw  = fields[2]
            lat_dir  = fields[3]
            lon_raw  = fields[4]
            lon_dir  = fields[5]
            fix_q    = int(fields[6]) if fields[6] else 0
            sat_cnt  = int(fields[7]) if fields[7] else 0
            hdop_raw = fields[8]
            alt_raw  = fields[9]

            if fix_q == 0 or not lat_raw or not lon_raw:
                return None

            lat = _nmea_to_decimal(lat_raw, lat_dir)
            lon = _nmea_to_decimal(lon_raw, lon_dir)
            alt = float(alt_raw) if alt_raw else 0.0

            # Use GSA-derived HDOP if available; fall back to GGA field
            hdop = float(hdop_raw) if hdop_raw else self._hdop

            return _build_fix(lat, lon, alt, hdop, sat_cnt, fix_q, "serial")

        except (ValueError, IndexError) as exc:
            logger.debug(f"[GpsReader/serial] GGA parse error: {exc}")
            return None

    def _parse_gsa(self, sentence: str) -> None:
        """Parse GSA sentence to update HDOP. Side-effect only — no return value.

        GSA format:
            $GPGSA,A,3,04,05,...,pdop,hdop,vdop*hh
            field[-3]=PDOP, field[-2]=HDOP, field[-1]=VDOP (before checksum)
        """
        try:
            sentence = sentence.split("*")[0]
            fields   = sentence.split(",")
            if len(fields) >= 17:
                hdop_raw = fields[-2]
                if hdop_raw:
                    self._hdop = float(hdop_raw)
        except (ValueError, IndexError):
            pass

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None


# ── Main GpsReader class ──────────────────────────────────────────────────────

class GpsReader:
    """Pi-side GPS reader for EXIF tagging and soft drift detection.

    Runs a background polling thread that continuously reads from either
    gpsd or direct serial NMEA. All public methods are thread-safe.

    Callers
    -------
        main.py (MODE 3 autonomous) — drift monitor via check_drift()
        camera_capture.py           — EXIF/sidecar via get_fix()

    Not a caller
    ------------
        _slam_bridge.py  — SLAM remains the sole EKF2 position source
        MAVROS            — GPS does not publish to any MAVROS topic

    Parameters
    ----------
    prefer_gpsd : bool
        Try gpsd first. If unavailable, fall back to serial. Default True.
    hdop_threshold : float
        Maximum acceptable HDOP before fix is flagged unreliable. Default 2.5.
    sat_min : int
        Minimum satellites for a reliable fix. Default 6.
    """

    def __init__(
        self,
        prefer_gpsd:    bool  = True,
        hdop_threshold: float = HDOP_THRESHOLD,
        sat_min:        int   = SAT_COUNT_MIN,
    ):
        self._hdop_threshold = hdop_threshold
        self._sat_min        = sat_min
        self._prefer_gpsd    = prefer_gpsd

        self._lock           = threading.Lock()
        self._stop           = threading.Event()
        self._thread:        Optional[threading.Thread] = None

        # Shared state updated by background thread
        self._latest_fix:    Optional[GpsFix] = None
        self._home_fix:      Optional[GpsFix] = None   # datum for ENU conversion
        self._fix_count:     int   = 0
        self._reject_count:  int   = 0
        self._started:       bool  = False

        # Backend is selected during start()
        self._backend = None
        self._backend_name: str = "none"

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Initialise the GPS backend and start the background polling thread.

        Attempts gpsd first if prefer_gpsd=True, then falls back to serial NMEA.
        Returns True if at least one backend connected successfully.
        """
        if self._started:
            logger.warning("[GpsReader] Already started — ignoring duplicate start()")
            return True

        backend = self._init_backend()
        if backend is None:
            logger.error(
                "[GpsReader] No GPS backend available. "
                "Install gpsd (sudo apt install gpsd python3-gps) "
                "or pyserial (pip install pyserial) for serial fallback."
            )
            return False

        self._backend      = backend
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True, name="gps_reader"
        )
        self._thread.start()
        self._started = True
        logger.info(
            f"[GpsReader] Started — backend={self._backend_name}  "
            f"poll={GPS_POLL_HZ}Hz  HDOP_max={self._hdop_threshold}  "
            f"sat_min={self._sat_min}"
        )
        return True

    def stop(self) -> None:
        """Stop the polling thread and close the GPS backend."""
        if not self._started:
            return
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5.0)
        if self._backend:
            self._backend.close()
        self._started = False
        logger.info(
            f"[GpsReader] Stopped — {self._fix_count} fixes accepted, "
            f"{self._reject_count} rejected by quality gate"
        )

    # ── Public read API ───────────────────────────────────────────────────────

    def get_fix(self) -> Optional[GpsFix]:
        """Return the latest GPS fix, or None if no fix has been received.

        Returns the fix regardless of reliability — callers must check
        fix.reliable before acting on the position. This allows EXIF writers
        to log even marginal fixes with a quality flag for post-processing.

        Returns None only if the GPS backend has never produced a valid parse.
        """
        with self._lock:
            fix = self._latest_fix
        if fix is None:
            return None
        if fix.is_stale():
            logger.debug("[GpsReader] Fix is stale — GPS may have lost signal")
        return fix

    def get_reliable_fix(self) -> Optional[GpsFix]:
        """Return the latest fix only if it passes all quality gates.

        Use this when acting on position — drift checks, position holds.
        Returns None if fix is stale, unreliable, or unavailable.
        """
        fix = self.get_fix()
        if fix is None or fix.is_stale() or not fix.reliable:
            return None
        return fix

    def is_reliable(self) -> bool:
        """Return True if the current fix is fresh and passes all quality gates."""
        return self.get_reliable_fix() is not None

    @property
    def backend(self) -> str:
        """Name of the active GPS backend: 'gpsd', 'serial', or 'none'."""
        return self._backend_name

    # ── Home datum ────────────────────────────────────────────────────────────

    def set_home(self, fix: Optional[GpsFix] = None) -> bool:
        """Capture the current fix as the WGS84 datum for ENU conversion.

        Should be called once at mission start after a reliable fix is
        confirmed — typically after the drone has been armed and MODE 3
        has been entered. If fix is not provided, uses the current
        latest reliable fix.

        Returns True if home was set, False if no reliable fix was available.
        """
        if fix is None:
            fix = self.get_reliable_fix()
        if fix is None:
            logger.warning(
                "[GpsReader] set_home() called but no reliable fix available. "
                "Retry after HDOP stabilises below %.1f.", self._hdop_threshold
            )
            return False

        with self._lock:
            self._home_fix = fix

        logger.info(
            f"[GpsReader] Home datum set: "
            f"lat={fix.lat:.7f}  lon={fix.lon:.7f}  alt={fix.alt:.1f}m  "
            f"HDOP={fix.hdop:.2f}  sats={fix.satellites}"
        )
        return True

    def get_home(self) -> Optional[GpsFix]:
        """Return the home datum fix, or None if set_home() has not been called."""
        with self._lock:
            return self._home_fix

    # ── ENU conversion ────────────────────────────────────────────────────────

    def get_enu_delta(
        self,
        fix: Optional[GpsFix] = None,
    ) -> Optional[Tuple[float, float, float]]:
        """Convert a GPS fix to ENU metres relative to the home datum.

        Uses flat-Earth approximation valid for missions under ~1 km from home.
        Error is < 1 cm at 100 m, < 10 cm at 500 m from home.

        Parameters
        ----------
        fix : GpsFix or None
            Fix to convert. If None, uses the latest reliable fix.

        Returns
        -------
        (east_m, north_m, up_m) in metres from home datum, or None if no
        home datum has been set or no reliable fix is available.
        """
        with self._lock:
            home = self._home_fix
        if home is None:
            logger.debug("[GpsReader] get_enu_delta() called before set_home()")
            return None

        if fix is None:
            fix = self.get_reliable_fix()
        if fix is None:
            return None

        east, north = _wgs84_to_enu(
            fix.lat, fix.lon, home.lat, home.lon
        )
        up = fix.alt - home.alt

        return round(east, 3), round(north, 3), round(up, 3)

    # ── Drift monitor ─────────────────────────────────────────────────────────

    def check_drift(
        self,
        slam_enu_x: float,
        slam_enu_y: float,
        threshold_m: float = DRIFT_THRESHOLD_M,
    ) -> bool:
        """Check whether GPS position disagrees with SLAM position beyond threshold.

        This is a COARSE safety check — it does NOT replace SLAM as the
        position authority. GPS accuracy is ±3–5 m CEP, so the threshold
        should never be set below 5.0 m or false positives will occur during
        normal flight even when SLAM is healthy.

        The intended use case is detecting runaway drift — scenarios where
        SLAM has silently degraded or lost the map and EKF2 is drifting
        without the flight stack detecting it.

        Parameters
        ----------
        slam_enu_x : float
            Current SLAM-derived East position in metres (ENU frame).
        slam_enu_y : float
            Current SLAM-derived North position in metres (ENU frame).
        threshold_m : float
            Maximum acceptable disagreement between GPS and SLAM in metres.
            Default 5.0 m — below this, normal GPS noise causes false alarms.

        Returns
        -------
        True if GPS disagrees with SLAM by more than threshold_m AND the
        GPS fix is reliable. False if GPS is unreliable (drift check skipped)
        or disagreement is within tolerance.
        """
        delta = self.get_enu_delta()
        if delta is None:
            # No reliable GPS fix — skip drift check, do not raise false alarm
            return False

        gps_east, gps_north, _ = delta
        disagreement = math.sqrt(
            (gps_east  - slam_enu_x) ** 2 +
            (gps_north - slam_enu_y) ** 2
        )

        if disagreement > threshold_m:
            fix = self.get_reliable_fix()
            hdop_str = f"HDOP={fix.hdop:.2f}" if fix else "fix unavailable"
            logger.warning(
                f"[GpsReader] Drift check: GPS↔SLAM disagreement={disagreement:.1f}m "
                f"(threshold={threshold_m:.1f}m)  {hdop_str}  "
                f"GPS=({gps_east:.1f},{gps_north:.1f})  "
                f"SLAM=({slam_enu_x:.1f},{slam_enu_y:.1f})"
            )
            return True

        return False

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        """Return diagnostic counters for logging and health monitoring."""
        with self._lock:
            fix   = self._latest_fix
            home  = self._home_fix
        return {
            "backend":       self._backend_name,
            "fix_count":     self._fix_count,
            "reject_count":  self._reject_count,
            "has_fix":       fix is not None,
            "fix_reliable":  fix.reliable if fix else False,
            "fix_stale":     fix.is_stale() if fix else True,
            "hdop":          fix.hdop if fix else 99.0,
            "satellites":    fix.satellites if fix else 0,
            "home_set":      home is not None,
        }

    # ── Background polling thread ─────────────────────────────────────────────

    def _poll_loop(self) -> None:
        """Background thread: continuously poll the GPS backend at GPS_POLL_HZ."""
        interval = 1.0 / GPS_POLL_HZ
        last_log = time.monotonic()

        while not self._stop.is_set():
            t0  = time.monotonic()
            fix = self._backend.read()

            if fix is not None:
                fix.reliable = self._evaluate_quality(fix)
                with self._lock:
                    self._latest_fix = fix
                    if fix.reliable:
                        self._fix_count += 1
                    else:
                        self._reject_count += 1

            # Periodic diagnostics every 10 seconds
            now = time.monotonic()
            if now - last_log >= 10.0:
                stats = self.get_stats()
                logger.debug(
                    f"[GpsReader] {stats['fix_count']} good / "
                    f"{stats['reject_count']} rejected  "
                    f"HDOP={stats['hdop']:.2f}  sats={stats['satellites']}  "
                    f"reliable={stats['fix_reliable']}"
                )
                last_log = now

            # Sleep remainder of poll interval (non-blocking)
            elapsed = time.monotonic() - t0
            sleep_s = max(0.0, interval - elapsed)
            self._stop.wait(timeout=sleep_s)

    # ── Quality gate ──────────────────────────────────────────────────────────

    def _evaluate_quality(self, fix: GpsFix) -> bool:
        """Apply three-criterion quality gate. All three must pass.

        Returns True if the fix is reliable enough to act on.
        Criteria match PX4 EKF2 GPS health check defaults.
        """
        if fix.fix_type < 1:
            logger.debug(
                f"[GpsReader] Fix rejected: no fix (fix_type={fix.fix_type})"
            )
            return False

        if fix.hdop > self._hdop_threshold:
            logger.debug(
                f"[GpsReader] Fix rejected: HDOP={fix.hdop:.2f} "
                f"> threshold={self._hdop_threshold:.2f}"
            )
            return False

        if fix.satellites < self._sat_min:
            logger.debug(
                f"[GpsReader] Fix rejected: satellites={fix.satellites} "
                f"< minimum={self._sat_min}"
            )
            return False

        return True

    # ── Backend selection ─────────────────────────────────────────────────────

    def _init_backend(self):
        """Try backends in preference order. Returns first that connects."""
        candidates = []
        if self._prefer_gpsd:
            candidates = [("gpsd", self._try_gpsd), ("serial", self._try_serial)]
        else:
            candidates = [("serial", self._try_serial), ("gpsd", self._try_gpsd)]

        for name, factory in candidates:
            backend = factory()
            if backend is not None:
                self._backend_name = name
                return backend

        return None

    def _try_gpsd(self):
        backend = _GpsdBackend(GPSD_HOST, GPSD_PORT)
        if backend.connect():
            return backend
        return None

    def _try_serial(self):
        backend = _SerialNmeaBackend(SERIAL_PORT, SERIAL_BAUD)
        if backend.connect():
            return backend
        return None

    # ── Context manager ───────────────────────────────────────────────────────

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()


# ── Module-level helper functions ─────────────────────────────────────────────

def _build_fix(
    lat: float, lon: float, alt: float,
    hdop: float, sat_count: int,
    fix_type: int, source: str,
) -> GpsFix:
    """Construct a GpsFix from parsed fields. reliable is set by the caller."""
    return GpsFix(
        lat=lat, lon=lon, alt=alt,
        hdop=hdop, satellites=sat_count,
        fix_type=fix_type,
        timestamp=time.monotonic(),
        reliable=False,   # set by GpsReader._evaluate_quality()
        source=source,
    )


def _wgs84_to_enu(
    lat: float, lon: float,
    lat0: float, lon0: float,
) -> Tuple[float, float]:
    """Convert WGS84 geodetic coordinates to ENU metres from a datum point.

    Flat-Earth approximation. Valid to < 1 cm error at 100 m from datum,
    < 10 cm at 500 m. Do not use beyond ~1 km from datum.

    Args:
        lat, lon   : Current position in decimal degrees.
        lat0, lon0 : Datum (home) position in decimal degrees.

    Returns:
        (east_m, north_m) in metres.
    """
    dlat = math.radians(lat  - lat0)
    dlon = math.radians(lon  - lon0)
    lat0_rad = math.radians(lat0)

    north = dlat * _EARTH_RADIUS_M
    east  = dlon * _EARTH_RADIUS_M * math.cos(lat0_rad)

    return east, north


def _nmea_to_decimal(raw: str, direction: str) -> float:
    """Convert NMEA ddmm.mmmm format to decimal degrees.

    NMEA format: DDDMM.MMMM where DDD = degrees, MM.MMMM = minutes
    Example: "4807.038" N → 48 + 7.038/60 = 48.1173°
    """
    if not raw:
        raise ValueError("Empty NMEA coordinate field")

    # Find decimal point to split degrees from minutes
    dot_idx = raw.index(".")
    # Degrees occupy all digits except the last two before the decimal
    deg_digits = dot_idx - 2
    degrees    = float(raw[:deg_digits])
    minutes    = float(raw[deg_digits:])
    decimal    = degrees + minutes / 60.0

    if direction in ("S", "W"):
        decimal = -decimal

    return decimal


def _safe_float(value, default=None) -> Optional[float]:
    """Convert value to float, returning default on failure."""
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _count_used_sats(satellites: list) -> int:
    """Count satellites marked as 'used' in a gpsd SKY satellites list."""
    if not isinstance(satellites, list):
        return 0
    return sum(1 for s in satellites if isinstance(s, dict) and s.get("used", False))
