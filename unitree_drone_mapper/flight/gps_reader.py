"""flight/gps_reader.py — MAVROS-backed GPS reader for EXIF tagging and soft drift detection.

ARCHITECTURAL POSITION
----------------------
This module is a RUNTIME MODULE — imported directly by main.py during MODE 3
autonomous flight. It does NOT replace or compete with Point-LIO SLAM as the
EKF2 position source. Instead, it listens to the MAVROS GPS topics already
forwarded from PX4 and exposes a small thread-safe API to the rest of the
application.

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

GPS FEED PATH
-------------
This version assumes the physical GPS receiver is connected to PX4, not
directly to the Raspberry Pi. The Pi therefore does not use gpsd or raw serial
NMEA. Instead it reads the already-bridged MAVROS topics:

    GPS receiver -> PX4 -> MAVLink -> MAVROS -> GpsReader -> main.py

Primary topics:
    1) /mavros/global_position/raw/fix      (sensor_msgs/NavSatFix)
       Provides WGS84 latitude / longitude / altitude.

    2) /mavros/gpsstatus/gps1/raw           (mavros_msgs/GPSRAW)
       Provides fix_type, satellites_visible, and EPH/EPV accuracy fields.

Optional fallback topic:
    /mavros/global_position/global

That higher-level topic is useful as a backup if PX4 is publishing it, but the
raw pair above remains the preferred source because it appears earlier and more
directly represents the health of the PX4→MAVROS GPS link.

QUALITY GATING
--------------
Fix quality is evaluated on every merged sample via three independent criteria:

    1. Fix type        — require fix_type ≥ 1
    2. HDOP proxy      — reject if HDOP_proxy > HDOP_THRESHOLD (default 2.5)
    3. Satellite count — reject if fewer than SAT_COUNT_MIN (default 6)

A fix that fails any criterion sets is_reliable() → False.
The coordinates are still returned by get_fix() so callers can log the raw data
for post-mission diagnostics, but the quality flag allows callers to decide
whether to act on the position.

Why "HDOP proxy" instead of true HDOP?
    MAVROS GPSRAW commonly exposes EPH (horizontal accuracy proxy) rather than a
    direct HDOP field. This module therefore derives a practical HDOP-like value
    from EPH when available:

        hdop_proxy = eph / 100.0

    Sentinel values such as 9999 / 65535 are treated as invalid and mapped to a
    deliberately bad value so the sample is rejected by the quality gate.

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

This is a secondary coarse reference only. SLAM remains the authoritative local
position source for flight control and obstacle-aware motion.

THREAD SAFETY
-------------
GpsReader runs its own ROS 2 executor in a background thread at startup.
All public methods acquire _lock before reading shared state. Safe to call
from main.py's orchestrator thread while the MAVROS subscriber thread runs.

DRIFT MONITOR INTEGRATION
--------------------------
main.py's handle_autonomous() may call check_drift() after each fly_to()
completes. If drift exceeds DRIFT_THRESHOLD_M, the method returns True and the
caller may issue a corrective setpoint. Example:

    if gps.check_drift(current_enu_x, current_enu_y, threshold_m=5.0):
        log("[GPS] Gross drift detected — holding position")
        node.fly_to(current_enu_x, current_enu_y, ez, 0.0)

This is a coarse catch (±3–5 m GPS accuracy). It is NOT a precision hover
stabiliser — the SLAM bridge handles that. It catches only runaway drift
scenarios where SLAM has silently degraded or lost its map.

DEPENDENCIES
------------
    ROS 2 Jazzy / rclpy
    sensor_msgs.msg.NavSatFix
    mavros_msgs.msg.GPSRAW

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

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GPSRAW

logger = logging.getLogger(__name__)

# ── Configuration constants ───────────────────────────────────────────────────
# All overridable via environment variables for bench testing and deployment.

GPS_POLL_HZ       = float(os.environ.get("GPS_POLL_HZ", "5.0"))
FIX_STALE_S       = float(os.environ.get("GPS_STALE_S", "3.0"))
TOPIC_STALE_S     = float(os.environ.get("GPS_TOPIC_STALE_S", "2.5"))

# Quality gate thresholds — intentionally aligned with the same practical gate
# used in the bench test and close to PX4 EKF2 expectations.
HDOP_THRESHOLD    = float(os.environ.get("GPS_HDOP_MAX", "2.5"))
SAT_COUNT_MIN     = int(os.environ.get("GPS_SAT_MIN", "6"))
FIX_TYPE_MIN      = int(os.environ.get("GPS_FIX_MIN", "1"))

# Drift monitor
DRIFT_THRESHOLD_M = float(os.environ.get("GPS_DRIFT_M", "5.0"))

# Topic names — user-overridable in case MAVROS namespaces differ in deployment.
RAW_FIX_TOPIC     = os.environ.get("GPS_RAW_FIX_TOPIC", "/mavros/global_position/raw/fix")
GPSRAW_TOPIC      = os.environ.get("GPS_GPSRAW_TOPIC", "/mavros/gpsstatus/gps1/raw")
GLOBAL_TOPIC      = os.environ.get("GPS_GLOBAL_TOPIC", "/mavros/global_position/global")

# Earth radius for flat-Earth ENU conversion
_EARTH_RADIUS_M = 6_371_000.0

# GPSRAW sentinel values commonly seen when PX4 has no valid solution.
_INVALID_EPH_VALUES = {0, 9999, 65535, 4294967295}
_INVALID_EPV_VALUES = {0, 9999, 65535, 4294967295}


# ── Data types ────────────────────────────────────────────────────────────────

@dataclass
class GpsFix:
    """Single GPS fix snapshot with quality metadata.

    Attributes
    ----------
    lat, lon, alt : float
        WGS84 geodetic coordinates. lat/lon in decimal degrees, alt in metres MSL.
    hdop : float
        Practical HDOP-like proxy derived from GPSRAW.eph when available.
        Lower is better. > 2.5 = unreliable.
    satellites : int
        Number of satellites visible / used in the fix solution.
    fix_type : int
        0 = no fix, 1 = GPS fix, 2 = DGPS fix, etc. A usable fix requires
        fix_type ≥ 1.
    timestamp : float
        monotonic time of this fix (time.monotonic()).
    reliable : bool
        True if fix passes all quality gates (hdop proxy, sat count, fix type).
    source : str
        Which topic combination produced this fix.
    eph, epv : float | None
        Raw MAVROS GPSRAW accuracy fields for diagnostics.
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
    eph:        Optional[float] = None
    epv:        Optional[float] = None

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
            "eph":        self.eph,
            "epv":        self.epv,
        }


# ── MAVROS topic reader node ─────────────────────────────────────────────────

class _MavrosGpsNode(Node):
    """Small ROS 2 node that merges the MAVROS GPS topics into a single fix.

    The node itself does not apply the final reliability gate; it only keeps the
    freshest topic data and exposes a merged fix candidate to the outer
    GpsReader class.
    """

    def __init__(self, raw_fix_topic: str, gpsraw_topic: str, global_topic: str):
        super().__init__("gps_reader_mavros")
        self._raw_fix_topic = raw_fix_topic
        self._gpsraw_topic = gpsraw_topic
        self._global_topic = global_topic

        self._navsat_raw: Optional[NavSatFix] = None
        self._navsat_raw_t: float = 0.0
        self._gpsraw: Optional[GPSRAW] = None
        self._gpsraw_t: float = 0.0
        self._global_fix: Optional[NavSatFix] = None
        self._global_fix_t: float = 0.0

        self.create_subscription(
            NavSatFix,
            raw_fix_topic,
            self._raw_fix_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GPSRAW,
            gpsraw_topic,
            self._gpsraw_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            global_topic,
            self._global_fix_cb,
            qos_profile_sensor_data,
        )

    def _raw_fix_cb(self, msg: NavSatFix) -> None:
        self._navsat_raw = msg
        self._navsat_raw_t = time.monotonic()

    def _gpsraw_cb(self, msg: GPSRAW) -> None:
        self._gpsraw = msg
        self._gpsraw_t = time.monotonic()

    def _global_fix_cb(self, msg: NavSatFix) -> None:
        self._global_fix = msg
        self._global_fix_t = time.monotonic()

    def build_fix(self) -> Optional[GpsFix]:
        """Merge latest MAVROS messages into one GpsFix candidate.

        Preferred path:
            raw NavSatFix + GPSRAW

        Fallback path:
            global NavSatFix + GPSRAW

        Returns None until at least GPSRAW and one NavSatFix source have both
        been seen recently.
        """
        now = time.monotonic()
        gpsraw = self._gpsraw
        if gpsraw is None or (now - self._gpsraw_t) > TOPIC_STALE_S:
            return None

        nav = None
        source = None
        if self._navsat_raw is not None and (now - self._navsat_raw_t) <= TOPIC_STALE_S:
            nav = self._navsat_raw
            source = "mavros_raw_fix+gpsraw"
        elif self._global_fix is not None and (now - self._global_fix_t) <= TOPIC_STALE_S:
            nav = self._global_fix
            source = "mavros_global_fix+gpsraw"

        if nav is None:
            return None

        hdop = _eph_to_hdop_proxy(int(gpsraw.eph))
        return GpsFix(
            lat=float(nav.latitude),
            lon=float(nav.longitude),
            alt=float(nav.altitude),
            hdop=hdop,
            satellites=int(gpsraw.satellites_visible),
            fix_type=int(gpsraw.fix_type),
            timestamp=now,
            reliable=False,  # set by outer GpsReader quality gate
            source=source,
            eph=float(gpsraw.eph),
            epv=float(gpsraw.epv),
        )

    def close(self) -> None:
        try:
            self.destroy_node()
        except Exception:
            pass


# ── Main GpsReader class ──────────────────────────────────────────────────────

class GpsReader:
    """MAVROS-backed GPS reader for EXIF tagging and soft drift detection.

    Runs a background ROS 2 executor thread that subscribes to MAVROS GPS
    topics. All public methods are thread-safe.

    Callers
    -------
        main.py (MODE 3 autonomous) — drift monitor via check_drift()
        camera_capture.py           — EXIF/sidecar via get_fix()

    Not a caller
    ------------
        _slam_bridge.py  — SLAM remains the sole EKF2 position source
        PX4              — this module only observes MAVROS; it does not send GPS

    Parameters
    ----------
    hdop_threshold : float
        Maximum acceptable HDOP proxy before fix is flagged unreliable.
        Default 2.5.
    sat_min : int
        Minimum satellites for a reliable fix. Default 6.
    fix_type_min : int
        Minimum acceptable GPSRAW.fix_type. Default 1.
    raw_fix_topic, gpsraw_topic, global_topic : str
        MAVROS topic names. Defaults match the current DronePi deployment.
    """

    def __init__(
        self,
        prefer_gpsd:    bool = True,
        hdop_threshold: float = HDOP_THRESHOLD,
        sat_min:        int = SAT_COUNT_MIN,
        fix_type_min:   int = FIX_TYPE_MIN,
        raw_fix_topic:  str = RAW_FIX_TOPIC,
        gpsraw_topic:   str = GPSRAW_TOPIC,
        global_topic:   str = GLOBAL_TOPIC,
    ):
        # prefer_gpsd is intentionally retained in the signature so main.py and
        # older callers do not break. It is ignored in the MAVROS-backed design.
        self._prefer_gpsd    = prefer_gpsd
        self._hdop_threshold = hdop_threshold
        self._sat_min        = sat_min
        self._fix_type_min   = fix_type_min

        self._raw_fix_topic  = raw_fix_topic
        self._gpsraw_topic   = gpsraw_topic
        self._global_topic   = global_topic

        self._lock           = threading.Lock()
        self._stop           = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Shared state updated by background thread
        self._latest_fix:   Optional[GpsFix] = None
        self._home_fix:     Optional[GpsFix] = None
        self._fix_count:    int = 0
        self._reject_count: int = 0
        self._started:      bool = False

        # ROS runtime state
        self._node: Optional[_MavrosGpsNode] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._owns_rclpy_context: bool = False
        self._backend_name: str = "mavros"

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Initialise the MAVROS reader backend and start the executor thread.

        Returns True if the ROS node was created successfully. The method does
        not require that a valid GPS fix already exists — it only requires that
        the subscriptions were created. Actual fix health is reported later via
        get_fix(), get_reliable_fix(), and get_stats().
        """
        if self._started:
            logger.warning("[GpsReader] Already started — ignoring duplicate start()")
            return True

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy_context = True
        except RuntimeError:
            # Another part of the process may already be racing to initialise
            # rclpy. If it is up afterwards, continue normally.
            if not rclpy.ok():
                logger.exception("[GpsReader] Failed to initialise ROS 2 context")
                return False

        try:
            self._node = _MavrosGpsNode(
                raw_fix_topic=self._raw_fix_topic,
                gpsraw_topic=self._gpsraw_topic,
                global_topic=self._global_topic,
            )
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as exc:
            logger.error(f"[GpsReader] Failed to create MAVROS GPS node: {exc}")
            self._cleanup_ros()
            return False

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._poll_loop,
            daemon=True,
            name="gps_reader_mavros",
        )
        self._thread.start()
        self._started = True

        logger.info(
            f"[GpsReader] Started — backend={self._backend_name}  "
            f"raw_fix={self._raw_fix_topic}  gpsraw={self._gpsraw_topic}  "
            f"global={self._global_topic}  poll={GPS_POLL_HZ}Hz  "
            f"HDOP_max={self._hdop_threshold}  sat_min={self._sat_min}  "
            f"fix_type_min={self._fix_type_min}"
        )
        return True

    def stop(self) -> None:
        """Stop the executor thread and release ROS resources."""
        if not self._started:
            return

        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5.0)

        self._cleanup_ros()
        self._started = False

        logger.info(
            f"[GpsReader] Stopped — {self._fix_count} fixes accepted, "
            f"{self._reject_count} rejected by quality gate"
        )

    def _cleanup_ros(self) -> None:
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
            self._executor = None

        if self._node is not None:
            self._node.close()
            self._node = None

        if self._owns_rclpy_context:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
            self._owns_rclpy_context = False

    # ── Public read API ───────────────────────────────────────────────────────

    def get_fix(self) -> Optional[GpsFix]:
        """Return the latest GPS fix, or None if no merged fix has been received.

        Returns the fix regardless of reliability — callers must check
        fix.reliable before acting on the position. This allows EXIF writers
        to log even marginal fixes with a quality flag for post-processing.

        Returns None only if the MAVROS reader has not yet received a matching
        GPSRAW + NavSatFix sample pair.
        """
        with self._lock:
            fix = self._latest_fix
        if fix is None:
            return None
        if fix.is_stale():
            logger.debug("[GpsReader] Fix is stale — PX4/MAVROS GPS may have stopped updating")
        return fix

    def get_reliable_fix(self) -> Optional[GpsFix]:
        """Return the latest fix only if it passes all quality gates.

        Use this when acting on position — drift checks, geotagging decisions,
        home capture. Returns None if fix is stale, unreliable, or unavailable.
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
        """Name of the active GPS backend."""
        return self._backend_name

    # ── Home datum ────────────────────────────────────────────────────────────

    def set_home(self, fix: Optional[GpsFix] = None) -> bool:
        """Capture the current fix as the WGS84 datum for ENU conversion.

        Should be called once at mission start after a reliable fix is
        confirmed. If fix is not provided, uses the current latest reliable fix.

        Returns True if home was set, False if no reliable fix was available.
        """
        if fix is None:
            fix = self.get_reliable_fix()
        if fix is None:
            logger.warning(
                "[GpsReader] set_home() called but no reliable fix available. "
                "Retry after MAVROS GPS quality improves below thresholds."
            )
            return False

        with self._lock:
            self._home_fix = fix

        logger.info(
            f"[GpsReader] Home datum set: "
            f"lat={fix.lat:.7f}  lon={fix.lon:.7f}  alt={fix.alt:.1f}m  "
            f"HDOP_proxy={fix.hdop:.2f}  sats={fix.satellites}  "
            f"source={fix.source}"
        )
        return True

    def get_home(self) -> Optional[GpsFix]:
        """Return the home datum fix, or None if set_home() has not been called."""
        with self._lock:
            return self._home_fix

    # ── ENU conversion ────────────────────────────────────────────────────────

    def get_enu_delta(self, fix: Optional[GpsFix] = None) -> Optional[Tuple[float, float, float]]:
        """Convert a GPS fix to ENU metres relative to the home datum.

        Uses flat-Earth approximation valid for missions under ~1 km from home.
        Error is small compared with consumer GPS noise.

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

        east, north = _wgs84_to_enu(fix.lat, fix.lon, home.lat, home.lon)
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

        Returns True if GPS disagrees with SLAM by more than threshold_m AND the
        GPS fix is reliable. False if GPS is unreliable (drift check skipped)
        or disagreement is within tolerance.
        """
        delta = self.get_enu_delta()
        if delta is None:
            # No reliable GPS fix — skip drift check, do not raise false alarm.
            return False

        gps_east, gps_north, _ = delta
        disagreement = math.sqrt(
            (gps_east - slam_enu_x) ** 2 +
            (gps_north - slam_enu_y) ** 2
        )

        if disagreement > threshold_m:
            fix = self.get_reliable_fix()
            hdop_str = f"HDOP_proxy={fix.hdop:.2f}" if fix else "fix unavailable"
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
            fix = self._latest_fix
            home = self._home_fix
        return {
            "backend": self._backend_name,
            "fix_count": self._fix_count,
            "reject_count": self._reject_count,
            "has_fix": fix is not None,
            "fix_reliable": fix.reliable if fix else False,
            "fix_stale": fix.is_stale() if fix else True,
            "hdop": fix.hdop if fix else 99.0,
            "satellites": fix.satellites if fix else 0,
            "fix_type": fix.fix_type if fix else 0,
            "source": fix.source if fix else "none",
            "eph": fix.eph if fix else None,
            "epv": fix.epv if fix else None,
            "home_set": home is not None,
            "raw_fix_topic": self._raw_fix_topic,
            "gpsraw_topic": self._gpsraw_topic,
            "global_topic": self._global_topic,
        }

    # ── Background subscriber / merger thread ────────────────────────────────

    def _poll_loop(self) -> None:
        """Background thread: spin the MAVROS node and merge topic samples."""
        interval = 1.0 / GPS_POLL_HZ
        last_log = time.monotonic()

        while not self._stop.is_set():
            t0 = time.monotonic()

            try:
                if self._executor is not None:
                    self._executor.spin_once(timeout_sec=0.2)
            except Exception as exc:
                logger.warning(f"[GpsReader] Executor spin error: {exc}")

            fix = None
            if self._node is not None:
                fix = self._node.build_fix()

            if fix is not None:
                fix.reliable = self._evaluate_quality(fix)
                with self._lock:
                    self._latest_fix = fix
                    if fix.reliable:
                        self._fix_count += 1
                    else:
                        self._reject_count += 1

            # Periodic diagnostics every 10 seconds.
            now = time.monotonic()
            if now - last_log >= 10.0:
                stats = self.get_stats()
                logger.debug(
                    f"[GpsReader] {stats['fix_count']} good / "
                    f"{stats['reject_count']} rejected  "
                    f"HDOP_proxy={stats['hdop']:.2f}  sats={stats['satellites']}  "
                    f"fix_type={stats['fix_type']}  reliable={stats['fix_reliable']}  "
                    f"source={stats['source']}"
                )
                last_log = now

            elapsed = time.monotonic() - t0
            sleep_s = max(0.0, interval - elapsed)
            self._stop.wait(timeout=sleep_s)

    # ── Quality gate ──────────────────────────────────────────────────────────

    def _evaluate_quality(self, fix: GpsFix) -> bool:
        """Apply three-criterion quality gate. All three must pass.

        Returns True if the fix is reliable enough to act on.
        Criteria are intentionally conservative and suitable for using GPS as a
        secondary coarse reference next to SLAM.
        """
        if fix.fix_type < self._fix_type_min:
            logger.debug(
                f"[GpsReader] Fix rejected: no usable fix "
                f"(fix_type={fix.fix_type}, min={self._fix_type_min})"
            )
            return False

        if fix.hdop > self._hdop_threshold:
            logger.debug(
                f"[GpsReader] Fix rejected: HDOP_proxy={fix.hdop:.2f} "
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

    # ── Context manager ───────────────────────────────────────────────────────

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()


# ── Module-level helper functions ─────────────────────────────────────────────

def _eph_to_hdop_proxy(eph_value: int) -> float:
    """Convert MAVROS GPSRAW EPH into a practical HDOP-like proxy.

    In MAVLink-derived GPSRAW data, EPH is typically horizontal accuracy in
    centimeters. This is not identical to HDOP, but it is a useful quality
    proxy for field acceptance checks and geotag reliability decisions.
    """
    if eph_value in _INVALID_EPH_VALUES:
        return 99.0
    if eph_value <= 0:
        return 99.0
    return round(eph_value / 100.0, 2)



def _wgs84_to_enu(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """Convert WGS84 geodetic coordinates to ENU metres from a datum point.

    Flat-Earth approximation. Valid to < 1 cm error at 100 m from datum,
    < 10 cm at 500 m. Do not use beyond ~1 km from datum.

    Args:
        lat, lon   : Current position in decimal degrees.
        lat0, lon0 : Datum (home) position in decimal degrees.

    Returns:
        (east_m, north_m) in metres.
    """
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    lat0_rad = math.radians(lat0)

    north = dlat * _EARTH_RADIUS_M
    east = dlon * _EARTH_RADIUS_M * math.cos(lat0_rad)
    return east, north
