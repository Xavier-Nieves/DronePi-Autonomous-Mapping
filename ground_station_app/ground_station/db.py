"""
ground_station/db.py — Local flight database (SQLite, PostgreSQL-ready).

Architecture
------------
FlightDatabase wraps SQLite via the stdlib sqlite3 module. The schema
uses only standard SQL types (TEXT, REAL, INTEGER) so migration to
PostgreSQL requires only swapping the connection string — no query
rewrites.

Migration path (future)
-----------------------
  1. Set DRONEPI_DB_URL=postgresql://user:pass@10.42.0.1/dronepi
  2. Replace sqlite3 calls with psycopg2 (schema unchanged)
  3. Add a sync loop in the orchestrator: pull new rows from RPi DB,
     upsert locally with RPi as authoritative source (updated_at wins).

Current state: local SQLite only at ~/.dronepi-ground/dronepi.db

Usage
-----
    db = FlightDatabase()
    db.upsert_flight(session_id="scan_20260504_120000", duration_s=187.4, ...)
    flights = db.get_all_flights()
    flight  = db.get_flight("scan_20260504_120000")

Design constraints
------------------
- No main() in this module.
- All methods are safe to call concurrently from the daemon and UI server
  (WAL journal mode + short transactions).
- Never raises on missing data — returns None / empty list on not found.
- Failures are logged and swallowed so a DB error never crashes the daemon.
"""

import json
import logging
import os
import sqlite3
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger(__name__)

# ── DB location ───────────────────────────────────────────────────────────────

def _db_path() -> Path:
    p = Path(os.environ.get("DRONEPI_DB_PATH",
                            Path.home() / ".dronepi-ground" / "dronepi.db"))
    p.parent.mkdir(parents=True, exist_ok=True)
    return p


# ── Schema ────────────────────────────────────────────────────────────────────

_DDL = """
CREATE TABLE IF NOT EXISTS flights (
    session_id          TEXT PRIMARY KEY,
    arm_time_iso        TEXT,
    duration_s          REAL,
    end_reason          TEXT,

    -- SLAM
    slam_points         INTEGER,
    drift_estimate_m    REAL,
    loop_closures       INTEGER,

    -- System health
    cpu_temp_max_c      REAL,
    throttling_events   INTEGER,
    memory_peak_pct     REAL,

    -- Battery
    battery_start_v     REAL,
    battery_end_v       REAL,
    battery_min_v       REAL,

    -- Mesh
    vertex_count        INTEGER,
    face_count          INTEGER,

    -- Anomalies
    anomaly_count       INTEGER,
    anomalies_json      TEXT,   -- JSON array of strings

    -- Report
    has_report          INTEGER DEFAULT 0,
    report_md           TEXT,

    -- Housekeeping
    created_at          TEXT,
    updated_at          TEXT
);

CREATE INDEX IF NOT EXISTS idx_flights_arm_time
    ON flights (arm_time_iso DESC);
"""


# ══════════════════════════════════════════════════════════════════════════════
# FlightDatabase
# ══════════════════════════════════════════════════════════════════════════════

class FlightDatabase:
    """
    SQLite flight log database.

    Thread-safe via a per-instance lock around connection creation.
    Uses WAL journal mode so concurrent reads from the UI server don't
    block writes from the daemon.
    """

    def __init__(self, db_path: Optional[Path] = None) -> None:
        self._path = db_path or _db_path()
        self._lock = threading.Lock()
        self._init_db()

    # ── Init ──────────────────────────────────────────────────────────────────

    def _init_db(self) -> None:
        try:
            with self._connect() as conn:
                conn.executescript(_DDL)
            log.info(f"[DB] Initialised at {self._path}")
        except Exception as exc:
            log.error(f"[DB] Init failed: {exc}")

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(str(self._path), timeout=10)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA foreign_keys=ON")
        return conn

    # ── Write ─────────────────────────────────────────────────────────────────

    def upsert_flight(
        self,
        session_id: str,
        *,
        arm_time_iso:      Optional[str]   = None,
        duration_s:        Optional[float] = None,
        end_reason:        Optional[str]   = None,
        slam_points:       Optional[int]   = None,
        drift_estimate_m:  Optional[float] = None,
        loop_closures:     Optional[int]   = None,
        cpu_temp_max_c:    Optional[float] = None,
        throttling_events: Optional[int]   = None,
        memory_peak_pct:   Optional[float] = None,
        battery_start_v:   Optional[float] = None,
        battery_end_v:     Optional[float] = None,
        battery_min_v:     Optional[float] = None,
        vertex_count:      Optional[int]   = None,
        face_count:        Optional[int]   = None,
        anomaly_count:     Optional[int]   = None,
        anomalies:         Optional[list]  = None,
        has_report:        bool            = False,
        report_md:         Optional[str]   = None,
    ) -> bool:
        """
        Insert or update a flight record.

        On conflict (same session_id) updates all provided fields and
        always refreshes updated_at. Fields not passed remain unchanged.

        Returns True on success, False on error.
        """
        now = datetime.now(timezone.utc).isoformat()
        anomalies_json = json.dumps(anomalies or [])

        sql = """
        INSERT INTO flights (
            session_id, arm_time_iso, duration_s, end_reason,
            slam_points, drift_estimate_m, loop_closures,
            cpu_temp_max_c, throttling_events, memory_peak_pct,
            battery_start_v, battery_end_v, battery_min_v,
            vertex_count, face_count,
            anomaly_count, anomalies_json,
            has_report, report_md,
            created_at, updated_at
        ) VALUES (
            :session_id, :arm_time_iso, :duration_s, :end_reason,
            :slam_points, :drift_estimate_m, :loop_closures,
            :cpu_temp_max_c, :throttling_events, :memory_peak_pct,
            :battery_start_v, :battery_end_v, :battery_min_v,
            :vertex_count, :face_count,
            :anomaly_count, :anomalies_json,
            :has_report, :report_md,
            :now, :now
        )
        ON CONFLICT(session_id) DO UPDATE SET
            arm_time_iso      = COALESCE(:arm_time_iso,      arm_time_iso),
            duration_s        = COALESCE(:duration_s,        duration_s),
            end_reason        = COALESCE(:end_reason,        end_reason),
            slam_points       = COALESCE(:slam_points,       slam_points),
            drift_estimate_m  = COALESCE(:drift_estimate_m,  drift_estimate_m),
            loop_closures     = COALESCE(:loop_closures,     loop_closures),
            cpu_temp_max_c    = COALESCE(:cpu_temp_max_c,    cpu_temp_max_c),
            throttling_events = COALESCE(:throttling_events, throttling_events),
            memory_peak_pct   = COALESCE(:memory_peak_pct,   memory_peak_pct),
            battery_start_v   = COALESCE(:battery_start_v,   battery_start_v),
            battery_end_v     = COALESCE(:battery_end_v,     battery_end_v),
            battery_min_v     = COALESCE(:battery_min_v,     battery_min_v),
            vertex_count      = COALESCE(:vertex_count,      vertex_count),
            face_count        = COALESCE(:face_count,        face_count),
            anomaly_count     = COALESCE(:anomaly_count,     anomaly_count),
            anomalies_json    = COALESCE(:anomalies_json,    anomalies_json),
            has_report        = MAX(has_report, :has_report),
            report_md         = COALESCE(:report_md,         report_md),
            updated_at        = :now
        """
        try:
            with self._lock, self._connect() as conn:
                conn.execute(sql, {
                    "session_id":       session_id,
                    "arm_time_iso":     arm_time_iso,
                    "duration_s":       duration_s,
                    "end_reason":       end_reason,
                    "slam_points":      slam_points,
                    "drift_estimate_m": drift_estimate_m,
                    "loop_closures":    loop_closures,
                    "cpu_temp_max_c":   cpu_temp_max_c,
                    "throttling_events":throttling_events,
                    "memory_peak_pct":  memory_peak_pct,
                    "battery_start_v":  battery_start_v,
                    "battery_end_v":    battery_end_v,
                    "battery_min_v":    battery_min_v,
                    "vertex_count":     vertex_count,
                    "face_count":       face_count,
                    "anomaly_count":    anomaly_count,
                    "anomalies_json":   anomalies_json,
                    "has_report":       int(has_report),
                    "report_md":        report_md,
                    "now":              now,
                })
            log.debug(f"[DB] Upserted {session_id}")
            return True
        except Exception as exc:
            log.error(f"[DB] upsert_flight failed for {session_id}: {exc}")
            return False

    # ── Read ──────────────────────────────────────────────────────────────────

    def get_all_flights(self, limit: int = 200) -> list[dict]:
        """
        Return all flights ordered newest first, up to `limit`.

        Returns empty list on error.
        """
        try:
            with self._connect() as conn:
                rows = conn.execute(
                    "SELECT * FROM flights ORDER BY arm_time_iso DESC, session_id DESC LIMIT ?",
                    (limit,)
                ).fetchall()
            return [self._row_to_dict(r) for r in rows]
        except Exception as exc:
            log.error(f"[DB] get_all_flights failed: {exc}")
            return []

    def get_flight(self, session_id: str) -> Optional[dict]:
        """
        Return a single flight record or None if not found.
        """
        try:
            with self._connect() as conn:
                row = conn.execute(
                    "SELECT * FROM flights WHERE session_id = ?", (session_id,)
                ).fetchone()
            return self._row_to_dict(row) if row else None
        except Exception as exc:
            log.error(f"[DB] get_flight failed for {session_id}: {exc}")
            return None

    def get_stats(self) -> dict:
        """
        Aggregate stats across all flights for the dashboard KPIs.

        Returns
        -------
        dict with keys:
            total, reported, avg_duration_s, total_anomalies,
            avg_slam_points, avg_drift_m, avg_cpu_temp_c
        """
        try:
            with self._connect() as conn:
                row = conn.execute("""
                    SELECT
                        COUNT(*)                     AS total,
                        SUM(has_report)              AS reported,
                        AVG(duration_s)              AS avg_duration_s,
                        SUM(anomaly_count)           AS total_anomalies,
                        AVG(slam_points)             AS avg_slam_points,
                        AVG(drift_estimate_m)        AS avg_drift_m,
                        AVG(cpu_temp_max_c)          AS avg_cpu_temp_c
                    FROM flights
                """).fetchone()
            if not row:
                return {}
            return dict(row)
        except Exception as exc:
            log.error(f"[DB] get_stats failed: {exc}")
            return {}

    def get_recent_anomalies(self, limit: int = 10) -> list[dict]:
        """
        Return the most recent anomaly entries across all flights.

        Returns list of dicts: {session_id, arm_time_iso, text}
        """
        try:
            with self._connect() as conn:
                rows = conn.execute("""
                    SELECT session_id, arm_time_iso, anomalies_json
                    FROM flights
                    WHERE anomaly_count > 0
                    ORDER BY arm_time_iso DESC
                    LIMIT ?
                """, (limit,)).fetchall()
            result = []
            for row in rows:
                try:
                    anms = json.loads(row["anomalies_json"] or "[]")
                    for a in anms:
                        result.append({
                            "session": row["session_id"],
                            "time":    row["arm_time_iso"],
                            "text":    a,
                        })
                except Exception:
                    pass
            return result[:limit]
        except Exception as exc:
            log.error(f"[DB] get_recent_anomalies failed: {exc}")
            return []

    def get_trend_series(self, metric: str, limit: int = 20) -> list[dict]:
        """
        Return time-series values for a single metric, oldest first.

        Supported metrics: duration_s, slam_points, drift_estimate_m,
                           cpu_temp_max_c, battery_min_v, anomaly_count

        Returns list of {session_id, arm_time_iso, value}
        """
        allowed = {
            "duration_s", "slam_points", "drift_estimate_m",
            "cpu_temp_max_c", "battery_min_v", "anomaly_count",
        }
        if metric not in allowed:
            log.warning(f"[DB] get_trend_series: invalid metric '{metric}'")
            return []
        try:
            with self._connect() as conn:
                rows = conn.execute(f"""
                    SELECT session_id, arm_time_iso, {metric} AS value
                    FROM flights
                    ORDER BY arm_time_iso DESC, session_id DESC
                    LIMIT ?
                """, (limit,)).fetchall()
            # Return oldest first for chart display
            return list(reversed([dict(r) for r in rows]))
        except Exception as exc:
            log.error(f"[DB] get_trend_series failed: {exc}")
            return []

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _row_to_dict(row: sqlite3.Row) -> dict:
        d = dict(row)
        try:
            d["anomalies"] = json.loads(d.get("anomalies_json") or "[]")
        except Exception:
            d["anomalies"] = []
        d["has_report"] = bool(d.get("has_report", 0))
        return d
