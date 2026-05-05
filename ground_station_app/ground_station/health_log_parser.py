"""
ground_station/health_log_parser.py — RPi health log parser.

Responsibilities
----------------
Parses the health_log.csv produced by scraping /rpi/health topic data
from the rosbag (or directly from rpi_health_node.py's logged output).

The CSV has one row per 2-second health sample. Column names match the
JSON fields published by rpi_health_node.py on /rpi/health:
    timestamp, cpu_percent, cpu_temp, cpu_freq_mhz, mem_percent,
    mem_used_mb, mem_total_mb, disk_percent, throttled, throttle_bits,
    load_avg_1m, load_avg_5m

Output
------
A summary dict with:
    cpu_temp_max_c      float — peak temperature during flight
    cpu_temp_mean_c     float — mean temperature
    throttling_events   int   — number of samples where throttled == True
    throttle_bits_seen  list  — unique non-zero throttle hex values
    memory_peak_pct     float — peak memory usage percentage
    memory_mean_pct     float — mean memory usage
    cpu_percent_max     float — peak CPU usage
    sample_count        int   — total health samples in log

If the file is absent or all fields are missing, safe defaults are returned.

rpi_health_node.py field reference (from rpi_server/rpi_health_node.py):
    cpu_temp     → degrees Celsius (float)
    throttled    → bool (True = throttling active)
    throttle_bits → hex string e.g. "0x50004"
    mem_percent  → 0-100 float
    cpu_percent  → 0-100 float

Design constraints
------------------
- No main() in this module.
- Returns dict; never raises — logs warning and returns defaults on any error.
"""

import logging
from pathlib import Path
from typing import Any, Optional

import pandas as pd

log = logging.getLogger(__name__)

_DEFAULTS: dict[str, Any] = {
    "cpu_temp_max_c":     0.0,
    "cpu_temp_mean_c":    0.0,
    "throttling_events":  0,
    "throttle_bits_seen": [],
    "memory_peak_pct":    0.0,
    "memory_mean_pct":    0.0,
    "cpu_percent_max":    0.0,
    "sample_count":       0,
}


class HealthLogParseError(RuntimeError):
    """Raised only when the file exists but is completely unparseable."""


# ══════════════════════════════════════════════════════════════════════════════
# HealthLogParser
# ══════════════════════════════════════════════════════════════════════════════

class HealthLogParser:
    """
    Parse health_log.csv and return a flight-health summary dict.

    Parameters
    ----------
    csv_path : str or Path
        Path to health_log.csv on the local filesystem.

    Usage
    -----
        parser = HealthLogParser("/cache/scan_20260422_143215/health_log.csv")
        data   = parser.parse()
        # data["cpu_temp_max_c"], data["throttling_events"], ...
    """

    def __init__(self, csv_path) -> None:
        self._path = Path(csv_path)

    def parse(self) -> dict:
        """
        Parse health_log.csv and return the summary dict.

        Returns
        -------
        dict with keys matching _DEFAULTS. Never raises on missing/bad data.

        Raises
        ------
        HealthLogParseError
            Only if the file exists but is structurally unparseable by pandas.
        """
        result = dict(_DEFAULTS)

        if not self._path.exists():
            log.warning(f"[HealthLogParser] File not found: {self._path} — using defaults.")
            return result

        try:
            df = pd.read_csv(self._path)
        except Exception as exc:
            raise HealthLogParseError(
                f"CSV parse failed for {self._path}: {exc}"
            ) from exc

        if df.empty:
            log.warning(f"[HealthLogParser] CSV is empty: {self._path}")
            return result

        result["sample_count"] = len(df)

        # ── CPU temperature ────────────────────────────────────────────────
        if "cpu_temp" in df.columns:
            temps = pd.to_numeric(df["cpu_temp"], errors="coerce").dropna()
            if not temps.empty:
                result["cpu_temp_max_c"]  = round(float(temps.max()), 1)
                result["cpu_temp_mean_c"] = round(float(temps.mean()), 1)

        # ── Throttling ─────────────────────────────────────────────────────
        # throttled column is bool-like: True/False or 1/0
        if "throttled" in df.columns:
            throttled_col = df["throttled"]
            # Normalise string "True"/"False" to bool
            if throttled_col.dtype == object:
                throttled_col = throttled_col.map(
                    lambda v: str(v).strip().lower() in ("true", "1", "yes")
                )
            result["throttling_events"] = int(throttled_col.sum())

        # Collect unique non-zero throttle_bits values for the report
        if "throttle_bits" in df.columns:
            bits_seen = set()
            for v in df["throttle_bits"].dropna():
                s = str(v).strip()
                if s not in ("0x0", "0x00000", "0", "", "nan"):
                    bits_seen.add(s)
            result["throttle_bits_seen"] = sorted(bits_seen)

        # ── Memory ────────────────────────────────────────────────────────
        if "mem_percent" in df.columns:
            mem = pd.to_numeric(df["mem_percent"], errors="coerce").dropna()
            if not mem.empty:
                result["memory_peak_pct"] = round(float(mem.max()), 1)
                result["memory_mean_pct"] = round(float(mem.mean()), 1)

        # ── CPU usage ─────────────────────────────────────────────────────
        if "cpu_percent" in df.columns:
            cpu = pd.to_numeric(df["cpu_percent"], errors="coerce").dropna()
            if not cpu.empty:
                result["cpu_percent_max"] = round(float(cpu.max()), 1)

        log.info(
            f"[HealthLogParser] Parsed {result['sample_count']} samples: "
            f"temp_max={result['cpu_temp_max_c']}°C, "
            f"throttling={result['throttling_events']} events, "
            f"mem_peak={result['memory_peak_pct']}%"
        )
        return result
