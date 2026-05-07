"""
ground_station/flight_samples_parser.py — Parser for _FlightDataSampler telemetry CSV.

Replaces ULogParser as the source of flight dynamics data for ReportGenerator.
Reads flight_samples.csv written at 2 Hz by safe_flight_mixin._FlightDataSampler
during every flight, and produces the same output dict schema that ULogParser
previously provided — so ReportGenerator.build_structured() requires only a
one-line swap.

CSV schema (from _FlightDataSampler.CSV_HEADER)
------------------------------------------------
timestamp_iso, t_flight_s,
cmd_x, cmd_y, cmd_z,
actual_x, actual_y, actual_z,
pos_error_m,
angvel_x, angvel_y, angvel_z, angvel_mag,
rc_ch1, rc_ch2, rc_ch3, rc_ch4, rc_ch7,
mode, armed

Output dict schema (matches ULogParser output — drop-in replacement)
---------------------------------------------------------------------
{
  "start_time":          str   — first timestamp_iso value, or "unknown"
  "duration_s":          float — t_flight_s of last row
  "modes":               list  — [{t: float, mode: str}] on mode transitions
  "ekf2_innovations":    dict  — pos_error proxy: {max, mean, spikes}
  "gps":                 dict  — always empty defaults (GPS not in this CSV)
  "rc_channel_7_events": list  — [float] t_flight_s where rc_ch7 crossed threshold
  "anomalies":           list  — [str] human-readable anomaly descriptions
}

Notes
-----
- pos_error_m (3D distance between commanded and actual EKF2 position) is used
  as a proxy for ekf2_innovations. It is not the same as a true EKF2 innovation
  magnitude but captures the same signal — position tracking error — and is
  flagged at the same 0.25 m threshold used by ULogParser for EKF2 spikes.
- GPS fields are always empty defaults because GPS is not subscribed in
  _FlightDataSampler. Add /mavros/gpsstatus/gps1/raw to BAG_TOPICS_REQUIRED
  and _FlightDataSampler subscriptions to enable GPS parsing in a future pass.
- RC CH7 threshold matches RC_KILL_THRESHOLD in safe_flight_mixin.py (1700 µs).
  A crossing above this threshold indicates a kill switch activation.
- angvel_mag > 2.0 rad/s sustained is flagged as an anomaly, matching
  MOTION_ANGVEL_THRESHOLD in safe_flight_mixin.py.

Design
------
No main(). Non-fatal on missing file — returns empty defaults.
Never raises. Importable by test scripts.
"""

import csv
import logging
from pathlib import Path
from typing import Union

log = logging.getLogger(__name__)

# ── Thresholds — must match safe_flight_mixin.py constants ───────────────────
_RC7_KILL_THRESHOLD   = 1700    # µs — matches RC_KILL_THRESHOLD
_ANGVEL_THRESHOLD     = 2.0     # rad/s — matches MOTION_ANGVEL_THRESHOLD
_POS_ERROR_THRESHOLD  = 0.25    # m — matches EKF2_SPIKE_THRESHOLD in ulog_parser


class FlightSamplesParser:
    """
    Parse flight_samples.csv and return structured telemetry summary.

    Parameters
    ----------
    csv_path : str or Path
        Path to flight_samples.csv. If the file does not exist or is empty
        all output fields return safe defaults — never raises.

    Usage
    -----
        parser = FlightSamplesParser("/path/to/flight_samples.csv")
        data   = parser.parse()
        # data["modes"], data["anomalies"], data["rc_channel_7_events"], ...
    """

    def __init__(self, csv_path: Union[str, Path]) -> None:
        self._path = Path(csv_path)

    def parse(self) -> dict:
        """
        Parse the CSV and return the structured summary dict.

        Returns
        -------
        dict with keys:
            start_time          str   — ISO timestamp of first sample or "unknown"
            duration_s          float — flight time in seconds (last t_flight_s)
            modes               list  — [{t: float, mode: str}] transition events
            ekf2_innovations    dict  — {max, mean, spikes} using pos_error_m proxy
            gps                 dict  — {hdop_max, hdop_mean, fix_loss_events} (empty)
            rc_channel_7_events list  — [float] t values where CH7 crossed threshold
            anomalies           list  — [str] human-readable anomaly descriptions
        """
        result = self._empty()

        if not self._path.exists():
            log.debug(f"[FlightSamplesParser] File not found: {self._path}")
            return result

        rows = self._read_rows()
        if not rows:
            log.debug(f"[FlightSamplesParser] No data rows in {self._path.name}")
            return result

        self._extract_start_time(rows, result)
        self._extract_duration(rows, result)
        self._extract_modes(rows, result)
        self._extract_pos_error(rows, result)
        self._extract_rc7_events(rows, result)
        self._extract_angvel_anomalies(rows, result)

        log.info(
            f"[FlightSamplesParser] {self._path.name}: "
            f"{len(rows)} rows, duration={result['duration_s']:.1f}s, "
            f"modes={len(result['modes'])}, anomalies={len(result['anomalies'])}"
        )
        return result

    # ── Private ───────────────────────────────────────────────────────────────

    @staticmethod
    def _empty() -> dict:
        return {
            "start_time":          "unknown",
            "duration_s":          0.0,
            "modes":               [],
            "ekf2_innovations":    {"max": 0.0, "mean": 0.0, "spikes": []},
            "gps":                 {"hdop_max": 0.0, "hdop_mean": 0.0, "fix_loss_events": []},
            "rc_channel_7_events": [],
            "anomalies":           [],
        }

    def _read_rows(self) -> list[dict]:
        """Read CSV into list of dicts. Returns [] on any parse error."""
        try:
            rows = []
            with open(self._path, newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    rows.append(row)
            return rows
        except Exception as exc:
            log.warning(f"[FlightSamplesParser] CSV read failed: {exc}")
            return []

    @staticmethod
    def _f(row: dict, key: str, default: float = 0.0) -> float:
        """Safe float extraction from a CSV row dict."""
        try:
            v = row.get(key, "")
            return float(v) if v not in ("", None) else default
        except (ValueError, TypeError):
            return default

    def _extract_start_time(self, rows: list[dict], result: dict) -> None:
        ts = rows[0].get("timestamp_iso", "").strip()
        if ts:
            result["start_time"] = ts

    def _extract_duration(self, rows: list[dict], result: dict) -> None:
        result["duration_s"] = self._f(rows[-1], "t_flight_s", 0.0)

    def _extract_modes(self, rows: list[dict], result: dict) -> None:
        """
        Detect flight mode transitions. Records a new entry whenever
        the mode column changes from the previous sample.
        """
        modes = []
        prev_mode = None
        for row in rows:
            mode = row.get("mode", "").strip()
            if not mode:
                continue
            if mode != prev_mode:
                modes.append({
                    "t":    self._f(row, "t_flight_s"),
                    "mode": mode,
                })
                prev_mode = mode
        result["modes"] = modes

    def _extract_pos_error(self, rows: list[dict], result: dict) -> None:
        """
        Build ekf2_innovations proxy from pos_error_m column.

        pos_error_m = 3D distance between commanded setpoint and actual
        EKF2 position. Spikes flagged at _POS_ERROR_THRESHOLD (0.25 m),
        matching the ULogParser EKF2 spike threshold.
        """
        errors = []
        spikes = []
        anomalies = result["anomalies"]

        for row in rows:
            err = self._f(row, "pos_error_m")
            t   = self._f(row, "t_flight_s")
            errors.append(err)
            if err > _POS_ERROR_THRESHOLD:
                spikes.append({"t": round(t, 2), "value": round(err, 3)})
                anomalies.append(
                    f"Position error {err:.2f}m at t={t:.1f}s "
                    f"(threshold {_POS_ERROR_THRESHOLD}m)"
                )

        if errors:
            result["ekf2_innovations"] = {
                "max":    round(max(errors), 3),
                "mean":   round(sum(errors) / len(errors), 3),
                "spikes": spikes,
            }

    def _extract_rc7_events(self, rows: list[dict], result: dict) -> None:
        """
        Detect RC CH7 kill switch activations — transitions above threshold.
        Records t_flight_s of each rising edge crossing.
        """
        events = []
        anomalies = result["anomalies"]
        prev_above = False

        for row in rows:
            ch7 = self._f(row, "rc_ch7")
            t   = self._f(row, "t_flight_s")
            above = ch7 > _RC7_KILL_THRESHOLD
            if above and not prev_above:
                events.append(round(t, 2))
                anomalies.append(
                    f"RC CH7 kill switch activated at t={t:.1f}s (PWM={int(ch7)})"
                )
            prev_above = above

        result["rc_channel_7_events"] = events

    def _extract_angvel_anomalies(self, rows: list[dict], result: dict) -> None:
        """
        Flag sustained angular velocity above MOTION_ANGVEL_THRESHOLD.
        Reports each window where angvel_mag exceeds threshold for >= 0.5s
        (matches the erratic_motion_monitor sustained threshold).
        """
        anomalies = result["anomalies"]
        above_since = None

        for row in rows:
            wmag = self._f(row, "angvel_mag")
            t    = self._f(row, "t_flight_s")

            if wmag > _ANGVEL_THRESHOLD:
                if above_since is None:
                    above_since = t
            else:
                if above_since is not None:
                    duration = t - above_since
                    if duration >= 0.5:
                        anomalies.append(
                            f"Erratic angular velocity {wmag:.2f} rad/s "
                            f"at t={above_since:.1f}s "
                            f"(sustained {duration:.1f}s, threshold {_ANGVEL_THRESHOLD} rad/s)"
                        )
                    above_since = None
