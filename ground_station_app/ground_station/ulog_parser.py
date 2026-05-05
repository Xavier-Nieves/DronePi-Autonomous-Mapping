"""
ground_station/ulog_parser.py — PX4 ULog flight data parser.

Responsibilities
----------------
Wraps pyulog to extract the subset of fields needed by ReportGenerator:
  - Flight duration and mode transition timeline
  - EKF2 innovation statistics (horizontal velocity, position)
  - GPS quality (HDOP, fix loss events)
  - Battery voltage (start, end, minimum)
  - RC Channel 7 events (manual override activations)
  - Anomaly flags (EKF2 spikes, GPS loss, voltage drop)

Output
------
A single dict matching the handoff's structured JSON intermediate schema
(session §8). Every field has a defined fallback so the report generator
always receives a complete dict even when a ULog message type is absent.

ULog message types used
-----------------------
  vehicle_status          → arm/disarm timestamps, nav_state transitions
  estimator_innovations   → ekf2 innovation fields (vel_ne_innov, pos_ne_innov)
  vehicle_gps_position    → fix_type, eph (HDOP proxy), epv (VDOP proxy)
  battery_status          → voltage_v
  input_rc                → channel 7 raw value (index 6, 0-based)

All message types are optional. If a type is absent from the log the
corresponding output key gets its documented fallback value.

PX4 ULog format reference:
  https://docs.px4.io/main/en/dev_log/ulog_file_format.html
pyulog API reference:
  https://github.com/PX4/pyulog

Design constraints
------------------
- No main() in this module.
- Returns dict; never raises on missing data — sets fallback and logs warning.
- ULogParseError raised only on file-not-found or file-corrupt conditions.
"""

import logging
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger(__name__)

# ── Exception ─────────────────────────────────────────────────────────────────

class ULogParseError(RuntimeError):
    """Raised when the .ulg file cannot be opened or is structurally corrupt."""


# ── RC Channel 7 threshold: values above this are treated as "switch ON" ──────
# PX4 RC channels: 1000 µs = min, 2000 µs = max, 1500 µs = centre.
# Channel 7 override is typically configured at >1750 µs.
# Reference: PX4 RC_MAP_* parameter docs.
RC7_THRESHOLD_US = 1750

# EKF2 innovation spike threshold (m/s for velocity, m for position).
# Values above this are flagged as anomalies.
# Reference: PX4 EKF2_EV_* tuning guidance.
EKF2_SPIKE_THRESHOLD = 0.25


# ══════════════════════════════════════════════════════════════════════════════
# ULogParser
# ══════════════════════════════════════════════════════════════════════════════

class ULogParser:
    """
    Parse a PX4 .ulg file and return a structured summary dict.

    Parameters
    ----------
    ulg_path : str or Path
        Path to the .ulg file on the local filesystem (after download by
        ArtifactFetcher).

    Usage
    -----
        parser = ULogParser("/path/to/flight.ulg")
        data   = parser.parse()
        # data["duration_s"], data["ekf2_innovations"], data["gps"], ...
    """

    def __init__(self, ulg_path) -> None:
        self._path = Path(ulg_path)

    # ── Public ────────────────────────────────────────────────────────────────

    def parse(self) -> dict:
        """
        Parse the ULog file and return the structured summary dict.

        Returns
        -------
        dict with keys:
            flight_id       str   — session ID (stem of file name)
            start_time      str   — ISO timestamp or "unknown"
            duration_s      float — total armed duration in seconds
            modes           list  — [{t: float, mode: str}, ...]
            ekf2_innovations dict — {max, mean, spikes: [{t, value}]}
            gps             dict  — {hdop_max, hdop_mean, fix_loss_events}
            battery         dict  — {start_v, end_v, min_v}
            rc_channel_7_events list — [float, ...] timestamps of CH7 ON
            anomalies       list  — [str, ...]

        Raises
        ------
        ULogParseError
            If the file does not exist or pyulog cannot parse its header.
        """
        if not self._path.exists():
            raise ULogParseError(f"ULog file not found: {self._path}")

        try:
            from pyulog import ULog
            ulog = ULog(str(self._path))
        except Exception as exc:
            raise ULogParseError(f"pyulog failed to open {self._path}: {exc}") from exc

        result: dict[str, Any] = {
            "flight_id":           self._path.stem,
            "start_time":          "unknown",
            "duration_s":          0.0,
            "modes":               [],
            "ekf2_innovations":    {"max": 0.0, "mean": 0.0, "spikes": []},
            "gps":                 {"hdop_max": 0.0, "hdop_mean": 0.0, "fix_loss_events": []},
            "battery":             {"start_v": 0.0, "end_v": 0.0, "min_v": 0.0},
            "rc_channel_7_events": [],
            "anomalies":           [],
        }

        self._extract_duration(ulog, result)
        self._extract_modes(ulog, result)
        self._extract_ekf2(ulog, result)
        self._extract_gps(ulog, result)
        self._extract_battery(ulog, result)
        self._extract_rc7(ulog, result)

        return result

    # ── Private extractors ────────────────────────────────────────────────────

    @staticmethod
    def _msg(ulog, name: str):
        """Return first ULogData for message type `name`, or None if absent."""
        msgs = ulog.get_dataset(name)
        # pyulog.get_dataset returns a list; older versions return a single obj.
        # Normalise to always return one object or None.
        if msgs is None:
            return None
        if isinstance(msgs, list):
            return msgs[0] if msgs else None
        return msgs

    def _extract_duration(self, ulog, result: dict) -> None:
        """
        Compute flight duration from log start/end timestamps.

        pyulog exposes start_timestamp (µs) and last_timestamp (µs) on the
        ULog object itself.

        Reference: pyulog ULog class attributes.
        """
        try:
            start_us = ulog.start_timestamp   # µs since epoch
            last_us  = ulog.last_timestamp    # µs since epoch
            duration_s = (last_us - start_us) / 1e6
            result["duration_s"] = round(max(duration_s, 0.0), 1)

            # ISO start time from epoch microseconds
            from datetime import datetime, timezone
            dt = datetime.fromtimestamp(start_us / 1e6, tz=timezone.utc)
            result["start_time"] = dt.isoformat()
        except Exception as exc:
            log.warning(f"[ULogParser] duration extraction failed: {exc}")

    def _extract_modes(self, ulog, result: dict) -> None:
        """
        Build mode transition timeline from vehicle_status nav_state field.

        nav_state integer codes → human-readable mode strings.
        Reference: PX4 vehicle_status.h nav_state enum.
        https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleStatus.msg
        """
        NAV_STATE = {
            0:  "MANUAL",
            1:  "ALTCTL",
            2:  "POSCTL",
            3:  "AUTO.MISSION",
            4:  "AUTO.LOITER",
            5:  "AUTO.RTL",
            10: "ACRO",
            14: "OFFBOARD",
            15: "STAB",
            17: "AUTO.TAKEOFF",
            18: "AUTO.LAND",
            19: "AUTO.FOLLOW_TARGET",
            20: "AUTO.PRECLAND",
        }
        try:
            ds = self._msg(ulog, "vehicle_status")
            if ds is None:
                log.warning("[ULogParser] vehicle_status not found in log.")
                return

            ts_us   = ds.data["timestamp"]
            states  = ds.data["nav_state"]
            t0_us   = ts_us[0]
            modes   = []
            prev    = None

            for ts, state in zip(ts_us, states):
                if state != prev:
                    label = NAV_STATE.get(int(state), f"STATE_{state}")
                    modes.append({"t": round((ts - t0_us) / 1e6, 2), "mode": label})
                    prev = state

            result["modes"] = modes
        except Exception as exc:
            log.warning(f"[ULogParser] mode extraction failed: {exc}")

    def _extract_ekf2(self, ulog, result: dict) -> None:
        """
        Extract EKF2 horizontal velocity innovation statistics.

        Uses estimator_innovations message, field vel_ne_innov (2-element
        vector: North, East innovations in m/s).
        Magnitude = sqrt(N^2 + E^2).

        Reference: PX4 estimator_innovations.msg
        https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorInnovations.msg
        """
        import math
        try:
            ds = self._msg(ulog, "estimator_innovations")
            if ds is None:
                log.warning("[ULogParser] estimator_innovations not found.")
                return

            ts_us = ds.data["timestamp"]
            t0_us = ts_us[0]

            # Fields may be vel_ne_innov[0] and vel_ne_innov[1] depending on
            # pyulog array expansion, or a single 2-element field.
            # Try common pyulog field name variants.
            vel_n, vel_e = None, None
            data_keys = set(ds.data.keys())

            if "vel_ne_innov[0]" in data_keys and "vel_ne_innov[1]" in data_keys:
                vel_n = ds.data["vel_ne_innov[0]"]
                vel_e = ds.data["vel_ne_innov[1]"]
            elif "vel_ne_innov" in data_keys:
                arr = ds.data["vel_ne_innov"]
                vel_n = arr[:, 0]
                vel_e = arr[:, 1]

            if vel_n is None:
                log.warning("[ULogParser] vel_ne_innov fields not found.")
                return

            magnitudes = [
                math.sqrt(float(n)**2 + float(e)**2)
                for n, e in zip(vel_n, vel_e)
            ]
            spikes = []
            for ts, mag in zip(ts_us, magnitudes):
                if mag > EKF2_SPIKE_THRESHOLD:
                    t_rel = round((ts - t0_us) / 1e6, 2)
                    spikes.append({"t": t_rel, "value": round(mag, 3)})
                    result["anomalies"].append(
                        f"EKF2 innovation spike {mag:.3f} m/s at t={t_rel}s"
                    )

            result["ekf2_innovations"] = {
                "max":    round(max(magnitudes), 3),
                "mean":   round(sum(magnitudes) / len(magnitudes), 3),
                "spikes": spikes,
            }
        except Exception as exc:
            log.warning(f"[ULogParser] EKF2 extraction failed: {exc}")

    def _extract_gps(self, ulog, result: dict) -> None:
        """
        Extract GPS quality from vehicle_gps_position.

        eph (estimated horizontal position error, m) is used as HDOP proxy
        because PX4 ULog stores eph rather than raw HDOP after PX4 v1.13.
        Values are normalised: eph / 5.0 gives an approximate HDOP equivalent
        (PX4 EKF2 HDOP gate default is 2.5, corresponding to eph ~12.5 m).

        fix_type: 0=No fix, 1=No fix, 2=2D, 3=3D, 4=DGPS, 5=RTK float, 6=RTK fixed.
        Fix loss event = transition from fix_type >= 3 to fix_type < 3.

        Reference: PX4 SensorGps.msg, EKF2_GPS_* parameters.
        https://docs.px4.io/main/en/advanced_config/parameter_reference.html
        """
        try:
            ds = self._msg(ulog, "vehicle_gps_position")
            if ds is None:
                log.warning("[ULogParser] vehicle_gps_position not found.")
                return

            ts_us    = ds.data["timestamp"]
            t0_us    = ts_us[0]
            eph      = ds.data.get("eph", None)
            fix_type = ds.data.get("fix_type", None)

            hdop_values = []
            if eph is not None:
                hdop_values = [round(float(v) / 5.0, 2) for v in eph]

            fix_loss_events = []
            if fix_type is not None:
                prev_fix = int(fix_type[0])
                for ts, ft in zip(ts_us[1:], fix_type[1:]):
                    ft = int(ft)
                    if prev_fix >= 3 and ft < 3:
                        t_rel = round((ts - t0_us) / 1e6, 2)
                        fix_loss_events.append(t_rel)
                        result["anomalies"].append(
                            f"GPS fix loss at t={t_rel}s (fix_type {prev_fix}→{ft})"
                        )
                    prev_fix = ft

            result["gps"] = {
                "hdop_max":         round(max(hdop_values), 2) if hdop_values else 0.0,
                "hdop_mean":        round(sum(hdop_values) / len(hdop_values), 2) if hdop_values else 0.0,
                "fix_loss_events":  fix_loss_events,
            }
        except Exception as exc:
            log.warning(f"[ULogParser] GPS extraction failed: {exc}")

    def _extract_battery(self, ulog, result: dict) -> None:
        """
        Extract battery voltage statistics from battery_status.

        voltage_v field (Volts). 6S LiPo nominal: 25.2 V full, 21.0 V cutoff.
        A drop below 22.0 V during flight is flagged as an anomaly.

        Reference: PX4 BatteryStatus.msg
        """
        VOLTAGE_ANOMALY_THRESHOLD = 22.0   # volts — configurable per battery spec

        try:
            ds = self._msg(ulog, "battery_status")
            if ds is None:
                log.warning("[ULogParser] battery_status not found.")
                return

            voltages = [float(v) for v in ds.data["voltage_v"] if float(v) > 5.0]
            if not voltages:
                return

            min_v = round(min(voltages), 2)
            result["battery"] = {
                "start_v": round(voltages[0], 2),
                "end_v":   round(voltages[-1], 2),
                "min_v":   min_v,
            }

            if min_v < VOLTAGE_ANOMALY_THRESHOLD:
                result["anomalies"].append(
                    f"Low battery voltage: {min_v} V (below {VOLTAGE_ANOMALY_THRESHOLD} V threshold)"
                )
        except Exception as exc:
            log.warning(f"[ULogParser] battery extraction failed: {exc}")

    def _extract_rc7(self, ulog, result: dict) -> None:
        """
        Detect RC Channel 7 override activations from input_rc.

        input_rc.values is an array of channel values in µs.
        Channel 7 is index 6 (0-based). Activation = rising edge above
        RC7_THRESHOLD_US.

        Reference: PX4 InputRc.msg, RC channel index convention.
        https://docs.px4.io/main/en/advanced_config/parameter_reference.html
        """
        try:
            ds = self._msg(ulog, "input_rc")
            if ds is None:
                log.info("[ULogParser] input_rc not found — RC7 events not extracted.")
                return

            ts_us  = ds.data["timestamp"]
            t0_us  = ts_us[0]

            # pyulog expands array fields as values[0], values[1], ...
            ch7_key = "values[6]"
            if ch7_key not in ds.data:
                log.warning(f"[ULogParser] {ch7_key} not in input_rc — RC7 skipped.")
                return

            ch7  = ds.data[ch7_key]
            events = []
            prev_state = False

            for ts, val in zip(ts_us, ch7):
                active = float(val) > RC7_THRESHOLD_US
                if active and not prev_state:
                    t_rel = round((ts - t0_us) / 1e6, 2)
                    events.append(t_rel)
                    result["anomalies"].append(f"RC Channel 7 override activated at t={t_rel}s")
                prev_state = active

            result["rc_channel_7_events"] = events
        except Exception as exc:
            log.warning(f"[ULogParser] RC7 extraction failed: {exc}")
