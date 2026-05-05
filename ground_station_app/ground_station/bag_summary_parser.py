"""
ground_station/bag_summary_parser.py — Rosbag summary CSV parser.

Responsibilities
----------------
Parses the bag_summary.csv produced by postprocess_mesh.py Phase X
(the BagReader / MLSSmoother / GroundClassifier pipeline).

The CSV is a single-row key-value file with one column per metric.
Column names are defined by postprocess_mesh.py's publisher stage.
If the file is absent or malformed, all fields fall back to safe defaults.

Expected CSV schema (single data row, header row):
    point_count_raw, point_count_final, drift_estimate_m, loop_closures,
    mls_iterations, sor_outliers_removed, processing_duration_s, bag_path

All fields are optional — absent columns are filled with None or 0.

Design constraints
------------------
- No main() in this module.
- Returns dict; never raises — logs warning and returns defaults on any error.
- pandas is used for CSV parsing (declared in pyproject.toml dependencies).
"""

import logging
from pathlib import Path
from typing import Any, Optional

import pandas as pd

log = logging.getLogger(__name__)

# ── Fallback values returned when a column is absent or unparseable ───────────

_DEFAULTS: dict[str, Any] = {
    "point_count_raw":          0,
    "point_count_final":        0,
    "drift_estimate_m":         None,
    "loop_closures":            0,
    "mls_iterations":           None,
    "sor_outliers_removed":     None,
    "processing_duration_s":    None,
    "bag_path":                 None,
}


# ── Exception ─────────────────────────────────────────────────────────────────

class BagSummaryParseError(RuntimeError):
    """Raised only when the file exists but is completely unparseable."""


# ══════════════════════════════════════════════════════════════════════════════
# BagSummaryParser
# ══════════════════════════════════════════════════════════════════════════════

class BagSummaryParser:
    """
    Parse the bag_summary.csv produced by postprocess_mesh.py.

    Parameters
    ----------
    csv_path : str or Path
        Path to bag_summary.csv on the local filesystem.

    Usage
    -----
        parser = BagSummaryParser("/cache/scan_20260422_143215/bag_summary.csv")
        data   = parser.parse()
        # data["point_count_final"], data["drift_estimate_m"], ...
    """

    def __init__(self, csv_path) -> None:
        self._path = Path(csv_path)

    def parse(self) -> dict:
        """
        Parse the CSV and return the structured SLAM summary dict.

        Returns
        -------
        dict with keys matching _DEFAULTS. All values have correct Python types
        (int, float, str, or None). Never raises on missing/bad data.

        Raises
        ------
        BagSummaryParseError
            Only if the file exists but pandas raises a structural CSV error
            that prevents any column from being read.
        """
        result = dict(_DEFAULTS)

        if not self._path.exists():
            log.warning(f"[BagSummaryParser] File not found: {self._path} — using defaults.")
            return result

        try:
            df = pd.read_csv(self._path, nrows=1)
        except Exception as exc:
            raise BagSummaryParseError(
                f"CSV parse failed for {self._path}: {exc}"
            ) from exc

        if df.empty:
            log.warning(f"[BagSummaryParser] CSV is empty: {self._path}")
            return result

        row = df.iloc[0]

        result["point_count_raw"]       = self._int(row, "point_count_raw")
        result["point_count_final"]     = self._int(row, "point_count_final")
        result["drift_estimate_m"]      = self._float(row, "drift_estimate_m")
        result["loop_closures"]         = self._int(row, "loop_closures")
        result["mls_iterations"]        = self._int(row, "mls_iterations")
        result["sor_outliers_removed"]  = self._int(row, "sor_outliers_removed")
        result["processing_duration_s"] = self._float(row, "processing_duration_s")
        result["bag_path"]              = self._str(row, "bag_path")

        log.info(
            f"[BagSummaryParser] Parsed: points={result['point_count_final']}, "
            f"drift={result['drift_estimate_m']} m, "
            f"loop_closures={result['loop_closures']}"
        )
        return result

    # ── Type-safe column accessors ────────────────────────────────────────────

    @staticmethod
    def _int(row: "pd.Series", col: str) -> Optional[int]:
        if col not in row.index:
            return _DEFAULTS.get(col)
        try:
            v = row[col]
            if pd.isna(v):
                return None
            return int(v)
        except (ValueError, TypeError):
            log.warning(f"[BagSummaryParser] Cannot convert '{col}' to int: {row[col]!r}")
            return None

    @staticmethod
    def _float(row: "pd.Series", col: str) -> Optional[float]:
        if col not in row.index:
            return _DEFAULTS.get(col)
        try:
            v = row[col]
            if pd.isna(v):
                return None
            return float(v)
        except (ValueError, TypeError):
            log.warning(f"[BagSummaryParser] Cannot convert '{col}' to float: {row[col]!r}")
            return None

    @staticmethod
    def _str(row: "pd.Series", col: str) -> Optional[str]:
        if col not in row.index:
            return _DEFAULTS.get(col)
        try:
            v = row[col]
            if pd.isna(v):
                return None
            return str(v).strip()
        except (ValueError, TypeError):
            return None
