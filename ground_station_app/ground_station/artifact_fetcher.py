"""
ground_station/artifact_fetcher.py — RPi artifact downloader.

Responsibilities
----------------
For a given flight session ID, fetch:
  1. flight_record.json  — flight logger structured record
  2. bag_summary.csv     — SLAM pipeline summary (if present)
  3. health_log.csv      — rpi_health samples (if present)
  4. metadata.json       — postprocess metadata (mesh vertex/face counts)

Files are saved to a local cache directory and their paths returned.
Absent files (404) are skipped silently — parsers handle missing files
via their own defaults.

ULog (.ulg) fetching is NOT implemented here. PX4 ULog files are large
(10-100 MB) and are not currently served by serve.py. The parser is
built and tested but will operate on stub/default data until a ULog
serving endpoint is added to serve.py in a future phase.

Cache layout
------------
DRONEPI_CACHE_DIR/<session_id>/
    flight_record.json
    bag_summary.csv
    health_log.csv
    metadata.json

References
----------
- serve.py endpoints confirmed from project knowledge:
    GET /api/flights              → flight index (FlightWatcher)
    GET /rosbags/<session>/<file> → files in the rosbag session folder
    GET /api/logs?session=<id>    → flight_events.json (not used here)
- httpx streaming download:
  https://www.python-httpx.org/advanced/clients/#streaming-responses
"""

import logging
import os
from pathlib import Path
from typing import Optional

import httpx

log = logging.getLogger(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────

def _rpi_base() -> str:
    host = os.environ.get("DRONEPI_RPI_HOST", "10.42.0.1")
    port = os.environ.get("DRONEPI_RPI_PORT", "8080")
    return f"http://{host}:{port}"

def _cache_dir() -> Path:
    return Path(os.environ.get(
        "DRONEPI_CACHE_DIR",
        Path.home() / ".dronepi-ground" / "cache",
    ))

# Artifact filenames as stored in each rosbag session folder on the RPi.
# These match the paths written by flight_logger.py and postprocess_mesh.py.
_ARTIFACTS = [
    "flight_record.json",
    "bag_summary.csv",
    "health_log.csv",
    "metadata.json",
]

_FETCH_TIMEOUT = httpx.Timeout(30.0, connect=5.0)


# ══════════════════════════════════════════════════════════════════════════════
# ArtifactFetcher
# ══════════════════════════════════════════════════════════════════════════════

class ArtifactFetcher:
    """
    Downloads flight artifacts from the RPi HTTP server to a local cache.

    Usage
    -----
        fetcher = ArtifactFetcher()
        paths   = fetcher.fetch("scan_20260422_143215")
        # paths["bag_summary_csv"] → Path or None

    Return dict keys
    ----------------
        flight_record_json  : Path | None
        bag_summary_csv     : Path | None
        health_log_csv      : Path | None
        metadata_json       : Path | None
    """

    def __init__(self) -> None:
        self._cache = _cache_dir()

    def fetch(self, session_id: str) -> dict[str, Optional[Path]]:
        """
        Download all artifacts for session_id.

        Returns dict mapping result keys to local Paths (or None if the
        artifact was absent or failed to download).
        Never raises.
        """
        session_cache = self._cache / session_id
        session_cache.mkdir(parents=True, exist_ok=True)

        results: dict[str, Optional[Path]] = {
            "flight_record_json": None,
            "bag_summary_csv":    None,
            "health_log_csv":     None,
            "metadata_json":      None,
        }

        for filename in _ARTIFACTS:
            local_path = self._fetch_one(session_id, filename, session_cache)
            key = self._filename_to_key(filename)
            results[key] = local_path

        fetched = sum(1 for v in results.values() if v is not None)
        log.info(f"[Fetcher] {session_id}: {fetched}/{len(_ARTIFACTS)} artifacts fetched.")
        return results

    def cached_path(self, session_id: str, filename: str) -> Path:
        """Return the local cache path for a file (may or may not exist)."""
        return self._cache / session_id / filename

    def clear_cache(self, session_id: str) -> None:
        """Remove cached artifacts for a session (used by replay command)."""
        import shutil
        target = self._cache / session_id
        if target.exists():
            shutil.rmtree(target)
            log.info(f"[Fetcher] Cleared cache for {session_id}.")

    # ── Private ───────────────────────────────────────────────────────────────

    def _fetch_one(
        self,
        session_id: str,
        filename: str,
        dest_dir: Path,
    ) -> Optional[Path]:
        """
        Download one file via GET /rosbags/<session_id>/<filename>.

        Returns local Path on success, None on 404 or any error.
        Uses streaming download to handle large files without loading
        the entire response into memory.
        """
        url = f"{_rpi_base()}/rosbags/{session_id}/{filename}"
        dest = dest_dir / filename

        # Skip download if already cached and non-empty.
        if dest.exists() and dest.stat().st_size > 0:
            log.debug(f"[Fetcher] Cache hit: {dest.name}")
            return dest

        try:
            with httpx.Client(timeout=_FETCH_TIMEOUT) as c:
                with c.stream("GET", url) as resp:
                    if resp.status_code == 404:
                        log.debug(f"[Fetcher] Not found (404): {filename}")
                        return None
                    resp.raise_for_status()

                    with open(dest, "wb") as f:
                        for chunk in resp.iter_bytes(chunk_size=65536):
                            f.write(chunk)

            size_kb = dest.stat().st_size // 1024
            log.info(f"[Fetcher] Downloaded {filename} ({size_kb} KB)")
            return dest

        except httpx.ConnectError:
            log.debug(f"[Fetcher] RPi unreachable — cannot fetch {filename}.")
            return None
        except httpx.TimeoutException:
            log.warning(f"[Fetcher] Timeout fetching {filename}.")
            self._remove_partial(dest)
            return None
        except Exception as exc:
            log.warning(f"[Fetcher] Error fetching {filename}: {exc}")
            self._remove_partial(dest)
            return None

    @staticmethod
    def _filename_to_key(filename: str) -> str:
        mapping = {
            "flight_record.json": "flight_record_json",
            "bag_summary.csv":    "bag_summary_csv",
            "health_log.csv":     "health_log_csv",
            "metadata.json":      "metadata_json",
        }
        return mapping.get(filename, filename.replace(".", "_"))

    @staticmethod
    def _remove_partial(path: Path) -> None:
        """Remove a partially downloaded file to prevent cache corruption."""
        try:
            if path.exists():
                path.unlink()
        except OSError:
            pass
