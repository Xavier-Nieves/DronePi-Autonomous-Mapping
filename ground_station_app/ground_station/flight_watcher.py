"""
ground_station/flight_watcher.py — RPi flight index poller.

Responsibilities
----------------
- Poll GET <rpi>/api/flights every DRONEPI_POLL_INTERVAL_S seconds.
- Compare the returned flight IDs against a local state file of
  already-processed IDs.
- Emit only new, complete flight IDs to the caller.
- Persist processed IDs to disk so daemon restart / laptop reboot does not
  re-process flights.

State file
----------
JSON file at DRONEPI_STATE_FILE (default ~/.dronepi-ground/state.json):
    {"processed_ids": ["scan_20260422_143215", "scan_20260423_091400"]}

Written atomically via a temp file + rename to prevent corruption on crash.

Polling contract
----------------
- Returns [] immediately if the RPi is unreachable (non-fatal).
- Returns [] if no new complete flights are found.
- Returns list of new session IDs (strings) when new complete flights exist.
- Never raises; all errors are logged and treated as empty poll result.

References
----------
- serve.py /api/flights response schema: confirmed from project knowledge.
  Keys used: flights[].id, flights[].status ('complete' | 'partial' | 'failed')
- httpx timeout docs: https://www.python-httpx.org/advanced/timeouts/
- Atomic file write pattern:
  https://docs.python.org/3/library/os.html#os.replace
"""

import json
import logging
import os
import tempfile
from pathlib import Path
from typing import Optional

import httpx

log = logging.getLogger(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────

def _rpi_base() -> str:
    host = os.environ.get("DRONEPI_RPI_HOST", "10.42.0.1")
    port = os.environ.get("DRONEPI_RPI_PORT", "8080")
    return f"http://{host}:{port}"

def _state_file() -> Path:
    return Path(os.environ.get(
        "DRONEPI_STATE_FILE",
        Path.home() / ".dronepi-ground" / "state.json",
    ))

def _poll_interval() -> float:
    try:
        return float(os.environ.get("DRONEPI_POLL_INTERVAL_S", "20"))
    except ValueError:
        return 20.0

_FETCH_TIMEOUT = httpx.Timeout(5.0, connect=3.0)


# ══════════════════════════════════════════════════════════════════════════════
# FlightWatcher
# ══════════════════════════════════════════════════════════════════════════════

class FlightWatcher:
    """
    Polls the RPi flight index and returns new complete flight IDs.

    Usage (called by FlightReportDaemon on each poll cycle)
    -----
        watcher = FlightWatcher()
        new_ids = watcher.poll()   # [] or ["scan_20260422_143215", ...]
    """

    def __init__(self) -> None:
        self._state_file = _state_file()
        self._processed: set[str] = self._load_state()

    # ── Public ────────────────────────────────────────────────────────────────

    def poll(self) -> list[str]:
        """
        Fetch /api/flights, return IDs of complete flights not yet processed.

        Never raises. Returns [] on any error or when nothing is new.
        """
        flights = self._fetch_index()
        if not flights:
            return []

        new_ids = []
        for flight in flights:
            fid    = flight.get("id") or flight.get("session_id", "")
            status = flight.get("status", "")
            if fid and status == "complete" and fid not in self._processed:
                new_ids.append(fid)

        if new_ids:
            log.info(f"[Watcher] {len(new_ids)} new complete flight(s): {new_ids}")

        return new_ids

    def mark_processed(self, flight_id: str) -> None:
        """
        Record a flight ID as processed and persist to state file.

        Called by FlightReportDaemon after successful report generation
        AND on failure — a failed report is not retried automatically.
        Retry logic is left to the CLI `replay` subcommand.
        """
        self._processed.add(flight_id)
        self._save_state()

    def is_processed(self, flight_id: str) -> bool:
        return flight_id in self._processed

    def reset(self) -> None:
        """Clear all processed IDs. Used by CLI `replay --all`."""
        self._processed.clear()
        self._save_state()
        log.info("[Watcher] State reset — all flights will be reprocessed.")

    # ── Private ───────────────────────────────────────────────────────────────

    def _fetch_index(self) -> list[dict]:
        """
        GET /api/flights from the RPi. Returns list of flight dicts or [].
        """
        url = f"{_rpi_base()}/api/flights"
        try:
            with httpx.Client(timeout=_FETCH_TIMEOUT) as c:
                resp = c.get(url)
                resp.raise_for_status()
                data = resp.json()
                flights = data.get("flights", [])
                log.debug(f"[Watcher] Fetched {len(flights)} flights from {url}")
                return flights
        except httpx.ConnectError:
            log.debug("[Watcher] RPi unreachable — skipping poll.")
            return []
        except httpx.TimeoutException:
            log.warning("[Watcher] Poll timed out.")
            return []
        except Exception as exc:
            log.warning(f"[Watcher] Poll error: {exc}")
            return []

    def _load_state(self) -> set[str]:
        """Load processed IDs from state file. Returns empty set if absent."""
        try:
            data = json.loads(self._state_file.read_text(encoding="utf-8"))
            ids = set(data.get("processed_ids", []))
            log.debug(f"[Watcher] Loaded {len(ids)} processed IDs from state.")
            return ids
        except FileNotFoundError:
            return set()
        except Exception as exc:
            log.warning(f"[Watcher] Could not load state file: {exc}")
            return set()

    def _save_state(self) -> None:
        """
        Persist processed IDs to state file atomically.

        Uses tempfile + os.replace for crash safety — a partial write
        never leaves the state file corrupt.
        """
        self._state_file.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(
            {"processed_ids": sorted(self._processed)}, indent=2
        )
        try:
            fd, tmp_path = tempfile.mkstemp(
                dir=self._state_file.parent, suffix=".tmp"
            )
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    f.write(payload)
                os.replace(tmp_path, self._state_file)
            except Exception:
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
        except Exception as exc:
            log.error(f"[Watcher] Could not save state: {exc}")
