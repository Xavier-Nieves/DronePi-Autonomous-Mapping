"""
ground_station/orchestrator.py — FlightReportDaemon background loop.

Responsibilities
----------------
The single production entry point for background processing. Coordinates:
    FlightWatcher → ArtifactFetcher → ReportGenerator → ReportPublisher

Loop design (commercial-grade crash resistance)
-----------------------------------------------
Each poll iteration is wrapped in a try/except. An exception in one
iteration does not kill the daemon — it logs, backs off, and retries
on the next cycle. This handles:
  - RPi WiFi dropout (ConnectError → Watcher returns [])
  - Ollama crash mid-generation (OllamaError caught in ReportGenerator)
  - Unexpected exceptions in any stage (caught at loop level)

Hang prevention
---------------
All HTTP calls in child classes have explicit timeouts (connect + read).
The daemon loop itself uses time.sleep() in small increments with a
shutdown flag check so Ctrl+C / stop_daemon() responds within 1 second
rather than waiting out a full poll interval.

Backoff on repeated errors
---------------------------
If N consecutive poll cycles all fail (RPi unreachable), the inter-poll
sleep doubles up to MAX_BACKOFF_S before resetting. This prevents log
spam during extended outages.

References
----------
- FlightWatcher:     ground_station/flight_watcher.py
- ArtifactFetcher:   ground_station/artifact_fetcher.py
- ReportGenerator:   ground_station/report_generator.py
- ReportPublisher:   ground_station/report_publisher.py
- OllamaClient:      ground_station/ollama_client.py
- ProcessManager:    ground_station/process_manager.py (manages this process)
"""

import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Optional

from ground_station.artifact_fetcher import ArtifactFetcher
from ground_station.flight_watcher   import FlightWatcher
from ground_station.ollama_client    import OllamaClient, OllamaError
from ground_station.report_generator import ReportGenerator
from ground_station.report_publisher import ReportPublisher
from ground_station.db               import FlightDatabase

log = logging.getLogger(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────

def _poll_interval() -> float:
    try:
        return float(os.environ.get("DRONEPI_POLL_INTERVAL_S", "20"))
    except ValueError:
        return 20.0

CONSECUTIVE_FAIL_THRESHOLD = 3
MAX_BACKOFF_S              = 300
SLEEP_TICK_S               = 1.0


# ══════════════════════════════════════════════════════════════════════════════
# FlightReportDaemon
# ══════════════════════════════════════════════════════════════════════════════

class FlightReportDaemon:
    """
    Background daemon that polls for new flights and generates LLM reports.

    Designed to run as a long-lived background process spawned by
    ProcessManager. Controlled via the stop() method or SIGTERM.

    Parameters
    ----------
    ollama_client : OllamaClient | None
        If provided, shared across all report generations. If None, a new
        client is created using env var configuration.
    """

    def __init__(self, ollama_client: Optional[OllamaClient] = None) -> None:
        self._client    = ollama_client or OllamaClient()
        self._watcher   = FlightWatcher()
        self._fetcher   = ArtifactFetcher()
        self._generator = ReportGenerator(ollama_client=self._client)
        self._publisher = ReportPublisher()

        self._stop_event        = threading.Event()
        self._consecutive_fails = 0
        self._current_backoff   = _poll_interval()

    # ── Public ────────────────────────────────────────────────────────────────

    def run_forever(self) -> None:
        """
        Main loop. Blocks until stop() is called or process is terminated.
        Called by cli.py _daemon-worker subcommand.
        """
        log.info("[Daemon] Starting FlightReportDaemon.")
        log.info(f"[Daemon] Poll interval: {_poll_interval()}s")
        log.info(f"[Daemon] Ollama model: {self._client.model}")

        try:
            self._client.ensure_ready()
        except OllamaError as exc:
            log.warning(
                f"[Daemon] Ollama not ready at startup: {exc} "
                "— will retry on first report generation."
            )

        while not self._stop_event.is_set():
            self._poll_cycle()
            self._sleep_interruptible(self._current_backoff)

        log.info("[Daemon] Stop event received — exiting.")

    def stop(self) -> None:
        """Signal the daemon to stop after the current sleep tick."""
        self._stop_event.set()

    def process_one(self, session_id: str) -> bool:
        """
        Process a single flight by ID. Used by CLI `replay` subcommand.
        Returns True if report was generated (Ollama success), False otherwise.
        """
        log.info(f"[Daemon] Replaying {session_id}...")
        return self._process_flight(session_id)

    # ── Private — poll cycle ──────────────────────────────────────────────────

    def _poll_cycle(self) -> None:
        """Single poll iteration. All exceptions are caught — never propagates."""
        try:
            new_ids = self._watcher.poll()
            if not new_ids:
                self._on_success()
                return

            for session_id in new_ids:
                if self._stop_event.is_set():
                    log.info("[Daemon] Stop requested mid-batch — exiting loop.")
                    return
                self._process_flight(session_id)
                self._watcher.mark_processed(session_id)

            self._on_success()

        except Exception as exc:
            self._on_failure(exc)

    def _process_flight(self, session_id: str) -> bool:
        """
        Full pipeline for one flight: fetch → generate → publish → persist to DB.
        Returns True if a report was generated and published.
        """
        log.info(f"[Daemon] Processing flight: {session_id}")

        # ── Fetch artifacts ───────────────────────────────────────────────────
        try:
            artifact_paths = self._fetcher.fetch(session_id)
        except Exception as exc:
            log.error(f"[Daemon] Artifact fetch failed for {session_id}: {exc}")
            return False

        # ── Build structured intermediate ─────────────────────────────────────
        try:
            structured = self._generator.build_structured(session_id, artifact_paths)
        except Exception as exc:
            log.error(f"[Daemon] Structured data build failed for {session_id}: {exc}")
            return False

        # ── Read extra fields from flight_record.json (not in structured) ─────
        # flight_type, script_name, and stage_timings come from artifacts
        # that aren't surfaced through the ReportGenerator structured dict.
        flight_type      = None
        script_name      = None
        stage_timings_s  = None
        bottleneck_stage = None

        fr_path = artifact_paths.get("flight_record_json")
        if fr_path and Path(fr_path).exists():
            try:
                fr = json.loads(Path(fr_path).read_text(encoding="utf-8"))
                flight_type = fr.get("flight_type")
                script_name = fr.get("script_name")
            except Exception as exc:
                log.debug(f"[Daemon] flight_record.json extra fields: {exc}")

        meta_path = artifact_paths.get("metadata_json")
        if meta_path and Path(meta_path).exists():
            try:
                md = json.loads(Path(meta_path).read_text(encoding="utf-8"))
                stage_timings_s  = md.get("stage_timings_s")
                bottleneck_stage = md.get("bottleneck_stage")
            except Exception as exc:
                log.debug(f"[Daemon] metadata.json stage timings: {exc}")

        # ── Generate report ───────────────────────────────────────────────────
        try:
            report_md = self._generator._call_ollama(structured)
        except Exception as exc:
            log.error(f"[Daemon] Report generation failed for {session_id}: {exc}")
            return False

        if not report_md:
            log.warning(f"[Daemon] Empty report for {session_id} — skipping upload.")
            return False

        # ── Publish ───────────────────────────────────────────────────────────
        try:
            self._publisher.publish(session_id, report_md)
        except Exception as exc:
            log.error(f"[Daemon] Publish failed for {session_id}: {exc}")
            # Non-fatal — report saved locally, still write to DB.

        # ── Persist to local DB ───────────────────────────────────────────────
        try:
            db = FlightDatabase()
            db.upsert_flight(
                session_id,
                arm_time_iso      = structured.get("start_time"),
                duration_s        = structured.get("duration_s"),
                end_reason        = structured.get("end_reason"),
                flight_type       = flight_type,
                script_name       = script_name,
                slam_points       = structured["slam"].get("point_count_final"),
                drift_estimate_m  = structured["slam"].get("drift_estimate_m"),
                loop_closures     = structured["slam"].get("loop_closures"),
                cpu_temp_max_c    = structured["rpi_health"].get("cpu_temp_max_c"),
                throttling_events = structured["rpi_health"].get("throttling_events"),
                memory_peak_pct   = structured["rpi_health"].get("memory_peak_pct"),
                battery_start_v   = structured["battery"].get("start_v"),
                battery_end_v     = structured["battery"].get("end_v"),
                battery_min_v     = structured["battery"].get("min_v"),
                vertex_count      = structured["mesh"].get("vertex_count"),
                face_count        = structured["mesh"].get("face_count"),
                stage_timings_s   = stage_timings_s,
                bottleneck_stage  = bottleneck_stage,
                anomaly_count     = len(structured.get("anomalies", [])),
                anomalies         = structured.get("anomalies", []),
                has_report        = True,
                report_md         = report_md,
            )
            log.info(f"[Daemon] DB upsert complete for {session_id}")
        except Exception as exc:
            log.error(f"[Daemon] DB upsert failed for {session_id}: {exc}")
            # Non-fatal — report still exists locally and on RPi.

        return True

    # ── Private — backoff management ──────────────────────────────────────────

    def _on_success(self) -> None:
        if self._consecutive_fails > 0:
            log.info("[Daemon] Connectivity restored — resetting backoff.")
        self._consecutive_fails = 0
        self._current_backoff   = _poll_interval()

    def _on_failure(self, exc: Exception) -> None:
        self._consecutive_fails += 1
        self._current_backoff = min(
            _poll_interval() * (2 ** self._consecutive_fails),
            MAX_BACKOFF_S,
        )
        log.warning(
            f"[Daemon] Poll cycle failed (consecutive={self._consecutive_fails}): "
            f"{exc}. Next poll in {self._current_backoff:.0f}s."
        )

    def _sleep_interruptible(self, duration_s: float) -> None:
        """
        Sleep for duration_s, checking stop_event every SLEEP_TICK_S.
        Ensures stop() response latency <= 1 second.
        """
        deadline = time.monotonic() + duration_s
        while not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(SLEEP_TICK_S, remaining))
