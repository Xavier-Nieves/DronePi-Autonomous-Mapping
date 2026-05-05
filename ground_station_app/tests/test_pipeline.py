#!/usr/bin/env python3
"""
tests/test_pipeline.py — Standalone Phase 3 pipeline test.

Covers: FlightWatcher, ArtifactFetcher, ReportGenerator,
        ReportPublisher, FlightReportDaemon.

Mock infrastructure
-------------------
A single mock HTTP server handles all RPi endpoints and Ollama endpoints
on two separate ports. No real RPi or Ollama instance required.

Run with:
    python tests/test_pipeline.py

Exits 0 on all pass, 1 on any failure. Takes ~5 seconds.
"""

import json
import logging
import os
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

results: list[tuple[str, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    status = "PASS" if condition else "FAIL"
    results.append((name, status))
    marker = "  [PASS]" if condition else "  [FAIL]"
    log.info(f"{marker}  {name}" + (f" — {detail}" if detail else ""))


# ══════════════════════════════════════════════════════════════════════════════
# Mock RPi server
# ══════════════════════════════════════════════════════════════════════════════

MOCK_SESSION = "scan_20260422_143215"

MOCK_FLIGHTS_RESPONSE = {
    "flights": [
        {"id": MOCK_SESSION, "session_id": MOCK_SESSION, "status": "complete"},
        {"id": "scan_20260421_090000", "session_id": "scan_20260421_090000", "status": "partial"},
    ],
    "total_flights": 2,
    "complete_count": 1,
    "total_size_gb": "1.2",
}

MOCK_BAG_SUMMARY = (
    "point_count_raw,point_count_final,drift_estimate_m,loop_closures,"
    "mls_iterations,sor_outliers_removed,processing_duration_s,bag_path\n"
    "2450000,1200000,0.18,3,5,18450,47.3,/mnt/ssd/rosbags/scan_20260422_143215\n"
)

MOCK_HEALTH_LOG = (
    "timestamp,cpu_percent,cpu_temp,mem_percent,throttled,throttle_bits\n"
    "1714000000.0,42.1,63.5,52.3,False,0x0\n"
    "1714000002.0,55.3,65.2,53.1,False,0x0\n"
    "1714000004.0,88.7,71.4,64.0,True,0x50004\n"
)

MOCK_METADATA = json.dumps({
    "id": MOCK_SESSION,
    "status": "complete",
    "vertex_count": 184231,
    "face_count": 362188,
    "processed_at": "2026-04-22 14:45",
})

MOCK_FLIGHT_RECORD = json.dumps({
    "flight_number": 12,
    "session_id": MOCK_SESSION,
    "script_name": "main.py",
    "flight_type": "AUTONOMOUS",
    "arm_time_iso": "2026-04-22T14:32:15+00:00",
    "duration_s": 187.4,
    "end_reason": "AUTO.LAND",
})

_put_received: dict = {}   # captures PUT body per session


class _RpiHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/api/flights":
            self._json(MOCK_FLIGHTS_RESPONSE)
        elif self.path == f"/rosbags/{MOCK_SESSION}/bag_summary.csv":
            self._text(MOCK_BAG_SUMMARY, "text/csv")
        elif self.path == f"/rosbags/{MOCK_SESSION}/health_log.csv":
            self._text(MOCK_HEALTH_LOG, "text/csv")
        elif self.path == f"/rosbags/{MOCK_SESSION}/metadata.json":
            self._text(MOCK_METADATA, "application/json")
        elif self.path == f"/rosbags/{MOCK_SESSION}/flight_record.json":
            self._text(MOCK_FLIGHT_RECORD, "application/json")
        else:
            self.send_response(404)
            self.end_headers()

    def do_PUT(self) -> None:
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8")
        # Extract session from path: /rosbags/<session>/report.md
        parts = self.path.strip("/").split("/")
        if len(parts) >= 3:
            session = parts[1]
            _put_received[session] = body
        self.send_response(200)
        self.end_headers()

    def _json(self, obj: dict) -> None:
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _text(self, text: str, mime: str) -> None:
        body = text.encode()
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args) -> None:
        pass


# ══════════════════════════════════════════════════════════════════════════════
# Mock Ollama server
# ══════════════════════════════════════════════════════════════════════════════

class _OllamaHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/api/tags":
            body = json.dumps({"models": [{"name": "qwen2.5:7b"}]}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", 0))
        self.rfile.read(length)
        body = json.dumps({
            "choices": [{"message": {"content": (
                "## Summary\nFlight completed nominally.\n\n"
                "## Anomalies\nNone detected.\n\n"
                "## SLAM Quality\n1.2M points, drift 0.18 m.\n\n"
                "## GPS Quality\nHDOP nominal.\n\n"
                "## System Health\nPeak temp 71.4°C, 1 throttle event.\n\n"
                "## Recommendations\nReview throttle event at t=4s."
            )}}]
        }).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args) -> None:
        pass


def _start(handler_cls) -> tuple[HTTPServer, str]:
    srv = HTTPServer(("127.0.0.1", 0), handler_cls)
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{port}"


# ══════════════════════════════════════════════════════════════════════════════
# Tests
# ══════════════════════════════════════════════════════════════════════════════

def test_flight_watcher(rpi_url: str) -> None:
    """FlightWatcher.poll() returns only complete, unprocessed flight IDs."""
    from ground_station.flight_watcher import FlightWatcher

    tmp_state = Path(tempfile.mktemp(suffix=".json"))
    os.environ["DRONEPI_RPI_HOST"] = "127.0.0.1"
    os.environ["DRONEPI_RPI_PORT"] = rpi_url.split(":")[-1]
    os.environ["DRONEPI_STATE_FILE"] = str(tmp_state)

    w = FlightWatcher()
    new_ids = w.poll()

    check("watcher_returns_complete_only",
          MOCK_SESSION in new_ids and "scan_20260421_090000" not in new_ids,
          str(new_ids))
    check("watcher_returns_list", isinstance(new_ids, list))

    # Mark processed and verify it's excluded on next poll
    w.mark_processed(MOCK_SESSION)
    new_ids2 = w.poll()
    check("watcher_excludes_processed", MOCK_SESSION not in new_ids2, str(new_ids2))

    # State file persists across instances
    w2 = FlightWatcher()
    check("watcher_state_persists", w2.is_processed(MOCK_SESSION))

    tmp_state.unlink(missing_ok=True)


def test_artifact_fetcher(rpi_url: str) -> None:
    """ArtifactFetcher downloads all available artifacts and returns correct paths."""
    from ground_station.artifact_fetcher import ArtifactFetcher

    tmp_cache = Path(tempfile.mkdtemp(prefix="dronepi_fetch_test_"))
    os.environ["DRONEPI_CACHE_DIR"] = str(tmp_cache)

    fetcher = ArtifactFetcher()
    paths = fetcher.fetch(MOCK_SESSION)

    check("fetcher_bag_summary_downloaded",
          paths["bag_summary_csv"] is not None and paths["bag_summary_csv"].exists(),
          str(paths["bag_summary_csv"]))
    check("fetcher_health_log_downloaded",
          paths["health_log_csv"] is not None and paths["health_log_csv"].exists(),
          str(paths["health_log_csv"]))
    check("fetcher_metadata_downloaded",
          paths["metadata_json"] is not None and paths["metadata_json"].exists(),
          str(paths["metadata_json"]))
    check("fetcher_flight_record_downloaded",
          paths["flight_record_json"] is not None and paths["flight_record_json"].exists(),
          str(paths["flight_record_json"]))

    # Cache hit — second fetch returns same paths without re-downloading
    paths2 = fetcher.fetch(MOCK_SESSION)
    check("fetcher_cache_hit",
          paths2["bag_summary_csv"] == paths["bag_summary_csv"])

    import shutil
    shutil.rmtree(tmp_cache, ignore_errors=True)


def test_report_generator_structured(rpi_url: str, ollama_url: str) -> None:
    """ReportGenerator.build_structured() assembles correct intermediate dict."""
    from ground_station.artifact_fetcher import ArtifactFetcher
    from ground_station.ollama_client import OllamaClient
    from ground_station.report_generator import ReportGenerator

    tmp_cache = Path(tempfile.mkdtemp(prefix="dronepi_gen_test_"))
    os.environ["DRONEPI_CACHE_DIR"] = str(tmp_cache)
    os.environ["DRONEPI_OLLAMA_HOST"] = ollama_url

    fetcher = ArtifactFetcher()
    paths   = fetcher.fetch(MOCK_SESSION)

    client    = OllamaClient(host=ollama_url, model="qwen2.5:7b")
    generator = ReportGenerator(ollama_client=client)
    structured = generator.build_structured(MOCK_SESSION, paths)

    check("structured_flight_id",   structured["flight_id"] == MOCK_SESSION)
    check("structured_slam_points", structured["slam"]["point_count_final"] == 1_200_000,
          str(structured["slam"]["point_count_final"]))
    check("structured_slam_drift",  abs(structured["slam"]["drift_estimate_m"] - 0.18) < 0.01,
          str(structured["slam"]["drift_estimate_m"]))
    check("structured_health_temp", structured["rpi_health"]["cpu_temp_max_c"] > 0,
          str(structured["rpi_health"]["cpu_temp_max_c"]))
    check("structured_mesh_verts",  structured["mesh"]["vertex_count"] == 184_231,
          str(structured["mesh"]["vertex_count"]))
    check("structured_json_safe",   bool(json.dumps(structured)))

    import shutil
    shutil.rmtree(tmp_cache, ignore_errors=True)


def test_report_generator_full(rpi_url: str, ollama_url: str) -> None:
    """ReportGenerator.generate() returns a non-empty markdown string."""
    from ground_station.artifact_fetcher import ArtifactFetcher
    from ground_station.ollama_client import OllamaClient
    from ground_station.report_generator import ReportGenerator

    tmp_cache = Path(tempfile.mkdtemp(prefix="dronepi_genfull_test_"))
    os.environ["DRONEPI_CACHE_DIR"] = str(tmp_cache)

    fetcher   = ArtifactFetcher()
    paths     = fetcher.fetch(MOCK_SESSION)
    client    = OllamaClient(host=ollama_url, model="qwen2.5:7b")
    generator = ReportGenerator(ollama_client=client)
    report    = generator.generate(MOCK_SESSION, paths)

    check("generate_returns_string",   isinstance(report, str))
    check("generate_nonempty",         bool(report and report.strip()))
    check("generate_has_summary",      "Summary" in (report or ""),
          (report or "")[:80])

    import shutil
    shutil.rmtree(tmp_cache, ignore_errors=True)


def test_report_publisher(rpi_url: str) -> None:
    """ReportPublisher saves locally and PUTs to RPi."""
    from ground_station.report_publisher import ReportPublisher

    tmp_cache = Path(tempfile.mkdtemp(prefix="dronepi_pub_test_"))
    os.environ["DRONEPI_CACHE_DIR"] = str(tmp_cache)

    publisher  = ReportPublisher()
    report_md  = "## Test Report\n\nThis is a test."
    uploaded   = publisher.publish(MOCK_SESSION, report_md)

    # Local save always works
    local_path = publisher.local_report_path(MOCK_SESSION)
    check("publisher_local_saved",   local_path.exists())
    check("publisher_local_content", local_path.read_text() == report_md)

    # RPi PUT received
    check("publisher_rpi_uploaded",  uploaded, "PUT returned 200")
    check("publisher_rpi_content",
          _put_received.get(MOCK_SESSION, "").strip() == report_md.strip(),
          str(_put_received.get(MOCK_SESSION, ""))[:60])

    import shutil
    shutil.rmtree(tmp_cache, ignore_errors=True)


def test_daemon_process_one(rpi_url: str, ollama_url: str) -> None:
    """FlightReportDaemon.process_one() runs the full pipeline for one flight."""
    from ground_station.ollama_client import OllamaClient
    from ground_station.orchestrator import FlightReportDaemon

    tmp_cache = Path(tempfile.mkdtemp(prefix="dronepi_daemon_test_"))
    tmp_state = Path(tempfile.mktemp(suffix=".json"))
    os.environ["DRONEPI_CACHE_DIR"] = str(tmp_cache)
    os.environ["DRONEPI_STATE_FILE"] = str(tmp_state)

    client = OllamaClient(host=ollama_url, model="qwen2.5:7b")
    daemon = FlightReportDaemon(ollama_client=client)

    result = daemon.process_one(MOCK_SESSION)
    check("daemon_process_one_ok", result is True, str(result))

    # Report must be saved locally
    from ground_station.report_publisher import ReportPublisher
    local = ReportPublisher().local_report_path(MOCK_SESSION)
    check("daemon_report_exists_locally", local.exists())

    tmp_state.unlink(missing_ok=True)
    import shutil
    shutil.rmtree(tmp_cache, ignore_errors=True)


def test_daemon_stop() -> None:
    """FlightReportDaemon.stop() terminates run_forever() within 2 seconds."""
    from ground_station.ollama_client import OllamaClient
    from ground_station.orchestrator import FlightReportDaemon

    # Point to unreachable RPi so poll cycles are instant no-ops
    os.environ["DRONEPI_RPI_HOST"] = "127.0.0.1"
    os.environ["DRONEPI_RPI_PORT"] = "19997"
    os.environ["DRONEPI_POLL_INTERVAL_S"] = "60"

    client = OllamaClient(host="http://127.0.0.1:19996", model="qwen2.5:7b")
    daemon = FlightReportDaemon(ollama_client=client)

    t = threading.Thread(target=daemon.run_forever, daemon=True)
    t.start()
    time.sleep(0.5)
    daemon.stop()
    t.join(timeout=3.0)

    check("daemon_stops_cleanly", not t.is_alive(),
          "thread still alive after stop()" if t.is_alive() else "stopped OK")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    log.info("=" * 60)
    log.info("Phase 3 pipeline test suite")
    log.info("=" * 60)

    rpi_srv,    rpi_url    = _start(_RpiHandler)
    ollama_srv, ollama_url = _start(_OllamaHandler)
    time.sleep(0.1)

    log.info(f"Mock RPi    : {rpi_url}")
    log.info(f"Mock Ollama : {ollama_url}")

    # Point all env vars at mock servers
    os.environ["DRONEPI_RPI_HOST"]      = "127.0.0.1"
    os.environ["DRONEPI_RPI_PORT"]      = rpi_url.split(":")[-1]
    os.environ["DRONEPI_OLLAMA_HOST"]   = ollama_url
    os.environ["DRONEPI_OLLAMA_MODEL"]  = "qwen2.5:7b"
    os.environ["DRONEPI_POLL_INTERVAL_S"] = "5"

    test_flight_watcher(rpi_url)
    test_artifact_fetcher(rpi_url)
    test_report_generator_structured(rpi_url, ollama_url)
    test_report_generator_full(rpi_url, ollama_url)
    test_report_publisher(rpi_url)
    test_daemon_process_one(rpi_url, ollama_url)
    test_daemon_stop()

    rpi_srv.shutdown()
    ollama_srv.shutdown()

    log.info("=" * 60)
    passed = sum(1 for _, s in results if s == "PASS")
    failed = sum(1 for _, s in results if s == "FAIL")
    log.info(f"Results: {passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
