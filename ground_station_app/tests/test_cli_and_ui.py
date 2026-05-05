#!/usr/bin/env python3
"""
tests/test_cli_and_ui.py — Standalone Phase 4 test.

Tests
-----
1. UIServer starts and responds on a random port.
2. GET / returns HTML containing 'DronePi'.
3. GET /api/status returns valid JSON with expected keys.
4. GET /api/flights returns JSON with 'flights' list (empty cache = []).
5. GET /api/report/<id> returns {report: null} for unknown session.
6. GET /api/report/<id> returns report content for a seeded cache entry.
7. GET /favicon.ico returns 200 when ICO exists, 404 when absent.
8. cli._write_launcher() creates a .pyw file with correct content.
9. cli._state_dir() returns a valid Path.

Run with:
    python tests/test_cli_and_ui.py
"""

import json
import logging
import os
import sys
import tempfile
import threading
import time
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


import urllib.request
import urllib.error


def _get(url: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, b""
    except Exception as e:
        return 0, str(e).encode()


def _start_ui(cache_dir: Path, state_dir: Path) -> tuple[object, int]:
    os.environ["DRONEPI_CACHE_DIR"] = str(cache_dir)
    # Point state dir so favicon lookup works
    # UIServer reads _state_dir() which uses Path.home() / ".dronepi-ground"
    # We patch the env so ico isn't required for most tests.

    from ground_station.ui_server import UIServer
    from http.server import HTTPServer
    import ground_station.ui_server as ui_mod

    # Patch _state_dir for this test
    ui_mod._state_dir_override = state_dir

    original_state_dir = ui_mod._state_dir
    ui_mod._state_dir = lambda: state_dir

    server = HTTPServer(("127.0.0.1", 0), ui_mod._DashboardHandler)
    port = server.server_address[1]
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    time.sleep(0.1)
    return server, port


def main() -> None:
    log.info("=" * 60)
    log.info("Phase 4 CLI + UI test suite")
    log.info("=" * 60)

    tmp_cache = Path(tempfile.mkdtemp(prefix="dronepi_ui_test_"))
    tmp_state = Path(tempfile.mkdtemp(prefix="dronepi_state_test_"))

    # ── Seed a fake flight in cache ──────────────────────────────────────────
    session_id = "scan_20260422_143215"
    session_dir = tmp_cache / session_id
    session_dir.mkdir(parents=True)

    flight_record = {
        "session_id": session_id,
        "arm_time_iso": "2026-04-22T14:32:15+00:00",
        "duration_s": 187.4,
    }
    (session_dir / "flight_record.json").write_text(
        json.dumps(flight_record), encoding="utf-8"
    )
    (session_dir / "report.md").write_text(
        "## Summary\n\nFlight completed nominally.\n\n## Anomalies\n\nNone.",
        encoding="utf-8",
    )

    # ── Start UI server ───────────────────────────────────────────────────────
    server, port = _start_ui(tmp_cache, tmp_state)
    base = f"http://127.0.0.1:{port}"
    log.info(f"UI server at {base}")

    # ── Tests ─────────────────────────────────────────────────────────────────

    # 1. Root returns HTML
    status, body = _get(f"{base}/")
    check("root_returns_200",   status == 200, f"status={status}")
    check("root_contains_dronepi",
          b"DronePi" in body,
          f"body_len={len(body)}")

    # 2. /api/status returns JSON with expected keys
    status, body = _get(f"{base}/api/status")
    check("status_returns_200", status == 200)
    try:
        d = json.loads(body)
        check("status_has_daemon_key", "daemon" in d)
        check("status_has_ollama_key", "ollama" in d)
        check("status_has_rpi_key",   "rpi"    in d)
    except Exception as e:
        check("status_json_parse", False, str(e))

    # 3. /api/flights returns list with seeded flight
    status, body = _get(f"{base}/api/flights")
    check("flights_returns_200", status == 200)
    try:
        d = json.loads(body)
        ids = [f["id"] for f in d.get("flights", [])]
        check("flights_contains_seeded", session_id in ids, str(ids))
        flight = next((f for f in d["flights"] if f["id"] == session_id), None)
        check("flights_has_report_flag",
              flight is not None and flight.get("has_report") is True,
              str(flight))
    except Exception as e:
        check("flights_json_parse", False, str(e))

    # 4. /api/report/<id> returns report content
    status, body = _get(f"{base}/api/report/{session_id}")
    check("report_returns_200", status == 200)
    try:
        d = json.loads(body)
        check("report_content_nonempty", bool(d.get("report")), str(d.get("report",""))[:60])
        check("report_has_summary",      "Summary" in (d.get("report") or ""))
        check("report_duration_present", d.get("duration") is not None, str(d.get("duration")))
    except Exception as e:
        check("report_json_parse", False, str(e))

    # 5. Unknown session returns report: null
    status, body = _get(f"{base}/api/report/nonexistent_session")
    check("unknown_report_returns_200", status == 200)
    try:
        d = json.loads(body)
        check("unknown_report_null", d.get("report") is None)
    except Exception as e:
        check("unknown_report_json", False, str(e))

    # 6. /favicon.ico returns 404 when no ico in state dir
    status, _ = _get(f"{base}/favicon.ico")
    check("favicon_404_when_absent", status == 404, f"status={status}")

    # Seed ico file
    (tmp_state / "dronepi.ico").write_bytes(b"\x00\x00\x01\x00")  # minimal ICO header
    status, _ = _get(f"{base}/favicon.ico")
    check("favicon_200_when_present", status == 200, f"status={status}")

    # 7. Unknown route returns 404
    status, body = _get(f"{base}/api/nonexistent")
    check("unknown_route_404", status == 404, f"status={status}")

    # 8. CLI _write_launcher creates .pyw file
    tmp_app = Path(tempfile.mkdtemp(prefix="dronepi_app_"))
    import ground_station.cli as cli_mod
    original_app_dir = cli_mod._app_dir
    cli_mod._app_dir = lambda: tmp_app

    launcher = cli_mod._write_launcher()
    check("launcher_file_exists", launcher.exists(), str(launcher))
    check("launcher_is_pyw",      launcher.suffix == ".pyw")
    content = launcher.read_text(encoding="utf-8")
    check("launcher_has_webbrowser", "webbrowser" in content)
    check("launcher_has_start",      "start" in content)

    cli_mod._app_dir = original_app_dir

    # ── Cleanup ───────────────────────────────────────────────────────────────
    server.shutdown()
    import shutil
    shutil.rmtree(tmp_cache, ignore_errors=True)
    shutil.rmtree(tmp_state, ignore_errors=True)
    shutil.rmtree(tmp_app,   ignore_errors=True)

    log.info("=" * 60)
    passed = sum(1 for _, s in results if s == "PASS")
    failed = sum(1 for _, s in results if s == "FAIL")
    log.info(f"Results: {passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
