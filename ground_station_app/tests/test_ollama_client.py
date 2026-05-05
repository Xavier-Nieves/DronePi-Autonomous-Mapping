#!/usr/bin/env python3
"""
tests/test_ollama_client.py — Standalone test for OllamaClient.

Tests
-----
1. is_reachable() returns False when no server is present.
2. is_reachable() returns True against a live mock server.
3. model_available() correctly identifies a present vs absent model.
4. chat() returns the expected response text from the mock server.
5. chat() retries on 503 and eventually raises OllamaError.
6. chat() raises OllamaError immediately on 400 (no retry).
7. chat() raises OllamaError on empty response content.
8. ensure_ready() raises OllamaError when server unreachable.

The mock server runs in a daemon thread using http.server (stdlib only).
No real Ollama installation is required for this test suite.

Run with:
    python tests/test_ollama_client.py

Exits with code 0 (all pass) or 1.
"""

import json
import logging
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).parent.parent))

from ground_station.ollama_client import OllamaClient, OllamaError

logging.basicConfig(
    level=logging.DEBUG,
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


# ── Mock Ollama HTTP server ────────────────────────────────────────────────────

class _MockHandler(BaseHTTPRequestHandler):
    """
    Minimal mock of the Ollama API surface used by OllamaClient:
      GET  /api/tags              → model list
      POST /v1/chat/completions   → chat response or configured error
    """

    # Class-level config — mutated per-test
    tags_response: dict        = {"models": [{"name": "qwen2.5:7b"}]}
    chat_status: int           = 200
    chat_body: Any             = None   # None → auto-generate success body
    force_fail_count: int      = 0      # how many times to return 503 before success
    _fail_counter: int         = 0

    def do_GET(self) -> None:
        if self.path == "/api/tags":
            body = json.dumps(self.tags_response).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self) -> None:
        content_len = int(self.headers.get("Content-Length", 0))
        self.rfile.read(content_len)  # consume body

        if self.path == "/api/pull":
            # Minimal pull response — stream a single status line
            self.send_response(200)
            self.send_header("Content-Type", "application/x-ndjson")
            self.end_headers()
            self.wfile.write(b'{"status":"success"}\n')
            return

        if self.path != "/v1/chat/completions":
            self.send_response(404)
            self.end_headers()
            return

        # Simulate transient failures before success
        if _MockHandler._fail_counter < _MockHandler.force_fail_count:
            _MockHandler._fail_counter += 1
            self.send_response(503)
            body = b'{"error":"service temporarily unavailable"}'
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        status = _MockHandler.chat_status
        if _MockHandler.chat_body is not None:
            body = json.dumps(_MockHandler.chat_body).encode()
        else:
            body = json.dumps({
                "choices": [{
                    "message": {"content": "## Flight Report\n\nAll systems nominal."}
                }]
            }).encode()

        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args) -> None:  # silence access log
        pass


def _start_mock_server() -> tuple[HTTPServer, str]:
    """Start mock server on a random free port. Returns (server, base_url)."""
    server = HTTPServer(("127.0.0.1", 0), _MockHandler)
    port = server.server_address[1]
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, f"http://127.0.0.1:{port}"


def _reset_mock() -> None:
    """Reset mock server state between tests."""
    _MockHandler.tags_response = {"models": [{"name": "qwen2.5:7b"}]}
    _MockHandler.chat_status   = 200
    _MockHandler.chat_body     = None
    _MockHandler.force_fail_count = 0
    _MockHandler._fail_counter = 0


# ── Tests ─────────────────────────────────────────────────────────────────────

def test_unreachable() -> None:
    """is_reachable() returns False for a port where nothing is listening."""
    client = OllamaClient(host="http://127.0.0.1:19999", model="qwen2.5:7b")
    result = client.is_reachable()
    check("is_reachable_false_when_down", not result)


def test_reachable(base_url: str) -> None:
    """is_reachable() returns True against the mock server."""
    _reset_mock()
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    result = client.is_reachable()
    check("is_reachable_true", result)


def test_model_available_present(base_url: str) -> None:
    """model_available() returns True when model appears in /api/tags."""
    _reset_mock()
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    result = client.model_available()
    check("model_available_present", result)


def test_model_available_absent(base_url: str) -> None:
    """model_available() returns False when model absent from /api/tags."""
    _reset_mock()
    _MockHandler.tags_response = {"models": [{"name": "llama3.1:8b"}]}
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    result = client.model_available()
    check("model_available_absent", not result)


def test_chat_success(base_url: str) -> None:
    """chat() returns assistant content on HTTP 200."""
    _reset_mock()
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    text = client.chat("You are a test assistant.", "Say hello.")
    check("chat_success", "Flight Report" in text, f"response={text[:60]}")


def test_chat_retry_on_503(base_url: str) -> None:
    """chat() retries on 503 and succeeds once the mock stops failing."""
    _reset_mock()
    _MockHandler.force_fail_count = 2   # fail twice, succeed on 3rd attempt
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    try:
        text = client.chat("system", "user")
        check("chat_retry_503_succeeds", "Flight Report" in text)
    except OllamaError as exc:
        check("chat_retry_503_succeeds", False, str(exc))


def test_chat_fail_after_max_retries(base_url: str) -> None:
    """chat() raises OllamaError after MAX_RETRIES exhausted."""
    _reset_mock()
    _MockHandler.force_fail_count = 99   # always fail
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    try:
        client.chat("system", "user")
        check("chat_fails_after_max_retries", False, "should have raised OllamaError")
    except OllamaError as exc:
        check("chat_fails_after_max_retries", True, str(exc)[:80])


def test_chat_400_no_retry(base_url: str) -> None:
    """chat() raises OllamaError immediately on 400 without retrying."""
    _reset_mock()
    _MockHandler.chat_status = 400
    _MockHandler.chat_body   = {"error": "model not found"}
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    try:
        client.chat("system", "user")
        check("chat_400_no_retry", False, "should have raised")
    except OllamaError as exc:
        check("chat_400_no_retry", True, str(exc)[:80])


def test_chat_empty_response(base_url: str) -> None:
    """chat() raises OllamaError when assistant content is empty string."""
    _reset_mock()
    _MockHandler.chat_body = {"choices": [{"message": {"content": ""}}]}
    client = OllamaClient(host=base_url, model="qwen2.5:7b")
    try:
        client.chat("system", "user")
        check("chat_empty_response", False, "should have raised")
    except OllamaError as exc:
        check("chat_empty_response", True, str(exc)[:80])


def test_ensure_ready_unreachable() -> None:
    """ensure_ready() raises OllamaError when server is down."""
    client = OllamaClient(host="http://127.0.0.1:19998", model="qwen2.5:7b")
    try:
        client.ensure_ready()
        check("ensure_ready_unreachable", False, "should have raised")
    except OllamaError as exc:
        check("ensure_ready_unreachable", True, str(exc)[:80])


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    log.info("=" * 60)
    log.info("OllamaClient test suite")
    log.info("=" * 60)

    server, base_url = _start_mock_server()
    log.info(f"Mock Ollama server running at {base_url}")

    # Give the server a moment to bind
    time.sleep(0.1)

    test_unreachable()
    test_reachable(base_url)
    test_model_available_present(base_url)
    test_model_available_absent(base_url)
    test_chat_success(base_url)
    test_chat_retry_on_503(base_url)
    test_chat_fail_after_max_retries(base_url)
    test_chat_400_no_retry(base_url)
    test_chat_empty_response(base_url)
    test_ensure_ready_unreachable()

    server.shutdown()

    log.info("=" * 60)
    passed = sum(1 for _, s in results if s == "PASS")
    failed = sum(1 for _, s in results if s == "FAIL")
    log.info(f"Results: {passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
