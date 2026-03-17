#!/usr/bin/env python3
"""
drone_mesh_server.py — CORS-enabled HTTP file server for the drone mesh viewer.

Serves .ply files, latest.json manifest, and meshview.html from
/mnt/ssd/maps/ on port 8080. Zero external dependencies — Python
standard library only.

Run directly:
    python3 /mnt/ssd/maps/serve.py

Or via systemd (see drone-mesh-server.service).

On laptop browser:
    http://10.42.0.1:8080/meshview.html
"""

import os
import sys
import logging
from http.server import SimpleHTTPRequestHandler, HTTPServer

# ── config ────────────────────────────────────────────────────────────────────

SERVE_DIR = "/mnt/ssd/maps"
PORT      = 8080
BIND_ADDR = "0.0.0.0"   # all interfaces — reachable over hotspot

# MIME types not in Python's default map
EXTRA_MIME = {
    ".ply":  "application/octet-stream",
    ".json": "application/json",
    ".html": "text/html; charset=utf-8",
    ".js":   "application/javascript",
}


# ── handler ───────────────────────────────────────────────────────────────────

class CORSHandler(SimpleHTTPRequestHandler):
    """SimpleHTTPRequestHandler extended with CORS headers and PLY MIME type."""

    def end_headers(self):
        # Allow any browser origin — safe for local LAN with no auth cookies
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        # Prevent latest.json from being cached — always return fresh manifest
        if self.path.endswith("latest.json"):
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        super().end_headers()

    def do_OPTIONS(self):
        """Handle CORS preflight requests."""
        self.send_response(200)
        self.end_headers()

    def guess_type(self, path):
        """Override to add PLY and other missing MIME types."""
        _, ext = os.path.splitext(path)
        if ext.lower() in EXTRA_MIME:
            return EXTRA_MIME[ext.lower()]
        return super().guess_type(path)

    def log_message(self, fmt, *args):
        """Suppress noisy poll requests from the viewer's 10s interval check."""
        msg = fmt % args
        # Suppress successful latest.json polls — they fire every 10s
        if "latest.json" in msg and '" 200 ' in msg:
            return
        logging.info(msg)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    if not os.path.isdir(SERVE_DIR):
        print(f"[FAIL] Serve directory not found: {SERVE_DIR}")
        print("       Mount the SSD first: sudo mount -a")
        sys.exit(1)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(message)s",
        datefmt="%H:%M:%S",
    )

    # Change working directory so relative paths resolve correctly
    os.chdir(SERVE_DIR)

    handler = lambda *a, **kw: CORSHandler(*a, directory=SERVE_DIR, **kw)
    server  = HTTPServer((BIND_ADDR, PORT), handler)

    logging.info(f"Drone mesh server running")
    logging.info(f"  Serving : {SERVE_DIR}")
    logging.info(f"  Port    : {PORT}")
    logging.info(f"  Viewer  : http://10.42.0.1:{PORT}/meshview.html")
    logging.info(f"  Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Server stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
