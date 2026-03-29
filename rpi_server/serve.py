#!/usr/bin/env python3
"""
drone_mesh_server.py — CORS-enabled HTTP file server for the drone mesh viewer.

Serves .ply files, latest.json manifest, and meshview.html from
/mnt/ssd/maps/ on port 8080. Zero external dependencies — Python
standard library only.

Endpoints
---------
GET  /meshview.html          Three.js viewer page
GET  /latest.json            Auto-load trigger polled every 10s by viewer
GET  /<filename>.ply         PLY mesh file download
GET  /api/flights            Flight database — JSON list of all sessions
                             Scans /mnt/ssd/rosbags/ and infers metadata
                             from directory names and file presence.
                             Overlays metadata.json per folder when present
                             (written by postprocess_mesh.py in a later phase).

Run directly:
    python3 /mnt/ssd/maps/serve.py

Or via systemd (see drone-mesh-server.service).

On laptop browser:
    http://10.42.0.1:8080/meshview.html
"""

import os
import sys
import json
import logging
import re
from datetime import datetime
from http.server import SimpleHTTPRequestHandler, HTTPServer
import sys as _sys

_sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from pythonconfig.config_loader import load_config as _load_config


# ── config ────────────────────────────────────────────────────────────────────
_cfg = _load_config()

SERVE_DIR  = _cfg["paths"]["maps_dir"]
ROSBAG_DIR = _cfg["paths"]["rosbags_dir"]
PORT       = _cfg["server"]["port"]
BIND_ADDR  = _cfg["server"]["bind_addr"]

# Directory name pattern written by ros2 bag record:
#   scan_YYYYMMDD_HHMMSS   e.g. scan_20260317_091400
# Also accepts flt_* and flight_* prefixes for forward compatibility.
BAG_DIR_RE = re.compile(
    r'^(?:scan|flt|flight)_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$'
)

EXTRA_MIME = {
    ".ply":  "application/octet-stream",
    ".json": "application/json",
    ".html": "text/html; charset=utf-8",
    ".js":   "application/javascript",
}

# ── flight database ───────────────────────────────────────────────────────────

def _folder_size_mb(path: str) -> float:
    """Return total size of all files in a directory tree, in MB."""
    total = 0
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            try:
                total += os.path.getsize(os.path.join(dirpath, f))
            except OSError:
                pass
    return round(total / (1024 * 1024), 1)


def _detect_files(path: str) -> list[str]:
    """
    Return list of file-type tags present in the bag folder.
    Tags: 'bag'  — any .db3 file (ROS 2 bag data)
          'ply'  — any .ply file
          'pcd'  — any .pcd file
    """
    tags = set()
    for f in os.listdir(path):
        lower = f.lower()
        if lower.endswith('.db3'):
            tags.add('bag')
        elif lower.endswith('.ply'):
            tags.add('ply')
        elif lower.endswith('.pcd'):
            tags.add('pcd')
    # Preserve display order
    order = ['ply', 'pcd', 'bag']
    return [t for t in order if t in tags]


def _derive_status(files: list[str]) -> str:
    """
    Infer session status from which file types are present.

    complete — PLY mesh exists (postprocess ran successfully)
    partial  — bag recorded but mesh not yet generated
    failed   — directory exists but no bag data found
    """
    if 'ply' in files:
        return 'complete'
    if 'bag' in files:
        return 'partial'
    return 'failed'


def _find_mesh_file(session_dir: str, session_id: str) -> str | None:
    """Return the best mesh PLY filename for a session, or None."""
    try:
        plys = [f for f in os.listdir(session_dir) if f.lower().endswith('.ply')]
    except OSError:
        return None
    # Priority order for mesh files
    priority = ['mesh_final.ply', 'mesh_poisson.ply']
    for name in priority:
        if name in plys:
            return name
    # Fallback: any file with _mesh in name
    for p in plys:
        if '_mesh' in p.lower() and 'debug' not in p.lower():
            return p
    return None


def _find_cloud_file(session_dir: str, session_id: str) -> str | None:
    """Return the best point cloud PLY filename for a session, or None."""
    try:
        plys = [f for f in os.listdir(session_dir) if f.lower().endswith('.ply')]
    except OSError:
        return None
    # Priority order for cloud files
    priority = ['combined_cloud.ply']
    for name in priority:
        if name in plys:
            return name
    # Fallback: any file with _cloud in name
    for p in plys:
        if '_cloud' in p.lower() and 'debug' not in p.lower():
            return p
    return None


def _find_all_ply_files(session_dir: str) -> list[str]:
    """Return all PLY filenames in a session directory."""
    try:
        return sorted([f for f in os.listdir(session_dir) if f.lower().endswith('.ply')])
    except OSError:
        return []


def build_flight_list() -> dict:
    """
    Scan ROSBAG_DIR and build the /api/flights response payload.

    Per-folder logic
    ----------------
    1. Parse timestamp from directory name via BAG_DIR_RE.
    2. Detect file types present (.db3, .ply, .pcd).
    3. Read folder size from disk.
    4. Derive status from file presence.
    5. If metadata.json exists in the folder, overlay its fields
       (written by postprocess_mesh.py — point_count, duration, notes, etc.).
       Any key in metadata.json takes precedence over the inferred value,
       allowing a clean upgrade to Option 1 without changing this function.

    Returns the exact schema expected by the meshview.html sidebar:
    {
        "flights": [ { ...per-flight fields... }, ... ],
        "total_flights":  N,
        "complete_count": N,
        "total_size_gb":  "X.X"
    }
    """
    flights = []

    if not os.path.isdir(ROSBAG_DIR):
        logging.warning(f"Rosbag directory not found: {ROSBAG_DIR}")
        return _empty_payload()

    try:
        entries = sorted(os.listdir(ROSBAG_DIR), reverse=True)  # newest first
    except OSError as e:
        logging.error(f"Cannot list {ROSBAG_DIR}: {e}")
        return _empty_payload()

    total_bytes = 0

    for entry in entries:
        folder_path = os.path.join(ROSBAG_DIR, entry)
        if not os.path.isdir(folder_path):
            continue

        m = BAG_DIR_RE.match(entry)
        if not m:
            continue  # skip directories that don't match the naming convention

        year, month, day, hour, minute, second = m.groups()

        # ── inferred fields ───────────────────────────────────────────────────
        session_id = entry
        timestamp  = f"{year}-{month}-{day} {hour}:{minute}"
        files      = _detect_files(folder_path)
        status     = _derive_status(files)
        size_mb    = _folder_size_mb(folder_path)
        mesh_file  = _find_mesh_file(folder_path, session_id)
        cloud_file = _find_cloud_file(folder_path, session_id)
        all_plys   = _find_all_ply_files(folder_path)

        total_bytes += size_mb * 1024 * 1024

        record = {
            "id":          session_id,
            "timestamp":   timestamp,
            "files":       files,
            "status":      status,
            "size_mb":     size_mb,
            # Filenames only (basename)
            "mesh_file":   mesh_file,
            "cloud_file":  cloud_file,
            "all_plys":    all_plys,
            # Full URL paths for browser to fetch via /rosbags/
            "mesh_url":    f"rosbags/{session_id}/{mesh_file}" if mesh_file else None,
            "cloud_url":   f"rosbags/{session_id}/{cloud_file}" if cloud_file else None,
            # Legacy field — keep for backward compat
            "ply_file":    f"rosbags/{session_id}/{mesh_file or cloud_file}" if (mesh_file or cloud_file) else None,
            # Fields populated by postprocess_mesh.py overlay (None until then)
            "point_count": None,
            "duration":    None,
        }

        # ── metadata.json overlay (Option 1 upgrade path) ────────────────────
        # When postprocess_mesh.py is built, it writes metadata.json into the
        # bag folder. Any key present there overrides the inferred value above,
        # with no other code changes required.
        meta_path = os.path.join(folder_path, "metadata.json")
        if os.path.isfile(meta_path):
            try:
                with open(meta_path) as f:
                    overlay = json.load(f)
                record.update(overlay)
            except (OSError, json.JSONDecodeError) as e:
                logging.warning(f"Could not read {meta_path}: {e}")

        flights.append(record)

    complete_count = sum(1 for f in flights if f['status'] == 'complete')
    total_gb       = round(total_bytes / (1024 ** 3), 1)

    return {
        "flights":       flights,
        "total_flights": len(flights),
        "complete_count": complete_count,
        "total_size_gb": str(total_gb),
    }


def _empty_payload() -> dict:
    return {
        "flights":        [],
        "total_flights":  0,
        "complete_count": 0,
        "total_size_gb":  "0.0",
    }


# ── handler ───────────────────────────────────────────────────────────────────

class CORSHandler(SimpleHTTPRequestHandler):
    """SimpleHTTPRequestHandler extended with CORS headers and /api/flights."""

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        if self.path.endswith("latest.json") or self.path.startswith("/api/"):
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()

    def do_GET(self):
        # ── /api/flights ──────────────────────────────────────────────────────
        if self.path == "/api/flights":
            self._serve_flights()
            return
        # ── /rosbags/<session>/<file.ply> — serve PLY files from rosbags dir ──
        if self.path.startswith("/rosbags/"):
            self._serve_rosbag_file()
            return
        # All other requests: fall through to static file serving
        super().do_GET()

    def _serve_rosbag_file(self):
        """Serve a file from ROSBAG_DIR by path: /rosbags/<session>/<filename>"""
        try:
            # Strip leading /rosbags/
            rel_path = self.path[len("/rosbags/"):]
            # Security — prevent directory traversal
            rel_path = rel_path.replace("..", "").lstrip("/")
            full_path = os.path.join(ROSBAG_DIR, rel_path)

            if not os.path.isfile(full_path):
                self.send_response(404)
                self.end_headers()
                return

            size = os.path.getsize(full_path)
            self.send_response(200)
            _, ext = os.path.splitext(full_path)
            self.send_header("Content-Type", EXTRA_MIME.get(ext.lower(), "application/octet-stream"))
            self.send_header("Content-Length", str(size))
            self.end_headers()
            with open(full_path, "rb") as f:
                self.wfile.write(f.read())
            logging.info(f"/rosbags/{rel_path} → {size} bytes")
        except Exception as e:
            logging.error(f"rosbag file serve error: {e}")
            self.send_response(500)
            self.end_headers()

    def _serve_flights(self):
        try:
            payload = build_flight_list()
            body    = json.dumps(payload, indent=2).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type",   "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            logging.info(
                f"/api/flights → {payload['total_flights']} flights, "
                f"{payload['complete_count']} complete"
            )
        except Exception as e:
            logging.error(f"/api/flights error: {e}")
            self.send_response(500)
            self.end_headers()

    def guess_type(self, path):
        _, ext = os.path.splitext(path)
        if ext.lower() in EXTRA_MIME:
            return EXTRA_MIME[ext.lower()]
        return super().guess_type(path)

    def log_message(self, fmt, *args):
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

    os.chdir(SERVE_DIR)

    handler = lambda *a, **kw: CORSHandler(*a, directory=SERVE_DIR, **kw)
    server  = HTTPServer((BIND_ADDR, PORT), handler)

    logging.info("Drone mesh server running")
    logging.info(f"  Serving   : {SERVE_DIR}")
    logging.info(f"  Rosbags   : {ROSBAG_DIR}")
    logging.info(f"  Port      : {PORT}")
    logging.info(f"  Viewer    : http://10.42.0.1:{PORT}/meshview.html")
    logging.info(f"  Flights   : http://10.42.0.1:{PORT}/api/flights")
    logging.info(f"  Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Server stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
