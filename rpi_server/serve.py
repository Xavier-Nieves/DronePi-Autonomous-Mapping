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
import http.client
import threading
from datetime import datetime
from http.server import SimpleHTTPRequestHandler, HTTPServer

# ── config ────────────────────────────────────────────────────────────────────

SERVE_DIR  = "/mnt/ssd/maps"
ROSBAG_DIR = "/mnt/ssd/rosbags"
LOG_DIR    = os.path.expanduser(
    os.environ.get("DRONEPI_LOG_DIR", "~/unitree_lidar_project/logs")
)
PORT       = 8080
BIND_ADDR  = "0.0.0.0"

# Camera stream proxy — forwards /stream and /snapshot to camera_stream_server
# on localhost:8081. This keeps the browser talking to a single origin (port 8080)
# and avoids cross-port CORS issues. If camera_stream_server is not running,
# requests to /stream return 503 gracefully rather than crashing serve.py.
CAM_STREAM_HOST = "127.0.0.1"
CAM_STREAM_PORT = 8081

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


def build_event_log(session_id: str = None, last_n: int = None) -> dict:
    """Load flight_events.json for a session and return as JSON payload.

    Lookup order:
      1. session_id provided → ROSBAG_DIR/<session_id>/flight_events.json (SSD primary)
      2. session_id provided but SSD absent → LOG_DIR/events/<session_id>_events.json
      3. session_id None → most recent session from ROSBAG_DIR

    Returns:
        {
          "session_id":  str,
          "source":      "ssd" | "repo_mirror" | "not_found",
          "event_count": int,
          "events":      [ {...} ]   # newest-first
        }
    """
    if session_id is None:
        try:
            entries = sorted(
                [e for e in os.listdir(ROSBAG_DIR)
                 if os.path.isdir(os.path.join(ROSBAG_DIR, e))
                 and BAG_DIR_RE.match(e)],
                reverse=True,
            )
            session_id = entries[0] if entries else None
        except OSError:
            session_id = None

    if session_id is None:
        return {"session_id": None, "source": "not_found", "event_count": 0, "events": []}

    ssd_path  = os.path.join(ROSBAG_DIR, session_id, "flight_events.json")
    repo_path = os.path.join(LOG_DIR, "events", f"{session_id}_events.json")

    events = None
    source = "not_found"
    for path, label in [(ssd_path, "ssd"), (repo_path, "repo_mirror")]:
        if os.path.isfile(path):
            try:
                with open(path) as f:
                    events = json.load(f)
                source = label
                break
            except (OSError, json.JSONDecodeError) as exc:
                logging.warning(f"build_event_log: could not read {path}: {exc}")

    if events is None:
        return {"session_id": session_id, "source": "not_found", "event_count": 0, "events": []}

    events = list(reversed(events))   # newest first
    if last_n:
        events = events[:last_n]

    return {"session_id": session_id, "source": source,
            "event_count": len(events), "events": events}


def build_log_history(last_n: int = 50) -> dict:
    """Return flight_history.log as structured JSON, newest-first.

    Returns:
        {
          "total_flights": int,
          "returned":      int,
          "flights":       [ {"raw": str, "flight_num": int,
                              "end_reason": str, "session_id": str} ]
        }
    """
    history_file = os.path.join(LOG_DIR, "flight_history.log")
    if not os.path.isfile(history_file):
        return {"total_flights": 0, "returned": 0, "flights": []}
    try:
        with open(history_file) as f:
            lines = [l.rstrip() for l in f if l.startswith("Flight ")]
    except OSError as exc:
        logging.warning(f"build_log_history: {exc}")
        return {"total_flights": 0, "returned": 0, "flights": []}

    total  = len(lines)
    subset = list(reversed(lines[-last_n:] if last_n else lines))
    parsed = []
    for line in subset:
        rec = {"raw": line}
        m = re.match(r'^Flight\s+(\d+)', line)
        if m:
            rec["flight_num"] = int(m.group(1))
        m = re.search(r'END:\s+(\S+)', line)
        if m:
            rec["end_reason"] = m.group(1).rstrip("|").strip()
        m = re.search(r'Bag:\s+(\S+)', line)
        if m:
            rec["session_id"] = m.group(1)
        parsed.append(rec)
    return {"total_flights": total, "returned": len(parsed), "flights": parsed}


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
        # ── /api/logs?session=<id>&last=<n> — flight event log JSON ──────────
        if self.path.startswith("/api/logs"):
            self._serve_event_log()
            return
        # ── /api/log-history?last=<n> — flight_history.log as JSON ───────────
        if self.path.startswith("/api/log-history"):
            self._serve_log_history()
            return
        # ── /api/log-download?session=<id> — download flight_events.json ─────
        if self.path.startswith("/api/log-download"):
            self._serve_log_download()
            return
        # ── /stream — proxy MJPEG stream from camera_stream_server:8081 ───────
        if self.path == "/stream":
            self._proxy_camera_stream()
            return
        # ── /snapshot — proxy single JPEG from camera_stream_server:8081 ──────
        if self.path == "/snapshot":
            self._proxy_camera_snapshot()
            return
        # ── /rosbags/<session>/<file.ply> — serve PLY files from rosbags dir ──
        if self.path.startswith("/rosbags/"):
            self._serve_rosbag_file()
            return
        # All other requests: fall through to static file serving
        super().do_GET()

    def _proxy_camera_stream(self):
        """
        Proxy the MJPEG stream from camera_stream_server (port 8081) to the
        browser. Streams indefinitely until the client disconnects.

        Design note — why proxy instead of redirect:
          A 302 redirect to http://rpi:8081/stream would work on the LAN but
          breaks when the viewer is accessed through a tunnel or when the
          browser enforces same-origin policy. Proxying through port 8080 keeps
          the origin consistent regardless of network topology.
        """
        try:
            conn = http.client.HTTPConnection(
                CAM_STREAM_HOST, CAM_STREAM_PORT, timeout=4
            )
            conn.request("GET", "/stream")
            upstream = conn.getresponse()

            if upstream.status != 200:
                self.send_response(upstream.status)
                self.end_headers()
                return

            # Forward the upstream Content-Type (carries the MJPEG boundary param)
            content_type = upstream.getheader(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=mjpegframe"
            )
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store, no-cache")
            self.end_headers()

            # Stream bytes directly — no buffering, minimum latency
            while True:
                chunk = upstream.read(8192)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()

        except (ConnectionRefusedError, http.client.HTTPException, OSError):
            # camera_stream_server not running — return 503, don't crash serve.py
            try:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(b"Camera stream unavailable")
            except OSError:
                pass

    def _proxy_camera_snapshot(self):
        """Proxy a single JPEG snapshot from camera_stream_server:8081/snapshot."""
        try:
            conn = http.client.HTTPConnection(
                CAM_STREAM_HOST, CAM_STREAM_PORT, timeout=4
            )
            conn.request("GET", "/snapshot")
            upstream = conn.getresponse()
            body = upstream.read()

            self.send_response(upstream.status)
            self.send_header(
                "Content-Type",
                upstream.getheader("Content-Type", "image/jpeg")
            )
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        except (ConnectionRefusedError, http.client.HTTPException, OSError):
            try:
                self.send_response(503)
                self.end_headers()
            except OSError:
                pass

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

    def _serve_event_log(self):
        """GET /api/logs?session=<id>&last=<n>"""
        try:
            session_id, last_n = None, None
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if "=" in part:
                        k, v = part.split("=", 1)
                        if k == "session":
                            session_id = v.strip()
                        elif k == "last":
                            try:
                                last_n = int(v)
                            except ValueError:
                                pass
            payload = build_event_log(session_id=session_id, last_n=last_n)
            body    = json.dumps(payload, indent=2).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type",   "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            logging.info(
                f"/api/logs → session={payload['session_id']} "
                f"events={payload['event_count']} source={payload['source']}"
            )
        except Exception as exc:
            logging.error(f"/api/logs error: {exc}")
            self.send_response(500)
            self.end_headers()

    def _serve_log_history(self):
        """GET /api/log-history?last=<n>"""
        try:
            last_n = 50
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if part.startswith("last="):
                        try:
                            last_n = int(part.split("=", 1)[1])
                        except ValueError:
                            pass
            payload = build_log_history(last_n=last_n)
            body    = json.dumps(payload, indent=2).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type",   "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            logging.info(
                f"/api/log-history → {payload['returned']}/{payload['total_flights']} flights"
            )
        except Exception as exc:
            logging.error(f"/api/log-history error: {exc}")
            self.send_response(500)
            self.end_headers()

    def _serve_log_download(self):
        """GET /api/log-download?session=<id>
        Serves flight_events.json as a file download attachment.
        If session omitted, uses the most recent session.
        """
        try:
            session_id = None
            if "?" in self.path:
                for part in self.path.split("?", 1)[1].split("&"):
                    if part.startswith("session="):
                        session_id = part.split("=", 1)[1].strip()

            # Resolve session
            if not session_id:
                try:
                    entries = sorted(
                        [e for e in os.listdir(ROSBAG_DIR)
                         if os.path.isdir(os.path.join(ROSBAG_DIR, e))
                         and BAG_DIR_RE.match(e)],
                        reverse=True,
                    )
                    session_id = entries[0] if entries else None
                except OSError:
                    session_id = None

            if not session_id:
                self.send_response(404)
                self.end_headers()
                return

            ssd_path  = os.path.join(ROSBAG_DIR, session_id, "flight_events.json")
            repo_path = os.path.join(LOG_DIR, "events", f"{session_id}_events.json")

            file_path = None
            for p in [ssd_path, repo_path]:
                if os.path.isfile(p):
                    file_path = p
                    break

            if not file_path:
                self.send_response(404)
                self.end_headers()
                return

            size     = os.path.getsize(file_path)
            filename = f"{session_id}_flight_events.json"
            self.send_response(200)
            self.send_header("Content-Type",        "application/json")
            self.send_header("Content-Length",      str(size))
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
            self.send_header("Cache-Control",       "no-store")
            self.end_headers()
            with open(file_path, "rb") as f:
                self.wfile.write(f.read())
            logging.info(f"/api/log-download → {filename} ({size} bytes)")
        except Exception as exc:
            logging.error(f"/api/log-download error: {exc}")
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
    logging.info(f"  Event log : http://10.42.0.1:{PORT}/api/logs")
    logging.info(f"  Log hist  : http://10.42.0.1:{PORT}/api/log-history")
    logging.info(f"  Download  : http://10.42.0.1:{PORT}/api/log-download")
    logging.info(f"  Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Server stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
