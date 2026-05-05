"""
ground_station/ui_server.py — Local dashboard HTTP server.

Serves the single-page dashboard HTML and a JSON API that the page
polls to get flight list and report content from the local cache.

Endpoints
---------
GET  /                          → dashboard HTML (embedded in this module)
GET  /api/status                → daemon + Ollama + RPi health JSON
GET  /api/flights               → list of locally cached flights
GET  /api/report/<session_id>   → markdown report text for a flight
GET  /favicon.ico               → DronePi logo ICO from state dir

All responses include CORS headers so the page can call the RPi
meshview directly from the browser without proxy issues.

Design constraints
------------------
- No main() in this module.
- Pure stdlib HTTP server — no Flask/FastAPI dependency.
- Dashboard HTML is embedded as a string constant (no separate file to
  manage or distribute).
- All failures return JSON error responses, never crash the server.
"""

import json
import logging
import os
import platform
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse, parse_qs

log = logging.getLogger(__name__)


def _cache_dir() -> Path:
    return Path(os.environ.get(
        "DRONEPI_CACHE_DIR",
        Path.home() / ".dronepi-ground" / "cache",
    ))

def _state_dir() -> Path:
    return Path.home() / ".dronepi-ground"

def _rpi_base() -> str:
    host = os.environ.get("DRONEPI_RPI_HOST", "10.42.0.1")
    port = os.environ.get("DRONEPI_RPI_PORT", "8080")
    return f"http://{host}:{port}"


# ══════════════════════════════════════════════════════════════════════════════
# Dashboard HTML (single-file, self-contained)
# ══════════════════════════════════════════════════════════════════════════════

DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>DronePi Ground Station</title>
<style>
  :root {
    --bg:       #0d1117;
    --surface:  #161b22;
    --border:   #30363d;
    --green:    #3fb950;
    --green-dim:#2ea043;
    --text:     #e6edf3;
    --muted:    #8b949e;
    --red:      #f85149;
    --yellow:   #d29922;
    --accent:   #58a6ff;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    font-size: 14px; display: flex; flex-direction: column; height: 100vh;
  }

  /* ── Header ── */
  header {
    background: var(--surface); border-bottom: 1px solid var(--border);
    padding: 12px 20px; display: flex; align-items: center; gap: 14px;
    flex-shrink: 0;
  }
  header img { width: 36px; height: 36px; border-radius: 50%; }
  header h1 { font-size: 16px; font-weight: 600; color: var(--text); }
  header h1 span { color: var(--green); }
  .status-bar { margin-left: auto; display: flex; gap: 16px; }
  .pill {
    font-size: 11px; padding: 3px 10px; border-radius: 12px;
    border: 1px solid var(--border); display: flex; align-items: center; gap: 5px;
  }
  .pill.ok  { border-color: var(--green);  color: var(--green); }
  .pill.err { border-color: var(--red);    color: var(--red); }
  .pill.warn{ border-color: var(--yellow); color: var(--yellow); }
  .dot { width: 7px; height: 7px; border-radius: 50%; background: currentColor; }

  /* ── Layout ── */
  .layout { display: flex; flex: 1; overflow: hidden; }

  /* ── Sidebar ── */
  .sidebar {
    width: 260px; min-width: 200px; background: var(--surface);
    border-right: 1px solid var(--border);
    display: flex; flex-direction: column; flex-shrink: 0;
  }
  .sidebar-header {
    padding: 12px 16px; font-size: 11px; text-transform: uppercase;
    letter-spacing: .08em; color: var(--muted); border-bottom: 1px solid var(--border);
    display: flex; justify-content: space-between; align-items: center;
  }
  .refresh-btn {
    background: none; border: none; color: var(--muted); cursor: pointer;
    font-size: 14px; padding: 2px 6px; border-radius: 4px;
  }
  .refresh-btn:hover { background: var(--border); color: var(--text); }
  .flight-list { overflow-y: auto; flex: 1; }
  .flight-item {
    padding: 10px 16px; cursor: pointer; border-bottom: 1px solid var(--border);
    transition: background .15s;
  }
  .flight-item:hover    { background: rgba(255,255,255,.04); }
  .flight-item.active   { background: rgba(63,185,80,.08); border-left: 3px solid var(--green); }
  .flight-item .fid     { font-size: 12px; font-weight: 500; color: var(--text); margin-bottom: 3px; }
  .flight-item .fmeta   { font-size: 11px; color: var(--muted); }
  .flight-item .badge   {
    display: inline-block; font-size: 10px; padding: 1px 6px;
    border-radius: 4px; margin-left: 6px; vertical-align: middle;
  }
  .badge.complete { background: rgba(63,185,80,.15); color: var(--green); }
  .badge.report   { background: rgba(88,166,255,.15); color: var(--accent); }

  /* ── Main content ── */
  .main { flex: 1; overflow-y: auto; padding: 24px 32px; }
  .empty-state {
    display: flex; flex-direction: column; align-items: center;
    justify-content: center; height: 60%; color: var(--muted); gap: 12px;
  }
  .empty-state .icon { font-size: 48px; }

  /* ── Report view ── */
  .report-header {
    display: flex; justify-content: space-between; align-items: flex-start;
    margin-bottom: 20px; flex-wrap: wrap; gap: 12px;
  }
  .report-title { font-size: 18px; font-weight: 600; color: var(--text); }
  .report-subtitle { font-size: 12px; color: var(--muted); margin-top: 4px; }
  .btn {
    padding: 6px 14px; border-radius: 6px; border: 1px solid var(--border);
    background: var(--surface); color: var(--text); cursor: pointer;
    font-size: 12px; font-weight: 500; text-decoration: none;
    display: inline-flex; align-items: center; gap: 6px; transition: all .15s;
  }
  .btn:hover      { border-color: var(--green); color: var(--green); }
  .btn.primary    { background: var(--green-dim); border-color: var(--green); color: #fff; }
  .btn.primary:hover { background: var(--green); }
  .btn-group      { display: flex; gap: 8px; flex-wrap: wrap; }

  /* ── Markdown rendering ── */
  .report-body {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: 8px; padding: 24px; line-height: 1.7;
  }
  .report-body h2 {
    font-size: 15px; font-weight: 600; color: var(--text);
    border-bottom: 1px solid var(--border); padding-bottom: 8px;
    margin: 20px 0 12px;
  }
  .report-body h2:first-child { margin-top: 0; }
  .report-body p  { color: var(--muted); margin-bottom: 10px; }
  .report-body ul { padding-left: 20px; color: var(--muted); }
  .report-body li { margin-bottom: 4px; }
  .report-body strong { color: var(--text); }
  .report-body code {
    background: var(--bg); padding: 1px 6px; border-radius: 4px;
    font-family: 'SFMono-Regular', Consolas, monospace; font-size: 12px;
    color: var(--accent);
  }

  /* ── No report placeholder ── */
  .no-report {
    background: var(--surface); border: 1px dashed var(--border);
    border-radius: 8px; padding: 40px; text-align: center; color: var(--muted);
  }
  .no-report .hint { font-size: 12px; margin-top: 8px; }

  /* ── Loading spinner ── */
  .spinner { display: inline-block; width: 16px; height: 16px; }
  .spinner::after {
    content: ''; display: block; width: 14px; height: 14px;
    border: 2px solid var(--border); border-top-color: var(--green);
    border-radius: 50%; animation: spin .7s linear infinite;
  }
  @keyframes spin { to { transform: rotate(360deg); } }
</style>
</head>
<body>

<header>
  <img src="/favicon.ico" alt="DronePi" onerror="this.style.display='none'">
  <h1>Drone<span>Pi</span> Ground Station</h1>
  <div class="status-bar" id="statusBar">
    <div class="pill warn"><div class="dot"></div>Loading...</div>
  </div>
</header>

<div class="layout">
  <aside class="sidebar">
    <div class="sidebar-header">
      Flights
      <button class="refresh-btn" onclick="loadFlights()" title="Refresh">↻</button>
    </div>
    <div class="flight-list" id="flightList">
      <div style="padding:16px;color:var(--muted);font-size:12px;">Loading...</div>
    </div>
  </aside>

  <main class="main" id="mainContent">
    <div class="empty-state">
      <div class="icon">🛸</div>
      <div>Select a flight to view its report</div>
      <div style="font-size:12px;">Reports are generated automatically after each flight lands.</div>
    </div>
  </main>
</div>

<script>
const RPI_BASE = "DRONEPI_RPI_BASE";
let selectedId  = null;
let statusTimer = null;

// ── Status bar ──────────────────────────────────────────────────────────────
async function loadStatus() {
  try {
    const r   = await fetch('/api/status');
    const d   = await r.json();
    const bar = document.getElementById('statusBar');
    bar.innerHTML = '';
    const items = [
      { label: 'Daemon',  ok: d.daemon  },
      { label: 'Ollama',  ok: d.ollama  },
      { label: 'RPi',     ok: d.rpi,   warn: true },
    ];
    items.forEach(({label, ok, warn}) => {
      const cls = ok ? 'ok' : (warn ? 'warn' : 'err');
      bar.innerHTML += `<div class="pill ${cls}">
        <div class="dot"></div>${label}
      </div>`;
    });
  } catch(e) {
    document.getElementById('statusBar').innerHTML =
      '<div class="pill err"><div class="dot"></div>UI Error</div>';
  }
}

// ── Flight list ─────────────────────────────────────────────────────────────
async function loadFlights() {
  try {
    const r   = await fetch('/api/flights');
    const d   = await r.json();
    const el  = document.getElementById('flightList');
    if (!d.flights || d.flights.length === 0) {
      el.innerHTML = '<div style="padding:16px;color:var(--muted);font-size:12px;">No flights in cache yet.</div>';
      return;
    }
    el.innerHTML = d.flights.map(f => `
      <div class="flight-item ${f.id === selectedId ? 'active' : ''}"
           onclick="selectFlight('${f.id}')">
        <div class="fid">${f.id}
          ${f.has_report ? '<span class="badge report">report</span>' : ''}
        </div>
        <div class="fmeta">${f.duration || '—'} &nbsp;·&nbsp; ${f.start_time || '—'}</div>
      </div>`).join('');
  } catch(e) {
    document.getElementById('flightList').innerHTML =
      '<div style="padding:16px;color:var(--muted);font-size:12px;">Failed to load flights.</div>';
  }
}

// ── Flight detail ────────────────────────────────────────────────────────────
async function selectFlight(id) {
  selectedId = id;
  loadFlights();  // re-render to show active state

  const main = document.getElementById('mainContent');
  main.innerHTML = '<div style="padding:40px;display:flex;gap:12px;align-items:center;color:var(--muted)"><div class="spinner"></div>Loading report...</div>';

  try {
    const r = await fetch(`/api/report/${id}`);
    const d = await r.json();

    const meshUrl = `${RPI_BASE}/meshview.html?session=${id}`;

    if (d.report) {
      main.innerHTML = `
        <div class="report-header">
          <div>
            <div class="report-title">${id}</div>
            <div class="report-subtitle">
              ${d.duration ? 'Duration: ' + d.duration + 's' : ''}
              ${d.start_time ? ' &nbsp;·&nbsp; ' + d.start_time : ''}
            </div>
          </div>
          <div class="btn-group">
            <a class="btn primary" href="${meshUrl}" target="_blank">🗺 Open Mesh Viewer</a>
            <button class="btn" onclick="copyReport()">📋 Copy Report</button>
          </div>
        </div>
        <div class="report-body" id="reportBody">${renderMarkdown(d.report)}</div>`;
    } else {
      main.innerHTML = `
        <div class="report-header">
          <div>
            <div class="report-title">${id}</div>
            <div class="report-subtitle">No report generated yet</div>
          </div>
          <div class="btn-group">
            <a class="btn primary" href="${meshUrl}" target="_blank">🗺 Open Mesh Viewer</a>
          </div>
        </div>
        <div class="no-report">
          <div>Report not yet available</div>
          <div class="hint">The daemon will generate it automatically, or run:<br>
            <code>ground-station replay ${id}</code>
          </div>
        </div>`;
    }
  } catch(e) {
    main.innerHTML = `<div style="padding:40px;color:var(--red)">Failed to load report: ${e}</div>`;
  }
}

// ── Simple markdown → HTML ───────────────────────────────────────────────────
function renderMarkdown(md) {
  return md
    .replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')
    .replace(/^## (.+)$/gm, '<h2>$1</h2>')
    .replace(/^### (.+)$/gm, '<h3 style="font-size:13px;color:var(--text);margin:14px 0 8px;">$1</h3>')
    .replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>')
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    .replace(/^- (.+)$/gm, '<li>$1</li>')
    .replace(/(<li>.*<\/li>\n?)+/g, s => `<ul>${s}</ul>`)
    .replace(/\n\n/g, '</p><p>')
    .replace(/^(?!<[hul])/gm, '')
    .replace(/(<p><\/p>)+/g, '')
    .trim();
}

function copyReport() {
  const body = document.getElementById('reportBody');
  if (body) navigator.clipboard.writeText(body.innerText);
}

// ── Init ────────────────────────────────────────────────────────────────────
loadStatus();
loadFlights();
setInterval(loadStatus,  10000);
setInterval(loadFlights, 20000);
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════════════════════
# Request handler
# ══════════════════════════════════════════════════════════════════════════════

class _DashboardHandler(BaseHTTPRequestHandler):

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path   = parsed.path.rstrip("/") or "/"

        try:
            if path == "/":
                self._serve_dashboard()
            elif path == "/favicon.ico":
                self._serve_favicon()
            elif path == "/api/status":
                self._serve_status()
            elif path == "/api/flights":
                self._serve_flights()
            elif path.startswith("/api/report/"):
                session_id = path[len("/api/report/"):]
                self._serve_report(session_id)
            else:
                self._json({"error": "not found"}, 404)
        except Exception as exc:
            log.error(f"[UI] Handler error: {exc}")
            self._json({"error": str(exc)}, 500)

    # ── Route handlers ────────────────────────────────────────────────────────

    def _serve_dashboard(self) -> None:
        html = DASHBOARD_HTML.replace("DRONEPI_RPI_BASE", _rpi_base())
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _serve_favicon(self) -> None:
        ico = _state_dir() / "dronepi.ico"
        if not ico.exists():
            self.send_response(404)
            self.end_headers()
            return
        data = ico.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/x-icon")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_status(self) -> None:
        from ground_station.process_manager import ProcessManager
        from ground_station.ollama_client import OllamaClient
        import httpx

        pm     = ProcessManager()
        ollama = OllamaClient()

        rpi_ok = False
        try:
            with httpx.Client(timeout=httpx.Timeout(3.0)) as c:
                r = c.get(f"{_rpi_base()}/api/flights")
                rpi_ok = r.status_code == 200
        except Exception:
            pass

        self._json({
            "daemon": pm.is_running(),
            "ollama": ollama.is_reachable(),
            "rpi":    rpi_ok,
            "model":  ollama.model,
        })

    def _serve_flights(self) -> None:
        cache = _cache_dir()
        flights = []
        if cache.exists():
            for session_dir in sorted(cache.iterdir(), reverse=True):
                if not session_dir.is_dir():
                    continue
                fid = session_dir.name
                has_report = (session_dir / "report.md").exists()

                # Try to read duration/start from flight_record.json
                duration   = None
                start_time = None
                fr_path    = session_dir / "flight_record.json"
                if fr_path.exists():
                    try:
                        fr         = json.loads(fr_path.read_text(encoding="utf-8"))
                        duration   = fr.get("duration_s")
                        start_time = fr.get("arm_time_iso", "")[:19]
                    except Exception:
                        pass

                flights.append({
                    "id":         fid,
                    "has_report": has_report,
                    "duration":   f"{duration:.0f}s" if duration else None,
                    "start_time": start_time,
                })

        self._json({"flights": flights, "total": len(flights)})

    def _serve_report(self, session_id: str) -> None:
        report_path = _cache_dir() / session_id / "report.md"
        if not report_path.exists():
            self._json({"report": None, "session_id": session_id})
            return

        report_md  = report_path.read_text(encoding="utf-8")
        duration   = None
        start_time = None
        fr_path    = _cache_dir() / session_id / "flight_record.json"
        if fr_path.exists():
            try:
                fr         = json.loads(fr_path.read_text(encoding="utf-8"))
                duration   = fr.get("duration_s")
                start_time = fr.get("arm_time_iso", "")[:19]
            except Exception:
                pass

        self._json({
            "session_id": session_id,
            "report":     report_md,
            "duration":   f"{duration:.0f}" if duration else None,
            "start_time": start_time,
        })

    # ── Response helpers ──────────────────────────────────────────────────────

    def _json(self, obj: dict, status: int = 200) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")

    def log_message(self, fmt, *args) -> None:
        log.debug(f"[UI] {fmt % args}")


# ══════════════════════════════════════════════════════════════════════════════
# UIServer
# ══════════════════════════════════════════════════════════════════════════════

class UIServer:
    """
    Local HTTP server for the dashboard UI.

    Usage
    -----
        server = UIServer(port=8765)
        server.serve_forever()   # blocks; run in a background process
    """

    def __init__(self, port: int = 8765) -> None:
        self._port = port

    def serve_forever(self) -> None:
        server = HTTPServer(("127.0.0.1", self._port), _DashboardHandler)
        log.info(f"[UI] Dashboard running at http://localhost:{self._port}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
            log.info("[UI] Dashboard server stopped.")
