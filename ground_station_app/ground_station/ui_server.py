"""
ground_station/ui_server.py — Local dashboard HTTP server.

Serves dashboard.html (a standalone file in the same directory) and a
JSON API the page polls for flight data.

Endpoints
---------
GET /                     dashboard.html (read from disk — no embedding)
GET /favicon.ico
GET /api/status           {daemon, ollama, rpi, rpi_base, model}
GET /api/flights          [{id, has_report, duration, slam_points, ...}]
GET /api/fleet-summary    {summary}  Ollama 2-3 sentence narrative
GET /api/report/<id>      {report, duration, metrics}

Data sources (priority)
-----------------------
1. SQLite at ~/.dronepi-ground/dronepi.db
2. CSV/JSON files in ~/.dronepi-ground/cache/  (fallback)
"""

import json
import logging
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
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

def _get_db():
    try:
        from ground_station.db import FlightDatabase
        return FlightDatabase()
    except Exception as exc:
        log.warning(f"[UI] DB unavailable, CSV fallback: {exc}")
        return None


class _DashboardHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        parsed = urlparse(self.path)
        path   = parsed.path.rstrip("/") or "/"
        qs     = parse_qs(parsed.query)
        try:
            if path == "/":                       self._serve_dashboard()
            elif path == "/favicon.ico":          self._serve_favicon()
            elif path == "/api/status":           self._serve_status()
            elif path == "/api/flights":          self._serve_flights()
            elif path == "/api/fleet-summary":    self._serve_fleet_summary(qs)
            elif path.startswith("/api/report/"): self._serve_report(path[len("/api/report/"):])
            else:                                 self._json({"error": "not found"}, 404)
        except Exception as exc:
            log.error(f"[UI] {exc}")
            self._json({"error": str(exc)}, 500)

    # ── Routes ────────────────────────────────────────────────────────────────

    def _serve_dashboard(self):
        # dashboard.html lives next to this file — same directory
        html_path = Path(__file__).parent / "dashboard.html"
        if not html_path.exists():
            self._json({"error": "dashboard.html not found — deploy it next to ui_server.py"}, 404)
            return
        body = html_path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _serve_favicon(self):
        ico = _state_dir() / "dronepi.ico"
        if not ico.exists():
            self.send_response(404); self.end_headers(); return
        data = ico.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/x-icon")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_status(self):
        from ground_station.process_manager import ProcessManager
        from ground_station.ollama_client import OllamaClient
        import httpx
        pm = ProcessManager(); ollama = OllamaClient(); rpi_ok = False
        try:
            with httpx.Client(timeout=httpx.Timeout(2.0)) as c:
                rpi_ok = c.get(f"{_rpi_base()}/api/flights").status_code == 200
        except Exception:
            pass
        self._json({
            "daemon":   pm.is_running(),
            "ollama":   ollama.is_reachable(),
            "rpi":      rpi_ok,
            "model":    ollama.model,
            "rpi_base": _rpi_base(),
        })

    def _serve_flights(self):
        db = _get_db()
        if db:
            rows = db.get_all_flights()
            if rows:
                known   = {r["session_id"] for r in rows}
                db_list = [self._db_to_api(r) for r in rows]
                extras  = [f for f in self._from_cache() if f["id"] not in known]
                result  = db_list + extras
                self._json({"flights": result, "total": len(result), "source": "db"})
                return
        flights = self._from_cache()
        self._json({"flights": flights, "total": len(flights), "source": "cache"})

    def _serve_fleet_summary(self, qs=None):
        try:
            from ground_station.ollama_client import OllamaClient, OllamaError
        except ImportError:
            self._json({"summary": "Ollama module not available."}); return

        db        = _get_db()
        stats     = db.get_stats() if db else {}
        anomalies = db.get_recent_anomalies(5) if db else []

        db_total  = int(stats.get("total", 0) or 0)
        # JS passes ?total= with the cache count — use whichever is larger
        js_total  = int((qs or {}).get("total", [0])[0] or 0)
        total     = max(db_total, js_total)
        reported  = int(stats.get("reported", 0) or 0)
        anm_total = int(stats.get("total_anomalies", 0) or 0)
        avg_dur   = round(float(stats.get("avg_duration_s") or 0), 1)
        avg_drift = round(float(stats.get("avg_drift_m") or 0), 3)

        if total == 0:
            self._json({"summary": "No flights recorded yet. System standing by."}); return

        anm_ctx = ""
        if anomalies:
            anm_ctx = "\nRecent anomalies:\n" + "\n".join(
                f"- [{a['session']}] {a['text']}" for a in anomalies[:5])

        prompt = (
            f"Fleet statistics: {total} total flights, {reported} with full reports. "
            f"Average flight duration: {avg_dur}s. Average SLAM drift: {avg_drift}m. "
            f"Total anomalies: {anm_total}.{anm_ctx}\n\n"
            "In 2-3 sentences, summarise overall fleet health and highlight any patterns. "
            "If no anomalies, confirm the fleet is operating normally."
        )
        system = (
            "You are a concise flight operations analyst for an autonomous LiDAR mapping drone. "
            "Respond in 2-3 sentences only. Be direct and specific."
        )
        try:
            summary = OllamaClient().chat(system_prompt=system, user_prompt=prompt)
            self._json({"summary": summary or "Fleet operating normally."})
        except OllamaError:
            if anm_total == 0:
                msg = f"All systems nominal \u2014 {total} flight{'s' if total!=1 else ''} logged, no anomalies detected."
            else:
                msg = (f"{total} flights logged. {anm_total} anomal{'ies' if anm_total!=1 else 'y'} "
                       f"detected across {reported} analysed sessions.")
            self._json({"summary": msg})
        except Exception as exc:
            log.error(f"[UI] fleet-summary: {exc}")
            self._json({"summary": "Fleet summary temporarily unavailable."})

    def _serve_report(self, session_id):
        db = _get_db()
        if db:
            row = db.get_flight(session_id)
            if row:
                dur = row.get("duration_s")
                self._json({
                    "session_id": session_id,
                    "report":     row.get("report_md"),
                    "duration":   f"{dur:.0f}" if dur else None,
                    "start_time": (row.get("arm_time_iso") or "")[:19],
                    "metrics": {
                        "slam_points":   row.get("slam_points"),
                        "drift":         row.get("drift_estimate_m"),
                        "cpu_temp":      row.get("cpu_temp_max_c"),
                        "anomaly_count": row.get("anomaly_count", 0),
                    },
                })
                return
        self._json(self._report_from_cache(session_id))

    # ── DB → API shape ────────────────────────────────────────────────────────

    @staticmethod
    def _db_to_api(row):
        dur = row.get("duration_s")
        return {
            "id":             row["session_id"],
            "has_report":     row.get("has_report", False),
            "duration":       f"{dur:.0f}s" if dur else None,
            "duration_raw":   round(dur, 1) if dur else 0,
            "start_time":     (row.get("arm_time_iso") or "")[:19],
            "slam_points":    row.get("slam_points") or 0,
            "drift_estimate": row.get("drift_estimate_m"),
            "cpu_temp_max":   row.get("cpu_temp_max_c"),
            "anomaly_count":  row.get("anomaly_count", 0),
            "anomalies":      row.get("anomalies", []),
        }

    # ── CSV cache fallback ────────────────────────────────────────────────────

    def _from_cache(self):
        cache = _cache_dir()
        flights = []
        if not cache.exists():
            return flights
        for sd in sorted(cache.iterdir(), reverse=True):
            if not sd.is_dir():
                continue
            fid = sd.name
            has_report = (sd / "report.md").exists()
            duration = None; start_time = None
            slam_points = 0; drift_estimate = None
            cpu_temp_max = None; anomaly_count = 0; anomalies = []

            fr = sd / "flight_record.json"
            if fr.exists():
                try:
                    d = json.loads(fr.read_text(encoding="utf-8-sig"))
                    duration   = d.get("duration_s")
                    start_time = (d.get("arm_time_iso") or "")[:19]
                except Exception:
                    pass

            bp = sd / "bag_summary.csv"
            if bp.exists():
                try:
                    lines = bp.read_text(encoding="utf-8-sig").strip().splitlines()
                    if len(lines) >= 2:
                        h = [x.strip() for x in lines[0].split(",")]
                        v = [x.strip() for x in lines[1].split(",")]
                        r = dict(zip(h, v))
                        slam_points = int(float(r.get("point_count_final", 0) or 0))
                        raw = r.get("drift_estimate_m", "")
                        drift_estimate = float(raw) if raw else None
                except Exception:
                    pass

            hp = sd / "health_log.csv"
            if hp.exists():
                try:
                    lines = hp.read_text(encoding="utf-8-sig").strip().splitlines()
                    if len(lines) >= 2:
                        h = [x.strip() for x in lines[0].split(",")]
                        temps = []
                        for line in lines[1:]:
                            rv = dict(zip(h, [x.strip() for x in line.split(",")]))
                            t  = rv.get("cpu_temp_c", "")
                            try: temps.append(float(t))
                            except ValueError: pass
                        cpu_temp_max = round(max(temps), 1) if temps else None
                except Exception:
                    pass

            rp = sd / "report.md"
            if rp.exists():
                try:
                    in_anm = False
                    for line in rp.read_text(encoding="utf-8").splitlines():
                        if "## Anomalies" in line: in_anm = True; continue
                        if in_anm and line.startswith("##"): break
                        if in_anm and line.strip().startswith("- "):
                            t = line.strip()[2:].strip()
                            if t: anomalies.append(t)
                    anomaly_count = len(anomalies)
                except Exception:
                    pass

            # metadata.json — pipeline timings
            stage_timings    = None
            bottleneck_stage = None
            mp = sd / "metadata.json"
            if mp.exists():
                try:
                    md = json.loads(mp.read_text(encoding="utf-8-sig"))
                    stage_timings    = md.get("stage_timings_s")
                    bottleneck_stage = md.get("bottleneck_stage")
                except Exception:
                    pass

            flights.append({
                "id":              fid,
                "has_report":      has_report,
                "duration":        f"{duration:.0f}s" if duration else None,
                "duration_raw":    round(duration, 1) if duration else 0,
                "start_time":      start_time,
                "slam_points":     slam_points,
                "drift_estimate":  drift_estimate,
                "cpu_temp_max":    cpu_temp_max,
                "anomaly_count":   anomaly_count,
                "anomalies":       anomalies,
                "stage_timings":   stage_timings,
                "bottleneck_stage":bottleneck_stage,
            })
        return flights

    def _report_from_cache(self, session_id):
        sd = _cache_dir() / session_id
        metrics = {}; duration = None; start_time = None

        fr = sd / "flight_record.json"
        if fr.exists():
            try:
                d = json.loads(fr.read_text(encoding="utf-8-sig"))
                duration   = d.get("duration_s")
                start_time = (d.get("arm_time_iso") or "")[:19]
            except Exception: pass

        bp = sd / "bag_summary.csv"
        if bp.exists():
            try:
                lines = bp.read_text(encoding="utf-8-sig").strip().splitlines()
                if len(lines) >= 2:
                    h = [x.strip() for x in lines[0].split(",")]
                    v = [x.strip() for x in lines[1].split(",")]
                    r = dict(zip(h, v))
                    metrics["slam_points"] = int(float(r.get("point_count_final", 0) or 0))
                    raw = r.get("drift_estimate_m", "")
                    if raw: metrics["drift"] = round(float(raw), 3)
            except Exception: pass

        hp = sd / "health_log.csv"
        if hp.exists():
            try:
                lines = hp.read_text(encoding="utf-8-sig").strip().splitlines()
                if len(lines) >= 2:
                    h = [x.strip() for x in lines[0].split(",")]
                    temps = []
                    for line in lines[1:]:
                        rv = dict(zip(h, [x.strip() for x in line.split(",")]))
                        t  = rv.get("cpu_temp_c", "")
                        try: temps.append(float(t))
                        except ValueError: pass
                    if temps: metrics["cpu_temp"] = round(max(temps), 1)
            except Exception: pass

        report_md = None; anomaly_count = 0
        rp = sd / "report.md"
        if rp.exists():
            try:
                report_md = rp.read_text(encoding="utf-8")
                in_anm = False
                for line in report_md.splitlines():
                    if "## Anomalies" in line: in_anm = True; continue
                    if in_anm and line.startswith("##"): break
                    if in_anm and line.strip().startswith("- "): anomaly_count += 1
            except Exception: pass
        metrics["anomaly_count"] = anomaly_count

        return {
            "session_id": session_id,
            "report":     report_md,
            "duration":   f"{duration:.0f}" if duration else None,
            "start_time": start_time,
            "metrics":    metrics,
        }

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")

    def log_message(self, fmt, *args):
        log.debug(f"[UI] {fmt % args}")


class UIServer:
    """Local HTTP dashboard server. Call serve_forever() in a background process."""

    def __init__(self, port=8765):
        self._port = port

    def serve_forever(self):
        server = HTTPServer(("127.0.0.1", self._port), _DashboardHandler)
        log.info(f"[UI] Dashboard at http://localhost:{self._port}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()
            log.info("[UI] Stopped.")
