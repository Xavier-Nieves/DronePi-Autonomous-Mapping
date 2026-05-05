"""
ground_station/report_publisher.py — Uploads markdown report to the RPi.

Responsibilities
----------------
PUT the generated report.md to the RPi server so it appears alongside
the mesh in meshview.html.

RPi endpoint (to be added to serve.py CORSHandler.do_PUT in Phase 5):
    PUT /rosbags/<session_id>/report.md
    Authorization: Bearer <DRONEPI_RPI_UPLOAD_TOKEN>
    Content-Type: text/markdown
    Body: raw markdown bytes

The serve.py do_PUT handler writes the body to:
    ROSBAG_DIR/<session_id>/report.md

meshview.html detects report.md presence via the /api/flights manifest
and renders a "View Report" link in the flight sidebar.

Auth model
----------
Bearer token via DRONEPI_RPI_UPLOAD_TOKEN env var (default: "dronepi").
Simple shared secret — sufficient for a closed hotspot network.
Not exposed to the internet.

Failure handling
----------------
Upload failure is non-fatal. The local report is always saved to cache
first. If the upload fails, the report is available locally at:
    DRONEPI_CACHE_DIR/<session_id>/report.md

References
----------
- serve.py confirmed as http.server.SimpleHTTPRequestHandler subclass —
  do_PUT will be added as a method override in Phase 5.
- httpx PUT: https://www.python-httpx.org/api/#httpx.Client.put
"""

import logging
import os
from pathlib import Path
from typing import Optional

import httpx

log = logging.getLogger(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────

def _rpi_base() -> str:
    host = os.environ.get("DRONEPI_RPI_HOST", "10.42.0.1")
    port = os.environ.get("DRONEPI_RPI_PORT", "8080")
    return f"http://{host}:{port}"

def _upload_token() -> str:
    return os.environ.get("DRONEPI_RPI_UPLOAD_TOKEN", "dronepi")

def _cache_dir() -> Path:
    return Path(os.environ.get(
        "DRONEPI_CACHE_DIR",
        Path.home() / ".dronepi-ground" / "cache",
    ))

_PUT_TIMEOUT = httpx.Timeout(15.0, connect=5.0)


# ══════════════════════════════════════════════════════════════════════════════
# ReportPublisher
# ══════════════════════════════════════════════════════════════════════════════

class ReportPublisher:
    """
    Saves report locally and uploads it to the RPi.

    Usage
    -----
        publisher = ReportPublisher()
        ok = publisher.publish(session_id, markdown_text)
        # ok=True: uploaded to RPi
        # ok=False: saved locally only (RPi upload failed)
    """

    def publish(self, session_id: str, report_md: str) -> bool:
        """
        Save report to local cache, then PUT to RPi.

        Returns True if RPi upload succeeded, False if only local save worked.
        Never raises.
        """
        # Always save locally first — this cannot be lost.
        local_path = self._save_local(session_id, report_md)
        if local_path:
            log.info(f"[Publisher] Report saved locally: {local_path}")
        else:
            log.error(f"[Publisher] Failed to save report locally for {session_id}.")

        # Upload to RPi — non-fatal if it fails.
        uploaded = self._put_to_rpi(session_id, report_md)
        if uploaded:
            log.info(f"[Publisher] Report uploaded to RPi for {session_id}.")
        else:
            log.warning(
                f"[Publisher] RPi upload failed for {session_id}. "
                "Report is available locally."
            )
        return uploaded

    def local_report_path(self, session_id: str) -> Path:
        """Return the local cache path for a session's report."""
        return _cache_dir() / session_id / "report.md"

    # ── Private ───────────────────────────────────────────────────────────────

    def _save_local(self, session_id: str, report_md: str) -> Optional[Path]:
        path = self.local_report_path(session_id)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(report_md, encoding="utf-8")
            return path
        except Exception as exc:
            log.error(f"[Publisher] Local save failed: {exc}")
            return None

    def _put_to_rpi(self, session_id: str, report_md: str) -> bool:
        """
        PUT report.md to RPi serve.py.

        Returns True on HTTP 200/201/204, False on any error.
        """
        url = f"{_rpi_base()}/rosbags/{session_id}/report.md"
        headers = {
            "Authorization": f"Bearer {_upload_token()}",
            "Content-Type":  "text/markdown; charset=utf-8",
        }
        try:
            with httpx.Client(timeout=_PUT_TIMEOUT) as c:
                resp = c.put(url, content=report_md.encode("utf-8"), headers=headers)
                if resp.status_code in (200, 201, 204):
                    return True
                log.warning(
                    f"[Publisher] RPi returned HTTP {resp.status_code} "
                    f"for PUT {url}: {resp.text[:100]}"
                )
                return False
        except httpx.ConnectError:
            log.debug("[Publisher] RPi unreachable — upload skipped.")
            return False
        except httpx.TimeoutException:
            log.warning("[Publisher] Upload timed out.")
            return False
        except Exception as exc:
            log.warning(f"[Publisher] Upload error: {exc}")
            return False
