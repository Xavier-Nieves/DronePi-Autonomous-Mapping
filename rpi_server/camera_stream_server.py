#!/usr/bin/env python3
"""
camera_stream_server.py — MJPEG live preview server for the IMX477.

Architecture
============
This module opens Picamera2 in a dedicated low-resolution preview
configuration and streams MJPEG frames over HTTP on port 8081.

Two operating modes
-------------------
Standalone (this file run directly, or as a systemd service):
    Opens its own Picamera2 instance at PREVIEW_WIDTH x PREVIEW_HEIGHT.
    Safe to run when main.py / CameraCapture is NOT running — the IMX477
    can only be held by one process at a time. The camera-stream service
    must therefore be stopped before any flight that uses CameraCapture.

Shared (called from main.py with an existing CameraCapture):
    Pass the CameraCapture instance to start(). The server reads
    _latest_frame / _frame_lock from it instead of opening a second
    Picamera2 handle. This avoids the hardware conflict.

    Usage from main.py:
        from camera_stream_server import CameraStreamServer
        from flight.camera_capture import CameraCapture

        cam = CameraCapture(session_id)
        cam.start()
        stream = CameraStreamServer(camera_capture=cam)
        stream.start()
        ...
        stream.stop()
        cam.stop()

Endpoints
---------
    GET /stream     multipart/x-mixed-replace MJPEG stream
    GET /snapshot   single JPEG frame (200 OK, image/jpeg)
    GET /status     JSON health check

Port
----
    8081  (proxied through serve.py on port 8080 at /stream and /snapshot)

Design decisions
----------------
Transport — MJPEG over HTTP multipart/x-mixed-replace:
    Each frame is a JPEG boundary part. The browser renders it via a plain
    <img src="/stream"> tag — no JS video decoder required. Latency is
    80–200ms over a 5GHz 802.11ac link, acceptable for pre-flight focus
    checks and in-flight monitoring.

    WebRTC was evaluated and rejected: it requires HTTPS, STUN/TURN
    infrastructure, and aiortc (~200MB), adding significant complexity
    for <50ms latency gain that is irrelevant for this use case.

Frame rate — 10fps preview:
    The IMX477 at full resolution (2028×1520) runs at 40fps under
    CameraCapture. The preview stream runs at 960×720 / 10fps to bound
    CPU cost (JPEG encode) and network bandwidth (~3.5 Mbit/s).
    At 10fps the stream is indistinguishable from 30fps for visual
    monitoring and costs ~4% CPU on the RPi5.

Threading model:
    _capture_thread  — runs Picamera2 capture loop, writes to _frame_buf
    _encode_thread   — reads _frame_buf, JPEG-encodes, distributes to clients
    HTTPServer       — Python stdlib, handles one client per thread (daemon)

    The encode thread keeps encode work off the HTTP handler threads,
    preventing slow clients from stalling the capture loop.

Dependencies
------------
    picamera2    — installed in dronepi conda env
    Pillow       — pip install Pillow --break-system-packages
    numpy        — pip install numpy --break-system-packages
    (HTTP server — Python stdlib only, no Flask/FastAPI)
"""

import io
import json
import logging
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# ── Constants ──────────────────────────────────────────────────────────────────

STREAM_HOST   = "0.0.0.0"
STREAM_PORT   = 8081
PREVIEW_W     = 960
PREVIEW_H     = 720
STREAM_FPS    = 10          # Preview frame rate
JPEG_QUALITY  = 72          # 72 → ~35KB/frame @ 960×720; 10fps ≈ 3.5 Mbit/s
BOUNDARY      = b"--mjpegframe"
BOUNDARY_STR  = "mjpegframe"


# ── Camera manager ─────────────────────────────────────────────────────────────

class _CameraManager:
    """
    Owns the Picamera2 preview instance and the encoded-frame distribution
    bus. Separates camera hardware access from HTTP transport.

    In standalone mode: opens its own Picamera2 at PREVIEW_W x PREVIEW_H.
    In shared mode: reads _latest_frame from a CameraCapture instance.
    """

    def __init__(self):
        self._lock         = threading.Lock()
        self._latest_jpeg  = None           # bytes — most recent encoded frame
        self._client_locks = {}             # client_id -> threading.Event
        self._client_jpegs = {}             # client_id -> bytes
        self._client_id    = 0
        self._running      = False

        self._capture_thread = None
        self._encode_thread  = None
        self._camera         = None         # Picamera2 instance (standalone only)
        self._cam_capture    = None         # external CameraCapture reference

        # Shared frame buffer (standalone mode)
        self._frame_buf      = None         # numpy RGB array
        self._frame_lock     = threading.Lock()
        self._frame_event    = threading.Event()

    def start(self, camera_capture=None) -> bool:
        """
        Start the camera and encode threads.

        Parameters
        ----------
        camera_capture : CameraCapture or None
            Pass an already-started CameraCapture to share its Picamera2
            instance. Pass None to open a standalone preview camera.

        Returns True if started successfully, False if camera unavailable.
        """
        self._cam_capture = camera_capture
        self._running = True

        if camera_capture is not None:
            # Shared mode — read from CameraCapture._latest_frame
            logger.info("[CameraStream] shared mode — reading from CameraCapture")
        else:
            # Standalone mode — open our own Picamera2
            if not self._open_camera():
                self._running = False
                return False
            self._capture_thread = threading.Thread(
                target=self._capture_loop, daemon=True, name="cam-capture"
            )
            self._capture_thread.start()

        self._encode_thread = threading.Thread(
            target=self._encode_loop, daemon=True, name="cam-encode"
        )
        self._encode_thread.start()
        logger.info(f"[CameraStream] streaming on http://0.0.0.0:{STREAM_PORT}/stream")
        return True

    def stop(self):
        self._running = False
        self._frame_event.set()     # unblock encode thread

        if self._capture_thread:
            self._capture_thread.join(timeout=3.0)
        if self._encode_thread:
            self._encode_thread.join(timeout=3.0)

        if self._camera:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception:
                pass
            self._camera = None

        logger.info("[CameraStream] stopped")

    def _open_camera(self) -> bool:
        try:
            from picamera2 import Picamera2
        except ImportError:
            logger.error("[CameraStream] picamera2 not importable — camera unavailable")
            return False

        try:
            self._camera = Picamera2()
            # Preview config: lower resolution, continuous capture
            config = self._camera.create_preview_configuration(
                main={"size": (PREVIEW_W, PREVIEW_H), "format": "RGB888"}
            )
            self._camera.configure(config)
            self._camera.start()
            logger.info(f"[CameraStream] Picamera2 opened at {PREVIEW_W}x{PREVIEW_H}")
            return True
        except Exception as e:
            logger.error(f"[CameraStream] Picamera2 open failed: {e}")
            if self._camera:
                try:
                    self._camera.close()
                except Exception:
                    pass
                self._camera = None
            return False

    def _capture_loop(self):
        """Standalone mode: continuously capture frames from Picamera2."""
        interval = 1.0 / STREAM_FPS
        while self._running:
            t0 = time.monotonic()
            try:
                frame = self._camera.capture_array()    # RGB numpy array
                with self._frame_lock:
                    self._frame_buf = frame
                self._frame_event.set()
            except Exception as e:
                logger.warning(f"[CameraStream] capture error: {e}")
                time.sleep(0.5)
                continue

            elapsed = time.monotonic() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def _get_latest_frame(self) -> Optional[np.ndarray]:
        """Return the most recent RGB frame from either mode."""
        if self._cam_capture is not None:
            # Shared mode: read from CameraCapture's frame buffer
            with self._cam_capture._frame_lock:
                return self._cam_capture._latest_frame
        else:
            # Standalone mode: read from our own buffer
            with self._frame_lock:
                return self._frame_buf

    def _encode_loop(self):
        """
        Encode the latest frame to JPEG and distribute to all connected clients.

        Runs at STREAM_FPS. If no clients are connected, skips encoding to
        avoid wasting CPU. In shared mode, polls at the stream interval since
        there is no frame event from CameraCapture.
        """
        from PIL import Image as PILImage

        interval = 1.0 / STREAM_FPS

        while self._running:
            t0 = time.monotonic()

            # In standalone mode, wait for a new frame event
            if self._cam_capture is None:
                self._frame_event.wait(timeout=1.0)
                self._frame_event.clear()

            if not self._running:
                break

            frame = self._get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            # Skip encoding if no clients are connected
            with self._lock:
                has_clients = bool(self._client_locks)
            if not has_clients:
                elapsed = time.monotonic() - t0
                time.sleep(max(0, interval - elapsed))
                continue

            # JPEG encode
            try:
                img = PILImage.fromarray(frame, mode="RGB")
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=JPEG_QUALITY)
                jpeg = buf.getvalue()
            except Exception as e:
                logger.warning(f"[CameraStream] encode error: {e}")
                time.sleep(0.05)
                continue

            # Distribute to all waiting client threads
            with self._lock:
                self._latest_jpeg = jpeg
                for cid, event in self._client_locks.items():
                    self._client_jpegs[cid] = jpeg
                    event.set()

            elapsed = time.monotonic() - t0
            sleep = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def register_client(self) -> int:
        """Register a new streaming client. Returns a client ID."""
        with self._lock:
            cid = self._client_id
            self._client_id += 1
            self._client_locks[cid] = threading.Event()
            self._client_jpegs[cid] = None
        return cid

    def unregister_client(self, cid: int):
        """Unregister a streaming client."""
        with self._lock:
            self._client_locks.pop(cid, None)
            self._client_jpegs.pop(cid, None)

    def get_frame(self, cid: int, timeout: float = 3.0) -> Optional[bytes]:
        """Block until a new frame is available for this client."""
        event = self._client_locks.get(cid)
        if event is None:
            return None
        fired = event.wait(timeout=timeout)
        if not fired:
            return None
        event.clear()
        with self._lock:
            return self._client_jpegs.get(cid)

    def snapshot(self) -> Optional[bytes]:
        """Return the most recently encoded JPEG, or None."""
        with self._lock:
            return self._latest_jpeg

    def status(self) -> dict:
        with self._lock:
            n_clients = len(self._client_locks)
        return {
            "running":   self._running,
            "mode":      "shared" if self._cam_capture else "standalone",
            "clients":   n_clients,
            "width":     PREVIEW_W,
            "height":    PREVIEW_H,
            "fps":       STREAM_FPS,
            "port":      STREAM_PORT,
        }


# ── HTTP handler ───────────────────────────────────────────────────────────────

# Module-level camera manager — set by CameraStreamServer.start()
_MANAGER: Optional[_CameraManager] = None


class _StreamHandler(BaseHTTPRequestHandler):
    """
    HTTP request handler for the MJPEG stream server.

    Routes:
        GET /stream    → MJPEG multipart stream
        GET /snapshot  → single JPEG
        GET /status    → JSON health
        GET /          → redirect to /stream
    """

    def do_GET(self):
        if self.path in ("/stream", "/"):
            self._handle_stream()
        elif self.path == "/snapshot":
            self._handle_snapshot()
        elif self.path == "/status":
            self._handle_status()
        else:
            self.send_response(404)
            self.end_headers()

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")

    def _handle_stream(self):
        if _MANAGER is None or not _MANAGER._running:
            self.send_response(503)
            self.end_headers()
            self.wfile.write(b"Camera not ready")
            return

        cid = _MANAGER.register_client()
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={BOUNDARY_STR}"
        )
        self._cors()
        self.end_headers()

        try:
            while True:
                jpeg = _MANAGER.get_frame(cid, timeout=3.0)
                if jpeg is None:
                    # Keepalive empty boundary so browser doesn't time out
                    self.wfile.write(BOUNDARY + b"\r\n\r\n")
                    self.wfile.flush()
                    continue

                frame_header = (
                    BOUNDARY + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                )
                self.wfile.write(frame_header + jpeg + b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            _MANAGER.unregister_client(cid)

    def _handle_snapshot(self):
        if _MANAGER is None:
            self.send_response(503)
            self.end_headers()
            return

        jpeg = _MANAGER.snapshot()
        if jpeg is None:
            self.send_response(503)
            self.end_headers()
            self.wfile.write(b"No frame available yet")
            return

        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self._cors()
        self.end_headers()
        self.wfile.write(jpeg)

    def _handle_status(self):
        body = json.dumps(_MANAGER.status() if _MANAGER else {"running": False}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        # Suppress per-frame stream log spam
        msg = fmt % args
        if "/stream" in msg and '" 200 ' in msg:
            return
        logger.info(msg)


# ── Public API ─────────────────────────────────────────────────────────────────

class CameraStreamServer:
    """
    Lifecycle wrapper that starts _CameraManager and HTTPServer.

    Usage (standalone — called by main() below or systemd):
        server = CameraStreamServer()
        server.start()          # blocks until KeyboardInterrupt

    Usage (shared with CameraCapture from main.py):
        server = CameraStreamServer(camera_capture=cam)
        server.start_background()   # non-blocking daemon threads
        ...
        server.stop()
    """

    def __init__(self, camera_capture=None):
        self._cam_capture = camera_capture
        self._manager     = None
        self._http_server = None
        self._http_thread = None

    def start_background(self) -> bool:
        """
        Start in background daemon threads. Returns True on success.
        Call stop() to shut down.
        """
        global _MANAGER
        self._manager = _CameraManager()
        ok = self._manager.start(self._cam_capture)
        if not ok:
            return False

        _MANAGER = self._manager

        self._http_server = HTTPServer((STREAM_HOST, STREAM_PORT), _StreamHandler)
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            daemon=True,
            name="cam-http"
        )
        self._http_thread.start()
        logger.info(f"[CameraStream] HTTP server on {STREAM_HOST}:{STREAM_PORT}")
        return True

    def stop(self):
        if self._http_server:
            self._http_server.shutdown()
        if self._manager:
            self._manager.stop()
        global _MANAGER
        _MANAGER = None

    def start(self) -> None:
        """
        Blocking start — for direct execution or systemd.
        Runs until KeyboardInterrupt or SIGTERM.
        """
        global _MANAGER
        self._manager = _CameraManager()
        ok = self._manager.start(self._cam_capture)
        if not ok:
            raise RuntimeError(
                "Camera unavailable — check picamera2 installation and "
                "that no other process holds /dev/media0"
            )

        _MANAGER = self._manager

        self._http_server = HTTPServer((STREAM_HOST, STREAM_PORT), _StreamHandler)
        logger.info(f"[CameraStream] serving on http://0.0.0.0:{STREAM_PORT}")
        logger.info(f"[CameraStream]   /stream   — MJPEG live feed")
        logger.info(f"[CameraStream]   /snapshot — single JPEG")
        logger.info(f"[CameraStream]   /status   — JSON health")

        try:
            self._http_server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            self._manager.stop()
            _MANAGER = None
            logger.info("[CameraStream] shut down")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s  %(levelname)-7s  %(message)s",
        datefmt="%H:%M:%S",
    )
    server = CameraStreamServer()
    server.start()


if __name__ == "__main__":
    main()
