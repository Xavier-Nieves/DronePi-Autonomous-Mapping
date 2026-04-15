"""
camera_capture.py — Continuous IMX477 capture with software waypoint trigger.

Architecture: Option B — persistent Picamera2 instance running in a background
thread. Waypoint arrival posts an event to the capture thread via threading.Event,
which saves the current frame immediately from the pre-rolling buffer. ISP
initialization occurs once at startup, not per capture, eliminating the
300–500ms per-capture overhead that a subprocess approach (Option A) would incur.

Design rationale vs. Option A (rpicam-still subprocess):
    Option A spawns a new process per waypoint. Each spawn re-initialises the
    ISP pipeline (~300–500ms), making capture latency proportional to the
    number of waypoints. At 20 waypoints this accumulates 6–10s of dead time
    during which the drone continues moving, introducing positional error into
    the photogrammetric reconstruction.

    Option B holds one Picamera2 instance alive for the entire mission. The ISP
    is already warm; trigger-to-save latency is bounded by frame period alone
    (~25ms at 40fps full resolution). Frame metadata (timestamp, GPS, ENU pose)
    is captured atomically with each frame and written to a sidecar JSON.

Sensor mode selection:
    2028×1520 @ 40fps, 12-bit SRGGB — maximum sensor coverage for mapping.
    The IMX477 does not expose a native 1280×720 mode; libcamera would
    subsample from 1332×990/120fps, reducing GSD. Full-frame is always
    preferred for photogrammetry.

Output layout (per session):
    /mnt/ssd/maps/<session>/flight_frames/
        frame_0001.jpg          JPEG, quality 95
        frame_0001.json         Sidecar: ros_timestamp, timestamp_iso,
                                ENU pose, GPS, waypoint index
        frame_0002.jpg
        frame_0002.json
        ...
        capture_log.csv         One row per frame for QA

    Frames are written to flight_frames/ inside the same session directory
    used by the mesh pipeline (/mnt/ssd/maps/<session>/), so all pipeline
    outputs for one flight live under a single session root.

    CHANGE LOG vs previous version:
        1. FLIGHT_BASE_DIR changed from /mnt/ssd/flights → /mnt/ssd/maps
           to unify session directories with postprocess_mesh.py output.
        2. output_dir now resolves to FLIGHT_BASE_DIR/session_id/flight_frames/
           to isolate frames from mesh outputs in the shared session root.
        3. ros_timestamp added to sidecar JSON. main.py injects the ROS node
           clock time via the trigger context dict so frame_ingestor.py can
           match frames to SLAM poses using PoseInterpolator without relying
           on wall-clock timestamps.

Thread safety:
    _trigger    threading.Event — main thread sets, capture thread clears
    _stop       threading.Event — main thread sets to shut down capture loop
    _frame_lock threading.Lock  — guards _latest_frame and _latest_metadata
    All public methods are safe to call from main.py's orchestrator thread.

Dependencies:
    picamera2   (pre-installed on Ubuntu 24.04 for RPi5, or: pip install picamera2)
    libcamera   (system package — present after rpicam-apps build)
    numpy, Pillow (pip install Pillow)

Usage (from handle_autonomous in main.py):
    camera = CameraCapture(session_id="scan_20260402_120000")
    camera.start()

    # At each waypoint arrival:
    meta = {
        "waypoint_index": i,
        "enu":            (ex, ey, ez),
        "gps":            (lat, lon, alt),
        "ros_timestamp":  node.get_clock().now().nanoseconds * 1e-9,
    }
    path = camera.trigger(meta)
    log(f"  [CAM] Saved {path}")

    camera.stop()
"""

import csv
import json
import logging
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────

# CHANGE 1: Unified with postprocess_mesh.py session root.
# All pipeline outputs for one flight now live under /mnt/ssd/maps/<session>/.
FLIGHT_BASE_DIR  = Path("/mnt/ssd/maps")

SENSOR_MODE      = {"size": (2028, 1520)}   # Full-frame, max GSD
FRAMERATE        = 40                        # IMX477 maximum at full resolution
JPEG_QUALITY     = 95                        # Lossless enough for photogrammetry
TRIGGER_TIMEOUT  = 2.0                       # Seconds to wait for trigger ack
CAPTURE_LOG_NAME = "capture_log.csv"
CSV_HEADERS      = [
    "frame_index", "filename", "timestamp_iso",
    "ros_timestamp",
    "waypoint_index",
    "enu_x", "enu_y", "enu_z",
    "gps_lat", "gps_lon", "gps_alt",
    "exif_written",
    "latency_ms",
]


class CameraCapture:
    """
    Continuous IMX477 capture controller with software waypoint trigger.

    Parameters
    ----------
    session_id : str
        Unique identifier for this flight session (e.g. 'scan_20260402_120000').
        Determines the output directory: FLIGHT_BASE_DIR / session_id / flight_frames/
    output_dir : Path or None
        Override output directory entirely. Defaults to the resolved path above.
    jpeg_quality : int
        JPEG compression quality (1–100). Default 95.
    """

    def __init__(
        self,
        session_id: str,
        output_dir: Optional[Path] = None,
        jpeg_quality: int = JPEG_QUALITY,
    ):
        self.session_id   = session_id
        # CHANGE 2: Frames live in flight_frames/ subfolder inside the session dir.
        # This isolates JPEGs and sidecars from PLY mesh outputs in the same root.
        self.output_dir   = (
            Path(output_dir) if output_dir
            else FLIGHT_BASE_DIR / session_id / "flight_frames"
        )
        self.jpeg_quality = jpeg_quality

        self._camera      = None          # Picamera2 instance
        self._thread      = None          # Background capture thread
        self._trigger     = threading.Event()
        self._stop        = threading.Event()
        self._frame_lock  = threading.Lock()
        self._latest_frame    = None      # numpy array (H, W, 3) RGB
        self._latest_meta     = {}        # Picamera2 frame metadata dict
        self._pending_context = {}        # dict posted by trigger() for sidecar
        self._frame_index     = 0
        self._csv_file        = None
        self._csv_writer      = None
        self._started         = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """
        Initialise Picamera2 and start the background capture thread.

        Returns True on success, False if Picamera2 is unavailable (e.g.
        running on a development machine without a camera).
        """
        try:
            from picamera2 import Picamera2
        except ImportError:
            logger.warning(
                "[CameraCapture] picamera2 not installed — camera disabled. "
                "Install with: pip install picamera2"
            )
            return False

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._open_csv()

        self._camera = Picamera2()

        # Configure for maximum-GSD full-frame mapping
        config = self._camera.create_still_configuration(
            main={"size": SENSOR_MODE["size"], "format": "RGB888"},
            controls={
                "FrameRate":      FRAMERATE,
                "AwbEnable":      True,
                "AeEnable":       True,
                "NoiseReductionMode": 2,   # HighQuality
            },
        )
        self._camera.configure(config)
        self._camera.start()

        # Allow AEC/AWB to converge before the first capture
        time.sleep(1.5)
        logger.info(
            f"[CameraCapture] IMX477 started — "
            f"{SENSOR_MODE['size'][0]}×{SENSOR_MODE['size'][1]} "
            f"@ {FRAMERATE}fps  output={self.output_dir}"
        )

        self._stop.clear()
        self._trigger.clear()
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="camera_capture"
        )
        self._thread.start()
        self._started = True
        return True

    def stop(self) -> None:
        """Stop the capture thread and release camera resources."""
        if not self._started:
            return
        self._stop.set()
        self._trigger.set()   # Unblock the capture loop if waiting
        if self._thread:
            self._thread.join(timeout=5.0)
        if self._camera:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception as exc:
                logger.warning(f"[CameraCapture] Error closing camera: {exc}")
        if self._csv_file:
            self._csv_file.close()
        self._started = False
        logger.info(
            f"[CameraCapture] Stopped — {self._frame_index} frames captured "
            f"to {self.output_dir}"
        )

    # ── Public Trigger API ────────────────────────────────────────────────────

    def trigger(self, context: dict = None, timeout: float = TRIGGER_TIMEOUT) -> Optional[Path]:
        """
        Signal the capture thread to save the current frame.

        Posts `context` as the sidecar metadata. The `ros_timestamp` key,
        if present, is written to the sidecar JSON so frame_ingestor.py
        can match frames to SLAM poses via PoseInterpolator.

        Parameters
        ----------
        context : dict
            Metadata attached to this capture. Recommended keys:
                waypoint_index : int
                enu            : (x, y, z) tuple in metres
                gps            : (lat, lon, alt) tuple
                ros_timestamp  : float — ROS clock time in seconds
                                 (node.get_clock().now().nanoseconds * 1e-9)
        timeout : float
            Maximum seconds to wait for the capture thread to acknowledge.

        Returns
        -------
        Path to saved JPEG, or None if camera is not running or timeout.
        """
        if not self._started:
            return None

        with self._frame_lock:
            self._pending_context = context or {}
        self._last_saved_path = None
        self._ack = threading.Event()

        self._trigger.set()

        if not self._ack.wait(timeout=timeout):
            logger.warning("[CameraCapture] Trigger timeout — frame not saved")
            return None

        return self._last_saved_path

    # ── Background Capture Loop ───────────────────────────────────────────────

    def _capture_loop(self) -> None:
        """
        Background thread: continuously captures frames into a rolling buffer
        and saves to disk when trigger() posts an event.

        The loop polls the camera at FRAMERATE; each iteration updates
        `_latest_frame`. When `_trigger` is set, the current frame is saved
        synchronously and the trigger is cleared.
        """
        while not self._stop.is_set():
            self._trigger.wait()
            if self._stop.is_set():
                break
            self._trigger.clear()

            t_trigger = time.monotonic()

            try:
                frame    = self._camera.capture_array("main")
                cam_meta = self._camera.capture_metadata()
            except Exception as exc:
                logger.error(f"[CameraCapture] Capture failed: {exc}")
                if hasattr(self, "_ack"):
                    self._ack.set()
                continue

            with self._frame_lock:
                context = dict(self._pending_context)

            self._frame_index += 1
            saved_path = self._save_frame(frame, cam_meta, context, t_trigger)
            self._last_saved_path = saved_path

            if hasattr(self, "_ack"):
                self._ack.set()

    # ── File I/O ──────────────────────────────────────────────────────────────

    def _save_frame(
        self,
        frame: np.ndarray,
        cam_meta: dict,
        context: dict,
        t_trigger: float,
    ) -> Path:
        """Save frame as JPEG and write sidecar JSON + CSV row."""
        from PIL import Image

        idx       = self._frame_index
        filename  = f"frame_{idx:04d}.jpg"
        jpg_path  = self.output_dir / filename
        json_path = self.output_dir / f"frame_{idx:04d}.json"

        enu    = context.get("enu",  (None, None, None))
        gps    = context.get("gps",  (None, None, None))
        ros_ts = context.get("ros_timestamp", None)

        # ── JPEG with GPS EXIF ────────────────────────────────────────────────
        # _build_gps_exif returns b"" gracefully if piexif is absent or GPS is
        # None — the plain JPEG is written instead and flagged in the sidecar.
        exif_bytes   = _build_gps_exif(gps[0], gps[1], gps[2])
        exif_written = len(exif_bytes) > 0

        img = Image.fromarray(frame, mode="RGB")
        if exif_written:
            img.save(str(jpg_path), format="JPEG",
                     quality=self.jpeg_quality, exif=exif_bytes)
        else:
            img.save(str(jpg_path), format="JPEG", quality=self.jpeg_quality)

        latency_ms = (time.monotonic() - t_trigger) * 1000.0
        from datetime import timezone as _tz
        ts_iso = datetime.now(tz=_tz.utc).isoformat()

        # ── Sidecar JSON ──────────────────────────────────────────────────────
        sidecar = {
            "frame_index":      idx,
            "filename":         filename,
            "timestamp_iso":    ts_iso,         # wall clock — human readable
            "ros_timestamp":    ros_ts,          # ROS clock — SLAM pose matching
            "waypoint_index":   context.get("waypoint_index"),
            "enu":              {"x": enu[0], "y": enu[1], "z": enu[2]},
            "gps":              {"lat": gps[0], "lon": gps[1], "alt": gps[2]},
            "gps_quality":      context.get("gps_quality", {}),
            "exif_gps_written": exif_written,   # QA flag for frame_ingestor
            "latency_ms":       round(latency_ms, 2),
            "sensor_mode":      f"{SENSOR_MODE['size'][0]}x{SENSOR_MODE['size'][1]}",
            "framerate":        FRAMERATE,
            "jpeg_quality":     self.jpeg_quality,
            "cam_exposure_us":  cam_meta.get("ExposureTime"),
            "cam_gain":         cam_meta.get("AnalogueGain"),
            "cam_awb_r":        cam_meta.get("ColourGains", [None, None])[0],
            "cam_awb_b":        cam_meta.get("ColourGains", [None, None])[1],
        }
        json_path.write_text(json.dumps(sidecar, indent=2))

        self._csv_writer.writerow([
            idx, filename, ts_iso,
            ros_ts,
            context.get("waypoint_index", ""),
            enu[0], enu[1], enu[2],
            gps[0], gps[1], gps[2],
            exif_written,
            round(latency_ms, 2),
        ])
        self._csv_file.flush()

        gps_str = (
            f"({gps[0]:.5f}, {gps[1]:.5f})" if gps[0] is not None else "no-fix"
        )
        logger.info(
            f"[CameraCapture] Frame {idx:04d}  "
            f"latency={latency_ms:.1f}ms  "
            f"exif={'ok' if exif_written else 'no-fix'}  "
            f"gps={gps_str}"
        )
        return jpg_path

    def _open_csv(self) -> None:
        """Open capture_log.csv for append (survives restarts)."""
        csv_path = self.output_dir / CAPTURE_LOG_NAME
        self.output_dir.mkdir(parents=True, exist_ok=True)
        write_header = not csv_path.exists()
        self._csv_file   = open(csv_path, "a", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        if write_header:
            self._csv_writer.writerow(CSV_HEADERS)
            self._csv_file.flush()

    # ── Context Manager Support ───────────────────────────────────────────────

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.stop()


# ── EXIF GPS helper (appended at module level) ────────────────────────────────

def _build_gps_exif(lat, lon, alt):
    """
    Build a minimal EXIF block containing a GPS IFD for embedding in JPEG.

    Writes six standard GPS tags recognised by ODM, QGIS, and consumer tools:
        GPSLatitudeRef / GPSLatitude
        GPSLongitudeRef / GPSLongitude
        GPSAltitudeRef  / GPSAltitude

    DMS rational encoding: degrees and minutes use denominator 1.
    Seconds use denominator 1000 for ~30 mm ground precision — well inside
    GPS CEP of 3-5 m for consumer receivers.

    Returns b"" if lat/lon is None or piexif is unavailable (non-fatal).
    """
    if lat is None or lon is None:
        return b""
    try:
        import piexif
    except ImportError:
        import logging
        logging.getLogger(__name__).warning(
            "[CameraCapture] piexif not installed - EXIF GPS skipped. "
            "Install: pip install piexif --break-system-packages"
        )
        return b""

    def to_dms(degrees):
        abs_d = abs(degrees)
        d = int(abs_d)
        m = int((abs_d - d) * 60)
        s = round(((abs_d - d) * 60 - m) * 60 * 1000)
        return ((d, 1), (m, 1), (s, 1000))

    alt_val = alt if alt is not None else 0.0
    gps_ifd = {
        piexif.GPSIFD.GPSLatitudeRef:   b"N" if lat >= 0 else b"S",
        piexif.GPSIFD.GPSLatitude:      to_dms(lat),
        piexif.GPSIFD.GPSLongitudeRef:  b"E" if lon >= 0 else b"W",
        piexif.GPSIFD.GPSLongitude:     to_dms(lon),
        piexif.GPSIFD.GPSAltitudeRef:   0,
        piexif.GPSIFD.GPSAltitude:      (int(abs(alt_val) * 100), 100),
    }
    try:
        return piexif.dump({"GPS": gps_ifd})
    except Exception as exc:
        import logging
        logging.getLogger(__name__).warning(
            f"[CameraCapture] piexif.dump failed: {exc} - EXIF GPS skipped"
        )
        return b""
