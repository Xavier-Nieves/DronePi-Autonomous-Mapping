"""
camera_capture.py — Continuous IMX477 capture with software waypoint trigger.

Architecture: single persistent Picamera2 instance running in a background
thread. The camera stays open for the full mission duration and continuously
refreshes an in-memory rolling frame buffer. Two consumers share that buffer:

    1. ROS image stream — a timer on the caller's ROS node publishes the latest
       frame on /arducam/image_raw at ROS_PUBLISH_FPS for Foxglove and
       HailoFlightNode. No second camera process is involved.

    2. Waypoint trigger — trigger() signals the capture loop to save the current
       rolling frame to disk as a JPEG + sidecar JSON immediately, without
       reopening or reconfiguring the camera.

Why one owner matters
---------------------
The IMX477 on RPi 5 with libcamera supports exactly one owner process at a
time. Two processes calling Picamera2() or rpicam-vid simultaneously will
produce a libcamera resource conflict. By keeping CameraCapture as the sole
owner and publishing /arducam/image_raw, all other consumers (Hailo, Foxglove)
subscribe to the ROS topic and receive frames without touching the hardware.

Camera configuration choice
----------------------------
create_preview_configuration() is used instead of create_still_configuration().
Preview configuration keeps the ISP pipeline warm and streams continuously at
the configured rate. Still configuration captures one frame per call at ~300ms
latency and is not designed for rolling use. buffer_count=4 provides a small
ring buffer inside libcamera to absorb brief processing spikes.

Sensor mode selection
---------------------
2028×1520 full-frame at 10fps for mapping quality. The ROS stream is published
from the same buffer. HailoFlightNode resizes frames internally to its HEF
input shapes (SCDepthV3: 320×256, YOLOv8m: 640×640) — the full-resolution
frames are passed through the topic without pre-scaling.

Output layout (per session)
---------------------------
/mnt/ssd/maps/<session>/flight_frames/
    frame_0001.jpg          JPEG, quality 95
    frame_0001.json         Sidecar: ros_timestamp, ENU pose, GPS, waypoint index
    frame_0002.jpg
    frame_0002.json
    ...
    capture_log.csv         One row per frame for QA

Thread safety
-------------
_trigger    threading.Event — main thread sets, capture loop clears
_stop       threading.Event — main thread sets to shut down
_frame_lock threading.Lock  — guards _latest_frame, _latest_meta, _latest_ros_stamp
All public methods are safe to call from main.py's orchestrator thread.

Usage (from _start_camera in main.py)
--------------------------------------
    cam = CameraCapture(
        session_id="scan_20260416_120000",
        ros_node=node._node,
        enable_ros_publish=True,
    )
    cam.start()

    # At each waypoint:
    cam.trigger({"waypoint_index": i, "enu": (ex, ey, ez), "gps": (lat, lon, alt),
                 "ros_timestamp": node._node.get_clock().now().nanoseconds * 1e-9})

    cam.stop()
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

FLIGHT_BASE_DIR  = Path("/mnt/ssd/rosbags")

SENSOR_MODE      = {"size": (2028, 1520)}   # Full-frame IMX477, max GSD
FRAMERATE        = 10                        # fps — preview stream rate
JPEG_QUALITY     = 95
TRIGGER_TIMEOUT  = 2.0
CAPTURE_LOG_NAME = "capture_log.csv"

# ROS image stream published from the rolling buffer
ROS_IMAGE_TOPIC  = "/arducam/image_raw"
ROS_PUBLISH_FPS  = 15.0   # Hz — must be ≤ FRAMERATE; 15Hz feeds Hailo reliably

CSV_HEADERS = [
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
        Unique identifier for this flight session.
    output_dir : Path or None
        Override output directory. Defaults to FLIGHT_BASE_DIR/session_id/flight_frames/
    jpeg_quality : int
        JPEG compression quality (1–100). Default 95.
    ros_node : rclpy.node.Node or None
        Existing ROS 2 node to attach the image stream publisher to.
        Pass node._node from MainNode during a real mission.
    enable_ros_publish : bool
        If True, publish the rolling frame buffer to /arducam/image_raw.
        Requires ros_node to be set.
    ros_publish_topic : str
        Override the default ROS image topic.
    ros_publish_fps : float
        Publish rate for the live stream. Should not exceed FRAMERATE.
    """

    def __init__(
        self,
        session_id: str,
        output_dir: Optional[Path] = None,
        jpeg_quality: int = JPEG_QUALITY,
        ros_node=None,
        enable_ros_publish: bool = False,
        ros_publish_topic: str = ROS_IMAGE_TOPIC,
        ros_publish_fps: float = ROS_PUBLISH_FPS,
    ):
        self.session_id  = session_id
        self.output_dir  = (
            Path(output_dir) if output_dir
            else FLIGHT_BASE_DIR / session_id / "flight_frames"
        )
        self.jpeg_quality = jpeg_quality

        # Picamera2 instance and capture thread
        self._camera     = None
        self._thread     = None
        self._trigger    = threading.Event()
        self._stop       = threading.Event()
        self._frame_lock = threading.Lock()

        # Rolling frame buffer — updated on every frame in _capture_loop
        self._latest_frame     = None   # numpy array (H, W, 3) RGB
        self._latest_meta      = {}     # Picamera2 metadata dict
        self._latest_ros_stamp = None   # _Stamp(sec, nanosec) or None
        self._live_frame_count = 0      # total frames captured (not saved)

        # Trigger state
        self._pending_context  = {}
        self._last_saved_path  = None
        self._ack              = None
        self._frame_index      = 0      # saved frame count

        # CSV log
        self._csv_file   = None
        self._csv_writer = None
        self._started    = False

        # ROS publisher (optional — attached to caller's node)
        self._ros_node           = ros_node
        self._enable_ros_publish = enable_ros_publish
        self._ros_publish_topic  = ros_publish_topic
        self._ros_publish_fps    = ros_publish_fps
        self._ros_pub            = None
        self._ros_timer          = None
        self._ros_image_type     = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """
        Initialise Picamera2 and start the rolling capture thread.

        Returns True on success, False if Picamera2 is unavailable.
        Camera failure is non-fatal — main.py continues without captures.
        """
        try:
            from picamera2 import Picamera2
        except ImportError:
            logger.warning(
                "[CameraCapture] picamera2 not importable — camera disabled."
            )
            return False

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._open_csv()

        self._camera = Picamera2()

        # Preview configuration for continuous streaming.
        # buffer_count=4 gives libcamera a small ring buffer to absorb
        # processing spikes without dropping frames.
        config = self._camera.create_preview_configuration(
            main={"size": SENSOR_MODE["size"], "format": "RGB888"},
            controls={
                "FrameRate": FRAMERATE,
                "AwbEnable": True,
                "AeEnable":  True,
            },
            buffer_count=4,
        )
        self._camera.configure(config)
        self._camera.start()

        # Allow AEC/AWB to converge before mission capture begins
        time.sleep(1.5)

        # Attach ROS publisher to the caller's node if requested
        if self._enable_ros_publish and self._ros_node is not None:
            self._attach_ros_publisher()

        self._stop.clear()
        self._trigger.clear()
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="camera_capture"
        )
        self._thread.start()
        self._started = True

        logger.info(
            f"[CameraCapture] IMX477 started — "
            f"{SENSOR_MODE['size'][0]}×{SENSOR_MODE['size'][1]} "
            f"@ {FRAMERATE}fps  output={self.output_dir}"
        )
        if self._ros_pub is not None:
            logger.info(
                f"[CameraCapture] ROS stream enabled — "
                f"topic={self._ros_publish_topic} @ {self._ros_publish_fps:.0f}Hz"
            )
        return True

    def stop(self) -> None:
        """Stop the capture thread and release all camera resources."""
        if not self._started:
            return

        self._stop.set()
        self._trigger.set()   # unblock any blocked wait path

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
            f"[CameraCapture] Stopped — "
            f"saved={self._frame_index}  "
            f"live_frames={self._live_frame_count}  "
            f"output={self.output_dir}"
        )

    # ── Public API ────────────────────────────────────────────────────────────

    def trigger(
        self, context: dict = None, timeout: float = TRIGGER_TIMEOUT
    ) -> Optional[Path]:
        """
        Save the current rolling frame to disk.

        The capture loop runs continuously. When trigger() is called it sets
        a flag that the loop checks on its next iteration — the current live
        frame is saved immediately without reopening or reconfiguring the camera.

        Parameters
        ----------
        context : dict
            Metadata attached to this capture. Keys used:
                waypoint_index : int
                enu            : (x, y, z) metres — SLAM position
                gps            : (lat, lon, alt)
                ros_timestamp  : float — ROS clock seconds
                gps_quality    : dict with hdop, satellites, reliable, source
        timeout : float
            Maximum seconds to wait for the capture loop to acknowledge.

        Returns
        -------
        Path to the saved JPEG, or None on timeout or if camera is not running.
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

    def get_latest_frame(self):
        """
        Return a copy of the latest rolling frame and its metadata.

        Returns
        -------
        (frame, meta, stamp) where:
            frame : numpy array (H, W, 3) RGB, or None if not yet available
            meta  : dict of Picamera2 metadata (ExposureTime, AnalogueGain, etc.)
            stamp : _Stamp(sec, nanosec) from FrameWallClock, or None
        """
        with self._frame_lock:
            if self._latest_frame is None:
                return None, {}, None
            return (
                self._latest_frame.copy(),
                dict(self._latest_meta),
                self._latest_ros_stamp,
            )

    # ── ROS Publishing ────────────────────────────────────────────────────────

    def _attach_ros_publisher(self) -> None:
        """
        Create a publisher and a timer on the caller's ROS node.

        The timer fires at ros_publish_fps and calls _publish_latest_frame(),
        which copies the current rolling buffer and publishes it as
        sensor_msgs/Image (rgb8). The publisher runs on the caller's executor —
        no separate ROS node or thread is created here.
        """
        try:
            from rclpy.qos import (
                QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            )
            from sensor_msgs.msg import Image
        except ImportError as exc:
            logger.warning(f"[CameraCapture] ROS publisher disabled: {exc}")
            return

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._ros_image_type = Image
        self._ros_pub = self._ros_node.create_publisher(
            Image, self._ros_publish_topic, sensor_qos
        )
        period = 1.0 / float(self._ros_publish_fps) if self._ros_publish_fps > 0 else 1.0
        self._ros_timer = self._ros_node.create_timer(period, self._publish_latest_frame)

    def _publish_latest_frame(self) -> None:
        """
        Publish the latest rolling frame as sensor_msgs/Image (rgb8).

        Frame is downscaled to 640x480 before publishing. The full-resolution
        2028x1520 frame = 9 MB per message which exceeds the Fast-DDS default
        fragment reassembly buffer (~1 MB) and is silently dropped between
        processes. 640x480 = 0.9 MB fits in a single DDS fragment and is
        received reliably by hailo_flight_node. Waypoint JPEGs are saved from
        the full-resolution buffer in _capture_loop and are unaffected.
        Hailo inference classes resize internally to their HEF input shapes
        (SCDepthV3: 320x256, YOLOv8m: 640x640) so 640x480 is sufficient.
        """
        if self._ros_pub is None or self._ros_image_type is None:
            return

        frame, _meta, ros_stamp = self.get_latest_frame()
        if frame is None:
            return

        import cv2
        publish_frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

        h, w = publish_frame.shape[:2]
        msg = self._ros_image_type()
        msg.header.frame_id = "camera_optical"

        if ros_stamp is not None:
            msg.header.stamp.sec     = ros_stamp.sec
            msg.header.stamp.nanosec = ros_stamp.nanosec
        else:
            msg.header.stamp = self._ros_node.get_clock().now().to_msg()

        msg.height       = h
        msg.width        = w
        msg.encoding     = "rgb8"
        msg.is_bigendian = False
        msg.step         = w * 3
        msg.data         = publish_frame.tobytes()
        self._ros_pub.publish(msg)

    # ── Background Capture Loop ───────────────────────────────────────────────

    def _capture_loop(self) -> None:
        """
        Continuously grab frames from Picamera2 and refresh the rolling buffer.

        This is the critical difference from the old implementation. The loop
        runs unconditionally — it does not wait for trigger(). Every frame
        updates _latest_frame so the ROS publisher always has fresh data.

        When _trigger is set by trigger(), the current frame is saved to disk
        on the next iteration and the trigger is cleared. The ack event is set
        so trigger() can return the saved path.

        Picamera2 API used here:
            capture_request() — acquires the next available frame from the
                                 libcamera buffer without blocking indefinitely.
            request.make_array("main") — extracts the frame as a numpy array.
            request.get_metadata() — returns per-frame ISP metadata.
            request.release() — returns the buffer to libcamera immediately.
        """
        while not self._stop.is_set():
            try:
                request  = self._camera.capture_request()
                frame    = request.make_array("main")
                cam_meta = request.get_metadata()
                request.release()
            except Exception as exc:
                logger.error(f"[CameraCapture] Capture failed: {exc}")
                time.sleep(0.1)
                continue

            ros_stamp = _ros_stamp_from_frame_wall_clock(cam_meta)

            # Update the rolling buffer under lock
            with self._frame_lock:
                self._latest_frame     = np.ascontiguousarray(frame.copy())
                self._latest_meta      = dict(cam_meta)
                self._latest_ros_stamp = ros_stamp
                self._live_frame_count += 1

                do_save = self._trigger.is_set()
                if do_save:
                    context       = dict(self._pending_context)
                    frame_to_save = self._latest_frame.copy()
                    meta_to_save  = dict(self._latest_meta)

            if not do_save:
                continue

            # Save triggered frame outside the lock
            self._trigger.clear()
            t_trigger = time.monotonic()
            self._frame_index += 1
            try:
                saved = self._save_frame(frame_to_save, meta_to_save, context, t_trigger)
                self._last_saved_path = saved
            except Exception as exc:
                logger.error(f"[CameraCapture] Save failed: {exc}")
                self._last_saved_path = None
            finally:
                if self._ack is not None:
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

        sidecar = {
            "frame_index":      idx,
            "filename":         filename,
            "timestamp_iso":    ts_iso,
            "ros_timestamp":    ros_ts,
            "waypoint_index":   context.get("waypoint_index"),
            "enu":              {"x": enu[0], "y": enu[1], "z": enu[2]},
            "gps":              {"lat": gps[0], "lon": gps[1], "alt": gps[2]},
            "gps_quality":      context.get("gps_quality", {}),
            "exif_gps_written": exif_written,
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


# ── Helper functions ──────────────────────────────────────────────────────────

def _ros_stamp_from_frame_wall_clock(cam_meta: dict):
    """
    Convert Picamera2 FrameWallClock nanoseconds to a lightweight stamp object.

    FrameWallClock is the wall-clock time in nanoseconds at which the frame
    was captured by the ISP. It is more accurate than time.time() taken after
    capture_request() returns because it avoids the libcamera processing delay.

    Returns a _Stamp(sec, nanosec) compatible with sensor_msgs/Image header,
    or None if FrameWallClock is not in the metadata.
    """
    ns = cam_meta.get("FrameWallClock")
    if ns is None:
        return None

    class _Stamp:
        __slots__ = ("sec", "nanosec")
        def __init__(self, sec, nanosec):
            self.sec     = sec
            self.nanosec = nanosec

    return _Stamp(int(ns // 1_000_000_000), int(ns % 1_000_000_000))


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
