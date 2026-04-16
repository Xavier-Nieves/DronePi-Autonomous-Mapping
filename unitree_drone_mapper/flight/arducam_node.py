#!/usr/bin/env python3
"""nodes/arducam_node.py — IMX477 ROS 2 publisher via rpicam-vid MJPEG pipe.

Publishes sensor_msgs/Image on /arducam/image_raw by reading MJPEG frames
from a rpicam-vid subprocess piped to stdout.

Architecture decision
---------------------
picamera2 requires system Python packages built alongside rpicam-apps.
On this Pi the rpicam stack was built from source without the Python
bindings. rpicam-vid is confirmed working (612KB/3s stdout test passed)
so this node uses it directly as a subprocess.

Frame extraction
----------------
rpicam-vid --codec mjpeg -o - writes a raw MJPEG stream: concatenated
JPEG images with no container. Each JPEG starts with FF D8 (SOI) and
ends with FF D9 (EOI). The reader thread scans for these markers to
extract complete frames. --flush ensures each frame is flushed to stdout
immediately rather than buffered, minimising pipe latency.

Timestamp
---------
The ROS clock stamp is taken the moment FF D9 is found in the stream,
which is the closest approximation to actual capture time achievable
without picamera2 metadata. --flush keeps this within ~30ms of
actual sensor exposure. The sync test tolerance is 45ms.

Topic
-----
  /arducam/image_raw   sensor_msgs/Image   encoding=bgr8

Usage
-----
  source /opt/ros/jazzy/setup.bash
  conda activate dronepi
  python3 nodes/arducam_node.py
  python3 nodes/arducam_node.py --width 1280 --height 960 --fps 5

Prerequisites
-------------
  rpicam-vid in PATH (confirmed /usr/local/bin/rpicam-vid)
  opencv-python in dronepi env:
    pip install opencv-python-headless
"""

import argparse
import os
import signal
import subprocess
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

_DEFAULT_WIDTH  = 1280
_DEFAULT_HEIGHT = 960
_DEFAULT_FPS    = 10

# JPEG boundary markers
_SOI = b'\xff\xd8'
_EOI = b'\xff\xd9'
_READ_CHUNK = 65536


class ArducamNode(Node):
    """IMX477 publisher using rpicam-vid MJPEG stdout pipe.

    Two-thread model:
      _reader_thread  — reads raw bytes from rpicam-vid stdout, assembles
                        complete JPEG frames, stores latest in _frame_buf
      ROS timer       — decodes latest JPEG, publishes sensor_msgs/Image

    This prevents the blocking pipe read from stalling the ROS executor.
    """

    def __init__(self, width: int, height: int, fps: int) -> None:
        super().__init__("arducam_node")

        self._width  = width
        self._height = height
        self._fps    = fps

        self._pub = self.create_publisher(Image, "/arducam/image_raw", _SENSOR_QOS)

        self._frame_buf:   bytes       = b""
        self._frame_stamp              = None
        self._frame_lock               = threading.Lock()
        self._frame_count: int         = 0
        self._pub_count:   int         = 0
        self._stop_event               = threading.Event()
        self._proc:        subprocess.Popen = None
        self._reader_thread: threading.Thread = None

        self._start_capture()
        self.create_timer(1.0 / self._fps, self._timer_cb)

        self.get_logger().info(
            f"ArducamNode started — {self._width}x{self._height} "
            f"@ {self._fps} Hz  topic=/arducam/image_raw"
        )

    def _start_capture(self) -> None:
        """Launch rpicam-vid writing MJPEG to stdout indefinitely."""
        cmd = [
            "rpicam-vid",
            "--codec",     "mjpeg",
            "-o",          "-",
            "--width",     str(self._width),
            "--height",    str(self._height),
            "--framerate", str(self._fps),
            "--timeout",   "0",        # run until killed
            "--nopreview",
            "--flush",                 # flush stdout per frame — reduces latency
        ]

        self.get_logger().info(f"Launching: {' '.join(cmd)}")

        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,             # unbuffered — critical for low latency
            preexec_fn=os.setsid,  # own process group for clean kill
        )

        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
            name="arducam_reader",
        )
        self._reader_thread.start()
        self.get_logger().info(f"rpicam-vid PID {self._proc.pid}")

    def _reader_loop(self) -> None:
        """Read MJPEG stream and extract complete JPEG frames.

        MJPEG is raw concatenated JPEGs with no container framing.
        Scan for FF D8 (SOI) and FF D9 (EOI) to find frame boundaries.
        Source: JPEG specification ISO/IEC 10918-1.
        """
        buf = b""
        start_idx = -1

        while not self._stop_event.is_set():
            try:
                chunk = self._proc.stdout.read(_READ_CHUNK)
            except Exception:
                break

            if not chunk:
                if not self._stop_event.is_set():
                    self.get_logger().warn("rpicam-vid stdout closed unexpectedly")
                break

            buf += chunk

            # Extract all complete frames from the current buffer
            while True:
                if start_idx < 0:
                    idx = buf.find(_SOI)
                    if idx < 0:
                        buf = buf[-1:]   # keep last byte — FF might be split
                        break
                    start_idx = idx

                eoi_idx = buf.find(_EOI, start_idx + 2)
                if eoi_idx < 0:
                    break              # frame incomplete — wait for more bytes

                # Complete frame found — stamp immediately at EOI
                frame_bytes = buf[start_idx: eoi_idx + 2]
                stamp = self.get_clock().now()

                with self._frame_lock:
                    self._frame_buf   = frame_bytes
                    self._frame_stamp = stamp
                    self._frame_count += 1

                buf       = buf[eoi_idx + 2:]
                start_idx = -1

        self.get_logger().info(
            f"Reader thread exiting — {self._frame_count} frames assembled"
        )

    def _timer_cb(self) -> None:
        """Decode latest JPEG frame and publish as sensor_msgs/Image."""
        with self._frame_lock:
            jpeg_bytes = self._frame_buf
            stamp      = self._frame_stamp

        if not jpeg_bytes or stamp is None:
            return

        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if img is None:
            self.get_logger().warn("cv2.imdecode failed — skipping frame")
            return

        h, w = img.shape[:2]

        msg = Image()
        msg.header.stamp    = stamp.to_msg()
        msg.header.frame_id = "camera_optical"
        msg.height          = h
        msg.width           = w
        msg.encoding        = "bgr8"
        msg.is_bigendian    = False
        msg.step            = w * 3
        msg.data            = img.tobytes()

        self._pub.publish(msg)
        self._pub_count += 1

        if self._pub_count % 50 == 0:
            self.get_logger().info(
                f"Published {self._pub_count} frames  ({w}x{h} bgr8  "
                f"assembled={self._frame_count})"
            )

    def destroy_node(self) -> None:
        """Kill rpicam-vid and join reader thread cleanly."""
        self.get_logger().info("Shutting down ArducamNode...")
        self._stop_event.set()

        if self._proc is not None and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except Exception:
                    pass

        if self._reader_thread is not None:
            self._reader_thread.join(timeout=3.0)

        self.get_logger().info(
            f"ArducamNode stopped — {self._pub_count} frames published"
        )
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ArducamNode — IMX477 via rpicam-vid MJPEG pipe"
    )
    parser.add_argument("--width",  type=int, default=_DEFAULT_WIDTH,
                        help=f"Capture width  (default {_DEFAULT_WIDTH})")
    parser.add_argument("--height", type=int, default=_DEFAULT_HEIGHT,
                        help=f"Capture height (default {_DEFAULT_HEIGHT})")
    parser.add_argument("--fps",    type=int, default=_DEFAULT_FPS,
                        help=f"Publish rate Hz (default {_DEFAULT_FPS})")
    args = parser.parse_args()

    rclpy.init()
    node = ArducamNode(width=args.width, height=args.height, fps=args.fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
