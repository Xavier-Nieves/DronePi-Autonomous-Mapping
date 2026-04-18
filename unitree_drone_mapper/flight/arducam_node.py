#!/usr/bin/env python3
"""
IMX477 ROS 2 publisher via Picamera2/libcamera.

Publishes sensor_msgs/Image on /arducam/image_raw using a single persistent
Picamera2 instance.

Architecture decision
---------------------
Picamera2 is now working in the target environment and uses libcamera under the
hood, so the old rpicam-vid MJPEG pipe workaround is no longer the preferred
path. This node is kept as a lightweight bench / development publisher for
camera-only validation.

Important runtime note
----------------------
Do not run this node at the same time as main.py mission capture. The Raspberry
Pi camera should have one owner process. During a real mapping mission, main.py
should own CameraCapture and publish /arducam/image_raw itself from the same
rolling buffer that it uses for waypoint captures.

Topic
-----
  /arducam/image_raw   sensor_msgs/Image   encoding=rgb8

Usage
-----
  source /opt/ros/jazzy/setup.bash
  conda activate dronepi
  python3 arducam_node.py
  python3 arducam_node.py --width 1280 --height 960 --fps 5

Prerequisites
-------------
  picamera2 importable in the active Python environment
  libcamera importable in the active Python environment
"""

import argparse
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class ArducamPublisher(Node):
    """Bench ROS 2 image publisher backed directly by Picamera2."""

    def __init__(self, width: int, height: int, fps: float, topic: str):
        super().__init__("arducam_publisher")

        self.width = width
        self.height = height
        self.fps = fps
        self.topic = topic
        self._stop = threading.Event()
        self._thread = None
        self._camera = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(Image, self.topic, sensor_qos)

        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError(
                "picamera2 is not importable in this environment. "
                "Fix the system/conda install first."
            ) from exc

        self._Picamera2 = Picamera2

    def start(self) -> None:
        """Open the camera once and begin the publish loop."""
        self._camera = self._Picamera2()
        config = self._camera.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            controls={"FrameRate": self.fps, "AwbEnable": True, "AeEnable": True},
            buffer_count=4,
        )
        self._camera.configure(config)
        self._camera.start()
        time.sleep(1.0)

        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"IMX477 started — publishing {self.width}x{self.height} @ {self.fps:.1f}Hz on {self.topic}"
        )

    def stop(self) -> None:
        """Stop the publish loop and release the camera."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._camera is not None:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception as exc:
                self.get_logger().warning(f"Camera close warning: {exc}")
        self.get_logger().info("IMX477 publisher stopped")

    def _publish_loop(self) -> None:
        """Grab frames continuously and publish them as rgb8 images."""
        period = 1.0 / self.fps if self.fps > 0 else 0.2
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                request = self._camera.capture_request()
                frame = request.make_array("main")
                meta = request.get_metadata()
                request.release()
            except Exception as exc:
                self.get_logger().error(f"Capture failed: {exc}")
                time.sleep(0.1)
                continue

            msg = Image()
            msg.header.frame_id = "camera_optical"
            frame_wall_clock = meta.get("FrameWallClock")
            if frame_wall_clock is not None:
                msg.header.stamp.sec = int(frame_wall_clock // 1_000_000_000)
                msg.header.stamp.nanosec = int(frame_wall_clock % 1_000_000_000)
            else:
                msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.pub.publish(msg)

            dt = time.monotonic() - t0
            sleep_s = period - dt
            if sleep_s > 0:
                time.sleep(sleep_s)


def parse_args():
    parser = argparse.ArgumentParser(description="IMX477 Picamera2 ROS 2 publisher")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=960)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--topic", default="/arducam/image_raw")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ArducamPublisher(args.width, args.height, args.fps, args.topic)

    def _shutdown(*_args):
        node.get_logger().info("Shutdown requested")
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            _shutdown()


if __name__ == "__main__":
    main()
