"""hailo/hailo_flight_node.py — ROS 2 node for in-flight Hailo-8 inference.

Runs in hailo_inference_env (NOT the Conda dronepi env). Communicates with
the main flight stack exclusively via ROS 2 topics — no shared imports,
no shared environment.

Camera architecture change
--------------------------
This node no longer owns the IMX477 camera. During a real mission,
CameraCapture (flight/camera_capture.py) holds the single persistent
Picamera2 instance and publishes /arducam/image_raw. This node subscribes
to that topic and runs inference on the received frames.

This eliminates the hardware conflict that occurred when both this node
and CameraCapture attempted to open /dev/media0 simultaneously.

Two consumers, one camera owner:
    CameraCapture (Picamera2, dronepi env)
        → /arducam/image_raw  (sensor_msgs/Image, rgb8, 15Hz)
            ├─ Foxglove Studio  (live map overlay, display only)
            └─ HailoFlightNode  (this file, hailo_inference_env)
                    → /hailo/optical_flow  (geometry_msgs/TwistStamped)
                    → /hailo/ground_class  (std_msgs/String)

Bench / dev use without main.py running
-----------------------------------------
If /arducam/image_raw is not publishing (e.g. bench test without mission),
this node falls back to RpicamCapture — the same direct rpicam-vid stdout
reader used in the test scripts. The fallback is triggered automatically
if no image message arrives within CAMERA_FALLBACK_TIMEOUT_S.

Published topics:
    /hailo/optical_flow  (geometry_msgs/TwistStamped)
        linear.x = vx (forward velocity, m/s, body frame)
        linear.y = vy (lateral velocity, m/s, body frame)
        linear.z = confidence (0.0–1.0, repurposed field)

    /hailo/ground_class  (std_msgs/String)
        data = JSON: {"label": "SAFE_LAND", "confidence": 0.87, "latency_ms": 14.2}

Subscribed topics:
    /arducam/image_raw   (sensor_msgs/Image)
        Primary frame source — published by CameraCapture during mission.
    /mavros/local_position/pose  (geometry_msgs/PoseStamped)
        Used to extract current AGL for metric flow scaling.

Environment: hailo_inference_env
    ~/hailo_inference_env/bin/python3 hailo/hailo_flight_node.py
"""

import json
import os
import sys
import threading
import time
from pathlib import Path

# ── Model Paths ───────────────────────────────────────────────────────────────

_HAILO_RESOURCES = Path("/usr/local/hailo/resources/models/hailo8")

FLOW_HEF_PATH  = Path(os.environ.get(
    "HAILO_FLOW_HEF",
    str(_HAILO_RESOURCES / "scdepthv3.hef"),
))
CLASS_HEF_PATH = Path(os.environ.get(
    "HAILO_CLASS_HEF",
    str(_HAILO_RESOURCES / "yolov8m.hef"),
))

# ── Camera Config ─────────────────────────────────────────────────────────────

# Resolution at which CameraCapture publishes /arducam/image_raw.
# Hailo inference classes resize internally to their HEF input shapes —
# this value is only used for the RpicamCapture fallback.
CAPTURE_W   = 640
CAPTURE_H   = 480
FOCAL_PX    = 5161.0    # pixels — update from calibration
DEFAULT_AGL = 5.0       # metres — fallback if pose topic unavailable

# Seconds to wait for /arducam/image_raw before activating RpicamCapture fallback
CAMERA_FALLBACK_TIMEOUT_S = 10.0

# Publish ground class every N inference cycles (~1 Hz at 15 fps input)
GROUND_CLASS_EVERY_N = 15

# ── Topic Names ───────────────────────────────────────────────────────────────

TOPIC_IMAGE   = "/arducam/image_raw"
TOPIC_FLOW    = "/hailo/optical_flow"
TOPIC_GROUND  = "/hailo/ground_class"
TOPIC_POSE    = "/mavros/local_position/pose"
NODE_NAME     = "hailo_flight_node"


def _import_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import TwistStamped, PoseStamped
    from sensor_msgs.msg   import Image
    from std_msgs.msg      import String
    return (rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy,
            TwistStamped, PoseStamped, Image, String)


# ── RpicamCapture fallback (bench / dev without main.py) ─────────────────────

# ══════════════════════════════════════════════════════════════════════════════
# RpicamCapture — direct rpicam-vid MJPEG stdout capture
# ══════════════════════════════════════════════════════════════════════════════

class RpicamCapture:
    """
    In-process camera capture using rpicam-vid MJPEG stdout pipe.

    IMX477 with rpicam-apps/libcamera does not expose a /dev/video* V4L2
    node — cv2.VideoCapture(CAP_V4L2) will always fail on this hardware.
    rpicam-vid is called directly and its MJPEG stdout is read frame-by-frame
    using FF D8 (SOI) / FF D9 (EOI) JPEG marker scanning.
    Source: JPEG specification ISO/IEC 10918-1.

    This is the same reader used in the flight test scripts.
    """

    _SOI   = b"\xff\xd8"
    _EOI   = b"\xff\xd9"
    _CHUNK = 65536

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30) -> None:
        import threading
        self._width  = width
        self._height = height
        self._fps    = fps
        self._proc   = None
        self._thread = None
        self._lock   = threading.Lock()
        self._frame  = None   # latest complete JPEG bytes
        self._count  = 0
        self._stop   = threading.Event()

    def start(self) -> bool:
        import subprocess, os, threading
        cmd = [
            "rpicam-vid",
            "--codec",     "mjpeg",
            "-o",          "-",
            "--width",     str(self._width),
            "--height",    str(self._height),
            "--framerate", str(self._fps),
            "--timeout",   "0",
            "--nopreview",
            "--flush",
        ]
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
                preexec_fn=os.setsid,
            )
        except FileNotFoundError:
            print("[RpicamCapture] rpicam-vid not found")
            return False
        except Exception as exc:
            print(f"[RpicamCapture] Launch failed: {exc}")
            return False

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="rpicam_hailo"
        )
        self._thread.start()

        import time
        deadline = time.time() + 6.0
        while time.time() < deadline:
            with self._lock:
                if self._count > 0:
                    print(f"[RpicamCapture] Ready  {self._width}×{self._height} @ {self._fps}fps")
                    return True
            time.sleep(0.1)
        print("[RpicamCapture] Started — no frames yet")
        return True

    def read(self):
        """cv2.VideoCapture-compatible read() — returns (True, bgr_array)."""
        import cv2
        import numpy as np
        import time
        deadline = time.time() + 0.5
        with self._lock:
            prev = self._count
        while time.time() < deadline:
            with self._lock:
                if self._count > prev and self._frame is not None:
                    jpeg = bytes(self._frame)
                    break
            time.sleep(0.01)
        else:
            return False, None
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return (img is not None), img

    def release(self) -> None:
        import os, signal
        self._stop.set()
        if self._proc is not None and self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        print(f"[RpicamCapture] Released  ({self._count} frames captured)")

    def isOpened(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _reader_loop(self) -> None:
        buf = b""
        start_idx = -1
        while not self._stop.is_set():
            try:
                chunk = self._proc.stdout.read(self._CHUNK)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk
            while True:
                if start_idx < 0:
                    idx = buf.find(self._SOI)
                    if idx < 0:
                        buf = buf[-1:]
                        break
                    start_idx = idx
                eoi_idx = buf.find(self._EOI, start_idx + 2)
                if eoi_idx < 0:
                    break
                frame_bytes = buf[start_idx : eoi_idx + 2]
                with self._lock:
                    self._frame = frame_bytes
                    self._count += 1
                buf = buf[eoi_idx + 2:]
                start_idx = -1


class RpicamCapture:
    """
    Direct rpicam-vid stdout MJPEG reader — used only when
    /arducam/image_raw is not publishing (bench mode without main.py).

    Identical to the implementation in the test scripts.
    Source: JPEG specification ISO/IEC 10918-1.
    """

    _SOI   = b"\xff\xd8"
    _EOI   = b"\xff\xd9"
    _CHUNK = 65536

    def __init__(self, width: int = 640, height: int = 480, fps: int = 15):
        self._width  = width
        self._height = height
        self._fps    = fps
        self._proc   = None
        self._thread = None
        self._lock   = threading.Lock()
        self._frame  = None
        self._count  = 0
        self._stop   = threading.Event()

    def start(self) -> bool:
        import subprocess, os as _os
        cmd = [
            "rpicam-vid", "--codec", "mjpeg", "-o", "-",
            "--width", str(self._width), "--height", str(self._height),
            "--framerate", str(self._fps), "--timeout", "0",
            "--nopreview", "--flush",
        ]
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                bufsize=0, preexec_fn=_os.setsid,
            )
        except FileNotFoundError:
            print("[RpicamCapture] rpicam-vid not found")
            return False
        except Exception as exc:
            print(f"[RpicamCapture] Launch failed: {exc}")
            return False

        self._stop.clear()
        self._thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="rpicam_hailo_fallback"
        )
        self._thread.start()

        deadline = time.time() + 6.0
        while time.time() < deadline:
            with self._lock:
                if self._count > 0:
                    print(f"[RpicamCapture] Fallback ready  "
                          f"{self._width}×{self._height} @ {self._fps}fps")
                    return True
            time.sleep(0.1)
        print("[RpicamCapture] Started — no frames yet")
        return True

    def read(self):
        """cv2.VideoCapture-compatible read() — returns (True, bgr_array)."""
        import cv2, numpy as np
        deadline = time.time() + 0.5
        with self._lock:
            prev = self._count
        while time.time() < deadline:
            with self._lock:
                if self._count > prev and self._frame is not None:
                    jpeg = bytes(self._frame)
                    break
            time.sleep(0.01)
        else:
            return False, None
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return (img is not None), img

    def release(self):
        import os as _os, signal as _sig
        self._stop.set()
        if self._proc is not None and self._proc.poll() is None:
            try:
                _os.killpg(_os.getpgid(self._proc.pid), _sig.SIGINT)
                self._proc.wait(timeout=5)
            except Exception:
                try:
                    _os.killpg(_os.getpgid(self._proc.pid), _sig.SIGKILL)
                except Exception:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        print(f"[RpicamCapture] Released  ({self._count} frames)")

    def isOpened(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _reader_loop(self):
        buf = b""
        start_idx = -1
        while not self._stop.is_set():
            try:
                chunk = self._proc.stdout.read(self._CHUNK)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk
            while True:
                if start_idx < 0:
                    idx = buf.find(self._SOI)
                    if idx < 0:
                        buf = buf[-1:]
                        break
                    start_idx = idx
                eoi_idx = buf.find(self._EOI, start_idx + 2)
                if eoi_idx < 0:
                    break
                frame_bytes = buf[start_idx: eoi_idx + 2]
                with self._lock:
                    self._frame = frame_bytes
                    self._count += 1
                buf = buf[eoi_idx + 2:]
                start_idx = -1


# ── HailoFlightNode ───────────────────────────────────────────────────────────

class HailoFlightNode:
    """
    ROS 2 node that subscribes to /arducam/image_raw, runs Hailo inference,
    and publishes optical flow and ground classification results.

    Lifecycle:
        node = HailoFlightNode()
        node.start()     # initialise device, load models, attach ROS
        node.spin()      # block — inference loop + ROS spin thread
        node.shutdown()  # clean teardown
    """

    def __init__(self):
        self._device        = None
        self._flow_model    = None
        self._class_model   = None
        self._optical_flow  = None
        self._ground_class  = None
        self._current_agl   = DEFAULT_AGL
        self._frame_count   = 0
        self._running       = False

        # Latest frame received from /arducam/image_raw or RpicamCapture fallback
        self._latest_frame        = None
        self._last_inferred_count = 0
        self._frame_lock          = threading.Lock()

        # ROS handles — set in start()
        self._rclpy      = None
        self._node       = None
        self._flow_pub   = None
        self._ground_pub = None
        self._image_sub  = None
        self._agl_lock   = threading.Lock()

        # Fallback camera (bench mode, no main.py running)
        self._fallback_camera = None
        self._using_fallback  = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def attach_to_node(self, existing_node) -> bool:
        """
        In-process mode — attach to an already-initialised rclpy node.

        Use this when hailo_platform is importable in the same Python
        environment as the caller (both using /opt/ros/jazzy rclpy).
        Subscriptions and publishers are created on existing_node so
        DDS discovery is instant — no cross-process delay.

        This is the preferred path for testing. In production main.py
        uses start() + spin() as a subprocess because hailo_platform
        lives in hailo_inference_env.

        Returns True on success.
        """
        if not self._load_hailo_models():
            return False

        try:
            (_, _, QoSProfile, ReliabilityPolicy,
             HistoryPolicy, TwistStamped, PoseStamped,
             Image, String) = _import_ros()
        except ImportError as exc:
            print(f"[HailoFlightNode] ROS import failed: {exc}")
            return False

        self._node  = existing_node
        self._rclpy = None   # caller owns spin — we do not call rclpy.init()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._flow_pub   = self._node.create_publisher(TwistStamped, TOPIC_FLOW,   reliable_qos)
        self._ground_pub = self._node.create_publisher(String,        TOPIC_GROUND, reliable_qos)
        self._image_sub  = self._node.create_subscription(
            Image, TOPIC_IMAGE, self._image_callback, sensor_qos)
        self._node.create_subscription(
            PoseStamped, TOPIC_POSE, self._pose_callback, sensor_qos)

        self._running = True

        # Start inference loop in a background thread —
        # the caller's spin loop drives the ROS callbacks
        self._inference_thread = threading.Thread(
            target=self._inference_loop, daemon=True, name="hailo_inference"
        )
        self._inference_thread.start()

        print(f"[HailoFlightNode] Attached to existing node (in-process)")
        print(f"  Subscribing  : {TOPIC_IMAGE}")
        print(f"  Publishing   : {TOPIC_FLOW}, {TOPIC_GROUND}")
        return True

    def stop_attached(self) -> None:
        """Stop the in-process inference thread and release Hailo device."""
        self._running = False
        if hasattr(self, "_inference_thread") and self._inference_thread is not None:
            self._inference_thread.join(timeout=3.0)
        if self._device is not None:
            self._device.shutdown()
        print("[HailoFlightNode] Stopped (in-process)")

    def _inference_loop(self) -> None:
        """
        Background inference thread for in-process (attached) mode.

        _image_callback is driven by the caller's ROS spin. This thread
        pulls the latest frame from the shared buffer and runs inference.
        """
        thermal_check_t = time.monotonic()
        while self._running:
            frame = self._get_next_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            self._frame_count += 1
            with self._agl_lock:
                agl = self._current_agl

            if self._optical_flow is not None:
                result = self._optical_flow.push_frame(frame, agl=agl)
                if result["valid"]:
                    self._publish_flow(result)

            if (self._ground_class is not None
                    and self._frame_count % GROUND_CLASS_EVERY_N == 0):
                classification = self._ground_class.classify(frame)
                self._publish_ground(classification)

            now = time.monotonic()
            if now - thermal_check_t > 60.0:
                self._device.log_thermal_status()
                thermal_check_t = now

    def start(self) -> bool:
        """
        Standalone mode — initialise device, create own ROS node, own spin.

        Used when running as a subprocess (production main.py launches
        hailo_flight_node.py via hailo_inference_env). For in-process
        use, call attach_to_node() instead.
        """
        if not self._load_hailo_models():
            return False

        # ROS 2 node
        try:
            (rclpy, Node, QoSProfile, ReliabilityPolicy,
             HistoryPolicy, TwistStamped, PoseStamped,
             Image, String) = _import_ros()
        except ImportError as exc:
            print(f"[HailoFlightNode] ROS 2 not available: {exc}")
            self._device.shutdown()
            return False

        self._rclpy = rclpy
        rclpy.init()
        self._node = Node(NODE_NAME)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._flow_pub   = self._node.create_publisher(TwistStamped, TOPIC_FLOW,   reliable_qos)
        self._ground_pub = self._node.create_publisher(String,        TOPIC_GROUND, reliable_qos)
        self._image_sub  = self._node.create_subscription(
            Image, TOPIC_IMAGE, self._image_callback, sensor_qos)
        self._node.create_subscription(
            PoseStamped, TOPIC_POSE, self._pose_callback, sensor_qos)

        self._running = True
        print(f"[HailoFlightNode] Started (standalone)")
        print(f"  Subscribing  : {TOPIC_IMAGE}")
        print(f"  Publishing   : {TOPIC_FLOW}, {TOPIC_GROUND}")
        print(f"  Fallback     : RpicamCapture if no frames within "
              f"{CAMERA_FALLBACK_TIMEOUT_S:.0f}s")
        return True

    def _load_hailo_models(self) -> bool:
        """Shared model loading for both start() and attach_to_node()."""
        sys.path.insert(0, str(Path(__file__).parent))
        from hailo_device import HailoDevice

        self._device = HailoDevice()
        if not self._device.is_available():
            print("[HailoFlightNode] /dev/hailo0 not found")
            return False
        try:
            self._device.open()
        except RuntimeError as exc:
            print(f"[HailoFlightNode] Device open failed: {exc}")
            return False

        self._flow_model  = self._load_model_safe("optical flow",  FLOW_HEF_PATH)
        self._class_model = self._load_model_safe("ground class",  CLASS_HEF_PATH)
        if self._flow_model is None and self._class_model is None:
            print("[HailoFlightNode] Both models failed to load")
            self._device.shutdown()
            return False

        if self._flow_model is not None:
            from hailo_optical_flow import HailoOpticalFlow
            self._optical_flow = HailoOpticalFlow(
                network_group=self._flow_model,
                focal_px=FOCAL_PX,
                agl_default=DEFAULT_AGL,
            )
            print(f"[HailoFlightNode] Optical flow ready: {FLOW_HEF_PATH.name}")

        if self._class_model is not None:
            from hailo_ground_class import HailoGroundClassifier
            self._ground_class = HailoGroundClassifier(
                network_group=self._class_model,
            )
            print(f"[HailoFlightNode] Ground classifier ready: {CLASS_HEF_PATH.name}")

        return True

    def spin(self) -> None:
        """Run inference loop. Blocks until shutdown() is called."""
        if not self._running:
            return

        # ROS spin in background thread — callbacks populate _latest_frame
        ros_thread = threading.Thread(
            target=self._rclpy.spin, args=(self._node,), daemon=True,
            name="hailo_ros_spin"
        )
        ros_thread.start()

        # Wait for first frame — activate fallback if none arrives
        print(f"[HailoFlightNode] Waiting for frames on {TOPIC_IMAGE}...")
        deadline = time.time() + CAMERA_FALLBACK_TIMEOUT_S
        while time.time() < deadline and self._running:
            with self._frame_lock:
                if self._latest_frame is not None:
                    break
            time.sleep(0.1)

        with self._frame_lock:
            has_frame = self._latest_frame is not None

        if not has_frame:
            print(f"[HailoFlightNode] No frames on {TOPIC_IMAGE} after "
                  f"{CAMERA_FALLBACK_TIMEOUT_S:.0f}s — activating RpicamCapture fallback")
            self._start_fallback_camera()

        thermal_check_t = time.monotonic()

        try:
            while self._running:
                frame = self._get_next_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                self._frame_count += 1

                with self._agl_lock:
                    agl = self._current_agl

                # Optical flow inference
                if self._optical_flow is not None:
                    result = self._optical_flow.push_frame(frame, agl=agl)
                    if result["valid"]:
                        self._publish_flow(result)

                # Ground classification every N frames
                if (self._ground_class is not None
                        and self._frame_count % GROUND_CLASS_EVERY_N == 0):
                    classification = self._ground_class.classify(frame)
                    self._publish_ground(classification)

                # Thermal check every 60 seconds
                now = time.monotonic()
                if now - thermal_check_t > 60.0:
                    self._device.log_thermal_status()
                    thermal_check_t = now

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Stop inference, release fallback camera, shut down ROS and Hailo."""
        self._running = False

        if self._fallback_camera is not None:
            self._fallback_camera.release()
            self._fallback_camera = None

        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()

        if self._device is not None:
            self._device.shutdown()

        print("[HailoFlightNode] Shutdown complete")

    # ── ROS Callbacks ─────────────────────────────────────────────────────────

    def _image_callback(self, msg) -> None:
        """
        Receive sensor_msgs/Image from /arducam/image_raw.

        CameraCapture publishes rgb8 at full mapping resolution (2028×1520).
        The inference classes resize internally to their HEF input shapes
        (SCDepthV3: 320×256, YOLOv8m: 640×640) so no resize is needed here.
        Encoding is normalised to BGR for OpenCV compatibility.
        """
        import numpy as np
        import cv2

        enc = getattr(msg, "encoding", "rgb8").lower()
        h   = int(msg.height)
        w   = int(msg.width)
        raw = bytes(msg.data)

        try:
            if enc in ("rgb8",):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3).copy()
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif enc == "bgr8":
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3).copy()
            elif enc == "mono8":
                gray = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
                arr  = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                print(f"[HailoFlightNode] Unsupported encoding: {enc} — skipping frame")
                return
        except Exception as exc:
            print(f"[HailoFlightNode] Frame decode error: {exc}")
            return

        with self._frame_lock:
            self._latest_frame = arr
            self._last_ros_count = getattr(self, "_last_ros_count", 0) + 1

    def _pose_callback(self, msg) -> None:
        """Update AGL from MAVROS local position z."""
        z = msg.pose.position.z
        if z > 0.1:
            with self._agl_lock:
                self._current_agl = float(z)

    # ── Frame Access ──────────────────────────────────────────────────────────

    def _get_next_frame(self):
        """
        Return the next unprocessed frame from either ROS subscription
        or RpicamCapture fallback. Returns None if no new frame is available.

        ROS path: compares _last_inferred_count against _last_ros_count.
        Fallback path: calls cap.read() which blocks up to 0.5s.
        """
        if self._using_fallback and self._fallback_camera is not None:
            ret, frame = self._fallback_camera.read()
            return frame if ret else None

        with self._frame_lock:
            ros_count = getattr(self, "_last_ros_count", 0)
            if ros_count <= self._last_inferred_count:
                return None
            frame = self._latest_frame
            self._last_inferred_count = ros_count
        return frame

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_flow(self, result: dict) -> None:
        """Publish optical flow velocity as TwistStamped.

        Encoding — twist.linear.z carries confidence (0–1) repurposed field.
        FlowBridge in the dronepi conda env reads and gates on this value.
        """
        from geometry_msgs.msg import TwistStamped
        msg = TwistStamped()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x  = result["vx"]
        msg.twist.linear.y  = result["vy"]
        msg.twist.linear.z  = result["confidence"]
        self._flow_pub.publish(msg)

    def _publish_ground(self, classification: dict) -> None:
        """Publish ground surface classification as JSON string."""
        from std_msgs.msg import String
        payload = {
            "label":      classification["label"],
            "confidence": round(classification["confidence"], 3),
            "latency_ms": round(classification["latency_ms"], 1),
        }
        msg      = String()
        msg.data = json.dumps(payload)
        self._ground_pub.publish(msg)

    # ── Fallback Camera ───────────────────────────────────────────────────────

    def _start_fallback_camera(self) -> None:
        """
        Start RpicamCapture as a fallback frame source.

        Used only in bench/dev mode when /arducam/image_raw is not
        publishing (main.py and CameraCapture not running). During a real
        mission CameraCapture is always the source — this code path will
        never activate in production.
        """
        self._fallback_camera = RpicamCapture(
            width=CAPTURE_W, height=CAPTURE_H, fps=15
        )
        ok = self._fallback_camera.start()
        if ok:
            self._using_fallback = True
            print("[HailoFlightNode] Fallback camera active via RpicamCapture")
        else:
            print("[HailoFlightNode] Fallback camera failed — inference will idle")
            self._fallback_camera = None

    # ── Private Helpers ───────────────────────────────────────────────────────

    def _load_model_safe(self, name: str, hef_path: Path):
        if not hef_path.exists():
            print(f"[HailoFlightNode] {name} HEF not found: {hef_path}")
            return None
        try:
            return self._device.load_model(str(hef_path))
        except Exception as exc:
            print(f"[HailoFlightNode] Failed to load {name}: {exc}")
            return None


# ── Entry Point ───────────────────────────────────────────────────────────────

def main():
    import signal

    node = HailoFlightNode()

    def _shutdown(signum=None, frame=None):
        print("\n[HailoFlightNode] Signal received — shutting down")
        node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    if not node.start():
        print("[HailoFlightNode] Startup failed — exiting")
        sys.exit(1)

    node.spin()


if __name__ == "__main__":
    main()
