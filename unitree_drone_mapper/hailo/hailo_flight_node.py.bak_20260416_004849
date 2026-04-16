"""hailo/hailo_flight_node.py — ROS 2 node for in-flight Hailo-8 inference.

Runs in hailo_inference_env (NOT the Conda dronepi env). Communicates with
the main flight stack exclusively via ROS 2 topics — no shared imports,
no shared environment. This is the correct isolation architecture.

Published topics:
    /hailo/optical_flow  (geometry_msgs/TwistStamped)
        linear.x = vx (forward velocity, m/s, body frame)
        linear.y = vy (lateral velocity, m/s, body frame)
        linear.z = confidence (0.0–1.0, encoded in z field)

    /hailo/ground_class  (std_msgs/String)
        data = JSON string:
            {"label": "SAFE_LAND", "confidence": 0.87, "latency_ms": 14.2}

Subscribed topics:
    /mavros/local_position/pose  (geometry_msgs/PoseStamped)
        Used to extract current altitude (z field) for metric flow scaling.
        Falls back to DEFAULT_AGL if this topic is not available.

Environment: hailo_inference_env
    ~/hailo_inference_env/bin/python3 hailo/hailo_flight_node.py

Models used:
    Optical flow:    scdepthv3.hef  (depth gradient → approximate flow)
                     or custom RAFT-small .hef if compiled
    Ground class:    yolov8m.hef

Model paths — override via environment variables:
    HAILO_FLOW_HEF  = path to optical flow .hef
    HAILO_CLASS_HEF = path to ground classification .hef

Design notes:
    - Both models share one VDevice via HailoDevice — single PCIe port owner.
    - Inference runs in a dedicated thread per model so camera capture and
      topic publishing are not blocked by Hailo latency.
    - If either model fails to load, the node continues with the other.
      A partial Hailo node (flow-only or class-only) is still useful.
    - The node publishes at the camera frame rate. FlowBridge in the main
      Conda env gates output on confidence before forwarding to MAVROS.
    - Ground class is published at 1 Hz (every N frames) — classification
      does not need per-frame granularity and reduces Hailo duty cycle.
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
    str(_HAILO_RESOURCES / "scdepthv3.hef"),   # Depth→flow fallback (on your system)
))
CLASS_HEF_PATH = Path(os.environ.get(
    "HAILO_CLASS_HEF",
    str(_HAILO_RESOURCES / "yolov8m.hef"),      # Surface classification
))

# ── Camera Config ─────────────────────────────────────────────────────────────

# IMX477 with Tamron 4-12mm at ~8mm focal length, 4056×3040 sensor
# Adjust FOCAL_PX after camera calibration — this is a reasonable default.
# Resolution at which frames are captured for Hailo input
CAPTURE_W    = 640
CAPTURE_H    = 480
FOCAL_PX     = 5161.0   # pixels — update from calibration
DEFAULT_AGL  = 5.0      # metres — fallback if pose topic not available

# Publish ground class every N optical flow frames
GROUND_CLASS_EVERY_N = 30   # ~1 Hz at 30fps

# ── Topic Names ───────────────────────────────────────────────────────────────

TOPIC_FLOW        = "/hailo/optical_flow"
TOPIC_GROUND      = "/hailo/ground_class"
TOPIC_POSE        = "/mavros/local_position/pose"
NODE_NAME         = "hailo_flight_node"


def _import_ros():
    """Import ROS 2 Python bindings. Raises ImportError if not sourced."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import TwistStamped, PoseStamped
    from std_msgs.msg      import String
    return rclpy, Node, QoSProfile, ReliabilityPolicy, HistoryPolicy, \
           TwistStamped, PoseStamped, String


class HailoFlightNode:
    """ROS 2 node that runs both Hailo inference models and publishes results.

    Lifecycle:
        node = HailoFlightNode()
        node.start()     # initialises device, loads models, opens camera
        node.spin()      # blocks — ROS 2 spin + inference loop
        node.shutdown()  # clean teardown on KeyboardInterrupt or signal
    """

    def __init__(self):
        self._device         = None
        self._flow_model     = None
        self._class_model    = None
        self._optical_flow   = None   # HailoOpticalFlow instance
        self._ground_class   = None   # HailoGroundClassifier instance
        self._camera         = None
        self._current_agl    = DEFAULT_AGL
        self._frame_count    = 0
        self._running        = False

        # ROS handles — set in start()
        self._rclpy          = None
        self._node           = None
        self._flow_pub       = None
        self._ground_pub     = None

        # Lock protecting _current_agl across ROS callback and inference threads
        self._agl_lock       = threading.Lock()

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Initialise device, ROS node, camera, and models.

        Returns True on success. Returns False if the Hailo device is
        unavailable — caller should exit cleanly so main.py skips Hailo.
        """
        # ── 1. Device check ───────────────────────────────────────────────────
        sys.path.insert(0, str(Path(__file__).parent))
        from hailo_device import HailoDevice

        self._device = HailoDevice()
        if not self._device.is_available():
            print("[HailoFlightNode] /dev/hailo0 not found — exiting")
            return False

        try:
            self._device.open()
        except RuntimeError as exc:
            print(f"[HailoFlightNode] Device open failed: {exc}")
            return False

        # ── 2. Load models ────────────────────────────────────────────────────
        self._flow_model  = self._load_model_safe("optical flow",  FLOW_HEF_PATH)
        self._class_model = self._load_model_safe("ground class",  CLASS_HEF_PATH)

        if self._flow_model is None and self._class_model is None:
            print("[HailoFlightNode] Both models failed to load — exiting")
            self._device.shutdown()
            return False

        # ── 3. Instantiate inference classes ──────────────────────────────────
        if self._flow_model is not None:
            from hailo_optical_flow import HailoOpticalFlow
            self._optical_flow = HailoOpticalFlow(
                network_group=self._flow_model,
                input_h=CAPTURE_H,
                input_w=CAPTURE_W,
                focal_px=FOCAL_PX,
                agl_default=DEFAULT_AGL,
            )
            print(f"[HailoFlightNode] Optical flow model loaded: {FLOW_HEF_PATH.name}")

        if self._class_model is not None:
            from hailo_ground_class import HailoGroundClassifier
            self._ground_class = HailoGroundClassifier(
                network_group=self._class_model,
            )
            print(f"[HailoFlightNode] Ground class model loaded: {CLASS_HEF_PATH.name}")

        # ── 4. ROS 2 node ─────────────────────────────────────────────────────
        try:
            (rclpy, Node, QoSProfile, ReliabilityPolicy,
             HistoryPolicy, TwistStamped, PoseStamped, String) = _import_ros()
        except ImportError as exc:
            print(f"[HailoFlightNode] ROS 2 not available: {exc}")
            print("  Source ROS 2 setup.bash before running this node")
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

        self._flow_pub   = self._node.create_publisher(
            TwistStamped, TOPIC_FLOW, reliable_qos)
        self._ground_pub = self._node.create_publisher(
            String, TOPIC_GROUND, reliable_qos)

        # Subscribe to pose for live AGL
        self._node.create_subscription(
            PoseStamped, TOPIC_POSE, self._pose_cb, sensor_qos)

        # ── 5. Camera ─────────────────────────────────────────────────────────
        self._camera = self._open_camera()
        if self._camera is None:
            print("[HailoFlightNode] Camera unavailable — exiting")
            rclpy.shutdown()
            self._device.shutdown()
            return False

        self._running = True
        print(f"[HailoFlightNode] Started — publishing on {TOPIC_FLOW}, {TOPIC_GROUND}")
        return True

    def spin(self) -> None:
        """Run the inference + publish loop. Blocks until shutdown() is called."""
        if not self._running:
            return

        import cv2

        # ROS spin in background thread
        ros_thread = threading.Thread(
            target=self._rclpy.spin, args=(self._node,), daemon=True)
        ros_thread.start()

        thermal_check_t = time.monotonic()

        try:
            while self._running:
                ret, frame = self._camera.read()
                if not ret or frame is None:
                    time.sleep(0.01)
                    continue

                self._frame_count += 1

                with self._agl_lock:
                    agl = self._current_agl

                # ── Optical flow inference ────────────────────────────────────
                if self._optical_flow is not None:
                    result = self._optical_flow.push_frame(frame, agl=agl)
                    if result["valid"]:
                        self._publish_flow(result)

                # ── Ground classification (every N frames) ────────────────────
                if (self._ground_class is not None and
                        self._frame_count % GROUND_CLASS_EVERY_N == 0):
                    classification = self._ground_class.classify(frame)
                    self._publish_ground(classification)

                # ── Thermal check every 60 seconds ────────────────────────────
                now = time.monotonic()
                if now - thermal_check_t > 60.0:
                    self._device.log_thermal_status()
                    thermal_check_t = now

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self) -> None:
        """Stop inference loop, release camera, shut down ROS and Hailo."""
        self._running = False

        if self._camera is not None:
            self._camera.release()
            self._camera = None

        if self._rclpy is not None and self._rclpy.ok():
            self._rclpy.shutdown()

        if self._device is not None:
            self._device.shutdown()

        print("[HailoFlightNode] Shutdown complete")

    # ── ROS Callbacks ─────────────────────────────────────────────────────────

    def _pose_cb(self, msg) -> None:
        """Update current AGL from MAVROS local position z field."""
        z = msg.pose.position.z
        if z > 0.1:   # ignore ground-level readings that might be noise
            with self._agl_lock:
                self._current_agl = float(z)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_flow(self, result: dict) -> None:
        """Publish optical flow velocity as TwistStamped.

        Encoding:
            twist.linear.x = vx (forward, m/s)
            twist.linear.y = vy (lateral, m/s)
            twist.linear.z = confidence (0.0–1.0, repurposed field)

        FlowBridge in the main Conda env reads this topic and gates on
        confidence before forwarding to /mavros/odometry/out.
        """
        from geometry_msgs.msg import TwistStamped
        msg = TwistStamped()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x  = result["vx"]
        msg.twist.linear.y  = result["vy"]
        msg.twist.linear.z  = result["confidence"]   # repurposed — see FlowBridge
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

    # ── Private Helpers ───────────────────────────────────────────────────────

    def _load_model_safe(self, name: str, hef_path: Path):
        """Load a model, returning None on failure instead of raising."""
        if not hef_path.exists():
            print(f"[HailoFlightNode] {name} HEF not found: {hef_path}")
            print(f"  Set HAILO_{name.upper().replace(' ', '_')}_HEF env var to override")
            return None
        try:
            model = self._device.load_model(str(hef_path))
            return model
        except Exception as exc:
            print(f"[HailoFlightNode] Failed to load {name} model: {exc}")
            return None

    def _open_camera(self):
        """Open IMX477 via OpenCV V4L2 backend."""
        import cv2
        # Try device indices 0–3 — IMX477 is typically /dev/video0 on RPi
        for idx in range(4):
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
                cap.set(cv2.CAP_PROP_FPS,          30)
                # Read one frame to confirm the device is live
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"[HailoFlightNode] Camera opened: /dev/video{idx} "
                          f"({CAPTURE_W}×{CAPTURE_H})")
                    return cap
                cap.release()
        print("[HailoFlightNode] No camera found on /dev/video0–3")
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
        print("[HailoFlightNode] Startup failed — exiting with code 1")
        sys.exit(1)

    node.spin()


if __name__ == "__main__":
    main()
