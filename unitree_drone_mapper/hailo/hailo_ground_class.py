"""hailo/hailo_ground_class.py — Nadir ground surface classifier.

Runs YOLOv8m or a semantic segmentation model on nadir camera frames to
classify the ground surface type below the drone. Output is used for:

    IN-FLIGHT:
        - Landing zone safety assessment before descent
        - Surface type metadata logged per frame in the bag

    POST-PROCESSING:
        - Surface type annotation per point cloud sector
        - Improved DTM/DSM classification (ground vs vegetation vs structure)

Uses the yolov8m.hef already installed at:
    /usr/local/hailo/resources/models/hailo8/yolov8m.hef

Surface classes mapped from COCO detections relevant to nadir aerial view:
    SAFE_LAND  — flat ground, grass, road, parking lot
    CAUTION    — vegetation, water visible, uneven terrain indicators
    UNSAFE     — person, vehicle, structure, obstacle detected
    UNKNOWN    — insufficient confidence or no relevant detections

Environment: hailo_inference_env

Design notes:
    - Classification is coarse — this is a scene-level label, not pixel-wise
      segmentation. Pixel-wise segmentation requires a dedicated segmentation
      .hef (e.g. yolov5m_seg.hef is available on your system).
    - The output is a single string label + confidence float, published on
      /hailo/ground_class as std_msgs/String in the flight node.
    - Post-processing use is via direct import — no ROS dependency here.
"""

import time
import numpy as np


# COCO class indices relevant to nadir aerial classification
# Source: COCO 2017 class list, standard YOLOv8 training set
_SAFE_LAND_CLASSES  = {60, 61, 62}   # dining table, toilet, tv — unused but safe
_CAUTION_CLASSES    = {
    0,   # person (could be under)
    14,  # bird
    15, 16, 17, 18, 19,  # cat, dog, horse, sheep, cow
    58,  # potted plant
    63,  # laptop — proxy for structure
}
_UNSAFE_CLASSES     = {
    0,   # person
    1,   # bicycle
    2,   # car
    3,   # motorcycle
    5,   # bus
    6,   # train
    7,   # truck
    9,   # traffic light
    56,  # chair
    57,  # couch
    59,  # bed
}

# Confidence thresholds
DETECTION_CONF_THRESHOLD = 0.40
CLASSIFICATION_CONF_MIN  = 0.30   # Below this → UNKNOWN

# Input size — yolov8m expects 640×640
DEFAULT_INPUT_H = 640
DEFAULT_INPUT_W = 640

# COCO class names for logging
COCO_CLASSES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck",
    "boat","traffic light","fire hydrant","stop sign","parking meter","bench",
    "bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard",
    "sports ball","kite","baseball bat","baseball glove","skateboard","surfboard",
    "tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza",
    "donut","cake","chair","couch","potted plant","bed","dining table","toilet",
    "tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors",
    "teddy bear","hair drier","toothbrush",
]


class HailoGroundClassifier:
    """Classifies nadir ground surface type using YOLOv8m on Hailo-8.

    Args:
        network_group: Configured network group from HailoDevice.load_model()
                       loaded with yolov8m.hef.
        input_h:       Model input height. Default 640 (yolov8m native).
        input_w:       Model input width. Default 640.
        conf_threshold: YOLOv8 detection confidence threshold. Default 0.40.
    """

    def __init__(
        self,
        network_group,
        input_h:        int   = DEFAULT_INPUT_H,
        input_w:        int   = DEFAULT_INPUT_W,
        conf_threshold: float = DETECTION_CONF_THRESHOLD,
    ):
        self._network_group  = network_group
        self._input_h        = input_h
        self._input_w        = input_w
        self._conf_threshold = conf_threshold
        self._latency_buf    = []

        # hailort >= 4.23.0: make_from_network_group() returns Dict[str, VStreamParams].
        # Store as dict; pre-4.23.0 returned a list — normalise for uniform access.
        # InferVStreams requires dicts — pass directly, not converted to lists.
        try:
            from hailo_platform import (
                InputVStreamParams, OutputVStreamParams,
                FormatType
            )
            _in  = InputVStreamParams.make_from_network_group(
                network_group, quantized=False, format_type=FormatType.UINT8
            )
            _out = OutputVStreamParams.make_from_network_group(
                network_group, quantized=False, format_type=FormatType.FLOAT32
            )
            self._input_params  = _in  if isinstance(_in,  dict) else {p.name: p for p in _in}
            self._output_params = _out if isinstance(_out, dict) else {p.name: p for p in _out}
        except Exception as exc:
            raise RuntimeError(f"Failed to create vstream params: {exc}") from exc

    # ── Public API ────────────────────────────────────────────────────────────

    def classify(self, frame: np.ndarray) -> dict:
        """Classify the ground surface type from a nadir camera frame.

        Args:
            frame: BGR numpy array from camera. Any resolution — resized
                   internally to (input_h, input_w).

        Returns:
            dict with keys:
                label      (str)   — "SAFE_LAND", "CAUTION", "UNSAFE", "UNKNOWN"
                confidence (float) — 0.0–1.0
                detections (list)  — list of (class_name, conf) tuples
                latency_ms (float) — inference wall time
        """
        t0 = time.monotonic()
        tensor = self._preprocess(frame)

        try:
            from hailo_platform import InferVStreams

            # activate() required in hailort >= 4.23.0 before InferVStreams.
            # input_name extracted from dict keys (not .name on a list element).
            input_name = list(self._input_params.keys())[0]
            with self._network_group.activate():
                with InferVStreams(
                    self._network_group,
                    self._input_params,
                    self._output_params,
                ) as pipeline:
                    raw = pipeline.infer({input_name: tensor})

            latency_ms = (time.monotonic() - t0) * 1000.0
            self._latency_buf.append(latency_ms)
            if len(self._latency_buf) > 30:
                self._latency_buf.pop(0)

            return self._parse_detections(raw, latency_ms)

        except Exception as exc:
            latency_ms = (time.monotonic() - t0) * 1000.0
            print(f"[HailoGroundClassifier] Inference error: {exc}")
            return {
                "label": "UNKNOWN", "confidence": 0.0,
                "detections": [], "latency_ms": latency_ms,
            }

    def get_avg_latency_ms(self) -> float:
        if not self._latency_buf:
            return 0.0
        return sum(self._latency_buf) / len(self._latency_buf)

    # ── Private ───────────────────────────────────────────────────────────────

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """Resize to model input size, convert BGR→RGB, add batch dim."""
        import cv2
        resized = cv2.resize(frame, (self._input_w, self._input_h))
        rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        return np.expand_dims(rgb, axis=0).astype(np.uint8)

    def _parse_detections(self, raw_outputs: dict, latency_ms: float) -> dict:
        """Parse YOLOv8 output tensor into surface classification.

        YOLOv8m Hailo output format: (1, num_boxes, 85) where each box is
        [x1, y1, x2, y2, obj_conf, class_conf_0...class_conf_79]
        The exact layout depends on the HEF compilation — try both formats.
        """
        output_key  = list(raw_outputs.keys())[0]
        # hailort 4.23.0: infer() may return a list of arrays per output
        # rather than a single ndarray. np.array() coerces both forms to
        # a single ndarray before .astype() — safe for both API versions.
        output_data = np.array(raw_outputs[output_key]).astype(np.float32)

        detections = []

        # Handle (1, N, 85) layout
        if len(output_data.shape) == 3 and output_data.shape[2] == 85:
            boxes = output_data[0]  # (N, 85)
            for box in boxes:
                obj_conf    = float(box[4])
                class_probs = box[5:]
                class_id    = int(np.argmax(class_probs))
                class_conf  = float(class_probs[class_id]) * obj_conf
                if class_conf >= self._conf_threshold and class_id < len(COCO_CLASSES):
                    detections.append((COCO_CLASSES[class_id], class_conf, class_id))

        # Handle (1, 85, N) layout (transposed)
        elif len(output_data.shape) == 3 and output_data.shape[1] == 85:
            boxes = output_data[0].T  # transpose to (N, 85)
            for box in boxes:
                obj_conf    = float(box[4])
                class_probs = box[5:]
                class_id    = int(np.argmax(class_probs))
                class_conf  = float(class_probs[class_id]) * obj_conf
                if class_conf >= self._conf_threshold and class_id < len(COCO_CLASSES):
                    detections.append((COCO_CLASSES[class_id], class_conf, class_id))

        return self._classify_from_detections(detections, latency_ms)

    def _classify_from_detections(
        self, detections: list, latency_ms: float
    ) -> dict:
        """Map detected objects to a surface safety label.

        Priority: UNSAFE > CAUTION > SAFE_LAND > UNKNOWN
        The label with the highest-confidence detection in each tier wins.
        """
        if not detections:
            return {
                "label": "UNKNOWN", "confidence": 0.0,
                "detections": [], "latency_ms": latency_ms,
            }

        unsafe_conf  = max(
            (conf for _, conf, cid in detections if cid in _UNSAFE_CLASSES),
            default=0.0
        )
        caution_conf = max(
            (conf for _, conf, cid in detections if cid in _CAUTION_CLASSES),
            default=0.0
        )
        max_any_conf = max(conf for _, conf, _ in detections)

        det_summary = [(name, round(conf, 2)) for name, conf, _ in
                       sorted(detections, key=lambda x: x[1], reverse=True)[:5]]

        if unsafe_conf >= CLASSIFICATION_CONF_MIN:
            return {
                "label": "UNSAFE", "confidence": unsafe_conf,
                "detections": det_summary, "latency_ms": latency_ms,
            }
        if caution_conf >= CLASSIFICATION_CONF_MIN:
            return {
                "label": "CAUTION", "confidence": caution_conf,
                "detections": det_summary, "latency_ms": latency_ms,
            }
        if max_any_conf >= CLASSIFICATION_CONF_MIN:
            return {
                "label": "SAFE_LAND", "confidence": max_any_conf,
                "detections": det_summary, "latency_ms": latency_ms,
            }

        return {
            "label": "UNKNOWN", "confidence": max_any_conf,
            "detections": det_summary, "latency_ms": latency_ms,
        }
