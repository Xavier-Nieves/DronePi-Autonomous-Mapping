"""hailo/hailo_optical_flow.py — Nadir optical flow velocity estimator.

Runs the SCDepthV3 depth estimation model on the Hailo-8 and derives
body-frame XY velocity estimates (vx, vy) from consecutive nadir frames.

The output feeds /hailo/optical_flow (geometry_msgs/TwistStamped) via
HailoFlightNode. FlowBridge then forwards confidence-gated estimates to
/mavros/odometry/out for EKF2 velocity fusion via EKF2_EV_CTRL bit 2.

Environment: hailo_inference_env

Model in use:
    SCDepthV3 — /usr/local/hailo/resources/models/hailo8/scdepthv3.hef
    Compiled input:  NHWC(256, 320, 3)  — single RGB frame
    Compiled output: single-channel depth map, same spatial resolution

    A true optical flow HEF (e.g. PWC-Net) is not publicly available for
    Hailo-8 without a registered Hailo developer account. SCDepthV3 is the
    operative model until a proper flow HEF is obtained.

Velocity derivation — depth-difference approach:
    SCDepthV3 produces a per-pixel inverse-depth map D(x,y) per frame.
    Between frame t0 and t1 the camera translates (dx, dy) pixels in the
    image plane. The mean shift of the depth field approximates this:

        depth_diff(x, y) = D_t1(x, y) - D_t0(x, y)
        mean_dx = mean(np.gradient(depth_diff)[1])  — x-axis (columns)
        mean_dy = mean(np.gradient(depth_diff)[0])  — y-axis (rows)

    Converted to metric velocity using the standard pinhole projection:
        vx = (mean_dx / fps) * (agl / focal_px)

    Confidence is the inverse of depth-difference gradient variance — high
    variance (turbulent/rotating scene) gives low confidence, which
    FlowBridge gates out before forwarding to EKF2.

    When a proper flow HEF becomes available, set HAILO_FLOW_HEF to point
    to it and update DEFAULT_INPUT_H/W to match its compiled shape.
    The RAFT-style path in _parse_flow_output() handles (1,2,H,W) tensors
    and will be selected automatically.

hailort 4.23.0 API notes:
    - network_group.activate() context manager is required before any
      InferVStreams call — omitting it causes NETWORK_GROUP_NOT_ACTIVATED.
    - make_from_network_group() returns Dict[str, VStreamParams]. The dict
      is passed directly to InferVStreams; the stream name comes from keys().
    - Frame pairs are maintained internally. Caller only pushes single frames.
    - All numpy operations use float32 (safe regardless of numpy version).
"""

import time
import numpy as np


# Camera intrinsics — IMX477 with Tamron 4-12mm at mid-zoom (~8mm).
# Update focal_px from camera_calibration.yaml after calibration.
# focal_px = (focal_length_mm / sensor_pixel_size_um) * 1000
# IMX477 pixel pitch = 1.55 µm, focal = 8mm → focal_px ≈ 5161
DEFAULT_FOCAL_PX  = 5161.0
DEFAULT_FPS       = 30.0

# SCDepthV3 compiled input shape — confirmed via:
#   hailortcli parse-hef scdepthv3.hef → Input NHWC(256x320x3)
# These must match the HEF exactly; the runtime enforces the tensor size.
# When a proper flow HEF is substituted, update these to its compiled shape.
DEFAULT_INPUT_H   = 256
DEFAULT_INPUT_W   = 320

# Minimum depth-difference gradient magnitude to count as valid motion signal
MIN_FLOW_MAGNITUDE = 0.5


class HailoOpticalFlow:
    """Estimates body-frame XY velocity from consecutive nadir camera frames.

    Args:
        network_group: Configured network group from HailoDevice.load_model().
        input_h:       Model input height in pixels. Default 480.
        input_w:       Model input width in pixels. Default 640.
        fps:           Camera frame rate used for velocity scaling. Default 30.
        focal_px:      Focal length in pixels from camera calibration.
        agl_default:   Default altitude AGL in metres if not supplied per-call.
    """

    def __init__(
        self,
        network_group,
        input_h:     int   = DEFAULT_INPUT_H,
        input_w:     int   = DEFAULT_INPUT_W,
        fps:         float = DEFAULT_FPS,
        focal_px:    float = DEFAULT_FOCAL_PX,
        agl_default: float = 5.0,
    ):
        self._network_group = network_group
        self._input_h       = input_h
        self._input_w       = input_w
        self._fps           = fps
        self._focal_px      = focal_px
        self._agl_default   = agl_default

        self._prev_frame    = None   # Preprocessed RGB tensor from t-1
        self._prev_depth    = None   # Depth map output from t-1 (SCDepthV3)
        self._latency_buf   = []     # Rolling latency measurements (ms)
        self._fps_buf       = []     # Rolling FPS measurements
        self._last_call_t   = None

        # Resolve vstream params once at construction.
        # hailort >= 4.23.0: make_from_network_group() returns
        # Dict[str, VStreamParams], not a list. Store as dict and pass
        # directly to InferVStreams — do not convert to list at call site.
        # Pre-4.23.0 returned a list; normalise to dict for uniform access.
        try:
            from hailo_platform import (
                InputVStreamParams, OutputVStreamParams,
                FormatType, HailoStreamInterface
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

    def push_frame(self, frame: np.ndarray, agl: float = None) -> dict:
        """Push a new nadir frame and return a velocity estimate.

        On the first call, stores the frame and returns a zero result —
        a frame pair is required for flow estimation.

        Args:
            frame: BGR or greyscale numpy array from camera. Any resolution —
                   resized internally to (input_h, input_w).
            agl:   Altitude above ground in metres. If None, uses agl_default.
                   More accurate AGL (from LiDAR or baro) gives more accurate
                   metric velocity — pass node.get_pos()[2] from MainNode.

        Returns:
            dict with keys:
                vx         (float) — forward velocity m/s (body frame)
                vy         (float) — lateral velocity m/s (body frame)
                confidence (float) — 0.0–1.0, reliability estimate
                latency_ms (float) — inference wall time in milliseconds
                valid      (bool)  — False on first call or inference failure
        """
        t0  = time.monotonic()
        agl = agl if agl is not None else self._agl_default

        current = self._preprocess(frame)

        if self._prev_frame is None:
            # First call — store frame and run inference to populate _prev_depth.
            # No velocity can be derived yet (need two depth maps to difference).
            self._prev_frame = current
            self._run_depth_only(current, t0)
            return self._zero_result(latency_ms=0.0)

        result = self._infer(current, agl, t0)
        self._prev_frame = current
        self._update_fps()
        return result

    def get_fps(self) -> float:
        """Return rolling average inference rate in Hz."""
        if len(self._fps_buf) < 2:
            return 0.0
        intervals = [self._fps_buf[i+1] - self._fps_buf[i]
                     for i in range(len(self._fps_buf) - 1)]
        avg_interval = sum(intervals) / len(intervals)
        return 1.0 / avg_interval if avg_interval > 0 else 0.0

    def get_avg_latency_ms(self) -> float:
        """Return rolling average inference latency in milliseconds."""
        if not self._latency_buf:
            return 0.0
        return sum(self._latency_buf) / len(self._latency_buf)

    def reset(self) -> None:
        """Clear stored frame and depth map. Call on mode transitions."""
        self._prev_frame = None
        self._prev_depth = None

    # ── Private ───────────────────────────────────────────────────────────────

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """Resize to SCDepthV3 input shape and format as (1, H, W, 3) uint8.

        SCDepthV3 compiled shape: NHWC(256, 320, 3) — single RGB frame.
        BGR→RGB conversion is applied because the model was trained on RGB.
        The output tensor feeds directly into InferVStreams as the input dict
        value; no two-frame concatenation is performed (that was for flow nets).
        """
        import cv2
        resized = cv2.resize(frame, (self._input_w, self._input_h))
        if len(resized.shape) == 2:
            # Greyscale input — replicate to 3 channels
            resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2RGB)
        else:
            resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # Shape: (1, H, W, 3) — batch=1, RGB
        return np.expand_dims(resized, axis=0).astype(np.uint8)

    def _infer(
        self,
        frame_t1: np.ndarray,
        agl:      float,
        t_start:  float,
    ) -> dict:
        """Run SCDepthV3 inference on the current frame and derive velocity.

        SCDepthV3 takes a single RGB frame and produces a depth map. Velocity
        is derived by differencing consecutive depth maps:

            depth_diff = depth_t1 - depth_t0
            flow ≈ spatial gradient of depth_diff

        hailort 4.23.0 requires network_group.activate() to be entered before
        any InferVStreams call. The activate() context wraps InferVStreams here.
        Both contexts are nested: activate → InferVStreams → infer().

        self._input_params / _output_params are Dict[str, VStreamParams].
        InferVStreams requires dicts — passed directly, not converted to lists.
        The input stream name is extracted from dict keys().
        """
        try:
            from hailo_platform import InferVStreams

            input_name = list(self._input_params.keys())[0]

            # activate() is required in hailort >= 4.23.0 before InferVStreams.
            # Omitting it produces HAILO_NETWORK_GROUP_NOT_ACTIVATED(69).
            with self._network_group.activate():
                with InferVStreams(
                    self._network_group,
                    self._input_params,
                    self._output_params,
                ) as pipeline:
                    raw_outputs = pipeline.infer({input_name: frame_t1})

            latency_ms = (time.monotonic() - t_start) * 1000.0
            self._latency_buf.append(latency_ms)
            if len(self._latency_buf) > 30:
                self._latency_buf.pop(0)

            return self._parse_depth_output(raw_outputs, agl, latency_ms)

        except Exception as exc:
            latency_ms = (time.monotonic() - t_start) * 1000.0
            print(f"[HailoOpticalFlow] Inference error: {exc}")
            return self._zero_result(latency_ms=latency_ms)

    def _parse_depth_output(
        self, raw_outputs: dict, agl: float, latency_ms: float
    ) -> dict:
        """Derive velocity from consecutive SCDepthV3 depth maps.

        SCDepthV3 output shape is (1, 1, H, W) or (1, H, W, 1) — a single
        inverse-depth channel. Velocity is estimated by differencing the
        current depth map against the stored previous depth map, then taking
        the mean spatial gradient of that difference field.

        The depth-difference approach:
            depth_diff = depth_t1 - depth_t0
            grad_y, grad_x = np.gradient(depth_diff)
            mean_dx = mean(grad_x)   → forward velocity proxy
            mean_dy = mean(grad_y)   → lateral velocity proxy

        Confidence is the inverse of the variance of the difference field —
        high variance means scene content or rotation dominates, not camera
        translation, so confidence is correctly low.

        If a proper flow HEF is loaded later, _parse_flow_output() handles
        (1,2,H,W) RAFT-style tensors and is called from _infer() instead.
        """
        output_key  = list(raw_outputs.keys())[0]
        # hailort 4.23.0: infer() may return a list of arrays per output
        # rather than a single ndarray. np.array() coerces both forms.
        output_data = np.array(raw_outputs[output_key]).astype(np.float32)
        shape       = output_data.shape

        # Extract depth map as (H, W) float32 regardless of channel ordering
        if len(shape) == 4 and shape[1] == 1:
            depth_now = output_data[0, 0]          # (1, 1, H, W)
        elif len(shape) == 4 and shape[3] == 1:
            depth_now = output_data[0, :, :, 0]    # (1, H, W, 1)
        elif len(shape) == 4 and shape[1] == 2:
            # RAFT-style flow tensor — two channels are dx, dy directly
            flow = np.transpose(output_data[0], (1, 2, 0))  # (H, W, 2)
            return self._velocity_from_flow(flow, agl, latency_ms)
        elif len(shape) == 4 and shape[3] == 2:
            flow = output_data[0]                            # (H, W, 2)
            return self._velocity_from_flow(flow, agl, latency_ms)
        else:
            return self._zero_result(latency_ms=latency_ms)

        # Depth difference between t1 and stored t0
        if self._prev_depth is None:
            # No previous depth yet — store and return zero
            self._prev_depth = depth_now
            return self._zero_result(latency_ms=latency_ms)

        depth_diff = depth_now - self._prev_depth
        self._prev_depth = depth_now

        # Spatial gradient of depth difference approximates pixel-level motion
        grad_y, grad_x = np.gradient(depth_diff)
        flow = np.stack([grad_x, grad_y], axis=-1)

        return self._velocity_from_flow(flow, agl, latency_ms)

    def _velocity_from_flow(
        self, flow: np.ndarray, agl: float, latency_ms: float
    ) -> dict:
        """Convert a (H, W, 2) flow field to a metric velocity dict.

        flow[:,:,0] is the x-axis displacement (forward, vx).
        flow[:,:,1] is the y-axis displacement (lateral, vy).

        Confidence is the inverse of total flow variance — high variance
        indicates rotation or scene change rather than translational motion.
        """
        mean_dx = float(np.mean(flow[:, :, 0]))
        mean_dy = float(np.mean(flow[:, :, 1]))
        var     = float(np.var(flow[:, :, 0]) + np.var(flow[:, :, 1]))
        magnitude  = float(np.sqrt(mean_dx**2 + mean_dy**2))
        confidence = float(np.clip(1.0 / (1.0 + var * 0.1), 0.0, 1.0))

        if magnitude < MIN_FLOW_MAGNITUDE:
            return {
                "vx": 0.0, "vy": 0.0,
                "confidence": confidence,
                "latency_ms": latency_ms,
                "valid": True,
            }

        # Pinhole projection: pixel/frame → m/s
        scale = (1.0 / self._fps) * (agl / self._focal_px)
        return {
            "vx":         float(mean_dx * scale),
            "vy":         float(mean_dy * scale),
            "confidence": confidence,
            "latency_ms": latency_ms,
            "valid":      True,
        }

    def _run_depth_only(self, frame: np.ndarray, t_start: float) -> None:
        """Run inference on the first frame to warm up _prev_depth.

        On the first push_frame() call there is no previous depth map to
        difference against. This method runs inference, stores the resulting
        depth map in _prev_depth, and discards the timing — no velocity is
        returned. The second call to push_frame() will produce the first
        valid velocity estimate.
        """
        try:
            from hailo_platform import InferVStreams
            input_name = list(self._input_params.keys())[0]
            with self._network_group.activate():
                with InferVStreams(
                    self._network_group,
                    self._input_params,
                    self._output_params,
                ) as pipeline:
                    raw_outputs = pipeline.infer({input_name: frame})

            output_key  = list(raw_outputs.keys())[0]
            output_data = np.array(raw_outputs[output_key]).astype(np.float32)
            shape = output_data.shape
            if len(shape) == 4 and shape[1] == 1:
                self._prev_depth = output_data[0, 0]
            elif len(shape) == 4 and shape[3] == 1:
                self._prev_depth = output_data[0, :, :, 0]
        except Exception as exc:
            print(f"[HailoOpticalFlow] Warm-up inference error: {exc}")

    def _zero_result(self, latency_ms: float = 0.0) -> dict:
        return {
            "vx": 0.0, "vy": 0.0,
            "confidence": 0.0,
            "latency_ms": latency_ms,
            "valid": False,
        }

    def _update_fps(self) -> None:
        now = time.monotonic()
        self._fps_buf.append(now)
        if len(self._fps_buf) > 30:
            self._fps_buf.pop(0)
