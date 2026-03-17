#!/usr/bin/env python3
"""IMX219 camera live preview — tries three backends in order.

Backend priority (Pi 5 / Ubuntu 24.04):
  1. picamera2   — native libcamera Python API (best for Pi CSI)
  2. GStreamer    — libcamerasrc pipeline via OpenCV CAP_GSTREAMER
  3. V4L2         — fallback for USB cameras

Run (no ROS needed, any terminal):
  python3 -m tests.test_camera_preview
  python3 -m tests.test_camera_preview --backend picamera2
  python3 -m tests.test_camera_preview --backend gst
  python3 -m tests.test_camera_preview --backend v4l2
  python3 -m tests.test_camera_preview --list
  python3 -m tests.test_camera_preview --width 640 --height 480

Controls:
  Q / Esc  : quit
  S        : save a snapshot to data/snapshots/

If the camera is not detected:
  sudo apt install -y libcamera-ipa libcamera-tools
  sudo apt install -y python3-picamera2
  Then run:  cam --list      (should show imx219)
"""

import argparse
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# ── paths ──────────────────────────────────────────────────────────────────────
_THIS_DIR    = Path(__file__).resolve().parent
MAPPER_DIR   = _THIS_DIR.parent
SNAPSHOT_DIR = MAPPER_DIR / "data/snapshots"


# ── camera backends ────────────────────────────────────────────────────────────

class Picamera2Backend:
    """Native libcamera via picamera2 (best for Pi 5 CSI cameras)."""

    def __init__(self, width: int, height: int):
        from picamera2 import Picamera2  # noqa: PLC0415
        self.cam = Picamera2()
        cfg = self.cam.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(0.5)  # sensor warm-up

    def read(self):
        """Return (True, BGR frame) or (False, None)."""
        import cv2  # noqa: PLC0415
        frame_rgb = self.cam.capture_array()
        if frame_rgb is None:
            return False, None
        return True, cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    def release(self):
        self.cam.stop()

    @staticmethod
    def name() -> str:
        return "picamera2"


class GStreamerBackend:
    """libcamerasrc GStreamer pipeline (works when picamera2 not installed)."""

    def __init__(self, width: int, height: int):
        import cv2  # noqa: PLC0415
        pipeline = (
            f"libcamerasrc ! "
            f"video/x-raw,width={width},height={height},framerate=30/1 ! "
            f"videoconvert ! appsink max-buffers=2 drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("GStreamer libcamerasrc pipeline failed to open")

    def read(self):
        import cv2  # noqa: PLC0415
        ret, frame = self.cap.read()
        return ret, frame

    def release(self):
        self.cap.release()

    @staticmethod
    def name() -> str:
        return "GStreamer/libcamerasrc"


class V4L2Backend:
    """Direct V4L2 — works for USB cameras; usually NOT for Pi 5 CSI."""

    def __init__(self, device: int, width: int, height: int):
        import cv2  # noqa: PLC0415
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if not self.cap.isOpened():
            raise RuntimeError(f"V4L2 device /dev/video{device} failed to open")
        # Verify at least one frame is readable
        ret, _ = self.cap.read()
        if not ret:
            self.cap.release()
            raise RuntimeError(f"/dev/video{device} opened but returned no frame")

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()

    @staticmethod
    def name() -> str:
        return "V4L2"


# ── helpers ────────────────────────────────────────────────────────────────────

def open_camera(backend_choice: str, device: int,
                width: int, height: int):
    """Try backends in priority order; return (backend_obj, backend_name)."""
    order = []
    if backend_choice == "auto":
        order = ["picamera2", "gst", "v4l2"]
    else:
        order = [backend_choice]

    for b in order:
        try:
            if b == "picamera2":
                print(f"  Trying picamera2 backend...")
                cam = Picamera2Backend(width, height)
                print(f"  [OK] picamera2 opened IMX219  ({width}×{height})")
                return cam
            elif b == "gst":
                print(f"  Trying GStreamer/libcamerasrc backend...")
                cam = GStreamerBackend(width, height)
                print(f"  [OK] GStreamer opened camera  ({width}×{height})")
                return cam
            elif b == "v4l2":
                print(f"  Trying V4L2 /dev/video{device}...")
                cam = V4L2Backend(device, width, height)
                print(f"  [OK] V4L2 /dev/video{device}  ({width}×{height})")
                return cam
        except Exception as exc:
            print(f"  [SKIP] {b}: {exc}")

    return None


def list_cameras() -> None:
    """Probe all backends and report what's available."""
    import cv2  # noqa: PLC0415

    print("=== Camera discovery ===\n")

    # picamera2
    try:
        from picamera2 import Picamera2  # noqa: PLC0415
        cams = Picamera2.global_camera_info()
        if cams:
            for i, c in enumerate(cams):
                print(f"  [picamera2] camera {i}: {c}")
        else:
            print("  [picamera2] No cameras found (check libcamera-ipa)")
    except ImportError:
        print("  [picamera2] Not installed  (sudo apt install python3-picamera2)")
    except Exception as exc:
        print(f"  [picamera2] Error: {exc}")

    # V4L2
    print()
    found_v4l2 = False
    for i in range(10):
        dev = f"/dev/video{i}"
        if not os.path.exists(dev):
            continue
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            status = "OK" if ret else "open but no frame"
            print(f"  [V4L2] /dev/video{i}  [{status}]")
            found_v4l2 = True
        cap.release()
    if not found_v4l2:
        print("  [V4L2] No /dev/video* devices found")

    print()
    print("Tip: if IMX219 is not listed run:")
    print("  sudo apt install -y libcamera-ipa libcamera-tools python3-picamera2")
    print("  cam --list")


def draw_overlay(frame: np.ndarray, fps: float, frame_no: int,
                 backend: str) -> np.ndarray:
    """Burn-in a status bar at the bottom of the frame."""
    import cv2  # noqa: PLC0415
    h, w = frame.shape[:2]
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, h - 36), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
    text = (f"{backend}  |  {w}×{h}  |  "
            f"FPS: {fps:.1f}  |  frame #{frame_no}  |  "
            f"[S] snapshot  [Q] quit")
    cv2.putText(frame, text,
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                (200, 230, 200), 1, cv2.LINE_AA)
    return frame


# ── main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    import cv2  # noqa: PLC0415

    ap = argparse.ArgumentParser(
        description="IMX219 camera preview — picamera2 / GStreamer / V4L2",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--backend", choices=["auto", "picamera2", "gst", "v4l2"],
                    default="auto",
                    help="Camera backend (default: auto — tries all in order)")
    ap.add_argument("--device", "-d", type=int, default=0,
                    help="V4L2 device index (only used with --backend v4l2)")
    ap.add_argument("--width",  "-W", type=int, default=1280,
                    help="Capture width  (default 1280)")
    ap.add_argument("--height", "-H", type=int, default=720,
                    help="Capture height (default  720)")
    ap.add_argument("--list", action="store_true",
                    help="List all available cameras and exit")
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return

    print("=== IMX219 Camera Preview ===\n")
    print(f"Requested: {args.width}×{args.height}  backend={args.backend}\n")

    cam = open_camera(args.backend, args.device, args.width, args.height)
    if cam is None:
        print("\n[FAIL] No camera backend succeeded.\n")
        print("Fix checklist:")
        print("  1. sudo apt install -y libcamera-ipa libcamera-tools python3-picamera2")
        print("  2. cam --list                    — confirm IMX219 is detected")
        print("  3. Ribbon cable fully seated in CAMERA port (not DISP)")
        print("  4. python3 -m tests.test_camera_preview --list")
        return

    SNAPSHOT_DIR.mkdir(parents=True, exist_ok=True)

    # Grab one frame to get actual dimensions
    ret, first_frame = cam.read()
    if not ret or first_frame is None:
        print("[FAIL] Camera opened but returned no frame.")
        cam.release()
        return

    actual_h, actual_w = first_frame.shape[:2]
    print(f"\nActual frame size: {actual_w}×{actual_h}")
    print("Controls:  Q / Esc = quit    S = snapshot\n")

    cv2.namedWindow("IMX219 Preview", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("IMX219 Preview", min(actual_w, 1280), min(actual_h, 720))

    fps_history: list[float] = []
    t_last = time.time()
    frame_no = 0
    backend_name = cam.name()

    # Feed the already-captured first frame into the loop
    pending = first_frame

    while True:
        if pending is not None:
            frame = pending
            pending = None
        else:
            ret, frame = cam.read()
            if not ret or frame is None:
                print("[WARN] Frame grab failed — retrying...")
                time.sleep(0.05)
                continue

        now = time.time()
        fps_history.append(1.0 / max(now - t_last, 1e-6))
        fps_history = fps_history[-20:]
        fps = sum(fps_history) / len(fps_history)
        t_last = now
        frame_no += 1

        display = draw_overlay(frame.copy(), fps, frame_no, backend_name)
        cv2.imshow("IMX219 Preview", display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            break
        if key in (ord("s"), ord("S")):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = str(SNAPSHOT_DIR / f"snapshot_{ts}.jpg")
            cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            print(f"  Snapshot saved: {path}")

    cam.release()
    cv2.destroyAllWindows()
    print(f"Preview closed after {frame_no} frames.")


if __name__ == "__main__":
    main()
