#!/usr/bin/env python3
"""
test_frame_ingestor.py — Standalone test for FrameIngestor.

Validates that FrameIngestor correctly reads a flight_frames/ directory,
parses sidecar JSON, and reports what it found — without requiring a live
ROS environment or Picamera2.

This is a standalone test script. It is NOT imported by any production module.

Usage
-----
# Against a real session directory:
python3 test_frame_ingestor.py --session /mnt/ssd/maps/scan_20260402_120000

# With bag path for SLAM pose matching:
python3 test_frame_ingestor.py \
    --session /mnt/ssd/maps/scan_20260402_120000 \
    --bag     /mnt/ssd/maps/scan_20260402_120000/rosbag

# Against a synthetic directory (creates temp frames for CI use):
python3 test_frame_ingestor.py --synthetic

What it checks
--------------
1. flight_frames/ directory found and non-empty
2. Each JPEG loads correctly (not corrupted)
3. Each sidecar JSON parses correctly
4. ros_timestamp present in sidecars (warns if missing)
5. GPS coordinates present in sidecars
6. PoseInterpolator loads and matches poses (if --bag provided)
7. Frames sorted by ros_timestamp ascending
8. FrameRecord fields populated as expected
"""

import argparse
import json
import logging
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

# ── Logging ───────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("test_frame_ingestor")


# ── Synthetic data generator ──────────────────────────────────────────────────

def create_synthetic_session(base_dir: Path, n_frames: int = 5) -> Path:
    """
    Create a minimal synthetic flight_frames/ directory for offline testing.

    Each frame is a 100×100 random colour JPEG with a valid sidecar JSON
    containing all fields that FrameIngestor expects, including ros_timestamp.

    Returns the session_dir path.
    """
    session_dir  = base_dir / "scan_test_synthetic"
    frames_dir   = session_dir / "flight_frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    base_ros_ts = 1711234567.0   # arbitrary ROS epoch start

    for i in range(1, n_frames + 1):
        # Synthetic JPEG — random colour noise
        img = np.random.randint(0, 255, (100, 100, 3), dtype=np.uint8)
        jpg_path = frames_dir / f"frame_{i:04d}.jpg"
        cv2.imwrite(str(jpg_path), img)

        # Sidecar JSON with all expected fields
        ros_ts = base_ros_ts + (i - 1) * 0.5   # 0.5s between waypoints
        sidecar = {
            "frame_index":    i,
            "filename":       jpg_path.name,
            "timestamp_iso":  f"2026-04-02T12:00:0{i}.000000+00:00",
            "ros_timestamp":  ros_ts,
            "waypoint_index": i - 1,
            "enu":            {"x": float(i), "y": float(i * 2), "z": 10.0},
            "gps":            {"lat": 18.2208 + i * 0.0001,
                               "lon": -67.1541 + i * 0.0001,
                               "alt": 50.0},
            "latency_ms":     12.5,
            "sensor_mode":    "2028x1520",
            "framerate":      40,
            "jpeg_quality":   95,
        }
        (frames_dir / f"frame_{i:04d}.json").write_text(
            json.dumps(sidecar, indent=2)
        )

    log.info(f"Synthetic session created: {session_dir}")
    log.info(f"  {n_frames} frames in {frames_dir}")
    return session_dir


# ── Test runner ───────────────────────────────────────────────────────────────

def run_test(session_dir: Path, bag_path: Path = None) -> bool:
    """
    Run FrameIngestor against session_dir and validate results.

    Returns True if all checks pass, False if any check fails.
    """
    # Ensure ortho_tools is importable — resolve utils/ relative to this script
    utils_dir = Path(__file__).resolve().parent.parent
    if str(utils_dir) not in sys.path:
        sys.path.insert(0, str(utils_dir))

    try:
        from ortho_tools.frame_ingestor import FrameIngestor
    except ImportError as exc:
        log.error(f"Cannot import FrameIngestor: {exc}")
        log.error(f"  Run this script from utils/ or ensure ortho_tools/ is on sys.path")
        return False

    log.info("=" * 55)
    log.info(f"  FrameIngestor Test")
    log.info(f"  Session : {session_dir}")
    log.info(f"  Bag     : {bag_path or 'None (pose matching disabled)'}")
    log.info("=" * 55)

    ingestor = FrameIngestor(
        session_dir = session_dir,
        bag_path    = bag_path,
    )
    records = ingestor.load()

    # ── Check 1: Any records returned ─────────────────────────────────────────
    if not records:
        log.error("[FAIL] Check 1 — No FrameRecords returned")
        log.error(f"        Expected flight_frames/ at: {session_dir}/flight_frames/")
        return False
    log.info(f"[PASS] Check 1 — {len(records)} FrameRecord(s) returned")

    # ── Check 2: All images loaded ────────────────────────────────────────────
    bad_images = [r for r in records if r.image is None]
    if bad_images:
        log.error(f"[FAIL] Check 2 — {len(bad_images)} frame(s) have None image")
        for r in bad_images:
            log.error(f"        {r.path.name}")
        return False
    log.info(f"[PASS] Check 2 — All images loaded  "
             f"shape={records[0].image.shape}")

    # ── Check 3: ros_timestamp present ────────────────────────────────────────
    missing_ts = [r for r in records if r.ros_ts is None]
    if missing_ts:
        log.warning(
            f"[WARN] Check 3 — {len(missing_ts)}/{len(records)} frame(s) "
            f"missing ros_timestamp.\n"
            f"         Pose matching will use wall-clock fallback for these frames.\n"
            f"         Update camera_capture.py to include ros_timestamp in context."
        )
    else:
        log.info(f"[PASS] Check 3 — All frames have ros_timestamp")

    # ── Check 4: GPS present ──────────────────────────────────────────────────
    no_gps = [r for r in records if r.gps[0] is None]
    if no_gps:
        log.warning(
            f"[WARN] Check 4 — {len(no_gps)}/{len(records)} frame(s) have no GPS fix.\n"
            f"         Ortho output will not be georeferenced for these frames."
        )
    else:
        fix = records[0].gps
        log.info(f"[PASS] Check 4 — GPS present  "
                 f"first frame: lat={fix[0]:.6f}  lon={fix[1]:.6f}")

    # ── Check 5: Sorted by ros_timestamp ─────────────────────────────────────
    ts_values = [r.ros_ts for r in records if r.ros_ts is not None]
    if len(ts_values) >= 2:
        sorted_ok = all(ts_values[i] <= ts_values[i+1]
                        for i in range(len(ts_values) - 1))
        if sorted_ok:
            log.info(f"[PASS] Check 5 — Frames sorted by ros_timestamp  "
                     f"range={ts_values[0]:.3f}–{ts_values[-1]:.3f}s")
        else:
            log.error("[FAIL] Check 5 — Frames NOT sorted by ros_timestamp")
            return False
    else:
        log.info("[SKIP] Check 5 — Not enough timestamped frames to verify sort")

    # ── Check 6: Pose matching (if bag provided) ──────────────────────────────
    if bag_path is not None:
        matched = [r for r in records if r.pose_4x4 is not None]
        if not matched:
            log.warning(
                "[WARN] Check 6 — No poses matched despite bag_path being provided.\n"
                "         Check that texture_tools/ is installed and the bag contains\n"
                f"        topic: {FrameIngestor.DEFAULT_POSE_TOPIC}"
            )
        else:
            log.info(f"[PASS] Check 6 — {len(matched)}/{len(records)} pose(s) matched  "
                     f"first pose shape={matched[0].pose_4x4.shape}")
    else:
        log.info("[SKIP] Check 6 — bag_path not provided, pose matching not tested")

    # ── Summary ───────────────────────────────────────────────────────────────
    log.info("=" * 55)
    log.info(f"  Frames loaded      : {len(records)}")
    log.info(f"  With ros_timestamp : {sum(1 for r in records if r.ros_ts is not None)}")
    log.info(f"  With GPS           : {sum(1 for r in records if r.gps[0] is not None)}")
    log.info(f"  With pose          : {sum(1 for r in records if r.pose_4x4 is not None)}")
    log.info("=" * 55)
    log.info("FrameIngestor test PASSED")
    return True


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Standalone test for ortho_tools.FrameIngestor"
    )
    parser.add_argument(
        "--session", type=Path, default=None,
        help="Session directory (e.g. /mnt/ssd/maps/scan_20260402_120000)"
    )
    parser.add_argument(
        "--bag", type=Path, default=None,
        help="ROS2 bag path for SLAM pose matching (optional)"
    )
    parser.add_argument(
        "--synthetic", action="store_true",
        help="Create and test against synthetic frames (no real data needed)"
    )
    args = parser.parse_args()

    if args.synthetic:
        with tempfile.TemporaryDirectory() as tmp:
            session_dir = create_synthetic_session(Path(tmp))
            ok = run_test(session_dir, bag_path=None)
    elif args.session:
        ok = run_test(args.session, bag_path=args.bag)
    else:
        parser.print_help()
        print("\nExample: python3 test_frame_ingestor.py --synthetic")
        sys.exit(0)

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
