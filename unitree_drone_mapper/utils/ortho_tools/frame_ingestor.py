"""
frame_ingestor.py — Flight frame loader for the ortho pipeline.

Reads the flight_frames/ directory written by camera_capture.py during
flight and produces a list of FrameRecord objects, one per JPEG frame,
each carrying the image array, matched SLAM pose, GPS coordinates, and
ROS timestamp.

Design
------
This module is the ortho pipeline equivalent of mesh_tools/bag_reader.py.
bag_reader.py reads LiDAR point cloud data from a ROS2 bag. frame_ingestor.py
reads camera frames from the filesystem — no bag access is needed because
camera_capture.py already wrote each frame and its sidecar JSON to disk
during flight.

Pose matching uses PoseInterpolator from texture_tools, which was validated
as part of Stage 7 texture projection. Both pipelines use the same class
so any extrinsic or timestamp fix applies to both automatically.

ros_timestamp vs wall clock
---------------------------
camera_capture.py writes two timestamps to each sidecar JSON:
    ros_timestamp  — ROS node clock at trigger time (float seconds).
                     Used for SLAM pose matching via PoseInterpolator.
    timestamp_iso  — UTC wall clock (ISO 8601 string).
                     Used for human-readable logging only.

PoseInterpolator keys on ROS timestamps from the SLAM trajectory topic
(/aft_mapped_to_init), not wall clock. If ros_timestamp is None in the
sidecar (camera_capture.py was an older version), this module falls back
to wall clock matching with a logged WARNING. The fallback will be less
accurate proportionally to the offset between ROS time and wall time during
flight (typically < 1 s on a freshly booted Pi, but not guaranteed).

Sorted order
------------
Frames are returned sorted by ros_timestamp (ascending) so the downstream
mosaic and orthorectification stages process them in flight sequence order.
If ros_timestamp is absent, sort falls back to filename sequence number.

Output
------
list[FrameRecord] — one entry per accepted frame.

Each FrameRecord contains:
    image       np.ndarray (H, W, 3)  — BGR, as loaded by cv2.imread
    pose_4x4    np.ndarray (4, 4)     — SLAM pose at frame time (world frame)
                                        None if PoseInterpolator unavailable
    gps         tuple (lat, lon, alt) — GPS at trigger, or (None, None, None)
    ros_ts      float                 — ROS timestamp in seconds
    waypoint_idx int                  — waypoint index at trigger
    path        Path                  — source JPEG path (for debug)

Dependencies
------------
    opencv-python  — cv2.imread
    numpy
    texture_tools.pose_interpolator — PoseInterpolator (optional, degrades gracefully)
"""

import json
import logging
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# ── FrameRecord ───────────────────────────────────────────────────────────────

@dataclass
class FrameRecord:
    """
    One camera frame with all associated metadata for the ortho pipeline.

    Attributes
    ----------
    image       : BGR image array (H, W, 3) as loaded by cv2.imread.
    pose_4x4    : 4×4 homogeneous SLAM pose matrix (world frame).
                  None if PoseInterpolator was unavailable or interpolation failed.
    gps         : (lat, lon, alt) tuple. Values may be None if GPS was absent.
    ros_ts      : ROS clock timestamp in seconds. Used as the primary sort key
                  and for pose matching.
    waypoint_idx: Zero-based waypoint index at which this frame was triggered.
    path        : Source JPEG path. Used for debug logging only.
    """
    image:        np.ndarray
    pose_4x4:     Optional[np.ndarray]
    gps:          tuple                   # (lat, lon, alt) — elements may be None
    ros_ts:       Optional[float]
    waypoint_idx: int
    path:         Path


# ── FrameIngestor ─────────────────────────────────────────────────────────────

class FrameIngestor:
    """
    Loads all camera frames from a session's flight_frames/ directory.

    Parameters
    ----------
    session_dir : Path
        Root session directory (e.g. /mnt/ssd/maps/scan_20260402_120000/).
        The flight_frames/ subdirectory is resolved relative to this path.
    bag_path : Path or None
        Path to the ROS2 bag directory for the same session. Required only
        when PoseInterpolator needs to load the SLAM trajectory. If None,
        pose matching is skipped and all FrameRecord.pose_4x4 values are None.
    pose_topic : str
        SLAM odometry topic to read from the bag for pose matching.
        Default: /aft_mapped_to_init (Point-LIO output topic).
    frames_subdir : str
        Name of the frames subdirectory inside session_dir.
        Default: flight_frames — matches camera_capture.py output_dir convention.
    """

    FRAMES_SUBDIR = "flight_frames"
    DEFAULT_POSE_TOPIC = "/aft_mapped_to_init"

    def __init__(
        self,
        session_dir: Path,
        bag_path:    Optional[Path] = None,
        pose_topic:  str = DEFAULT_POSE_TOPIC,
        frames_subdir: str = FRAMES_SUBDIR,
    ):
        self.session_dir   = Path(session_dir)
        self.bag_path      = Path(bag_path) if bag_path else None
        self.pose_topic    = pose_topic
        self.frames_dir    = self.session_dir / frames_subdir
        self._interpolator = None   # Loaded lazily in load()

    # ── Public API ────────────────────────────────────────────────────────────

    def load(self) -> list[FrameRecord]:
        """
        Load all frames from flight_frames/, match SLAM poses, return sorted list.

        Steps:
            1. Validate frames directory exists and contains JPEGs.
            2. Load PoseInterpolator from SLAM trajectory if bag_path is set.
            3. For each JPEG in filename order:
                a. Load sidecar JSON — extract ros_ts, gps, waypoint_idx.
                b. Load image via cv2.imread.
                c. Interpolate SLAM pose at ros_ts.
                d. Build FrameRecord.
            4. Sort by ros_ts ascending (flight sequence order).
            5. Return list.

        Returns
        -------
        list[FrameRecord] — empty list if frames directory is missing or empty.
        """
        if not self.frames_dir.exists():
            logger.error(
                f"[FrameIngestor] flight_frames directory not found: {self.frames_dir}\n"
                f"  Expected location: {self.session_dir}/{self.FRAMES_SUBDIR}/\n"
                f"  This directory is written by camera_capture.py during MODE 3 flight.\n"
                f"  Check that camera_capture.py FLIGHT_BASE_DIR matches this session_dir."
            )
            return []

        jpeg_paths = sorted(self.frames_dir.glob("frame_*.jpg"))
        if not jpeg_paths:
            logger.warning(
                f"[FrameIngestor] No frame_*.jpg files found in {self.frames_dir}"
            )
            return []

        logger.info(
            f"[FrameIngestor] Found {len(jpeg_paths)} frame(s) in {self.frames_dir}"
        )

        self._interpolator = self._load_pose_interpolator()

        records = []
        missing_ros_ts = 0

        for path in jpeg_paths:
            record = self._load_one_frame(path)
            if record is None:
                continue
            if record.ros_ts is None:
                missing_ros_ts += 1
            records.append(record)

        if missing_ros_ts > 0:
            logger.warning(
                f"[FrameIngestor] {missing_ros_ts}/{len(records)} frame(s) have "
                f"ros_timestamp=None in their sidecar JSON.\n"
                f"  This indicates camera_capture.py was an older version that did "
                f"not include ros_timestamp in the trigger context.\n"
                f"  Pose matching for these frames will use wall-clock fallback "
                f"(less accurate). Update camera_capture.py and re-fly for best results."
            )

        # Sort by ROS timestamp ascending. Frames with ros_ts=None sort last.
        records.sort(key=lambda r: (r.ros_ts is None, r.ros_ts or 0.0))

        logger.info(
            f"[FrameIngestor] Loaded {len(records)} frame(s)  "
            f"pose_matched={sum(1 for r in records if r.pose_4x4 is not None)}  "
            f"gps_tagged={sum(1 for r in records if r.gps[0] is not None)}"
        )
        return records

    # ── Internal Helpers ──────────────────────────────────────────────────────

    def _load_one_frame(self, jpeg_path: Path) -> Optional[FrameRecord]:
        """
        Load one JPEG and its sidecar JSON into a FrameRecord.

        Returns None if the image cannot be loaded (corrupted file, disk error).
        A missing sidecar is tolerated — the record is created with None fields
        so the frame is still included in the mosaic even without metadata.
        """
        # ── Load image ────────────────────────────────────────────────────────
        image = cv2.imread(str(jpeg_path))
        if image is None:
            logger.warning(f"[FrameIngestor] Failed to load image: {jpeg_path.name}")
            return None

        # ── Load sidecar JSON ─────────────────────────────────────────────────
        json_path = jpeg_path.with_suffix(".json")
        sidecar   = {}
        if json_path.exists():
            try:
                sidecar = json.loads(json_path.read_text())
            except json.JSONDecodeError as exc:
                logger.warning(
                    f"[FrameIngestor] Sidecar JSON malformed for {jpeg_path.name}: {exc}"
                )
        else:
            logger.warning(
                f"[FrameIngestor] No sidecar JSON for {jpeg_path.name} — "
                f"metadata fields will be None"
            )

        # ── Extract metadata fields ───────────────────────────────────────────
        ros_ts       = sidecar.get("ros_timestamp")       # float or None
        waypoint_idx = sidecar.get("waypoint_index") or 0

        gps_block = sidecar.get("gps", {})
        gps = (
            gps_block.get("lat"),
            gps_block.get("lon"),
            gps_block.get("alt"),
        )

        # ── SLAM pose interpolation ───────────────────────────────────────────
        pose_4x4 = None
        if self._interpolator is not None and ros_ts is not None:
            pose_4x4 = self._interpolate_pose(ros_ts)
        elif self._interpolator is not None and ros_ts is None:
            # Wall-clock fallback using timestamp_iso
            # Less accurate — logged at warning level in load() already
            pass

        return FrameRecord(
            image        = image,
            pose_4x4     = pose_4x4,
            gps          = gps,
            ros_ts       = ros_ts,
            waypoint_idx = waypoint_idx,
            path         = jpeg_path,
        )

    def _load_pose_interpolator(self):
        """
        Load PoseInterpolator from texture_tools with the session SLAM trajectory.

        Returns the interpolator instance, or None if:
          - bag_path was not provided (caller opted out of pose matching)
          - texture_tools is not importable (package not yet installed)
          - The SLAM trajectory topic is absent from the bag

        Failure here is non-fatal. FrameRecord.pose_4x4 will be None for all
        frames, and the fast-path MosaicBuilder does not require pose data.
        The full-path Orthorectifier does require it and will log an error.
        """
        if self.bag_path is None:
            logger.info(
                "[FrameIngestor] bag_path not provided — "
                "SLAM pose matching disabled. Fast-path mosaic will proceed "
                "without pose data. Full-path orthorectifier requires a bag_path."
            )
            return None

        try:
            # texture_tools must be on sys.path — same requirement as Stage 7
            from texture_tools.pose_interpolator import PoseInterpolator
        except ImportError as exc:
            logger.warning(
                f"[FrameIngestor] texture_tools not importable: {exc}\n"
                f"  SLAM pose matching disabled. Check that utils/texture_tools/ "
                f"is installed and sys.path includes utils/."
            )
            return None

        try:
            interpolator = PoseInterpolator(
                bag_path   = str(self.bag_path),
                pose_topic = self.pose_topic,
            )
            interpolator.load()
            pose_count = len(interpolator)
            logger.info(
                f"[FrameIngestor] PoseInterpolator loaded  "
                f"poses={pose_count:,}  topic={self.pose_topic}"
            )
            return interpolator

        except Exception as exc:
            logger.warning(
                f"[FrameIngestor] PoseInterpolator failed to load: {exc}\n"
                f"  SLAM pose matching disabled for this session."
            )
            return None

    def _interpolate_pose(self, ros_ts: float) -> Optional[np.ndarray]:
        """
        Return the 4×4 SLAM pose at ros_ts via PoseInterpolator.

        Returns None if interpolation fails (timestamp out of range, etc.).
        Failure per-frame is non-fatal — the frame is still included in
        the mosaic, just without a matched pose.
        """
        try:
            pose = self._interpolator.get_pose_at(ros_ts)
            return pose   # 4×4 np.ndarray, world frame
        except Exception as exc:
            logger.debug(
                f"[FrameIngestor] Pose interpolation failed at ros_ts={ros_ts:.3f}: {exc}"
            )
            return None
