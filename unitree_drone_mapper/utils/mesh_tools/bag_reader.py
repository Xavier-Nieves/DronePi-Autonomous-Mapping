"""
bag_reader.py — Extract point cloud from a ROS 2 bag.

Reads /cloud_registered frames (sensor_msgs/PointCloud2) from a
rosbag2 .mcap file and returns a combined Nx3 float32 numpy array.

Also parses bag metadata.yaml for duration, timestamp, and frame count.

Dependencies: rosbags (pip install rosbags) -- no ROS install needed.
"""

import struct
import time
from pathlib import Path

import numpy as np


# Topic written by Point-LIO containing the registered point cloud
CLOUD_TOPIC = "/cloud_registered"


class BagReader:
    """
    Reads a ROS 2 bag and extracts the point cloud.

    Parameters
    ----------
    bag_path : Path or str
        Path to the bag directory (e.g. /mnt/ssd/rosbags/scan_20260319_230358)
    max_frames : int, optional
        Maximum number of frames to read. None = all frames.
    cap : int, optional
        Maximum total points after combining all frames.
        Uniform subsample applied if exceeded. None = no cap.

    Example
    -------
        reader = BagReader("/mnt/ssd/rosbags/scan_20260319_230358")
        pts    = reader.extract()          # Nx3 float32
        meta   = reader.metadata          # dict with duration, timestamp etc
    """

    def __init__(self,
                 bag_path,
                 max_frames: int | None = None,
                 cap: int | None = None):
        self.bag_path   = Path(bag_path)
        self.max_frames = max_frames
        self.cap        = cap
        self.metadata   = {}

    # ── public ────────────────────────────────────────────────────────────────

    def extract(self) -> np.ndarray:
        """
        Extract and return combined point cloud as Nx3 float32 array.
        Also populates self.metadata with bag timing information.
        """
        t0 = time.time()
        self._parse_metadata()
        pts = self._read_cloud()
        elapsed = time.time() - t0
        print(f"  [BagReader] {len(pts):,} points extracted in {elapsed:.1f}s")
        return pts

    # ── metadata ──────────────────────────────────────────────────────────────

    def _parse_metadata(self):
        """Parse metadata.yaml from bag directory."""
        import yaml
        meta_path = self.bag_path / "metadata.yaml"
        if not meta_path.exists():
            print("  [BagReader] metadata.yaml not found -- duration unknown")
            self.metadata = {
                "duration_s":    120.0,
                "duration_str":  "??m ??s",
                "timestamp_iso": "unknown",
                "cloud_frames":  0,
                "ros_distro":    "jazzy",
            }
            return

        with open(meta_path) as f:
            raw = yaml.safe_load(f)

        info        = raw.get("rosbag2_bagfile_information", {})
        duration_ns = info.get("duration", {}).get("nanoseconds", 0)
        duration_s  = duration_ns / 1e9
        start_ns    = info.get("starting_time", {}).get("nanoseconds_since_epoch", 0)

        from datetime import datetime, timezone
        ts = datetime.fromtimestamp(start_ns / 1e9, tz=timezone.utc)

        # Count cloud frames
        cloud_frames = 0
        for t in info.get("topics_with_message_count", []):
            if t.get("topic_metadata", {}).get("name") == CLOUD_TOPIC:
                cloud_frames = t.get("message_count", 0)

        self.metadata = {
            "duration_s":    round(duration_s, 1),
            "duration_str":  f"{int(duration_s//60):02d}m {int(duration_s%60):02d}s",
            "timestamp_iso": ts.strftime("%Y-%m-%dT%H:%M:%SZ"),
            "cloud_frames":  cloud_frames,
            "ros_distro":    info.get("ros_distro", "jazzy"),
        }
        print(f"  [BagReader] Duration: {self.metadata['duration_str']}  "
              f"Frames: {cloud_frames:,}")

    # ── point cloud reader ────────────────────────────────────────────────────

    def _read_cloud(self) -> np.ndarray:
        """Read all /cloud_registered frames and stack into Nx3 array."""
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        pts_all   = []
        frames    = 0

        with Reader(str(self.bag_path)) as reader:
            connections = [c for c in reader.connections
                           if c.topic == CLOUD_TOPIC]
            if not connections:
                raise RuntimeError(
                    f"Topic {CLOUD_TOPIC} not found in bag.\n"
                    f"Available topics: "
                    f"{[c.topic for c in reader.connections]}"
                )

            for conn, ts, data in reader.messages(connections=connections):
                if self.max_frames and frames >= self.max_frames:
                    break
                msg = typestore.deserialize_cdr(data, conn.msgtype)
                pts = self._unpack_pointcloud2(msg)
                if pts is not None and len(pts) > 0:
                    pts_all.append(pts)
                frames += 1

        if not pts_all:
            raise RuntimeError("No valid point cloud data found in bag.")

        combined = np.vstack(pts_all)

        # Apply cap if specified
        if self.cap and len(combined) > self.cap:
            idx      = np.random.choice(len(combined), self.cap, replace=False)
            combined = combined[idx]
            print(f"  [BagReader] Cap applied: {len(pts_all)*pts_all[0].shape[0]:,}"
                  f" -> {len(combined):,}")

        return combined

    def _unpack_pointcloud2(self, msg) -> np.ndarray | None:
        """Unpack a PointCloud2 message into Nx3 float32 array."""
        fields = {f.name: f.offset for f in msg.fields}
        if "x" not in fields:
            return None

        ps   = msg.point_step
        raw  = bytes(msg.data)
        n    = msg.width * msg.height
        x_o  = fields["x"]
        y_o  = fields.get("y", x_o + 4)
        z_o  = fields.get("z", x_o + 8)

        pts = np.empty((n, 3), dtype=np.float32)
        for i in range(n):
            b = i * ps
            pts[i, 0] = struct.unpack_from("<f", raw, b + x_o)[0]
            pts[i, 1] = struct.unpack_from("<f", raw, b + y_o)[0]
            pts[i, 2] = struct.unpack_from("<f", raw, b + z_o)[0]

        # Remove NaN/Inf
        valid = np.isfinite(pts).all(axis=1)
        return pts[valid]
