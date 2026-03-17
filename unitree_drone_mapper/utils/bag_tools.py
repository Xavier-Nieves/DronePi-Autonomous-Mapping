"""ROS 2 bag reading, topic extraction, and point cloud conversion.

Migrated and generalized from ws/src/tools/bag_tools/bag_to_pcd.py
"""

import os
import numpy as np
from pathlib import Path
from typing import List, Tuple

from utils.logger import setup_logger

logger = setup_logger(__name__)


def read_pointclouds_from_bag(
    bag_path: str,
    topic: str = "/cloud_registered",
    fields: tuple = ("x", "y", "z", "intensity"),
) -> Tuple[List[np.ndarray], int]:
    """Read all point cloud messages from a ROS 2 bag file.

    Args:
        bag_path: Path to the bag directory.
        topic: Topic name to extract point clouds from.
        fields: Point fields to extract.

    Returns:
        Tuple of (list of Nx4 arrays per message, total message count).
    """
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    storage_id = _detect_storage_format(bag_path)

    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    clouds = []
    msg_count = 0

    logger.info(f"Reading bag: {bag_path} (topic: {topic})")

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, PointCloud2)
            points = []
            for point in pc2.read_points(msg, skip_nans=True, field_names=fields):
                points.append(list(point))

            if points:
                clouds.append(np.array(points, dtype=np.float64))
                msg_count += 1

                if msg_count % 50 == 0:
                    total_pts = sum(len(c) for c in clouds)
                    logger.info(f"  {msg_count} messages, {total_pts:,} points...")

    total_pts = sum(len(c) for c in clouds)
    logger.info(f"Read complete: {msg_count} messages, {total_pts:,} points")
    return clouds, msg_count


def read_trajectory_from_bag(
    bag_path: str,
    topic: str = "/Odometry",
) -> List[dict]:
    """Read odometry/trajectory poses from a bag file.

    Args:
        bag_path: Path to the bag directory.
        topic: Odometry topic name.

    Returns:
        List of pose dicts with keys: timestamp, position, orientation.
    """
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

    storage_id = _detect_storage_format(bag_path)
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    poses = []

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            poses.append({
                "timestamp": timestamp,
                "position": [p.x, p.y, p.z],
                "orientation": [q.w, q.x, q.y, q.z],
            })

    logger.info(f"Read {len(poses)} poses from {topic}")
    return poses


def merge_clouds(clouds: List[np.ndarray]) -> np.ndarray:
    """Merge a list of point cloud arrays into a single array."""
    if not clouds:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(clouds)


def save_pcd_ascii(points: np.ndarray, filepath: str, fields: str = "x y z intensity"):
    """Save points to PCD file in ASCII format.

    Args:
        points: Nx3 or Nx4 array of points.
        filepath: Output file path.
        fields: Space-separated field names.
    """
    field_list = fields.split()
    n_fields = len(field_list)
    n_points = len(points)

    Path(filepath).parent.mkdir(parents=True, exist_ok=True)

    with open(filepath, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write(f"FIELDS {fields}\n")
        f.write(f"SIZE {' '.join(['4'] * n_fields)}\n")
        f.write(f"TYPE {' '.join(['F'] * n_fields)}\n")
        f.write(f"COUNT {' '.join(['1'] * n_fields)}\n")
        f.write(f"WIDTH {n_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {n_points}\n")
        f.write("DATA ascii\n")

        for p in points:
            values = " ".join(f"{v:.6f}" for v in p[:n_fields])
            f.write(f"{values}\n")

    size_mb = os.path.getsize(filepath) / (1024 * 1024)
    logger.info(f"Saved PCD: {filepath} ({n_points:,} pts, {size_mb:.2f} MB)")


def _detect_storage_format(bag_path: str) -> str:
    """Detect bag storage format (sqlite3 or mcap)."""
    bag_dir = Path(bag_path)
    if any(bag_dir.glob("*.mcap")):
        return "mcap"
    return "sqlite3"
