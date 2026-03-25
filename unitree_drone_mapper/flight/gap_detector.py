#!/usr/bin/env python3
"""gap_detector.py — LiDAR point-cloud coverage gap detector.

Monitors /cloud_registered and maintains a 2D occupancy grid of point
counts in the XY plane. Any grid cell whose count falls below the
configured minimum density is flagged as a gap and returned as a
candidate gap-fill waypoint.

Called by handle_autonomous() in main.py during MODE 3 mission execution.

Usage
-----
    from flight.gap_detector import GapDetector

    gap_det = GapDetector(cell_size_m=footprint_r * 2.0, min_density=5)
    gap_det.start(ros2_node)                      # begin accumulating

    gaps = gap_det.find_gaps_near(cx, cy, radius_m=footprint_r)
    for gx, gy in gaps:
        node.fly_to(gx, gy, ez, 0.0, timeout=60.0)

Dependencies
------------
    rclpy, sensor_msgs (ROS 2 Jazzy), numpy
"""

import math
import threading
from typing import List, Tuple

import numpy as np


class GapDetector:
    """Monitors /cloud_registered and detects under-covered survey areas.

    Maintains a flat 2D grid of point counts in the XY plane. Grid cells
    are sized to the camera footprint radius at mission altitude. A cell
    is flagged as a gap if its accumulated point count is below min_density.

    Attributes:
        cell_size_m:  Width of each grid cell in metres.
        min_density:  Minimum point count required for adequate coverage.
    """

    def __init__(self, cell_size_m: float, min_density: int):
        """Initialise the gap detector.

        Args:
            cell_size_m:  Grid cell size in metres (typically footprint_r * 2).
            min_density:  Minimum points per cell before a gap is declared.
        """
        self._cell = cell_size_m
        self._min  = min_density
        self._grid: dict = {}        # (ix, iy) -> int count
        self._lock = threading.Lock()
        self._sub  = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self, node) -> None:
        """Subscribe to /cloud_registered and begin accumulating point counts.

        Args:
            node: An rclpy Node instance (e.g. MainNode._node).
        """
        try:
            from sensor_msgs.msg import PointCloud2
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self._sub = node.create_subscription(
                PointCloud2, "/cloud_registered", self._cloud_cb, qos
            )
            print("  [GapDetector] Subscribed to /cloud_registered")
        except Exception as exc:
            print(f"  [GapDetector][WARN] Could not subscribe: {exc}")

    # ── ROS Callback ──────────────────────────────────────────────────────────

    def _cloud_cb(self, msg) -> None:
        """Accumulate XY positions from each incoming PointCloud2 message."""
        try:
            fields = {f.name: f.offset for f in msg.fields}
            if "x" not in fields or "y" not in fields:
                return

            x_off = fields["x"]
            y_off = fields["y"]
            ps    = msg.point_step
            data  = bytes(msg.data)
            n     = msg.width * msg.height

            # Fast path — x and y are tightly packed as the first two floats
            if x_off == 0 and y_off == 4:
                stride = ps // 4
                raw = np.frombuffer(data, dtype=np.float32).reshape(-1, stride)
                xs  = raw[:, 0]
                ys  = raw[:, 1]
            else:
                import struct
                xs = np.empty(n, dtype=np.float32)
                ys = np.empty(n, dtype=np.float32)
                for i in range(n):
                    base    = i * ps
                    xs[i]   = struct.unpack_from("<f", data, base + x_off)[0]
                    ys[i]   = struct.unpack_from("<f", data, base + y_off)[0]

            valid = np.isfinite(xs) & np.isfinite(ys)
            xs = xs[valid]
            ys = ys[valid]

            with self._lock:
                for x, y in zip(xs, ys):
                    key = (int(x / self._cell), int(y / self._cell))
                    self._grid[key] = self._grid.get(key, 0) + 1

        except Exception:
            pass  # Never let a bad cloud message crash the mission

    # ── Query Interface ───────────────────────────────────────────────────────

    def check_coverage(self, ex: float, ey: float) -> bool:
        """Return True if the grid cell at (ex, ey) has adequate coverage.

        Args:
            ex: East position in metres (ENU frame).
            ey: North position in metres (ENU frame).

        Returns:
            True if coverage meets the minimum density threshold.
        """
        key = (int(ex / self._cell), int(ey / self._cell))
        with self._lock:
            return self._grid.get(key, 0) >= self._min

    def find_gaps_near(
        self, ex: float, ey: float, radius_m: float
    ) -> List[Tuple[float, float]]:
        """Return gap cell centres within radius_m of position (ex, ey).

        Scans a circular region around the given position and returns the
        centre coordinates of every cell whose point count is below the
        minimum density threshold. These coordinates are used directly as
        gap-fill waypoint targets.

        Args:
            ex:       East position in metres.
            ey:       North position in metres.
            radius_m: Search radius in metres.

        Returns:
            List of (east, north) tuples for each gap cell centre found.
        """
        r_cells = int(math.ceil(radius_m / self._cell))
        cx      = int(ex / self._cell)
        cy      = int(ey / self._cell)
        gaps: List[Tuple[float, float]] = []

        with self._lock:
            for dx in range(-r_cells, r_cells + 1):
                for dy in range(-r_cells, r_cells + 1):
                    if dx * dx + dy * dy > r_cells * r_cells:
                        continue
                    key = (cx + dx, cy + dy)
                    if self._grid.get(key, 0) < self._min:
                        gaps.append((
                            (cx + dx + 0.5) * self._cell,
                            (cy + dy + 0.5) * self._cell,
                        ))
        return gaps
