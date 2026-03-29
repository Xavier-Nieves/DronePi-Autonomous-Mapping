#!/usr/bin/env python3
"""collision_monitor.py — Three-zone LiDAR collision monitor + downward height estimator.

Zone distances calculated from physical drone dimensions:
  Prop tip to LiDAR centre : 0.66 m
  Leg detection radius     : ~0.50 m
  Position drift margin    : ~0.20 m

  IGNORE_RADIUS   = 0.70 m  Legs (0.50) + drift margin (0.20) — nothing here is an obstacle
  OBSTACLE_RADIUS = 2.00 m  Props (0.66) + 1.34 m braking distance
  CAUTION_RADIUS  = 3.50 m  Outer early-warning ring — begin speed reduction

  CP_DIST in QGC must match OBSTACLE_RADIUS = 2.0 m.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  FUNCTION 1 — Horizontal collision detection (three-zone proximity)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Zone 1 IGNORE    Drone body and legs stripped entirely from the cloud.
  Zone 2 OBSTACLE  Encoded into 72-sector OBSTACLE_DISTANCE → MAVROS → PX4.
  Zone 3 CAUTION   Publishes "CAUTION" on /dronepi/collision_zone so
                   main.py scales down fly_to() speed early.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  FUNCTION 2 — Downward height-above-ground (AGL) estimation
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Extracts downward-pointing returns from /cloud_registered and publishes
  sensor_msgs/Range on /mavros/distance_sensor/lidar_down.
  MAVROS forwards this to PX4 as DISTANCE_SENSOR (PITCH_270 = downward).
  PX4 EKF2 fuses it in conditional rangefinder mode (EKF2_RNG_CTRL = 1)
  — active only at low speed/altitude during takeoff, landing, and hover.

MAVROS px4_config.yaml entry required:
  lidar_down_sub:
    subscriber: true
    id: 4
    orientation: PITCH_270
    field_of_view: 0.26

Dependencies
------------
  rclpy, sensor_msgs, mavros_msgs, std_msgs (ROS 2 Jazzy), numpy
"""

import math
import struct
import threading
import time
import sys as _sys
import math as _math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).parents[2]))

from pythonconfig.config_loader import load_config as _load_config


from sensor_msgs.msg import PointCloud2, Range, LaserScan
from std_msgs.msg import String

# ── Zone Radii and tuning — loaded from config/dronepi.yaml ──────────────────
_cfg = _load_config()

IGNORE_RADIUS   = _cfg["collision"]["ignore_radius"]
OBSTACLE_RADIUS = _cfg["collision"]["obstacle_radius"]
CAUTION_RADIUS  = _cfg["collision"]["caution_radius"]

# ── Downward Height Estimator ─────────────────────────────────────────────────

DOWN_CONE_DEG   = _cfg["collision"]["down_cone_deg"]                # Half-angle of downward extraction cone (degrees)
DOWN_MAX_M      = _cfg["collision"]["down_max_m"]                   # Maximum believable ground distance (metres)
DOWN_MIN_POINTS = _cfg["collision"]["down_min_points"]              # Minimum in-cone returns before publishing AGL
DOWN_FOV_RAD    = math.radians(_cfg["collision"]["down_fov_deg"])   # Beam spread reported to PX4


# ── Publish Rate and Sector Config ────────────────────────────────────────────
# MAVROS obstacle_distance plugin accepts sensor_msgs/LaserScan on
# /mavros/obstacle/send and converts it to MAVLink OBSTACLE_DISTANCE for PX4.
# LaserScan convention: angle_min = -pi, angle_max = pi, 72 beams = 5 deg each.

PUBLISH_HZ  = _cfg["collision"]["publish_hz"]
NUM_SECTORS = 72
SECTOR_RAD  = (2.0 * _math.pi) / NUM_SECTORS   # ~0.0873 rad per sector

# ── QoS Profiles ─────────────────────────────────────────────────────────────

CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class CollisionMonitor(Node):
    """Three-zone proximity monitor and downward AGL estimator.

    Publishes three topics from one /cloud_registered subscription:
      /mavros/obstacle/send              LaserScan — horizontal collision map
                                         (MAVROS converts to OBSTACLE_DISTANCE for PX4)
      /dronepi/collision_zone            String    — CLEAR | CAUTION | OBSTACLE
      /mavros/distance_sensor/lidar_down Range     — AGL height for PX4 EKF2
    """

    def __init__(self):
        super().__init__("collision_monitor")

        # Shared state protected by lock
        # Sector distances in metres, float('inf') = no obstacle detected
        self._sectors    = [float('inf')] * NUM_SECTORS
        self._zone       = "CLEAR"
        self._agl_m      = None
        self._lock       = threading.Lock()
        self._last_cloud = 0.0
        self._last_log   = 0.0

        # Subscriptions
        self._cloud_sub = self.create_subscription(
            PointCloud2, "/cloud_registered",
            self._cloud_callback, CLOUD_QOS,
        )

        # Publishers
        # LaserScan: MAVROS obstacle_distance plugin converts this to MAVLink
        # OBSTACLE_DISTANCE message consumed by PX4 collision prevention
        self._obs_pub   = self.create_publisher(
            LaserScan, "/mavros/obstacle/send", RELIABLE_QOS)
        self._zone_pub  = self.create_publisher(
            String, "/dronepi/collision_zone", RELIABLE_QOS)
        self._range_pub = self.create_publisher(
            Range, "/mavros/distance_sensor/lidar_down_sub", RELIABLE_QOS)

        # Fixed-rate publish timer
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_callback)

        self.get_logger().info(
            f"Collision monitor started  "
            f"ignore={IGNORE_RADIUS}m  obstacle={OBSTACLE_RADIUS}m  "
            f"caution={CAUTION_RADIUS}m  down_cone={DOWN_CONE_DEG}deg"
        )

    # ── Cloud Callback ────────────────────────────────────────────────────────

    def _cloud_callback(self, msg: PointCloud2) -> None:
        """Process point cloud — updates collision sectors and AGL estimate."""
        try:
            points = self._extract_xyz(msg)
            if points is None or len(points) == 0:
                return

            # ── Horizontal collision sectors ──────────────────────────────────
            xy    = points[:, :2]
            dists = np.sqrt(xy[:, 0]**2 + xy[:, 1]**2)

            # Strip Zone 1 (drone self-detections)
            mask  = dists >= IGNORE_RADIUS
            xy    = xy[mask]
            dists = dists[mask]

            # Initialise all sectors as no-obstacle (inf)
            new_sectors = [float('inf')] * NUM_SECTORS
            if len(xy) > 0:
                # arctan2 returns [-pi, pi] — map to [0, 2pi] then to sector index
                angles_rad = np.arctan2(xy[:, 1], xy[:, 0])
                angles_rad = angles_rad % (2.0 * math.pi)
                for angle, dist in zip(angles_rad, dists):
                    if dist > CAUTION_RADIUS:
                        continue
                    idx = int(angle / SECTOR_RAD) % NUM_SECTORS
                    if dist < new_sectors[idx]:
                        new_sectors[idx] = round(dist, 3)

            finite = [d for d in new_sectors if d < float('inf')]
            if finite:
                closest = min(finite)
                zone = (
                    "OBSTACLE" if closest < OBSTACLE_RADIUS else
                    "CAUTION"  if closest < CAUTION_RADIUS  else
                    "CLEAR"
                )
            else:
                zone = "CLEAR"

            # ── Downward AGL estimate ─────────────────────────────────────────
            agl = self._estimate_agl(points)

            with self._lock:
                self._sectors    = new_sectors
                self._zone       = zone
                self._agl_m      = agl
                self._last_cloud = time.time()

        except Exception as exc:
            self.get_logger().warn(f"Cloud processing error: {exc}")

    # ── AGL Estimator ─────────────────────────────────────────────────────────

    def _estimate_agl(self, points: np.ndarray):
        """Estimate altitude above ground from downward-pointing cloud returns.

        Selects points below sensor origin whose elevation angle exceeds
        DOWN_CONE_DEG, then returns the median Z-magnitude of those returns.

        Returns:
            AGL in metres, or None if insufficient data or out of range.
        """
        try:
            # Ground returns have negative Z in ENU frame (below sensor)
            below = points[points[:, 2] < 0]
            if len(below) == 0:
                return None

            slant = np.sqrt(below[:, 0]**2 + below[:, 1]**2 + below[:, 2]**2)
            with np.errstate(divide="ignore", invalid="ignore"):
                elev = np.arcsin(
                    np.abs(below[:, 2]) / np.where(slant > 0, slant, 1.0)
                )

            cone_rad = math.radians(DOWN_CONE_DEG)
            z_vals   = np.abs(below[:, 2][elev >= cone_rad])

            if len(z_vals) < DOWN_MIN_POINTS:
                return None

            agl = float(np.median(z_vals))
            return round(agl, 3) if 0.05 <= agl <= DOWN_MAX_M else None

        except Exception:
            return None

    # ── Point Extraction ──────────────────────────────────────────────────────

    def _extract_xyz(self, msg: PointCloud2):
        """Extract (x, y, z) from a PointCloud2 message as (N, 3) float32."""
        try:
            fields = {f.name: f.offset for f in msg.fields}
            if not all(k in fields for k in ("x", "y", "z")):
                return None

            x_off, y_off, z_off = fields["x"], fields["y"], fields["z"]
            ps   = msg.point_step
            data = bytes(msg.data)
            n    = msg.width * msg.height

            if x_off == 0 and y_off == 4 and z_off == 8:
                stride = ps // 4
                raw    = np.frombuffer(data, dtype=np.float32).reshape(-1, stride)
                xyz    = raw[:, :3].copy()
            else:
                xyz = np.empty((n, 3), dtype=np.float32)
                for i in range(n):
                    base     = i * ps
                    xyz[i,0] = struct.unpack_from("<f", data, base + x_off)[0]
                    xyz[i,1] = struct.unpack_from("<f", data, base + y_off)[0]
                    xyz[i,2] = struct.unpack_from("<f", data, base + z_off)[0]

            valid = np.isfinite(xyz).all(axis=1)
            return xyz[valid]

        except Exception:
            return None

    # ── Publish Callback ──────────────────────────────────────────────────────

    def _publish_callback(self) -> None:
        """Publish obstacle map, zone status, and AGL at fixed rate."""
        with self._lock:
            sectors = list(self._sectors)
            zone    = self._zone
            agl     = self._agl_m
            stale   = (time.time() - self._last_cloud) > 1.0

        # Zone status for main.py velocity scaling
        zone_msg      = String()
        zone_msg.data = "CLEAR" if stale else zone
        self._zone_pub.publish(zone_msg)

        # Obstacle map as LaserScan — MAVROS obstacle_distance plugin converts
        # this to the MAVLink OBSTACLE_DISTANCE message for PX4 collision prevention.
        # angle_min=-pi, angle_max=pi covers full 360 degrees in 72 beams.
        scan                = LaserScan()
        scan.header.stamp   = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min      = -math.pi
        scan.angle_max      =  math.pi
        scan.angle_increment = SECTOR_RAD
        scan.time_increment  = 0.0
        scan.scan_time       = 1.0 / PUBLISH_HZ
        scan.range_min       = IGNORE_RADIUS
        scan.range_max       = CAUTION_RADIUS
        # LaserScan uses inf for no obstacle detected; stale = report all inf
        scan.ranges          = (
            [float('inf')] * NUM_SECTORS if stale else
            [float(d) for d in sectors]   # already inf where no obstacle
        )
        self._obs_pub.publish(scan)

        # Downward AGL rangefinder for PX4 EKF2 conditional height fusion
        if not stale and agl is not None:
            rng                 = Range()
            rng.header.stamp    = self.get_clock().now().to_msg()
            rng.header.frame_id = "lidar_down"
            rng.radiation_type  = Range.INFRARED
            rng.field_of_view   = DOWN_FOV_RAD
            rng.min_range       = 0.10
            rng.max_range       = DOWN_MAX_M
            rng.range           = agl
            self._range_pub.publish(rng)

        # Periodic diagnostics every 5 seconds
        now = time.time()
        if now - self._last_log >= 5.0:
            finite  = [d for d in sectors if d < float('inf')]
            closest = f"{min(finite):.2f}m" if finite else "none"
            agl_str = f"{agl:.2f}m" if agl is not None else "no data"
            self.get_logger().info(
                f"Zone={zone_msg.data}  closest={closest}  "
                f"sectors={len(finite)}/{NUM_SECTORS}  AGL={agl_str}"
                + ("  [STALE — no cloud data]" if stale else "")
            )
            self._last_log = now


def main():
    rclpy.init()
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Collision monitor stopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
