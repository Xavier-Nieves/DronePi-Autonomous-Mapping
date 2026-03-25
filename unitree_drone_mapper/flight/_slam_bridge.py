#!/usr/bin/env python3
"""_slam_bridge.py — Point-LIO odometry → MAVROS vision pose bridge.

Converts Point-LIO's /aft_mapped_to_init odometry output to the format
expected by MAVROS on /mavros/vision_pose/pose for EKF fusion.

Frame Convention
----------------
Point-LIO outputs poses in ENU (East-North-Up), which is the ROS standard.
MAVROS expects PoseStamped in ENU on /mavros/vision_pose/pose and handles
the ENU→NED conversion internally before forwarding to PX4 via MAVLink.

Do NOT manually convert ENU→NED here. MAVROS is the correct interface for
PX4 when using ROS 2 with MAVLink.

PX4 Configuration Required
--------------------------
Enable vision position fusion in QGroundControl:

    EKF2_EV_CTRL = 15      (enable all vision inputs: pos, vel, yaw, height)
    EKF2_HGT_REF = 3       (use vision as height reference, optional)
    EKF2_EV_DELAY = 20     (adjust based on observed lag, typically 10-50ms)

Launched as a subprocess by main.py (MODE 3) and drone_watchdog.py (MODE 2).
Not intended to be run standalone in normal operation.

Dependencies
------------
    rclpy, nav_msgs, geometry_msgs (ROS 2 Jazzy)
    Point-LIO must be running and publishing /aft_mapped_to_init
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# ── QoS Profiles ──────────────────────────────────────────────────────────────

# Match Point-LIO's publisher QoS (BEST_EFFORT for sensor data)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# MAVROS vision_pose subscriber expects RELIABLE QoS
VISION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class SLAMBridgeNode(Node):
    """Bridges Point-LIO odometry to MAVROS vision pose input.

    Subscribes to /aft_mapped_to_init (nav_msgs/Odometry) from Point-LIO
    and republishes as /mavros/vision_pose/pose (geometry_msgs/PoseStamped)
    for PX4 EKF fusion via MAVROS.
    """

    def __init__(self):
        super().__init__("slam_to_mavros_bridge")

        # Publisher: MAVROS vision pose input
        self.pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            VISION_QOS,
        )

        # Subscriber: Point-LIO accumulated SLAM pose
        self.sub = self.create_subscription(
            Odometry,
            "/aft_mapped_to_init",
            self._callback,
            SENSOR_QOS,
        )

        self.msg_count     = 0
        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            "SLAM bridge started: /aft_mapped_to_init → /mavros/vision_pose/pose"
        )
        self.get_logger().info(
            "Ensure EKF2_EV_CTRL=15 in PX4 for vision fusion"
        )

    def _callback(self, msg: Odometry) -> None:
        """Convert Odometry to PoseStamped and publish to MAVROS."""
        out                  = PoseStamped()
        out.header.stamp     = msg.header.stamp  # preserve original timestamp
        out.header.frame_id  = "map"             # MAVROS expects map or odom
        out.pose             = msg.pose.pose     # ENU pass-through — no conversion

        self.pub.publish(out)

        # Periodic diagnostics every 2 seconds
        self.msg_count += 1
        now     = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9

        if elapsed >= 2.0:
            p    = msg.pose.pose.position
            rate = self.msg_count / elapsed
            self.get_logger().info(
                f"Bridge: {self.msg_count} frames ({rate:.1f} Hz)  "
                f"ENU=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            )
            self.msg_count     = 0
            self.last_log_time = now


def main():
    rclpy.init()
    node = SLAMBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SLAM bridge stopped")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
