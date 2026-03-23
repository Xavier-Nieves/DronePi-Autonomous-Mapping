#!/usr/bin/env python3
"""SLAM bridge: Point-LIO odometry → MAVROS vision pose input.

Converts Point-LIO's /aft_mapped_to_init odometry output to the format
expected by MAVROS on /mavros/vision_pose/pose for EKF fusion.

Frame Convention
----------------
Point-LIO outputs poses in ENU (East-North-Up), which is the ROS standard.
MAVROS expects PoseStamped in ENU on /mavros/vision_pose/pose and handles
the ENU→NED conversion internally before forwarding to PX4 via MAVLink.

**Do NOT manually convert ENU→NED here.** That was the bug in the previous
version which published to the DDS topic /fmu/in/vehicle_visual_odometry.
MAVROS is the correct interface for PX4 when using ROS 2 with MAVLink.

PX4 Configuration Required
--------------------------
Enable vision position fusion in QGroundControl:

    EKF2_EV_CTRL = 15      (enable all vision inputs: pos, vel, yaw, height)
    EKF2_HGT_REF = 3       (use vision as height reference, optional)
    EKF2_EV_DELAY = 20     (adjust based on observed lag, typically 10-50ms)

To verify fusion is active, check the EKF2 innovations panel in QGC.
Vision position residuals should be visible and small (<0.5m).

Usage
-----
  # Standalone (for testing)
  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
  python3 _slam_bridge.py

  # As part of flight stack (launched by drone_watchdog or flight_mission)
  # See flight_controller.py for integration.

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

# ── QoS profiles ──────────────────────────────────────────────────────────────

# Match Point-LIO's publisher QoS (BEST_EFFORT for sensor data)
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# MAVROS vision_pose subscriber uses default QoS (RELIABLE)
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
        # MAVROS forwards this to PX4 as VISION_POSITION_ESTIMATE MAVLink msg
        self.pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            VISION_QOS,
        )

        # Subscriber: Point-LIO odometry output
        # /aft_mapped_to_init is the accumulated SLAM pose in the map frame
        self.sub = self.create_subscription(
            Odometry,
            "/aft_mapped_to_init",
            self._callback,
            SENSOR_QOS,
        )

        # Diagnostics
        self.msg_count = 0
        self.last_log_time = self.get_clock().now()

        self.get_logger().info(
            "SLAM bridge started: /aft_mapped_to_init → /mavros/vision_pose/pose"
        )
        self.get_logger().info(
            "Ensure EKF2_EV_CTRL=15 in PX4 for vision fusion"
        )

    def _callback(self, msg: Odometry):
        """Convert Odometry to PoseStamped and publish to MAVROS."""
        
        out = PoseStamped()
        
        # Use the original timestamp for proper sensor fusion timing
        out.header.stamp = msg.header.stamp
        
        # Frame ID: MAVROS expects 'map' or 'odom' frame
        # Point-LIO uses 'camera_init' but the pose is in the map frame
        out.header.frame_id = "map"
        
        # Pass through the pose directly — ENU frame, no conversion needed
        # MAVROS handles ENU→NED conversion internally before MAVLink TX
        out.pose = msg.pose.pose
        
        self.pub.publish(out)
        
        # Periodic logging (every 2 seconds worth of messages at ~50Hz)
        self.msg_count += 1
        now = self.get_clock().now()
        elapsed = (now - self.last_log_time).nanoseconds / 1e9
        
        if elapsed >= 2.0:
            p = msg.pose.pose.position
            rate = self.msg_count / elapsed
            self.get_logger().info(
                f"Bridge: {self.msg_count} frames ({rate:.1f}Hz)  "
                f"ENU=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            )
            self.msg_count = 0
            self.last_log_time = now


def main():
    rclpy.init()
    node = SLAMBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SLAM bridge stopped by user")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
