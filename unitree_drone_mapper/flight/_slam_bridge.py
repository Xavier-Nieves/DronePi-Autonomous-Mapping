#!/usr/bin/env python3
"""SLAM odometry bridge: /Odometry (ENU) -> /fmu/in/vehicle_visual_odometry (NED).

Point-LIO outputs poses in ENU (East-North-Up, ROS standard).
PX4 expects NED (North-East-Down).

Conversion:
    NED.x =  ENU.y   (North = East in ENU... wait, no)
    NED.x =  ENU.x   ... actually ENU->NED:
    N =  ENU.y
    E =  ENU.x
    D = -ENU.z

Quaternion ENU->NED: rotate by 90 deg around Z then flip Z.
"""

import math
import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy,
    QoSDurabilityPolicy, QoSHistoryPolicy,
)

from nav_msgs.msg import Odometry as ROSOdometry
from px4_msgs.msg import VehicleOdometry

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def enu_to_ned_position(x_enu, y_enu, z_enu):
    """Convert ENU position to NED."""
    return (y_enu, x_enu, -z_enu)


def enu_to_ned_quaternion(qx, qy, qz, qw):
    """Rotate quaternion from ENU to NED frame.

    Apply a fixed rotation: R_ENU_to_NED = Rz(90°) * Rx(180°)
    Simplified: swap and negate components.
    """
    # ENU->NED: multiply by quaternion for 90deg Z + 180deg X
    # Using the known formula:
    return (qy, qx, -qz, qw)   # (new_x, new_y, new_z, new_w)


class SLAMBridgeNode(Node):
    """Bridges Point-LIO odometry to PX4 visual odometry input."""

    def __init__(self):
        super().__init__("slam_to_px4_bridge")

        self.pub = self.create_publisher(
            VehicleOdometry,
            "/fmu/in/vehicle_visual_odometry",
            RELIABLE_QOS,
        )

        self.sub = self.create_subscription(
            ROSOdometry,
            "/Odometry",
            self._callback,
            SENSOR_QOS,
        )

        self.msg_count = 0
        self.get_logger().info(
            "SLAM bridge ready: /Odometry -> /fmu/in/vehicle_visual_odometry"
        )

    def _callback(self, msg: ROSOdometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear

        # Convert position ENU -> NED
        n, e, d = enu_to_ned_position(p.x, p.y, p.z)

        # Convert quaternion ENU -> NED
        qx_ned, qy_ned, qz_ned, qw_ned = enu_to_ned_quaternion(q.x, q.y, q.z, q.w)

        # Convert velocity ENU -> NED
        vn, ve, vd = enu_to_ned_position(v.x, v.y, v.z)

        out = VehicleOdometry()
        ts = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp        = ts
        out.timestamp_sample = ts

        out.pose_frame    = VehicleOdometry.POSE_FRAME_NED
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        out.position = [float(n), float(e), float(d)]
        out.q        = [float(qw_ned), float(qx_ned), float(qy_ned), float(qz_ned)]
        out.velocity = [float(vn), float(ve), float(vd)]

        # Set covariance diagonals (uncertainty)
        out.position_variance = [0.01, 0.01, 0.01]
        out.orientation_variance = [0.01, 0.01, 0.01]
        out.velocity_variance = [0.05, 0.05, 0.05]

        self.pub.publish(out)

        self.msg_count += 1
        if self.msg_count % 50 == 0:
            self.get_logger().info(
                f"Bridge: {self.msg_count} frames  "
                f"NED=({n:.2f},{e:.2f},{d:.2f})"
            )


def main():
    rclpy.init()
    node = SLAMBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
