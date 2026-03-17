#!/usr/bin/env python3
"""Unified launch: MicroXRCEAgent + LiDAR + Point-LIO SLAM + PX4 DDS bridge.

This ties together the full stack:
  - MicroXRCEAgent  : bridges Pixhawk serial <-> ROS 2 DDS
  - unitree_lidar   : LiDAR driver -> /unilidar/cloud + /unilidar/imu
  - point_lio       : SLAM -> /cloud_registered + /Odometry
  - slam_to_px4     : forwards /Odometry -> /fmu/in/vehicle_visual_odometry
  - (optional) RViz : live visualisation

Usage:
    source /opt/ros/jazzy/setup.bash
    source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
    ros2 launch ~/unitree_lidar_project/unitree_drone_mapper/launch/dds_slam.launch.py

    ros2 launch ... rviz:=false
    ros2 launch ... px4_port:=/dev/ttyACM0 px4_baud:=921600
    ros2 launch ... lidar_port:=/dev/ttyUSB0
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    px4_port_arg  = DeclareLaunchArgument("px4_port",  default_value="/dev/ttyACM0")
    px4_baud_arg  = DeclareLaunchArgument("px4_baud",  default_value="921600")
    lidar_port_arg = DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0")
    rviz_arg      = DeclareLaunchArgument("rviz",      default_value="true")
    slam_to_px4_arg = DeclareLaunchArgument(
        "slam_to_px4", default_value="true",
        description="Forward SLAM odometry to PX4 visual odometry input"
    )

    # ── 1. MicroXRCEAgent (serial bridge Pixhawk <-> DDS) ─────────────────────
    # Uses snap 'micro-xrce-dds-agent' (preferred) — install via:
    #   sudo snap install micro-xrce-dds-agent
    # For UDP/ethernet mode pass: px4_port:=udp  (then baud is ignored)
    xrce_agent = ExecuteProcess(
        cmd=[
            "micro-xrce-dds-agent", "serial",
            "--dev", LaunchConfiguration("px4_port"),
            "-b",   LaunchConfiguration("px4_baud"),
        ],
        output="screen",
        name="xrce_agent",
    )

    # ── 2. Unitree LiDAR driver ───────────────────────────────────────────────
    lidar_node = Node(
        package="unitree_lidar_ros2",
        executable="unitree_lidar_ros2_node",
        name="unitree_lidar_ros2_node",
        output="screen",
        parameters=[{
            "port":          LaunchConfiguration("lidar_port"),
            "rotate_yaw_bias": 0.0,
            "range_scale":   0.001,
            "range_bias":    0.0,
            "range_max":     50.0,
            "range_min":     0.0,
            "cloud_frame":   "unilidar_lidar",
            "cloud_topic":   "unilidar/cloud",
            "cloud_scan_num": 18,
            "imu_frame":     "unilidar_imu",
            "imu_topic":     "unilidar/imu",
        }],
    )

    # ── 3. Point-LIO SLAM ─────────────────────────────────────────────────────
    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("point_lio"),
                "config", "unilidar_l1.yaml",
            ]),
            {
                "use_imu_as_input":      False,
                "prop_at_freq_of_imu":   True,
                "check_satu":            True,
                "init_map_size":         10,
                "point_filter_num":      1,
                "space_down_sample":     True,
                "filter_size_surf":      0.1,
                "filter_size_map":       0.1,
                "cube_side_length":      1000.0,
                "runtime_pos_log_enable": False,
                "pcd_save.pcd_save_en":  False,  # managed by pipeline instead
            },
        ],
    )

    # ── 4. SLAM -> PX4 visual odometry bridge ─────────────────────────────────
    # Inline Python node: subscribes /Odometry (Point-LIO, ENU)
    #                     publishes  /fmu/in/vehicle_visual_odometry (NED)
    slam_bridge_node = Node(
        package="rclpy",        # uses generic python node runner
        executable="python3",   # replaced below with actual script
        name="slam_to_px4_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam_to_px4")),
    )

    # Better: use a standalone python executable via ExecuteProcess
    # (avoids needing to install as ROS package)
    BRIDGE_SCRIPT = os.path.join(
        os.path.dirname(__file__), "..", "flight", "_slam_bridge.py"
    )
    slam_bridge = ExecuteProcess(
        cmd=["python3", BRIDGE_SCRIPT],
        output="screen",
        name="slam_bridge",
        condition=IfCondition(LaunchConfiguration("slam_to_px4")),
    )

    # ── 5. RViz ───────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("point_lio"),
            "rviz_cfg", "loam_livox.rviz",
        ])],
        condition=IfCondition(LaunchConfiguration("rviz")),
        prefix="nice",
    )

    return LaunchDescription([
        # Args
        px4_port_arg, px4_baud_arg, lidar_port_arg, rviz_arg, slam_to_px4_arg,

        # Nodes — agent first, then sensors, then SLAM
        xrce_agent,
        lidar_node,
        point_lio_node,
        slam_bridge,
        rviz_node,
    ])
