"""DronePi Flight Module — MAVROS-based flight control for PX4.

This module provides reusable components for autonomous and manual
flight operations with the DronePi system.

Components
----------
FlightController
    Reusable MAVROS interface for state monitoring, setpoint publishing,
    arming, and mode switching. Thread-safe with background ROS spin.

_slam_bridge
    SLAM bridge node that forwards Point-LIO poses to MAVROS for EKF fusion.
    Publishes to /mavros/vision_pose/pose (corrected from DDS topic).

drone_watchdog
    Systemd-managed supervisor that auto-starts Point-LIO, SLAM bridge,
    and bag recording when the drone arms in OFFBOARD mode.

flight_mission
    Full autonomous survey mission orchestrator with pre-flight checks,
    waypoint execution, gap detection, and post-flight processing.

Usage
-----
    from flight.flight_controller import FlightController

    fc = FlightController(node_name="my_script")
    fc.wait_for_connection()
    fc.wait_for_ekf(require_gps=True)
    fc.stream_setpoint(x, y, z, yaw=0.0, duration=3.0)
    fc.arm()
    fc.set_mode("OFFBOARD")
    fc.fly_to(target_x, target_y, target_z)
    fc.set_mode("AUTO.LAND")
    fc.shutdown()

See individual module docstrings for detailed documentation.
"""

from .flight_controller import FlightController, create_controller

__all__ = [
    "FlightController",
    "create_controller",
]

__version__ = "2.0.0"
__author__ = "DronePi Capstone Team"
