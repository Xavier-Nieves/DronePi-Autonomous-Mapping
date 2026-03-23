#!/usr/bin/env python3
"""flight_controller.py — Reusable flight control primitives for MAVROS.

This module provides a clean, reusable interface for MAVROS-based flight
control. All flight scripts (mission_executor, flight_mission, test scripts)
should import from here instead of reimplementing the same logic.

Design Principles
-----------------
1. Thread-safe: All state access is protected by locks
2. Non-blocking: ROS spin runs in background thread
3. Defensive: All methods validate preconditions
4. Observable: State changes are logged

Usage
-----
  from flight_controller import FlightController

  # Initialize (starts ROS node and subscriptions)
  fc = FlightController(node_name="my_flight_script")

  # Wait for system ready
  fc.wait_for_connection(timeout=15.0)
  fc.wait_for_ekf(timeout=30.0, require_gps=True)

  # Pre-stream setpoints (required before OFFBOARD)
  fc.stream_setpoint(x, y, z, yaw=0.0, duration=3.0)

  # Arm and switch mode
  fc.set_mode("OFFBOARD")
  fc.arm()

  # Fly to position
  fc.fly_to(x, y, z, yaw=0.0, timeout=30.0)

  # Land
  fc.set_mode("AUTO.LAND")
  fc.wait_for_disarm(timeout=30.0)

  # Cleanup
  fc.shutdown()

Thread Safety
-------------
The FlightController runs rclpy.spin in a background daemon thread so the
main thread can perform blocking waits and flight logic without missing
callbacks. All state variables (_state, _pose, _home, etc.) are protected
by threading.Lock.

Service calls (arm, set_mode) use spin_until_future_complete which is
safe to call from the main thread while the spin thread is running.

Dependencies
------------
  rclpy, geometry_msgs, mavros_msgs, sensor_msgs (ROS 2 Jazzy)
  MAVROS must be running and connected to PX4
"""

import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix


# ── QoS profiles ──────────────────────────────────────────────────────────────

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ── FlightController ──────────────────────────────────────────────────────────

class FlightController:
    """Reusable MAVROS flight control interface.
    
    Provides high-level methods for arming, mode switching, setpoint
    publishing, and position tracking. All methods are thread-safe.
    
    Attributes:
        node_name: Name of the ROS node (for logging)
        setpoint_hz: Rate for setpoint publishing (default: 20 Hz)
    """

    def __init__(self, node_name: str = "flight_controller", setpoint_hz: int = 20):
        """Initialize the flight controller.
        
        Args:
            node_name: ROS node name
            setpoint_hz: Setpoint publish rate in Hz (PX4 requires > 2 Hz)
        """
        self._lock = threading.Lock()
        self._stop = threading.Event()
        
        self.node_name = node_name
        self.setpoint_hz = setpoint_hz
        self._setpoint_period = 1.0 / setpoint_hz
        
        # State variables (protected by _lock)
        self._state = State()
        self._pose: Optional[PoseStamped] = None
        self._home: Optional[HomePosition] = None
        self._gps: Optional[NavSatFix] = None
        self._home_z: float = 0.0  # Captured Z at initialization
        
        # Initialize ROS
        rclpy.init()
        self._node = Node(node_name)
        
        # Subscriptions
        self._node.create_subscription(
            State, "/mavros/state", self._state_cb, RELIABLE_QOS)
        self._node.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, SENSOR_QOS)
        self._node.create_subscription(
            HomePosition, "/mavros/home_position/home",
            self._home_cb, SENSOR_QOS)
        self._node.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            self._gps_cb, SENSOR_QOS)
        
        # Publisher
        self._sp_pub = self._node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", RELIABLE_QOS)
        
        # Service clients
        self._arm_client = self._node.create_client(
            CommandBool, "/mavros/cmd/arming")
        self._mode_client = self._node.create_client(
            SetMode, "/mavros/set_mode")
        
        # Start spin thread
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()
        
        self._log(f"FlightController initialized (node: {node_name})")

    # ── Logging ───────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        """Log via ROS logger."""
        self._node.get_logger().info(msg)

    def _warn(self, msg: str):
        """Warn via ROS logger."""
        self._node.get_logger().warn(msg)

    def _error(self, msg: str):
        """Error via ROS logger."""
        self._node.get_logger().error(msg)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _state_cb(self, msg: State):
        with self._lock:
            self._state = msg

    def _pose_cb(self, msg: PoseStamped):
        with self._lock:
            self._pose = msg

    def _home_cb(self, msg: HomePosition):
        with self._lock:
            self._home = msg

    def _gps_cb(self, msg: NavSatFix):
        with self._lock:
            self._gps = msg

    # ── State Getters ─────────────────────────────────────────────────────────

    def get_state(self) -> State:
        """Get current MAVROS state (connected, armed, mode)."""
        with self._lock:
            return self._state

    def get_pose(self) -> Optional[PoseStamped]:
        """Get current local position pose."""
        with self._lock:
            return self._pose

    def get_position(self) -> Tuple[float, float, float]:
        """Get current position as (x, y, z) tuple."""
        with self._lock:
            if self._pose is None:
                return (0.0, 0.0, 0.0)
            p = self._pose.pose.position
            return (p.x, p.y, p.z)

    def get_altitude(self) -> float:
        """Get current altitude (Z position)."""
        _, _, z = self.get_position()
        return z

    def get_altitude_above_home(self) -> float:
        """Get altitude relative to captured home Z."""
        return self.get_altitude() - self._home_z

    def get_yaw(self) -> float:
        """Get current yaw in radians from quaternion."""
        with self._lock:
            if self._pose is None:
                return 0.0
            q = self._pose.pose.orientation
            # Yaw from quaternion (assuming NED or ENU with Z-up)
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

    def get_home(self) -> Optional[HomePosition]:
        """Get home position (GPS-based)."""
        with self._lock:
            return self._home

    def get_gps(self) -> Optional[NavSatFix]:
        """Get current GPS fix."""
        with self._lock:
            return self._gps

    def is_connected(self) -> bool:
        """Check if FCU is connected."""
        return self.get_state().connected

    def is_armed(self) -> bool:
        """Check if drone is armed."""
        return self.get_state().armed

    def get_mode(self) -> str:
        """Get current flight mode."""
        return self.get_state().mode

    # ── Initialization Waits ──────────────────────────────────────────────────

    def wait_for_connection(self, timeout: float = 15.0) -> bool:
        """Block until MAVROS reports FCU connected.
        
        Args:
            timeout: Maximum seconds to wait
            
        Returns:
            True if connected, False if timeout
        """
        self._log(f"Waiting for FCU connection (timeout: {timeout:.0f}s)...")
        deadline = time.time() + timeout
        
        while time.time() < deadline:
            if self.is_connected():
                self._log(f"FCU connected (mode: {self.get_mode()})")
                return True
            time.sleep(0.1)
        
        self._error("FCU connection timeout")
        return False

    def wait_for_ekf(self, timeout: float = 30.0, require_gps: bool = False,
                     z_tolerance: float = 0.03, stable_count: int = 20) -> bool:
        """Block until EKF local position is stable.
        
        Waits for the local position estimate to stabilize (Z drift below
        tolerance for stable_count consecutive checks).
        
        Args:
            timeout: Maximum seconds to wait
            require_gps: If True, also wait for GPS home position
            z_tolerance: Maximum Z drift per check (metres)
            stable_count: Required consecutive stable checks
            
        Returns:
            True if EKF stable, False if timeout
        """
        self._log(f"Waiting for EKF stability (timeout: {timeout:.0f}s)...")
        if require_gps:
            self._log("  GPS home position required")
        
        deadline = time.time() + timeout
        prev_z = None
        stable = 0
        
        while time.time() < deadline:
            z = self.get_altitude()
            home_ok = (self.get_home() is not None) if require_gps else True
            
            if prev_z is not None:
                drift = abs(z - prev_z)
                if drift < z_tolerance:
                    stable += 1
                else:
                    stable = 0
            prev_z = z
            
            # Progress display
            home_str = f"home={'OK' if self.get_home() else 'waiting'}  " if require_gps else ""
            print(f"\r  z={z:.3f}m  {home_str}stable={stable}/{stable_count}    ",
                  end="", flush=True)
            
            if stable >= stable_count and home_ok:
                print()  # Newline after progress
                # Capture home Z for relative altitude calculations
                self._home_z = z
                self._log(f"EKF stable. Home Z captured: {self._home_z:.3f}m")
                return True
            
            time.sleep(0.1)
        
        print()  # Newline after progress
        self._error("EKF stability timeout")
        return False

    def wait_for_gps(self, timeout: float = 30.0, min_satellites: int = 6) -> bool:
        """Block until GPS fix is acquired.
        
        Args:
            timeout: Maximum seconds to wait
            min_satellites: Minimum satellites for valid fix (if available)
            
        Returns:
            True if GPS acquired, False if timeout
        """
        self._log(f"Waiting for GPS fix (timeout: {timeout:.0f}s)...")
        deadline = time.time() + timeout
        
        while time.time() < deadline:
            gps = self.get_gps()
            if gps is not None and gps.status.status >= 0:
                self._log(f"GPS fix: lat={gps.latitude:.6f} lon={gps.longitude:.6f}")
                return True
            time.sleep(0.5)
        
        self._error("GPS fix timeout")
        return False

    # ── Setpoint Publishing ───────────────────────────────────────────────────

    def publish_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Publish a single position setpoint.
        
        Args:
            x, y, z: Position in local ENU frame (metres)
            yaw: Heading in radians
        """
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def stream_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0,
                        duration: float = 1.0):
        """Stream setpoints at setpoint_hz for duration seconds.
        
        Use this to pre-stream setpoints before switching to OFFBOARD mode
        (PX4 requires an active setpoint stream before accepting OFFBOARD).
        
        Args:
            x, y, z: Position in local ENU frame (metres)
            yaw: Heading in radians
            duration: Seconds to stream
        """
        end_time = time.time() + duration
        while time.time() < end_time and not self._stop.is_set():
            self.publish_setpoint(x, y, z, yaw)
            time.sleep(self._setpoint_period)

    # ── Arming and Mode ───────────────────────────────────────────────────────

    def arm(self, timeout: float = 5.0) -> bool:
        """Send arm command.
        
        Args:
            timeout: Service call timeout
            
        Returns:
            True if command accepted, False otherwise
        """
        if not self._arm_client.wait_for_service(timeout_sec=timeout):
            self._error("Arm service not available")
            return False
        
        req = CommandBool.Request()
        req.value = True
        
        future = self._arm_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        
        if future.result() is not None and future.result().success:
            self._log("Arm command accepted")
            return True
        else:
            self._error("Arm command rejected")
            return False

    def disarm(self, timeout: float = 5.0) -> bool:
        """Send disarm command.
        
        Args:
            timeout: Service call timeout
            
        Returns:
            True if command accepted, False otherwise
        """
        if not self._arm_client.wait_for_service(timeout_sec=timeout):
            self._error("Arm service not available")
            return False
        
        req = CommandBool.Request()
        req.value = False
        
        future = self._arm_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        
        if future.result() is not None and future.result().success:
            self._log("Disarm command accepted")
            return True
        else:
            self._warn("Disarm command rejected (may already be disarmed)")
            return False

    def set_mode(self, mode: str, timeout: float = 5.0) -> bool:
        """Set flight mode.
        
        Args:
            mode: PX4 mode string (e.g., "OFFBOARD", "AUTO.LAND", "AUTO.RTL")
            timeout: Service call timeout
            
        Returns:
            True if mode change accepted, False otherwise
        """
        if not self._mode_client.wait_for_service(timeout_sec=timeout):
            self._error("SetMode service not available")
            return False
        
        req = SetMode.Request()
        req.custom_mode = mode
        
        future = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        
        if future.result() is not None and future.result().mode_sent:
            self._log(f"Mode change to {mode} accepted")
            return True
        else:
            self._error(f"Mode change to {mode} rejected")
            return False

    def wait_for_arm(self, timeout: float = 10.0) -> bool:
        """Block until armed.
        
        Args:
            timeout: Maximum seconds to wait
            
        Returns:
            True if armed, False if timeout
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_armed():
                return True
            time.sleep(0.1)
        return False

    def wait_for_disarm(self, timeout: float = 30.0) -> bool:
        """Block until disarmed.
        
        Args:
            timeout: Maximum seconds to wait
            
        Returns:
            True if disarmed, False if timeout
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_armed():
                return True
            time.sleep(0.1)
        return False

    def wait_for_mode(self, mode: str, timeout: float = 10.0) -> bool:
        """Block until in specified mode.
        
        Args:
            mode: Expected mode string
            timeout: Maximum seconds to wait
            
        Returns:
            True if mode matches, False if timeout
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.get_mode() == mode:
                return True
            time.sleep(0.1)
        return False

    # ── Position Control ──────────────────────────────────────────────────────

    def fly_to(self, x: float, y: float, z: float, yaw: float = 0.0,
               timeout: float = 30.0, tolerance: float = 0.5,
               progress: bool = True) -> bool:
        """Fly to position and wait for arrival.
        
        Continuously publishes setpoints to the target position until
        within tolerance or timeout.
        
        Args:
            x, y, z: Target position in local ENU frame (metres)
            yaw: Target heading in radians
            timeout: Maximum seconds to reach target
            tolerance: Arrival tolerance in metres (3D distance)
            progress: Print progress to stdout
            
        Returns:
            True if target reached, False if timeout
        """
        deadline = time.time() + timeout
        
        while time.time() < deadline and not self._stop.is_set():
            self.publish_setpoint(x, y, z, yaw)
            
            cx, cy, cz = self.get_position()
            dist = math.sqrt((x - cx)**2 + (y - cy)**2 + (z - cz)**2)
            
            if progress:
                print(f"\r  Target: ({x:.1f},{y:.1f},{z:.1f})  "
                      f"Current: ({cx:.1f},{cy:.1f},{cz:.1f})  "
                      f"Dist: {dist:.2f}m    ", end="", flush=True)
            
            if dist < tolerance:
                if progress:
                    print()  # Newline
                self._log(f"Position reached within {tolerance:.2f}m")
                return True
            
            time.sleep(self._setpoint_period)
        
        if progress:
            print()  # Newline
        self._warn(f"Position timeout (dist: {dist:.2f}m)")
        return False

    def wait_for_altitude(self, target_z: float, timeout: float = 30.0,
                          tolerance: float = 0.15) -> bool:
        """Wait for specific altitude while maintaining setpoint stream.
        
        Args:
            target_z: Target Z position (metres, absolute)
            timeout: Maximum seconds to wait
            tolerance: Altitude tolerance (metres)
            
        Returns:
            True if altitude reached, False if timeout
        """
        x, y, _ = self.get_position()
        yaw = self.get_yaw()
        return self.fly_to(x, y, target_z, yaw, timeout, tolerance)

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def shutdown(self):
        """Clean shutdown of the flight controller."""
        self._stop.set()
        self._log("FlightController shutting down")
        
        # Node cleanup
        self._node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


# ── Convenience Functions ─────────────────────────────────────────────────────

def create_controller(node_name: str = "flight_controller") -> FlightController:
    """Factory function to create a FlightController instance.
    
    Use this when you want a quick setup:
    
        fc = create_controller("my_test")
        fc.wait_for_connection()
        ...
    """
    return FlightController(node_name=node_name)


# ── Test Harness ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    """Quick test: connect, check EKF, print state."""
    print("FlightController Test")
    print("=" * 40)
    
    fc = FlightController(node_name="fc_test")
    
    try:
        if not fc.wait_for_connection(timeout=10.0):
            print("FAIL: No FCU connection")
            fc.shutdown()
            exit(1)
        
        print(f"Connected. Mode: {fc.get_mode()}  Armed: {fc.is_armed()}")
        
        if not fc.wait_for_ekf(timeout=20.0, require_gps=False):
            print("FAIL: EKF not stable")
            fc.shutdown()
            exit(1)
        
        x, y, z = fc.get_position()
        print(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"Yaw: {math.degrees(fc.get_yaw()):.1f} deg")
        
        print("\nFlightController OK")
        
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        fc.shutdown()
