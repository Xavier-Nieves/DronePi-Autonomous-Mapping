"""
flight/_flow_bridge.py — Hailo optical flow → MAVROS EKF2 velocity bridge.

Runs in the Conda dronepi environment alongside the rest of the flight stack.
Subscribes to /hailo/optical_flow published by hailo_flight_node.py (which
runs in its own hailo_inference_env) and forwards valid, confidence-gated
velocity estimates to /mavros/odometry/out for EKF2 velocity fusion.

This file is the integration point between the two isolated Python
environments. The Hailo env publishes the flow topic over ROS 2 DDS.
The Conda env (this file) consumes it. No shared imports, no shared venv.

Architecture
------------
  hailo_inference_env:
    hailo_flight_node.py
      → /hailo/optical_flow  (geometry_msgs/TwistStamped)
          twist.linear.x  = vx m/s  (body frame forward)
          twist.linear.y  = vy m/s  (body frame lateral)
          twist.linear.z  = confidence 0.0–1.0  (repurposed field)

  dronepi conda env (this file):
    FlowBridge._flow_callback()
      ← /hailo/optical_flow
      → /mavros/odometry/out  (nav_msgs/Odometry)
          EKF2_EV_CTRL bit 2 fuses this as 3D velocity

EKF2 parameter requirements
----------------------------
  EKF2_EV_CTRL  = 15   All four bits — pos + height + velocity + yaw
  EKF2_EV_DELAY = 50   ms — accounts for Hailo inference latency (~15–25ms)
                        on top of the SLAM bridge latency already in this param
  EKF2_HGT_REF  = 3    SLAM as primary height reference

Safety gates applied before forwarding
---------------------------------------
  1. Confidence threshold (default 0.50)
     Hailo reports confidence per-frame based on flow magnitude consistency.
     Low confidence = turbulent scene, rotation, or motion blur.

  2. Message age guard (default 150 ms)
     Rejects stale messages that arrived late through the DDS transport.
     Stale velocity in EKF2 is worse than no velocity.

  3. Velocity magnitude clamp (default 15 m/s)
     Rejects physically implausible spikes from inference artefacts.
     15 m/s is well above the drone's maximum speed.

  4. Consecutive fallback counter
     After FALLBACK_ESCALATE_COUNT consecutive rejections, calls
     degraded_callback() so the watchdog/main can flag HAILO_DEGRADED.

Usage — instantiated by main.py
--------------------------------
  from _flow_bridge import FlowBridge

  flow_bridge = FlowBridge(
      node=node._node,                     # shared rclpy Node
      degraded_callback=_on_flow_degraded, # optional escalation hook
  )
  # No further calls needed — runs on the shared ROS 2 executor.
  # FlowBridge is passive — it only reacts to incoming messages.

  # Diagnostics (call periodically):
  stats = flow_bridge.get_stats()
  # → {"received": N, "forwarded": N, "pass_rate": 0.73, ...}

  # On shutdown:
  flow_bridge.destroy()

Design notes
------------
  - FlowBridge does NOT modify the Hailo node's output. The original
    header timestamp is preserved in the outgoing Odometry message so
    EKF2 can apply its delay compensation correctly.

  - The twist.linear.z confidence field is extracted here and then
    discarded. The outgoing Odometry.twist.twist.linear.z = 0.0 because
    the Hailo flow model does not estimate vertical velocity.

  - FlowBridge is NOT a ROS 2 Node subclass. It is a helper class that
    attaches subscriptions and publishers to an existing Node. This avoids
    a second rclpy context and keeps the executor single-threaded.

  - Thread safety: _flow_callback is called from the ROS executor thread.
    All mutable state is protected by _lock. _forward_velocity builds and
    publishes the Odometry message while the lock is NOT held to avoid
    blocking the executor.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional


# ── Constants ─────────────────────────────────────────────────────────────────

CONFIDENCE_THRESHOLD    = 0.50    # Reject flow below this value
MAX_MSG_AGE_S           = 0.150   # Reject messages older than 150 ms
MAX_VELOCITY_M_S        = 15.0    # Clamp — above this is physically implausible
FALLBACK_ESCALATE_COUNT = 30      # Consecutive rejections before degraded flag

# Odometry covariance diagonal values (m/s)²
# Increase if EKF2 velocity innovations are large or jittery
_COV_VX = 0.10   # vx variance
_COV_VY = 0.10   # vy variance
_COV_VZ = 0.50   # vz variance — not estimated, set high so EKF ignores it


class FlowBridge:
    """
    Gates Hailo optical flow and forwards valid estimates to MAVROS.

    Parameters
    ----------
    node : rclpy.node.Node
        Shared ROS 2 node. Subscriptions and publishers are created on it.
    confidence_threshold : float
        Minimum Hailo confidence to forward. Default 0.50.
    degraded_callback : callable, optional
        Called once when consecutive fallback count exceeds
        FALLBACK_ESCALATE_COUNT. Use to set HAILO_DEGRADED LED state.
    """

    def __init__(
        self,
        node,
        confidence_threshold: float = CONFIDENCE_THRESHOLD,
        degraded_callback: Optional[Callable] = None,
    ) -> None:
        self._node             = node
        self._conf_threshold   = confidence_threshold
        self._degraded_cb      = degraded_callback

        # Statistics
        self._total_received   = 0
        self._total_forwarded  = 0
        self._fallback_count   = 0
        self._degraded_mode    = False
        self._last_forwarded_t: Optional[float] = None

        self._lock             = threading.Lock()

        # ROS handles — set in _setup_ros
        self._flow_sub  = None
        self._odom_pub  = None

        self._setup_ros()

        print(
            f"  [FlowBridge] Initialised  "
            f"confidence_threshold={confidence_threshold}  "
            f"max_age={MAX_MSG_AGE_S*1000:.0f}ms  "
            f"max_vel={MAX_VELOCITY_M_S}m/s"
        )

    # ── Public API ─────────────────────────────────────────────────────────────

    def get_stats(self) -> dict:
        """
        Return forwarding statistics dictionary.

        Keys: received, forwarded, pass_rate, fallback_count, degraded
        """
        with self._lock:
            rcv = self._total_received
            fwd = self._total_forwarded
            return {
                "received":       rcv,
                "forwarded":      fwd,
                "pass_rate":      round(fwd / rcv, 3) if rcv > 0 else 0.0,
                "fallback_count": self._fallback_count,
                "degraded":       self._degraded_mode,
            }

    def is_active(self) -> bool:
        """True if a message was forwarded in the last 2 seconds."""
        with self._lock:
            if self._last_forwarded_t is None:
                return False
            return (time.monotonic() - self._last_forwarded_t) < 2.0

    def destroy(self) -> None:
        """Remove subscriptions and publishers from the shared node."""
        try:
            if self._flow_sub is not None:
                self._node.destroy_subscription(self._flow_sub)
            if self._odom_pub is not None:
                self._node.destroy_publisher(self._odom_pub)
        except Exception:
            pass
        self._flow_sub = None
        self._odom_pub = None
        print("  [FlowBridge] Destroyed")

    # ── Private — ROS setup ────────────────────────────────────────────────────

    def _setup_ros(self) -> None:
        """Create subscriber and publisher on the shared node."""
        try:
            from geometry_msgs.msg import TwistStamped
            from nav_msgs.msg      import Odometry
            from rclpy.qos         import (
                QoSProfile, ReliabilityPolicy, HistoryPolicy
            )
        except ImportError as exc:
            print(f"  [FlowBridge] ROS 2 import failed: {exc}")
            return

        # BEST_EFFORT matches the QoS that hailo_flight_node uses for publishing.
        # Using RELIABLE here would cause the subscription to receive no messages
        # because QoS policies must be compatible — reliable subscriber can only
        # receive from reliable publisher, not best_effort.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._flow_sub = self._node.create_subscription(
            TwistStamped,
            "/hailo/optical_flow",
            self._flow_callback,
            sensor_qos,
        )
        self._odom_pub = self._node.create_publisher(
            Odometry,
            "/mavros/odometry/out",
            reliable_qos,
        )

        print("  [FlowBridge] ROS setup complete")
        print("    subscribed : /hailo/optical_flow")
        print("    publishing : /mavros/odometry/out")

    # ── Private — callback ─────────────────────────────────────────────────────

    def _flow_callback(self, msg) -> None:
        """
        Process one incoming optical flow message.

        The Hailo flight node encodes confidence in twist.linear.z.
        All gates run under the lock. The actual publish call runs outside
        the lock to avoid blocking the ROS executor.
        """
        vx         = float(msg.twist.linear.x)
        vy         = float(msg.twist.linear.y)
        confidence = float(msg.twist.linear.z)   # repurposed — see hailo_flight_node.py

        forward    = False
        header     = msg.header

        with self._lock:
            self._total_received += 1

            # ── Gate 1: confidence ────────────────────────────────────────────
            if confidence < self._conf_threshold:
                self._fallback_count += 1
                self._check_escalation()
                return

            # ── Gate 2: message age ───────────────────────────────────────────
            # Build a rclpy.time.Time from the message header and compare to now.
            # Messages that sat in the DDS queue too long are stale and harmful.
            try:
                import rclpy.time
                msg_ts  = rclpy.time.Time.from_msg(msg.header.stamp)
                now_ts  = self._node.get_clock().now()
                age_s   = (now_ts - msg_ts).nanoseconds / 1e9
                if age_s > MAX_MSG_AGE_S:
                    self._fallback_count += 1
                    self._check_escalation()
                    return
            except Exception:
                pass   # clock unavailable — skip age check, do not reject

            # ── Gate 3: velocity magnitude clamp ─────────────────────────────
            speed = (vx**2 + vy**2) ** 0.5
            if speed > MAX_VELOCITY_M_S:
                self._fallback_count += 1
                self._check_escalation()
                return

            # ── All gates passed ──────────────────────────────────────────────
            self._fallback_count   = 0
            self._degraded_mode    = False
            self._total_forwarded += 1
            self._last_forwarded_t = time.monotonic()
            forward                = True

        # Publish outside the lock so the executor is not blocked
        if forward:
            self._forward_velocity(header, vx, vy)

    def _forward_velocity(self, header, vx: float, vy: float) -> None:
        """
        Build and publish nav_msgs/Odometry with velocity fields populated.

        The original Hailo header timestamp is preserved so EKF2 can apply
        its EKF2_EV_DELAY compensation correctly. Changing the timestamp here
        would cause EKF2 to miscompute the measurement delay and potentially
        reject or incorrectly weight the velocity measurement.

        Frame convention:
          odom.header.frame_id    = "odom"       (world frame)
          odom.child_frame_id     = "base_link"  (body frame)
          odom.twist.twist.linear = body-frame velocity

        EKF2 with EKF2_EV_CTRL bit 2 fuses twist.twist.linear as external
        vision velocity. Position fields (odom.pose) are left at zero —
        EKF2 ignores them when only the velocity bit is enabled.
        """
        if self._odom_pub is None:
            return

        try:
            from nav_msgs.msg import Odometry
        except ImportError:
            return

        odom                       = Odometry()
        odom.header                = header       # preserve Hailo timestamp
        odom.header.frame_id       = "odom"
        odom.child_frame_id        = "base_link"

        # Body-frame velocity — EKF2 fuses these
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.linear.z  = 0.0   # vertical not estimated by flow model

        # Covariance diagonal — row-major 6×6, indices [0]=vx, [7]=vy, [14]=vz
        # Conservative starting values — increase if EKF2 innovations are large
        odom.twist.covariance[0]   = _COV_VX
        odom.twist.covariance[7]   = _COV_VY
        odom.twist.covariance[14]  = _COV_VZ

        self._odom_pub.publish(odom)

    def _check_escalation(self) -> None:
        """
        Fire the degraded callback once when the fallback threshold is crossed.

        Called under self._lock — do not acquire the lock inside this method.
        The callback itself is called outside the lock via a one-shot flag to
        avoid holding the lock during an arbitrary user callback.
        """
        if (self._fallback_count >= FALLBACK_ESCALATE_COUNT
                and not self._degraded_mode):
            self._degraded_mode = True
            # Schedule callback outside the lock
            if self._degraded_cb is not None:
                import threading
                threading.Thread(
                    target=self._degraded_cb, daemon=True
                ).start()
