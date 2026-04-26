#!/usr/bin/env python3
"""
flight/main.py — DronePi autonomous survey mission.

Triggered by the watchdog via RC CH6 toggle or by direct invocation.
Runs the full survey: takeoff → waypoints → gap fill → camera trigger →
postflight processing.

SAFETY CONTEXT: CONTEXT_MISSION
  OFFBOARD lost → 30s pause window with RC stick activity sampling.
  - If pilot static (sticks < 30 PWM from baseline) and OFFBOARD restored
    → resume mission from current waypoint.
  - If pilot active OR timeout → AUTO.RTL.
  - RC CH7 > 1700 PWM → immediate teardown in all contexts.

WATCHDOG COORDINATION:
  - Writes /tmp/dronepi_mission.lock with mode="autonomous" and PID.
  - Watchdog reads lock["mode"] — unchanged API, backward-compatible.
  - Watchdog yields all control when lock mode is "autonomous".
  - main_status.json heartbeat written to keep watchdog's dead-man check happy.

OFFBOARD LOSS HANDLING — how the waypoint loop interacts with the mixin:

  OLD (replaced):
      while node.mode != 'OFFBOARD' and node.armed:
          if elapsed > OFFBOARD_RESUME_S:
              node.set_mode('AUTO.RTL')
              return
          time.sleep(0.5)

  NEW (mixin-driven):
      if self._offboard_pause_active:
          time.sleep(0.5)   # mixin is managing the pause window — just wait
          continue
      if self._offboard_aborted:
          break              # mixin already commanded AUTO.RTL

  The mixin's _rc_override_monitor_loop handles all state transitions.
  The waypoint loop only needs to check two booleans.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from flight.preflight_checks import PreflightChecker
from flight.waypoint_validator import WaypointValidator
from flight.gps_reader import GpsReader

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

# SafeFlightMixin lives alongside this file in flight/
sys.path.insert(0, str(Path(__file__).parent))
from safe_flight_mixin import (  # noqa: E402
    CONTEXT_MISSION,
    SafeFlightMixin,
)

# ── Constants ─────────────────────────────────────────────────────────────────
ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = str(Path.home() / "unitree_lidar_project/RPI5/ros2_ws/install/setup.bash")

POINTLIO_LAUNCH = (
    "source {ros} && source {ws} && "
    "ros2 launch point_lio mapping_mid360.launch.py"
).format(ros=ROS_SETUP, ws=WS_SETUP)
SLAM_BRIDGE_CMD = (
    "source {ros} && source {ws} && "
    "python3 {script}"
).format(
    ros=ROS_SETUP,
    ws=WS_SETUP,
    script=str(Path(__file__).parent / "slam_bridge.py"),
)
POSTFLIGHT_CMD = (
    "source {ros} && source {ws} && "
    "python3 {script}"
).format(
    ros=ROS_SETUP,
    ws=WS_SETUP,
    script=str(Path(__file__).parent / "postflight.py"),
)

SETPOINT_RATE_HZ = 20.0
SETPOINT_PERIOD  = 1.0 / SETPOINT_RATE_HZ
WARMUP_S         = 3.0
OFFBOARD_RESUME_S = 30.0        # must match SafeFlightMixin offboard_resume_s
POS_TOLERANCE_M  = 0.20
ALT_TOLERANCE_M  = 0.15

MAIN_STATUS_FILE = Path("/tmp/main_status.json")   # heartbeat for watchdog


def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


class MainNode(SafeFlightMixin, Node):
    """
    Autonomous survey mission node.

    Inherits SafeFlightMixin with CONTEXT_MISSION — OFFBOARD loss triggers
    a 30s pause window with RC activity detection instead of immediate teardown.
    """

    def __init__(self, waypoints_enu: list[tuple[float, float, float]]) -> None:
        Node.__init__(self, "dronepi_main")
        SafeFlightMixin.__init__(
            self,
            script_name=__file__,
            context=CONTEXT_MISSION,
            offboard_resume_s=OFFBOARD_RESUME_S,
        )

        self._waypoints = waypoints_enu
        self.mode:    str   = ""
        self.armed:   bool  = False
        self.local_x: float = 0.0
        self.local_y: float = 0.0
        self.local_z: float = 0.0

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sp_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self.create_subscription(
            State, "/mavros/state", self._state_cb, qos_reliable
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, qos_sensor
        )

        self.arming_client   = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode,     "/mavros/set_mode")

        self._pointlio_proc:    subprocess.Popen | None = None
        self._slam_bridge_proc: subprocess.Popen | None = None

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _state_cb(self, msg: State) -> None:
        self.mode  = msg.mode
        self.armed = msg.armed

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.local_x = msg.pose.position.x
        self.local_y = msg.pose.position.y
        self.local_z = msg.pose.position.z

    # ── Service helpers ──────────────────────────────────────────────────────

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return bool(future.result() and future.result().mode_sent)

    def arm(self) -> bool:
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        return bool(future.result() and future.result().success)

    # ── Setpoint publish ─────────────────────────────────────────────────────

    def publish_sp(self, x: float, y: float, z: float) -> None:
        """
        Publish one setpoint AND update both mixin heartbeat fields.
        MUST NOT be called when self._pilot_override is True.
        """
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)

        # REQUIRED: feeds the setpoint watchdog
        self._sp_heartbeat = time.monotonic()
        self._alt_setpoint = z

    # ── Watchdog heartbeat ────────────────────────────────────────────────────

    def write_main_status(self, status: str) -> None:
        try:
            MAIN_STATUS_FILE.write_text(json.dumps({
                "status":     status,
                "updated_at": time.time(),
                "pid":        self._pid,
            }))
        except Exception:
            pass

    # ── Subprocesses ─────────────────────────────────────────────────────────

    def start_pointlio(self) -> bool:
        try:
            self._pointlio_proc = subprocess.Popen(
                ["bash", "-c", POINTLIO_LAUNCH],
                preexec_fn=__import__("os").setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            log(f"Point-LIO started  pid={self._pointlio_proc.pid}")
            time.sleep(4.0)
            return True
        except Exception as exc:
            log(f"Point-LIO start failed: {exc}")
            return False

    def start_slam_bridge(self) -> bool:
        try:
            self._slam_bridge_proc = subprocess.Popen(
                ["bash", "-c", SLAM_BRIDGE_CMD],
                preexec_fn=__import__("os").setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            log(f"SLAM bridge started  pid={self._slam_bridge_proc.pid}")
            time.sleep(2.0)
            return True
        except Exception as exc:
            log(f"SLAM bridge start failed: {exc}")
            return False

    def _kill_proc(self, proc: subprocess.Popen | None) -> None:
        if proc and proc.poll() is None:
            import os, signal
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=8)
            except Exception:
                pass

    # ── Navigation ───────────────────────────────────────────────────────────

    def fly_to(
        self, x: float, y: float, z: float, timeout_s: float = 30.0
    ) -> bool:
        """
        Fly toward (x, y, z) until within tolerance.
        Returns False if pilot override activates mid-transit.
        Does NOT publish during _offboard_pause_active.
        """
        import math
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if self._pilot_override or self._offboard_aborted:
                return False
            if self._offboard_pause_active:
                time.sleep(0.5)
                continue
            self.publish_sp(x, y, z)
            rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)
            dist_h = math.hypot(self.local_x - x, self.local_y - y)
            dist_v = abs(self.local_z - z)
            if dist_h < POS_TOLERANCE_M and dist_v < ALT_TOLERANCE_M:
                return True
        return not (self._pilot_override or self._offboard_aborted)

    # ── Camera trigger ────────────────────────────────────────────────────────

    def trigger_camera(self) -> None:
        try:
            subprocess.run(
                ["bash", "-c",
                 f"source {ROS_SETUP} && source {WS_SETUP} && "
                 "ros2 topic pub --once /mavros/cmd/command "
                 "mavros_msgs/msg/CommandCode '{command: 203}'"],
                timeout=3, capture_output=True,
            )
            log("Camera triggered.")
            self._log_event("CAMERA_TRIGGER", {
                "actual_enu": [
                    round(self.local_x, 2),
                    round(self.local_y, 2),
                    round(self.local_z, 2),
                ],
            })
        except Exception as exc:
            log(f"Camera trigger failed: {exc}")
            self._log_event("CAMERA_TRIGGER_FAILED", {"error": str(exc)})

    # ── Main handle_autonomous ────────────────────────────────────────────────

    def handle_autonomous(self) -> bool:
        """
        Execute the autonomous survey mission.

        OFFBOARD loss is handled by the mixin's RC Override Monitor.
        The waypoint loop polls two booleans:
          self._offboard_pause_active  — True while the 30s window is open
          self._offboard_aborted       — True after AUTO.RTL was commanded

        This replaces the old while-loop that was in this function.
        Do NOT add another while-loop for OFFBOARD recovery here.
        """
        log(f"handle_autonomous: {len(self._waypoints)} waypoints.")
        self.write_main_status("autonomous_running")
        self._log_event("TAKEOFF", {
            "waypoint_count": len(self._waypoints),
            "first_target_enu": [
                round(self._waypoints[0][0], 2),
                round(self._waypoints[0][1], 2),
                round(self._waypoints[0][2], 2),
            ] if self._waypoints else None,
        })

        # ── SLAM staleness tracking ───────────────────────────────────────────
        # If Point-LIO stops publishing mid-flight, the SLAM bridge keeps
        # forwarding the last known pose indefinitely. EKF2 believes the drone
        # is holding position while it physically drifts. Detect this by
        # monitoring /aft_mapped_to_init freshness and commanding AUTO.LAND
        # if the topic goes silent beyond SLAM_STALE_S.
        # Source: Point-LIO /aft_mapped_to_init, nav_msgs/Odometry, ~10 Hz
        _SLAM_STALE_S = float(os.environ.get("SLAM_STALE_S", "5.0"))
        _slam_last_t: list[float] = [time.monotonic()]

        def _slam_live_cb(_msg) -> None:
            _slam_last_t[0] = time.monotonic()

        from nav_msgs.msg import Odometry as _OdomMsg
        from rclpy.qos import QoSProfile as _QoSProfile
        from rclpy.qos import ReliabilityPolicy as _Rel, HistoryPolicy as _Hist
        _slam_live_sub = self.create_subscription(
            _OdomMsg, "/aft_mapped_to_init", _slam_live_cb,
            _QoSProfile(reliability=_Rel.BEST_EFFORT,
                        history=_Hist.KEEP_LAST, depth=5),
        )

        try:
            for i, (ex, ey, ez) in enumerate(self._waypoints):
                # ── CONTEXT_MISSION pause window check ───────────────────────
                if self._offboard_aborted:
                    log("Mission aborted by mixin (AUTO.RTL commanded). Exiting loop.")
                    return False

                if self._offboard_pause_active:
                    log("OFFBOARD pause active — waiting for mixin resolution...")
                    while self._offboard_pause_active and not self._teardown_called:
                        time.sleep(0.5)
                    if self._offboard_aborted:
                        log("Mixin resolved: aborted. Exiting loop.")
                        return False
                    log(f"Mixin resolved: resuming from waypoint {i+1}.")

                if self._pilot_override:
                    return False

                # ── SLAM staleness check ──────────────────────────────────────
                _slam_age = time.monotonic() - _slam_last_t[0]
                if _slam_age > _SLAM_STALE_S:
                    log(
                        f"CRITICAL: No Point-LIO output for {_slam_age:.1f}s "
                        f"(limit {_SLAM_STALE_S:.1f}s). "
                        "EKF2 position frozen. Commanding AUTO.LAND."
                    )
                    self._log_event("SLAM_STALE_LAND",
                                    {"stale_s": round(_slam_age, 1)})
                    self.set_mode("AUTO.LAND")
                    return False

                # ── Fly to waypoint ───────────────────────────────────────────
                log(f"Waypoint {i+1}/{len(self._waypoints)}: "
                    f"ENU=({ex:.1f}, {ey:.1f}, {ez:.1f})")
                self.write_main_status(f"wp_{i+1}_of_{len(self._waypoints)}")
                self._log_event("WAYPOINT_DISPATCH", {
                    "wp_index":   i + 1,
                    "wp_total":   len(self._waypoints),
                    "target_enu": [round(ex, 2), round(ey, 2), round(ez, 2)],
                })

                if not self.fly_to(ex, ey, ez):
                    log(f"Waypoint {i+1} not reached (override or abort).")
                    self._log_event("WAYPOINT_ABORTED", {
                        "wp_index": i + 1,
                        "actual_enu": [
                            round(self.local_x, 2),
                            round(self.local_y, 2),
                            round(self.local_z, 2),
                        ],
                    })
                    return False

                log(f"  Waypoint {i+1} reached.")
                self._log_event("WAYPOINT_REACHED", {
                    "wp_index":  i + 1,
                    "actual_enu": [
                        round(self.local_x, 2),
                        round(self.local_y, 2),
                        round(self.local_z, 2),
                    ],
                    "pos_error_m": round(
                        ((self.local_x - ex)**2 +
                         (self.local_y - ey)**2 +
                         (self.local_z - ez)**2) ** 0.5, 3
                    ),
                })

                # ── Camera trigger at waypoint ────────────────────────────────
                self.trigger_camera()

            log("All waypoints complete.")
            return True

        finally:
            self.destroy_subscription(_slam_live_sub)

    # ── Full mission run ──────────────────────────────────────────────────────

    def run(self) -> bool:
        """
        Full mission sequence:
          start_safety_monitors → Point-LIO → SLAM bridge →
          preflight checks (3a) → waypoint validation (3b) →
          warmup → OFFBOARD + arm → handle_autonomous → AUTO.LAND → postflight
        """

        # ── 1. Safety monitors (bag recorder + lock + all monitors) ───────────
        if not self.start_safety_monitors():
            log("ERROR: Could not acquire mission lock. Another mission running?")
            return False
        if self._pilot_override:
            log("ERROR: Pilot override active before arm. Aborting.")
            return False

        # Export session directory so subprocesses (collision_monitor, etc.)
        # can append to the same flight_events.json without a shared reference.
        os.environ["DRONEPI_SESSION_DIR"] = str(self._session_dir)

        # ── 2. Subprocesses ───────────────────────────────────────────────────
        if not self.start_pointlio():
            self._teardown("pointlio_failed")
            return False

        if not self.start_slam_bridge():
            self._teardown("slam_bridge_failed")
            return False

        # ── 3. Wait for MAVROS ────────────────────────────────────────────────
        log("Waiting for MAVROS...")
        t0 = time.monotonic()
        while self.mode == "" and time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.mode == "":
            log("ERROR: MAVROS timeout.")
            self._teardown("mavros_timeout")
            return False

        # ── 3a. Preflight checks ──────────────────────────────────────────────
        # PreflightChecker runs three independent gates in sequence:
        #   1. stale_lock      — clears dead-PID lock from prior session
        #   2. ssd_mount       — verifies /mnt/ssd is mounted with free space
        #   3. slam_convergence — confirms Point-LIO is publishing before arm
        # All fatal failures abort here before any setpoint or arm call.
        _pfc = PreflightChecker(
            node=self,
            spin_fn=lambda: rclpy.spin_once(self, timeout_sec=0.5),
            log_event_fn=self._log_event,
        )
        for _result in _pfc.run_all():
            if _result.passed:
                if _result.reason != "ok":
                    log(f"PREFLIGHT [{_result.check}] WARNING: {_result.reason} "
                        f"{_result.detail}")
                else:
                    log(f"PREFLIGHT [{_result.check}] OK  {_result.detail}")
            else:
                log(f"PREFLIGHT [{_result.check}] FAILED: {_result.reason} "
                    f"{_result.detail}")
                if _result.fatal:
                    self._teardown(f"preflight_{_result.check}_failed")
                    return False

        # ── 3b. Waypoint validation ───────────────────────────────────────────
        _gps = GpsReader()
        _gps.start()

        _vr = WaypointValidator(_gps, max_radius_m=float(
            os.environ.get("WAYPOINT_MAX_RADIUS_M", "200.0")
        ))
        _val_result = _vr.check(self._waypoints)

        if not _val_result.passed:
            log(
                f"ERROR: Waypoint validation FAILED — {_val_result.reason}. "
                f"Offending waypoint index: {_val_result.offending_index}, "
                f"radius: {_val_result.offending_radius_m} m. Aborting pre-arm."
            )
            _gps.stop()
            self._teardown("waypoint_validation_failed")
            return False

        if _val_result.skipped:
            log(f"WARNING: Waypoint validation skipped — {_val_result.reason}. Proceeding.")

        _gps.stop()

        # ── 4. Warmup setpoints ───────────────────────────────────────────────
        log(f"Setpoint warmup ({WARMUP_S}s)...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < WARMUP_S:
            if self._pilot_override:
                self._teardown("pilot_override_warmup")
                return False
            # Warm up toward first waypoint altitude
            first_alt = self._waypoints[0][2] if self._waypoints else 3.0
            self.publish_sp(0.0, 0.0, first_alt)
            rclpy.spin_once(self, timeout_sec=SETPOINT_PERIOD)

        # ── 5. OFFBOARD + arm ─────────────────────────────────────────────────
        if not self.set_mode("OFFBOARD"):
            log("ERROR: Failed to enter OFFBOARD.")
            self._teardown("offboard_failed")
            return False

        if not self.arm():
            log("ERROR: Failed to arm.")
            self._teardown("arm_failed")
            return False

        log("Armed and in OFFBOARD. Starting mission.")
        self._log_event("ARMED", {
            "pointlio_pid":    self._pointlio_proc.pid if self._pointlio_proc else None,
            "slam_bridge_pid": self._slam_bridge_proc.pid if self._slam_bridge_proc else None,
            "waypoint_count":  len(self._waypoints),
        })

        success = False
        try:
            success = self.handle_autonomous()

        finally:
            # ── 6. Land (always) ──────────────────────────────────────────────
            if self.armed and not self._teardown_called:
                log("Landing...")
                self._log_event("LANDING", {
                    "reason":     "mission_complete" if success else "mission_aborted",
                    "actual_enu": [
                        round(self.local_x, 2),
                        round(self.local_y, 2),
                        round(self.local_z, 2),
                    ],
                })
                self.set_mode("AUTO.LAND")
                t0 = time.monotonic()
                while self.armed and time.monotonic() - t0 < 45.0:
                    rclpy.spin_once(self, timeout_sec=0.5)

            # ── 7. Stop subprocesses ──────────────────────────────────────────
            self._kill_proc(self._slam_bridge_proc)
            self._kill_proc(self._pointlio_proc)

            # ── 8. Mixin teardown (stops bag, releases lock, flushes log) ─────
            self._teardown("mission_complete" if success else "mission_aborted")

            self.write_main_status("idle")

        # ── 9. Postflight processing ───────────────────────────────────────────
        if success:
            log("Starting postflight processing...")
            try:
                subprocess.Popen(
                    ["bash", "-c", POSTFLIGHT_CMD],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception as exc:
                log(f"Postflight launch failed: {exc}")

        return success


# ── Entry point ────────────────────────────────────────────────────────────────

def parse_waypoint(s: str) -> tuple[float, float, float]:
    parts = [float(v) for v in s.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"Waypoint must be x,y,z — got: {s}")
    return (parts[0], parts[1], parts[2])


def main() -> None:
    parser = argparse.ArgumentParser(
        description="DronePi autonomous survey mission."
    )
    parser.add_argument(
        "--waypoints", nargs="+", type=parse_waypoint,
        required=True, metavar="x,y,z",
        help="ENU waypoints in metres, space-separated e.g. 5,0,4 5,5,4 0,5,4",
    )
    args = parser.parse_args()

    rclpy.init()
    node = MainNode(waypoints_enu=args.waypoints)

    try:
        success = node.run()
        log(f"Mission {'COMPLETE' if success else 'ABORTED'}.")
        sys.exit(0 if success else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
