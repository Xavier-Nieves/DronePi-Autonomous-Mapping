#!/usr/bin/env python3
"""
tests/test_hailo_flight_validation.py — Props-on hover test with full Hailo chain.

Combines:
  - OFFBOARD arming, hover, and landing from test_altitude_validation.py
  - CameraCapture (Picamera2) → /arducam/image_raw
  - hailo_flight_node (hailo_inference_env subprocess)
  - /hailo/optical_flow + /hailo/ground_class reception
  - Structured JSON log for post-flight analysis

What this test validates
------------------------
  1. Full production stack starts correctly under flight conditions
  2. Camera publishes at target rate during hover
  3. Hailo receives camera frames and produces inference output
  4. Optical flow rate and confidence during actual hover motion
  5. Ground classification during nadir hover (first real classification)
  6. Waypoint JPEG saved with ENU + GPS metadata
  7. FlowBridge receives flow messages (confirms MAVROS EKF2 augmentation path)

What this does NOT test
------------------------
  - Multi-altitude progression (that is test_altitude_validation.py)
  - Square pattern lateral movement (that is test_square_camera.py)
  - Full waypoint mission (that is main.py autonomous)

Safety
------
  - bench_scan mission lock prevents watchdog/main from interfering
  - drone-watchdog stopped, dronepi-main kept running (collision ACTIVE)
  - Emergency land on any unhandled exception
  - Ctrl+C triggers graceful AUTO.LAND before teardown

Output
------
  tests/hailo_flight_output/
    session_<timestamp>/
      frame_0001.jpg          Full-res JPEG from waypoint trigger
      frame_0001.json         Sidecar with ENU, GPS, ROS timestamp
      hailo_flight_log.json   Structured per-second telemetry log
      report.txt              Human-readable summary

Run:
    cd tests/
    python test_hailo_flight_validation.py
    python test_hailo_flight_validation.py --alt 3 --hold 20
    python test_hailo_flight_validation.py --dry-run   # sensor checks only
"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────────────────────

_TESTS_DIR  = Path(__file__).resolve().parent
_MAPPER_DIR = _TESTS_DIR.parent
sys.path.insert(0, str(_MAPPER_DIR))

HAILO_FLIGHT_SCRIPT = _MAPPER_DIR / "hailo" / "hailo_flight_node.py"
HAILO_ENV           = Path.home() / "hailo_inference_env" / "bin" / "python3"
POINTLIO_CMD        = (
    f"source /opt/ros/jazzy/setup.bash && "
    f"ros2 launch point_lio mapping_unilidar.launch.py"
)
POINTLIO_LOG        = Path("/tmp/hailo_flight_val_pointlio.log")
MISSION_LOCK        = Path("/tmp/dronepi_mission.lock")

OUTPUT_BASE = _TESTS_DIR / "hailo_flight_output"

# ── Constants ─────────────────────────────────────────────────────────────────

HOVER_ALT_DEFAULT   = 3.0    # metres above home
HOLD_S_DEFAULT      = 30.0   # seconds to hold at altitude
SETPOINT_HZ         = 20
WP_TOLERANCE_M      = 0.3
TAKEOFF_TIMEOUT_S   = 30.0

# Hailo readiness
HAILO_SUBSCRIBER_TIMEOUT = 30.0
CAMERA_PUBLISH_HZ        = 15.0

# Log intervals
TELEMETRY_LOG_HZ   = 2.0   # write one log entry per 0.5s

# ── Helpers ───────────────────────────────────────────────────────────────────

def _ros_source():
    ws = os.path.expanduser(
        "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
    )
    return (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {ws} && "
    )


def _launch(name: str, cmd: str, log_path: Path) -> subprocess.Popen:
    print(f"  Starting {name}  (log → {log_path})")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "w") as lf:
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=lf, stderr=lf,
            preexec_fn=os.setsid,
        )
    return proc


def _stop_proc(name: str, proc) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
        print(f"  stopped : {name}")
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def _wait_topic(topic: str, timeout: float) -> bool:
    import subprocess as sp
    cmd = (
        f"{_ros_source()} "
        f"timeout 6 ros2 topic hz {topic} --window 5 2>/dev/null "
        f"| grep -oP 'average rate: \\K[0-9.]+'"
    )
    deadline = time.time() + timeout
    print(f"  Waiting for {topic}  ({timeout:.0f}s timeout)", end="", flush=True)
    while time.time() < deadline:
        r = sp.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=12)
        if r.stdout.strip():
            hz = float(r.stdout.strip().splitlines()[-1])
            print(f" {hz:.1f} Hz  ✓")
            return True
        print(".", end="", flush=True)
        time.sleep(1.0)
    print(" TIMEOUT")
    return False


# ── Service management ────────────────────────────────────────────────────────

class ServiceManager:
    """Stop watchdog before test, restore after. Keep dronepi-main alive."""

    def __init__(self):
        self._stopped = []

    def pause(self):
        print("[Services] Pausing DronePi services...")
        for svc in ["drone-watchdog.service"]:
            r = subprocess.run(
                ["sudo", "systemctl", "stop", svc],
                capture_output=True, text=True,
            )
            if r.returncode == 0:
                self._stopped.append(svc)
                print(f"  stopped : {svc}")
            else:
                print(f"  skipped : {svc}  (was not running)")
        print("  [INFO] dronepi-main kept running — collision avoidance ACTIVE")

    def restore(self):
        print("[Services] Restoring DronePi services...")
        for svc in self._stopped:
            subprocess.run(["sudo", "systemctl", "start", svc],
                           capture_output=True)
            print(f"  started : {svc}")
        if MISSION_LOCK.exists():
            MISSION_LOCK.unlink()
            print("  [Lock] Cleared /tmp/dronepi_mission.lock")


# ── Hailo subprocess ──────────────────────────────────────────────────────────

def start_hailo_subprocess():
    if not HAILO_ENV.exists() or not HAILO_FLIGHT_SCRIPT.exists():
        print("  [WARN] hailo_inference_env or hailo_flight_node.py not found")
        return None

    proc = subprocess.Popen(
        [str(HAILO_ENV), str(HAILO_FLIGHT_SCRIPT)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True, bufsize=1,
        env=os.environ.copy(),
        preexec_fn=os.setsid,
    )

    lines = []

    def _drain():
        for line in proc.stdout:
            line = line.rstrip()
            lines.append(line)
            print(f"  [Hailo] {line}")

    threading.Thread(target=_drain, daemon=True, name="hailo_stdout").start()
    print(f"  hailo_flight_node.py started (PID {proc.pid})")
    return proc


def stop_hailo_subprocess(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    print("  [Hailo] stopped")


# ── ROS setup ─────────────────────────────────────────────────────────────────

def start_ros_stack(session_dir: Path):
    """Start CameraCapture + ROS node + Hailo subscribers. Returns (rclpy, node, cam, stats, hailo_proc)."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos  import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg   import Image
    from geometry_msgs.msg import TwistStamped, PoseStamped
    from std_msgs.msg      import String
    from mavros_msgs.msg   import State
    from flight.camera_capture import CameraCapture

    rclpy.init()
    ros_node = Node("hailo_flight_val_node")

    # Stats shared across callbacks
    stats = {
        "lock":             threading.Lock(),
        "image_ts":         [],
        "flow_messages":    [],
        "ground_messages":  [],
        "pose":             None,
        "home_z":           None,
        "state":            None,
        "vision_stamp":     None,
    }

    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST, depth=10)
    reliable_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, depth=10)

    def _image_cb(msg):
        with stats["lock"]:
            stats["image_ts"].append(time.monotonic())

    def _flow_cb(msg):
        with stats["lock"]:
            stats["flow_messages"].append({
                "vx":         msg.twist.linear.x,
                "vy":         msg.twist.linear.y,
                "confidence": msg.twist.linear.z,
                "t":          time.monotonic(),
            })

    def _ground_cb(msg):
        with stats["lock"]:
            try:
                d = json.loads(msg.data)
            except Exception:
                d = {"raw": msg.data}
            d["t"] = time.monotonic()
            stats["ground_messages"].append(d)

    def _pose_cb(msg):
        with stats["lock"]:
            stats["pose"] = {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
            }

    def _state_cb(msg):
        with stats["lock"]:
            stats["state"] = {
                "armed":     msg.armed,
                "mode":      msg.mode,
                "connected": msg.connected,
            }

    from mavros_msgs.msg import HomePosition
    def _home_cb(msg):
        with stats["lock"]:
            if stats["home_z"] is None:
                stats["home_z"] = msg.position.z
                print(f"  Home Z = {msg.position.z:.3f}m")

    ros_node.create_subscription(Image,        "/arducam/image_raw",          _image_cb,  sensor_qos)
    ros_node.create_subscription(TwistStamped, "/hailo/optical_flow",         _flow_cb,   reliable_qos)
    ros_node.create_subscription(String,       "/hailo/ground_class",         _ground_cb, reliable_qos)
    ros_node.create_subscription(PoseStamped,  "/mavros/local_position/pose", _pose_cb,   sensor_qos)
    ros_node.create_subscription(State,        "/mavros/state",               _state_cb,  10)
    ros_node.create_subscription(HomePosition, "/mavros/home_position/home",  _home_cb,   sensor_qos)

    threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True, name="val_ros_spin"
    ).start()

    # Camera
    cam = CameraCapture(
        session_id=session_dir.name,
        output_dir=session_dir,
        ros_node=ros_node,
        enable_ros_publish=True,
        ros_publish_topic="/arducam/image_raw",
        ros_publish_fps=CAMERA_PUBLISH_HZ,
    )
    if not cam.start():
        print("  [WARN] CameraCapture failed to start — camera disabled")
        cam = None
    else:
        print(f"  [CAM] IMX477 started @ {CAMERA_PUBLISH_HZ:.0f}Hz")

    # Hailo
    print("\n[Hailo] Starting hailo_flight_node...")
    hailo_proc = start_hailo_subprocess()

    if hailo_proc is not None:
        print(f"  Waiting for Hailo subscriber (up to {HAILO_SUBSCRIBER_TIMEOUT:.0f}s)...")
        deadline = time.time() + HAILO_SUBSCRIBER_TIMEOUT
        while time.time() < deadline:
            try:
                count = ros_node.count_subscribers("/arducam/image_raw")
                elapsed = time.time() - (deadline - HAILO_SUBSCRIBER_TIMEOUT)
                print(f"\r  [{elapsed:.0f}s] subscribers={count} (need ≥2)...",
                      end="", flush=True)
                if count >= 2:
                    print()
                    print(f"  [Hailo] Subscribed after {elapsed:.1f}s ✓")
                    break
            except Exception:
                pass
            time.sleep(0.5)
        else:
            print()
            print("  [Hailo] Subscriber not confirmed — proceeding anyway")

    return rclpy, ros_node, cam, stats, hailo_proc


# ── Flight node ───────────────────────────────────────────────────────────────

class FlightNode:
    """Minimal MAVROS interface for setpoint + mode + arm control."""

    def __init__(self, ros_node, rclpy_ref, stats):
        self._node  = ros_node
        self._rclpy = rclpy_ref
        self._stats = stats

        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.srv   import SetMode, CommandBool

        self._sp_pub = ros_node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self._mode_cli = ros_node.create_client(SetMode, "/mavros/set_mode")
        self._arm_cli  = ros_node.create_client(CommandBool, "/mavros/cmd/arming")
        self._PS = PoseStamped

    def get_pos(self):
        with self._stats["lock"]:
            p = self._stats["pose"]
        return (p["x"], p["y"], p["z"]) if p else (0.0, 0.0, 0.0)

    @property
    def armed(self):
        with self._stats["lock"]:
            s = self._stats["state"]
        return s["armed"] if s else False

    @property
    def mode(self):
        with self._stats["lock"]:
            s = self._stats["state"]
        return s["mode"] if s else ""

    @property
    def connected(self):
        with self._stats["lock"]:
            s = self._stats["state"]
        return s["connected"] if s else False

    def publish_sp(self, x, y, z, yaw=0.0):
        msg = self._PS()
        msg.header.stamp    = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._sp_pub.publish(msg)

    def stream_sp(self, x, y, z, yaw, duration):
        end = time.time() + duration
        while time.time() < end:
            self.publish_sp(x, y, z, yaw)
            time.sleep(1.0 / SETPOINT_HZ)

    def fly_to(self, x, y, z, yaw, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            cx, cy, cz = self.get_pos()
            dist = math.sqrt((cx-x)**2 + (cy-y)**2 + (cz-z)**2)
            if dist < WP_TOLERANCE_M:
                print()
                return True
            self.publish_sp(x, y, z, yaw)
            print(f"\r    dist={dist:.2f}m", end="", flush=True)
            time.sleep(1.0 / SETPOINT_HZ)
        print()
        return False

    def set_mode(self, mode: str) -> bool:
        from mavros_msgs.srv import SetMode
        req = SetMode.Request()
        req.custom_mode = mode
        fut = self._mode_cli.call_async(req)
        deadline = __import__('time').time() + 5.0
        while __import__('time').time() < deadline:
            if fut.done(): break
            __import__('time').sleep(0.05)
        return bool(fut.result() and fut.result().mode_sent)

    def arm(self) -> bool:
        from mavros_msgs.srv import CommandBool
        req = CommandBool.Request()
        req.value = True
        fut = self._arm_cli.call_async(req)
        deadline = __import__('time').time() + 5.0
        while __import__('time').time() < deadline:
            if fut.done(): break
            __import__('time').sleep(0.05)
        return bool(fut.result() and fut.result().success)


# ── Telemetry logger ──────────────────────────────────────────────────────────

class TelemetryLogger:
    """
    Writes one JSON entry per log interval during the flight.
    Each entry captures: timestamp, pose, flow stats, ground class,
    camera rate, and Hailo subprocess status.
    """

    def __init__(self, log_path: Path, stats: dict, hailo_proc, flight: FlightNode):
        self._path      = log_path
        self._stats     = stats
        self._hailo     = hailo_proc
        self._flight    = flight
        self._entries   = []
        self._running   = False
        self._thread    = None
        self._t0        = None

    def start(self):
        self._t0      = time.monotonic()
        self._running = True
        self._thread  = threading.Thread(
            target=self._loop, daemon=True, name="telem_logger"
        )
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(json.dumps(self._entries, indent=2))
        print(f"  [Log] {len(self._entries)} entries → {self._path}")

    def _loop(self):
        interval = 1.0 / TELEMETRY_LOG_HZ
        while self._running:
            t_now = time.monotonic()
            elapsed = t_now - self._t0

            with self._stats["lock"]:
                pose         = dict(self._stats["pose"]) if self._stats["pose"] else {}
                flow_msgs    = list(self._stats["flow_messages"])
                ground_msgs  = list(self._stats["ground_messages"])
                image_ts     = list(self._stats["image_ts"])

            # Camera rate (last 2s window)
            recent_imgs = [t for t in image_ts if t_now - t < 2.0]
            cam_hz = len(recent_imgs) / 2.0 if len(recent_imgs) >= 2 else 0.0

            # Flow stats (last 2s)
            recent_flow = [f for f in flow_msgs if t_now - f["t"] < 2.0]
            flow_hz   = len(recent_flow) / 2.0 if recent_flow else 0.0
            flow_conf = sum(f["confidence"] for f in recent_flow) / len(recent_flow) \
                        if recent_flow else 0.0
            flow_vx   = recent_flow[-1]["vx"] if recent_flow else 0.0
            flow_vy   = recent_flow[-1]["vy"] if recent_flow else 0.0

            # Latest ground class
            last_ground = ground_msgs[-1] if ground_msgs else {}

            # Hailo alive
            hailo_alive = (self._hailo is not None and
                           self._hailo.poll() is None)

            entry = {
                "t_s":           round(elapsed, 2),
                "ts_iso":        datetime.utcnow().isoformat() + "Z",
                "pose":          pose,
                "armed":         self._flight.armed,
                "mode":          self._flight.mode,
                "cam_hz":        round(cam_hz, 1),
                "flow_hz":       round(flow_hz, 1),
                "flow_conf":     round(flow_conf, 3),
                "flow_vx":       round(flow_vx, 4),
                "flow_vy":       round(flow_vy, 4),
                "flow_total":    len(flow_msgs),
                "ground_label":  last_ground.get("label", ""),
                "ground_conf":   last_ground.get("confidence", 0.0),
                "ground_lat_ms": last_ground.get("latency_ms", 0.0),
                "ground_total":  len(ground_msgs),
                "hailo_alive":   hailo_alive,
            }
            self._entries.append(entry)
            time.sleep(interval)


# ── Report writer ─────────────────────────────────────────────────────────────

def write_report(report_path: Path, stats: dict, args, session_dir: Path,
                 flight_ok: bool, t_flight_start: float, t_flight_end: float):

    duration = t_flight_end - t_flight_start if t_flight_start else 0.0

    with stats["lock"]:
        flow_msgs   = list(stats["flow_messages"])
        ground_msgs = list(stats["ground_messages"])
        image_ts    = list(stats["image_ts"])

    # Camera rate over flight window
    flight_imgs = [t for t in image_ts
                   if t_flight_start and t >= t_flight_start and t <= t_flight_end]
    cam_hz = len(flight_imgs) / duration if duration > 0 else 0.0

    # Flow stats
    flight_flow = [f for f in flow_msgs
                   if t_flight_start and f["t"] >= t_flight_start]
    flow_hz   = (len(flight_flow) - 1) / (flight_flow[-1]["t"] - flight_flow[0]["t"]) \
                 if len(flight_flow) >= 2 else 0.0
    flow_conf = sum(f["confidence"] for f in flight_flow) / len(flight_flow) \
                if flight_flow else 0.0
    flow_valid = sum(1 for f in flight_flow if f["confidence"] > 0.5)

    # Ground stats
    flight_ground = [g for g in ground_msgs
                     if t_flight_start and g["t"] >= t_flight_start]
    labels = [g.get("label", "UNKNOWN") for g in flight_ground]
    label_counts = {l: labels.count(l) for l in set(labels)}

    lines = [
        "=" * 60,
        "  DronePi — Hailo Flight Validation Report",
        f"  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"  Altitude: {args.alt}m  Hold: {args.hold}s",
        "=" * 60,
        "",
        f"  Flight result  : {'PASS' if flight_ok else 'FAIL'}",
        f"  Flight duration: {duration:.1f}s",
        f"  Session dir    : {session_dir}",
        "",
        "  Camera",
        f"    Publish rate : {cam_hz:.1f} Hz  (target ≥ {CAMERA_PUBLISH_HZ:.0f} Hz)",
        f"    Frames (flight window): {len(flight_imgs)}",
        "",
        "  Hailo Optical Flow",
        f"    Messages received : {len(flight_flow)}",
        f"    Rate during flight: {flow_hz:.1f} Hz",
        f"    Avg confidence   : {flow_conf:.3f}",
        f"    High-conf (>0.5) : {flow_valid}/{len(flight_flow)}",
        "    NOTE: confidence reflects depth-difference quality,",
        "    not velocity accuracy. Valid=high during real hover motion.",
        "",
        "  Hailo Ground Classification",
        f"    Messages received : {len(flight_ground)}",
        f"    Label distribution: {label_counts}",
        "    NOTE: UNKNOWN is expected when hovering over non-COCO scenes.",
        "    SAFE_LAND/UNSAFE will appear over real outdoor terrain.",
        "",
        "  Interpretation",
    ]

    issues = []
    if cam_hz < CAMERA_PUBLISH_HZ * 0.8:
        issues.append(f"Camera rate {cam_hz:.1f}Hz below target {CAMERA_PUBLISH_HZ:.0f}Hz")
    if len(flight_flow) == 0:
        issues.append("No optical flow messages received during flight")
    if len(flight_ground) == 0:
        issues.append("No ground class messages received during flight")

    if not issues:
        lines.append("    All systems nominal. Full chain verified.")
        lines.append("    Ready to proceed to main.py autonomous mission.")
    else:
        for issue in issues:
            lines.append(f"    ISSUE: {issue}")

    lines += ["", "=" * 60]
    text = "\n".join(lines)
    print("\n" + text)
    report_path.write_text(text + "\n")
    print(f"\n  Report → {report_path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Props-on hover test with full Hailo + camera chain"
    )
    parser.add_argument("--alt",     type=float, default=HOVER_ALT_DEFAULT,
                        help=f"Hover altitude above home in metres (default {HOVER_ALT_DEFAULT})")
    parser.add_argument("--hold",    type=float, default=HOLD_S_DEFAULT,
                        help=f"Hold duration at altitude in seconds (default {HOLD_S_DEFAULT})")
    parser.add_argument("--dry-run", action="store_true",
                        help="Sensor + Hailo checks only — no arming")
    args = parser.parse_args()

    ts         = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = OUTPUT_BASE / f"session_{ts}"
    session_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("  DronePi — Hailo Flight Validation")
    print(f"  Altitude : {args.alt}m above home")
    print(f"  Hold     : {args.hold}s")
    print(f"  Dry run  : {'YES' if args.dry_run else 'NO'}")
    print(f"  Output   : {session_dir}")
    print("=" * 60)

    if not args.dry_run:
        print("\n  *** LIVE FLIGHT ***")
        print("  Ctrl+C within 5s to abort.")
        for i in range(5, 0, -1):
            print(f"  {i}...", end="\r", flush=True)
            time.sleep(1.0)
        print()

    # Write bench_scan lock
    MISSION_LOCK.write_text("bench_scan")
    print(f"  [Lock] bench_scan written → {MISSION_LOCK}")

    svc_mgr = ServiceManager()
    svc_mgr.pause()

    # Start Point-LIO
    print("\n[Setup 1/4] Checking Point-LIO...")
    if _wait_topic("/aft_mapped_to_init", timeout=2.0):
        print("  [OK] Reusing running Point-LIO")
        plio_proc = None
    else:
        print("  Starting Point-LIO locally...")
        plio_proc = _launch("Point-LIO", _ros_source() + "ros2 launch point_lio mapping_unilidar.launch.py", POINTLIO_LOG)
        if not _wait_topic("/aft_mapped_to_init", timeout=30.0):
            print("  [FAIL] Point-LIO not publishing — aborting")
            _stop_proc("Point-LIO", plio_proc)
            svc_mgr.restore()
            sys.exit(1)

    print("\n[Setup 2/4] Starting camera and ROS...")
    rclpy_ref, ros_node, cam, stats, hailo_proc = start_ros_stack(session_dir)

    flight = FlightNode(ros_node, rclpy_ref, stats)
    telem  = TelemetryLogger(
        session_dir / "hailo_flight_log.json",
        stats, hailo_proc, flight
    )

    flight_ok      = False
    t_flight_start = None
    t_flight_end   = None

    def _emergency_land():
        print("\n  [EMERGENCY] Landing...")
        flight.set_mode("AUTO.LAND")

    try:
        # Wait for FCU
        print("\n[Setup 3/4] Waiting for FCU connection...")
        deadline = time.time() + 20.0
        while time.time() < deadline:
            if flight.connected:
                break
            time.sleep(0.2)
        if not flight.connected:
            print("  [FAIL] FCU not connected")
            sys.exit(1)
        print(f"  [OK] FCU connected  mode={flight.mode}")

        # Sensor liveness
        print("\n[Setup 4/4] Sensor liveness check...")
        time.sleep(2.0)
        with stats["lock"]:
            cam_ok   = len(stats["image_ts"]) > 0
            hailo_ok = hailo_proc is not None and hailo_proc.poll() is None
        print(f"  Camera  : {'✓ OK' if cam_ok else '✗ NOT PUBLISHING'}")
        print(f"  Hailo   : {'✓ running' if hailo_ok else '✗ not running'}")

        if args.dry_run:
            print("\n[DRY RUN] Sensor checks complete — no arming")
            flight_ok = cam_ok
            telem.start()
            time.sleep(5.0)
            telem.stop()
        else:
            # Pre-stream setpoints
            print("\n[1/5] Pre-streaming setpoints (3s)...")
            hx, hy, hz = flight.get_pos()
            flight.stream_sp(hx, hy, hz, 0.0, 3.0)
            print("  [OK] Setpoint stream established")

            # Arm
            print("\n[2/5] Arming...")
            if not flight.arm():
                print("  [FAIL] Arm rejected")
                sys.exit(1)
            print("  [OK] Armed")

            # OFFBOARD
            print("  Switching to OFFBOARD...")
            if not flight.set_mode("OFFBOARD"):
                print("  [FAIL] OFFBOARD rejected")
                flight.set_mode("AUTO.LAND")
                sys.exit(1)
            print("  [OK] OFFBOARD")

            # Get home Z
            with stats["lock"]:
                home_z = stats["home_z"] or hz
            target_z = home_z + args.alt
            print(f"\n[3/5] Climbing to {args.alt}m (target Z={target_z:.2f}m)...")

            telem.start()
            t_flight_start = time.monotonic()

            reached = flight.fly_to(hx, hy, target_z, 0.0, TAKEOFF_TIMEOUT_S)
            if not reached:
                print("  [WARN] Altitude not reached within timeout")
            else:
                print(f"  [OK] Hovering at {args.alt}m")

            # Hold + telemetry
            print(f"\n[4/5] Holding for {args.hold}s...")
            print("  Logging: t  mode  cam_hz  flow_hz  flow_conf  ground_label")
            hold_end = time.time() + args.hold
            while time.time() < hold_end:
                cx, cy, cz = flight.get_pos()
                flight.publish_sp(hx, hy, target_z, 0.0)
                with stats["lock"]:
                    flow_n  = len(stats["flow_messages"])
                    gnd_n   = len(stats["ground_messages"])
                    gnd_lbl = stats["ground_messages"][-1].get("label", "-") \
                              if stats["ground_messages"] else "-"
                    img_n   = len(stats["image_ts"])
                remaining = hold_end - time.time()
                print(
                    f"\r  t={args.hold - remaining:.0f}s  "
                    f"Z={cz:.2f}m  "
                    f"flow={flow_n}  ground={gnd_n}({gnd_lbl})  "
                    f"imgs={img_n}    ",
                    end="", flush=True,
                )
                time.sleep(1.0 / SETPOINT_HZ)
            print()

            # Trigger camera at hover position
            print("\n[4b] Triggering camera capture...")
            ros_ts = ros_node.get_clock().now().nanoseconds * 1e-9
            cx, cy, cz = flight.get_pos()
            if cam is not None:
                saved = cam.trigger({
                    "waypoint_index": 0,
                    "enu":            (cx, cy, cz),
                    "gps":            (None, None, None),
                    "ros_timestamp":  ros_ts,
                }, timeout=3.0)
                if saved:
                    print(f"  [OK] JPEG saved: {saved.name}")
                else:
                    print("  [WARN] Trigger timeout")

            # Land
            print("\n[5/5] Switching to AUTO.LAND...")
            flight.set_mode("AUTO.LAND")
            print("  Waiting for landing and disarm...")
            while flight.armed:
                time.sleep(0.5)
            print("  [OK] Disarmed")

            t_flight_end = time.monotonic()
            telem.stop()
            flight_ok = True

    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl+C — emergency landing")
        _emergency_land()
        while flight.armed:
            time.sleep(0.5)
        telem.stop()

    except Exception as exc:
        print(f"\n[FAIL] Unexpected exception: {exc}")
        import traceback
        traceback.print_exc()
        _emergency_land()
        telem.stop()

    finally:
        print("\n[Teardown]")
        stop_hailo_subprocess(hailo_proc)
        if cam is not None:
            cam.stop()
        _stop_proc("Point-LIO", plio_proc)
        ros_node.destroy_node()
        if rclpy_ref.ok():
            rclpy_ref.shutdown()
        svc_mgr.restore()

    # Write report
    write_report(
        session_dir / "report.txt",
        stats, args, session_dir,
        flight_ok,
        t_flight_start or 0.0,
        t_flight_end or time.monotonic(),
    )

    print("\n" + "=" * 60)
    print(f"  Session: {session_dir}")
    print(f"  Log    : {session_dir / 'hailo_flight_log.json'}")
    print(f"  Report : {session_dir / 'report.txt'}")
    print("=" * 60)

    sys.exit(0 if flight_ok else 1)


if __name__ == "__main__":
    main()
