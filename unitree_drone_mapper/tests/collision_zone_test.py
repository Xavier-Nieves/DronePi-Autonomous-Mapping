#!/usr/bin/env python3
"""collision_zone_test.py — Collision zone characterisation tool for DronePi.

Launches Point-LIO, collects real LiDAR point clouds, classifies every point
into the three collision zones, and exports three output files:

  raw_points.csv      — All points exactly as received from /cloud_registered
  filtered_points.csv — All points with IGNORE / OBSTACLE / CAUTION / CLEAR labels
  zones.ply           — Coloured point cloud for 3D visualisation
                        IGNORE   → grey   (128, 128, 128)
                        OBSTACLE → red    (220,  50,  50)
                        CAUTION  → yellow (255, 200,   0)
                        CLEAR    → green  ( 50, 200,  50)

All outputs saved to /mnt/ssd/collision_test/<timestamp>/

Usage
-----
  Drone must be stationary with LiDAR connected to /dev/ttyUSB0.
  Run as two phases:
    Phase 1 — Free space (no obstacles around drone) — 45 s or Ctrl+C
    Phase 2 — With obstacles placed at known distances — 45 s or Ctrl+C

  source /opt/ros/jazzy/setup.bash
  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
  python3 collision_zone_test.py

  Optional args:
    --duration 30       Override collection time (seconds, default 45)
    --output /tmp/test  Override output directory
    --no-pointlio       Skip Point-LIO launch (use if already running)

Zone Radii (from collision_monitor.py)
--------------------------------------
  IGNORE_RADIUS   = 0.70 m  Drone body + legs + drift margin
  OBSTACLE_RADIUS = 2.00 m  Prop tip + braking distance → CP_DIST in QGC
  CAUTION_RADIUS  = 3.50 m  Outer early-warning ring

Dependencies
------------
  rclpy, sensor_msgs (ROS 2 Jazzy), numpy, pandas
  Point-LIO must be installed in ~/unitree_lidar_project/RPI5/ros2_ws
"""

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# ── Zone Constants — must match collision_monitor.py ─────────────────────────

IGNORE_RADIUS   = 0.70   # m — drone legs + drift margin
OBSTACLE_RADIUS = 2.00   # m — prop tip + braking distance
CAUTION_RADIUS  = 3.50   # m — outer warning ring

# ── Zone colours for PLY (R, G, B) ───────────────────────────────────────────

COLOUR_IGNORE   = (128, 128, 128)   # grey
COLOUR_OBSTACLE = (220,  50,  50)   # red
COLOUR_CAUTION  = (255, 200,   0)   # yellow
COLOUR_CLEAR    = ( 50, 200,  50)   # green

# ── Project Paths — match main.py ─────────────────────────────────────────────

ROS_SETUP   = "/opt/ros/jazzy/setup.bash"
WS_SETUP    = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
LIDAR_PORT   = "/dev/ttyUSB0"
DEFAULT_OUT  = Path("/mnt/ssd/collision_test")
DEFAULT_DUR  = 45    # seconds
MISSION_LOCK = Path("/tmp/dronepi_mission.lock")
POINTLIO_TIMEOUT_S = 30   # Max seconds to wait for /cloud_registered

# ── Logging helpers ───────────────────────────────────────────────────────────

def ts():
    return datetime.now().strftime("%H:%M:%S")

def info(msg):  print(f"[{ts()}]  ·  {msg}", flush=True)
def ok(msg):    print(f"[{ts()}]  ✓  {msg}", flush=True)
def warn(msg):  print(f"[{ts()}]  ⚠  {msg}", flush=True)
def header(msg):
    print(f"\n{'═' * 58}", flush=True)
    print(f"  {msg}", flush=True)
    print(f"{'═' * 58}", flush=True)


# ── Watchdog Lock Helpers ─────────────────────────────────────────────────────

def write_bench_lock() -> None:
    """Write bench_scan lock so the watchdog yields for the duration of this test.

    The watchdog treats bench_scan identically to autonomous mode — it stops
    any running stack and loops passively until the lock is cleared. This
    prevents a port conflict on /dev/ttyUSB0 when this script launches
    Point-LIO directly.
    """
    MISSION_LOCK.write_text(
        json.dumps({"mode": "bench_scan",
                    "started_at": datetime.now().isoformat()}))
    info("Watchdog lock written (bench_scan) — watchdog yielding")


def clear_bench_lock() -> None:
    """Clear the bench_scan lock so the watchdog resumes normal operation."""
    if MISSION_LOCK.exists():
        try:
            data = json.loads(MISSION_LOCK.read_text())
            if data.get("mode") == "bench_scan":
                MISSION_LOCK.unlink()
                info("Watchdog lock cleared — watchdog resuming")
        except Exception:
            pass


# ── Point Cloud Collector ─────────────────────────────────────────────────────

class PointCollector:
    """Subscribes to /cloud_registered and accumulates raw XYZ points.

    Thread-safe. All collected points are stored as a flat list of
    (x, y, z, timestamp_ms) tuples for downstream processing.
    """

    def __init__(self):
        self._points: list = []      # (x, y, z, t_ms)
        self._lock   = threading.Lock()
        self._count  = 0             # total messages received
        self._node   = None
        self._sub    = None

    def start(self, node) -> None:
        """Attach to an rclpy node and begin subscribing."""
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2

        self._node = node
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub = node.create_subscription(
            PointCloud2, "/cloud_registered", self._callback, qos
        )
        info("Subscribed to /cloud_registered")

    def _callback(self, msg) -> None:
        """Extract XYZ from PointCloud2 and append to buffer."""
        try:
            fields = {f.name: f.offset for f in msg.fields}
            if not all(k in fields for k in ("x", "y", "z")):
                return

            x_off = fields["x"]
            y_off = fields["y"]
            z_off = fields["z"]
            ps    = msg.point_step
            data  = bytes(msg.data)
            n     = msg.width * msg.height
            t_ms  = int(time.time() * 1000)

            import numpy as np
            if x_off == 0 and y_off == 4 and z_off == 8:
                stride = ps // 4
                raw    = np.frombuffer(data, dtype=np.float32).reshape(-1, stride)
                xyz    = raw[:, :3].copy()
            else:
                xyz = np.empty((n, 3), dtype=np.float32)
                for i in range(n):
                    base      = i * ps
                    xyz[i, 0] = struct.unpack_from("<f", data, base + x_off)[0]
                    xyz[i, 1] = struct.unpack_from("<f", data, base + y_off)[0]
                    xyz[i, 2] = struct.unpack_from("<f", data, base + z_off)[0]

            valid = np.isfinite(xyz).all(axis=1)
            xyz   = xyz[valid]

            pts = [(float(r[0]), float(r[1]), float(r[2]), t_ms)
                   for r in xyz]

            with self._lock:
                self._points.extend(pts)
                self._count += 1

        except Exception as exc:
            warn(f"Cloud callback error: {exc}")

    @property
    def total_points(self) -> int:
        with self._lock:
            return len(self._points)

    @property
    def total_messages(self) -> int:
        with self._lock:
            return self._count

    def get_all(self) -> list:
        with self._lock:
            return list(self._points)


# ── Zone Classifier ───────────────────────────────────────────────────────────

def classify_zone(x: float, y: float) -> str:
    """Classify a point by its 2D horizontal distance from origin."""
    r = math.sqrt(x * x + y * y)
    if r < IGNORE_RADIUS:
        return "IGNORE"
    elif r < OBSTACLE_RADIUS:
        return "OBSTACLE"
    elif r < CAUTION_RADIUS:
        return "CAUTION"
    else:
        return "CLEAR"


# ── Raw CSV Export ────────────────────────────────────────────────────────────

def export_raw_csv(points: list, out_dir: Path) -> Path:
    """Export all collected points as raw_points.csv."""
    import pandas as pd
    import numpy as np

    info(f"Building raw CSV ({len(points):,} points)...")
    arr = np.array(points, dtype=np.float64)
    df  = pd.DataFrame({
        "x":          arr[:, 0],
        "y":          arr[:, 1],
        "z":          arr[:, 2],
        "range_2d":   np.sqrt(arr[:, 0]**2 + arr[:, 1]**2).round(4),
        "range_3d":   np.sqrt(arr[:, 0]**2 + arr[:, 1]**2 + arr[:, 2]**2).round(4),
        "timestamp_ms": arr[:, 3].astype(np.int64),
    })

    path = out_dir / "raw_points.csv"
    df.to_csv(path, index=False)
    ok(f"Raw CSV saved → {path}  ({len(df):,} rows)")
    return path


# ── Filtered + Classified CSV Export ─────────────────────────────────────────

def export_filtered_csv(points: list, out_dir: Path) -> Path:
    """Export all points with zone labels as filtered_points.csv.

    Includes IGNORE-labelled points so the full picture is preserved.
    A separate 'filtered' view (Zone 1 removed) can be obtained by
    filtering rows where zone != 'IGNORE'.
    """
    import pandas as pd
    import numpy as np

    info(f"Classifying {len(points):,} points into zones...")
    arr    = np.array(points, dtype=np.float64)
    r2d    = np.sqrt(arr[:, 0]**2 + arr[:, 1]**2)
    zones  = []
    for r in r2d:
        if r < IGNORE_RADIUS:
            zones.append("IGNORE")
        elif r < OBSTACLE_RADIUS:
            zones.append("OBSTACLE")
        elif r < CAUTION_RADIUS:
            zones.append("CAUTION")
        else:
            zones.append("CLEAR")

    df = pd.DataFrame({
        "x":            arr[:, 0].round(4),
        "y":            arr[:, 1].round(4),
        "z":            arr[:, 2].round(4),
        "range_2d":     r2d.round(4),
        "zone":         zones,
        "timestamp_ms": arr[:, 3].astype(np.int64),
    })

    # Zone summary
    counts = df["zone"].value_counts()
    header("Zone Classification Summary")
    for zone, colour in [
        ("IGNORE",   "grey   — drone body/legs"),
        ("OBSTACLE", "red    — imminent danger"),
        ("CAUTION",  "yellow — slow down"),
        ("CLEAR",    "green  — safe"),
    ]:
        n   = counts.get(zone, 0)
        pct = 100 * n / len(df) if len(df) else 0
        bar = "█" * int(pct / 2)
        print(f"  {zone:<10}  {n:>8,} pts  {pct:5.1f}%  {bar}", flush=True)

    path = out_dir / "filtered_points.csv"
    df.to_csv(path, index=False)
    ok(f"Filtered CSV saved → {path}  ({len(df):,} rows)")
    return path


# ── PLY Export ────────────────────────────────────────────────────────────────

def export_ply(points: list, out_dir: Path) -> Path:
    """Export colour-coded point cloud as zones.ply.

    Format: ASCII PLY with x y z red green blue per point.
    Colours:
      IGNORE   → grey   (128, 128, 128)
      OBSTACLE → red    (220,  50,  50)
      CAUTION  → yellow (255, 200,   0)
      CLEAR    → green  ( 50, 200,  50)
    """
    import numpy as np

    info(f"Writing PLY file ({len(points):,} points)...")

    colour_map = {
        "IGNORE":   COLOUR_IGNORE,
        "OBSTACLE": COLOUR_OBSTACLE,
        "CAUTION":  COLOUR_CAUTION,
        "CLEAR":    COLOUR_CLEAR,
    }

    arr  = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
    r2d  = np.sqrt(arr[:, 0]**2 + arr[:, 1]**2)

    # Classify each point
    colours = []
    for r in r2d:
        if r < IGNORE_RADIUS:
            colours.append(COLOUR_IGNORE)
        elif r < OBSTACLE_RADIUS:
            colours.append(COLOUR_OBSTACLE)
        elif r < CAUTION_RADIUS:
            colours.append(COLOUR_CAUTION)
        else:
            colours.append(COLOUR_CLEAR)

    path = out_dir / "zones_cloud.ply"
    with open(path, "w") as f:
        # PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"comment DronePi collision zone characterisation\n")
        f.write(f"comment IGNORE={IGNORE_RADIUS}m "
                f"OBSTACLE={OBSTACLE_RADIUS}m "
                f"CAUTION={CAUTION_RADIUS}m\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")

        # Data rows
        for i, (x, y, z) in enumerate(arr):
            r, g, b = colours[i]
            f.write(f"{x:.4f} {y:.4f} {z:.4f} {r} {g} {b}\n")

    ok(f"PLY saved → {path}")
    return path


# ── Point-LIO Launcher ────────────────────────────────────────────────────────

class PointLIOLauncher:
    """Launches and manages the Point-LIO ROS 2 process."""

    def __init__(self):
        self._proc = None

    def start(self) -> bool:
        """Launch Point-LIO. Returns True if started successfully."""
        if not Path(LAUNCH_FILE).exists():
            warn(f"Launch file not found: {LAUNCH_FILE}")
            return False
        if not Path(LIDAR_PORT).exists():
            warn(f"LiDAR port not found: {LIDAR_PORT}")
            return False

        cmd = (
            f"source {ROS_SETUP} && "
            f"source {WS_SETUP} && "
            f"ros2 launch {LAUNCH_FILE} rviz:=false port:={LIDAR_PORT}"
        )
        info("Launching Point-LIO...")
        self._proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        info(f"Point-LIO PID {self._proc.pid} — waiting for /cloud_registered...")

        # Wait for /cloud_registered to publish before returning.
        # This replaces a fixed sleep and ensures the collector node never
        # starts spinning before SLAM output is actually available.
        if not self._wait_for_cloud(timeout=POINTLIO_TIMEOUT_S):
            if self._proc.poll() is not None:
                warn("Point-LIO exited before publishing — check LiDAR connection")
                return False
            warn(f"/cloud_registered not seen after {POINTLIO_TIMEOUT_S}s — proceeding anyway")

        if self._proc.poll() is not None:
            warn("Point-LIO exited immediately — check LiDAR connection")
            return False
        ok("Point-LIO running and publishing /cloud_registered")
        return True

    def _wait_for_cloud(self, timeout: float) -> bool:
        """Block until /cloud_registered publishes or timeout expires.

        Runs a temporary rclpy context in a background thread. This is safe
        here because PointLIOLauncher.start() is called before the main
        rclpy.init() in main(), so no context conflict exists at this point.
        """
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2

        received  = threading.Event()
        last_tick = [-1]

        def _spin():
            rclpy.init()
            node = Node("pointlio_readiness_check")
            qos  = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            node.create_subscription(
                PointCloud2, "/cloud_registered",
                lambda msg: received.set(), qos
            )
            while not received.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        spin_thread = threading.Thread(target=_spin, daemon=True)
        spin_thread.start()

        t0 = time.time()
        while not received.is_set():
            elapsed = time.time() - t0
            if elapsed >= timeout:
                return False
            tick = int(elapsed) // 5
            if tick != last_tick[0] and int(elapsed) > 0:
                info(f"Waiting for /cloud_registered... ({elapsed:.0f}s / {timeout:.0f}s)")
                last_tick[0] = tick
            # Also abort early if Point-LIO process died
            if self._proc.poll() is not None:
                return False
            time.sleep(0.5)

        info(f"/cloud_registered confirmed ({time.time() - t0:.1f}s after launch)")
        return True

    def stop(self) -> None:
        if self._proc and self._proc.poll() is None:
            info("Stopping Point-LIO...")
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            ok("Point-LIO stopped")


# ── Live Status Printer ───────────────────────────────────────────────────────

def live_status(collector: PointCollector, stop_event: threading.Event,
                duration: int) -> None:
    """Print collection progress every 5 seconds until stop_event is set."""
    t0 = time.time()
    while not stop_event.is_set():
        elapsed  = time.time() - t0
        pts      = collector.total_points
        msgs     = collector.total_messages
        remain   = max(0, duration - elapsed)

        if pts > 0:
            # Quick zone sample for live display (sample last 5000 points)
            raw  = collector.get_all()
            samp = raw[-5000:] if len(raw) > 5000 else raw
            zones = {"IGNORE": 0, "OBSTACLE": 0, "CAUTION": 0, "CLEAR": 0}
            for p in samp:
                zones[classify_zone(p[0], p[1])] += 1
            total = len(samp)
            pct = {k: 100 * v / total for k, v in zones.items()}
            print(
                f"[{ts()}]  "
                f"{elapsed:5.0f}s / {duration}s  |  "
                f"{pts:>8,} pts  {msgs} msgs  |  "
                f"🔴 {pct['OBSTACLE']:.0f}%  "
                f"🟡 {pct['CAUTION']:.0f}%  "
                f"🟢 {pct['CLEAR']:.0f}%  "
                f"⬜ {pct['IGNORE']:.0f}%  "
                f"| ⏱ {remain:.0f}s left",
                flush=True,
            )
        else:
            info(f"{elapsed:.0f}s — waiting for first cloud message...")

        stop_event.wait(timeout=5)


# ── Metadata Export ───────────────────────────────────────────────────────────

def export_metadata(points: list, out_dir: Path) -> None:
    """Write metadata.json so the viewer's info panel populates correctly.

    The viewer reads metadata.json from the same folder as the PLY and displays
    duration and processed_at in the Scan Info panel (lines 1843-1849 of viewer).
    """
    import json
    import numpy as np

    arr = np.array(points, dtype=np.float64)
    t0  = int(arr[:, 3].min())
    t1  = int(arr[:, 3].max())
    dur = (t1 - t0) / 1000.0

    # Count zones
    r2d   = np.sqrt(arr[:, 0]**2 + arr[:, 1]**2)
    n_ign = int(np.sum(r2d < IGNORE_RADIUS))
    n_obs = int(np.sum((r2d >= IGNORE_RADIUS) & (r2d < OBSTACLE_RADIUS)))
    n_cau = int(np.sum((r2d >= OBSTACLE_RADIUS) & (r2d < CAUTION_RADIUS)))
    n_clr = int(np.sum(r2d >= CAUTION_RADIUS))

    meta = {
        "duration":     f"{dur:.1f}s",
        "processed_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "point_count":  len(points),
        "zone_radii": {
            "ignore_m":   IGNORE_RADIUS,
            "obstacle_m": OBSTACLE_RADIUS,
            "caution_m":  CAUTION_RADIUS,
        },
        "zone_counts": {
            "IGNORE":   n_ign,
            "OBSTACLE": n_obs,
            "CAUTION":  n_cau,
            "CLEAR":    n_clr,
        },
        "colour_key": {
            "IGNORE":   "grey   (128,128,128)  — drone body/legs < 0.70m",
            "OBSTACLE": "red    (220, 50, 50)  — imminent danger 0.70–2.00m",
            "CAUTION":  "yellow (255,200,  0)  — slow down zone  2.00–3.50m",
            "CLEAR":    "green  ( 50,200, 50)  — safe            > 3.50m",
        },
    }

    path = out_dir / "metadata.json"
    with open(path, "w") as f:
        json.dump(meta, f, indent=2)
    ok(f"Metadata saved → {path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="DronePi collision zone characterisation test")
    parser.add_argument("--duration", type=int, default=DEFAULT_DUR,
                        help=f"Collection duration in seconds (default {DEFAULT_DUR})")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT,
                        help="Output directory root (default /mnt/ssd/collision_test)")
    parser.add_argument("--no-pointlio", action="store_true",
                        help="Skip Point-LIO launch (use if already running externally)")
    args = parser.parse_args()

    # ── Setup output directory ────────────────────────────────────────────────
    run_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.output / run_tag
    out_dir.mkdir(parents=True, exist_ok=True)

    header(f"DronePi Collision Zone Characterisation Test")
    info(f"Output directory : {out_dir}")
    info(f"Collection time  : up to {args.duration} seconds (Ctrl+C to stop early)")
    info(f"Zone radii       : ignore={IGNORE_RADIUS}m  "
         f"obstacle={OBSTACLE_RADIUS}m  caution={CAUTION_RADIUS}m")
    print(flush=True)
    info("PHASE GUIDANCE:")
    info("  Phase 1 — Clear area, no obstacles. Let run for ~20 s, then Ctrl+C")
    info("  Phase 2 — Place objects at 1.0 m, 2.5 m, 4.0 m from drone centre")
    info("            Run again with --output pointing to same folder or new folder")
    print(flush=True)

    # Write bench_scan lock so the watchdog yields while this test owns
    # the Point-LIO process and /dev/ttyUSB0 port.
    write_bench_lock()

    # ── Launch Point-LIO ──────────────────────────────────────────────────────
    launcher = PointLIOLauncher()
    if not args.no_pointlio:
        if not launcher.start():
            warn("Point-LIO failed to start. Exiting.")
            clear_bench_lock()
            sys.exit(1)
    else:
        info("Skipping Point-LIO launch (--no-pointlio flag set)")

    # ── Start ROS 2 node ──────────────────────────────────────────────────────
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        warn("rclpy not found — source ROS 2 setup before running this script")
        launcher.stop()
        sys.exit(1)

    rclpy.init()
    node      = Node("collision_zone_test")
    collector = PointCollector()
    collector.start(node)

    stop_event = threading.Event()

    # ── Ctrl+C / timeout handler ──────────────────────────────────────────────
    def _stop(sig=None, frame=None):
        if not stop_event.is_set():
            print(flush=True)
            info("Stop signal received — finishing collection...")
        stop_event.set()

    signal.signal(signal.SIGINT, _stop)

    # ── Live status thread ────────────────────────────────────────────────────
    status_thread = threading.Thread(
        target=live_status,
        args=(collector, stop_event, args.duration),
        daemon=True,
    )
    status_thread.start()

    # ── Spin until timeout or Ctrl+C ─────────────────────────────────────────
    t0 = time.time()
    header("Collecting — press Ctrl+C to stop early")
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 >= args.duration:
            info(f"Duration limit ({args.duration}s) reached — stopping")
            stop_event.set()

    stop_event.set()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    clear_bench_lock()
    launcher.stop()

    # ── Process and export ────────────────────────────────────────────────────
    points = collector.get_all()
    header(f"Collection complete — {len(points):,} points from "
           f"{collector.total_messages} cloud messages")

    if len(points) == 0:
        warn("No points collected — check LiDAR connection and Point-LIO output")
        clear_bench_lock()
        sys.exit(1)

    raw_csv      = export_raw_csv(points, out_dir)
    filtered_csv = export_filtered_csv(points, out_dir)
    ply_path     = export_ply(points, out_dir)
    export_metadata(points, out_dir)

    # ── Final summary ─────────────────────────────────────────────────────────
    header("Output Files")
    print(f"  Raw CSV      : {raw_csv}", flush=True)
    print(f"  Filtered CSV : {filtered_csv}", flush=True)
    print(f"  PLY cloud    : {ply_path}", flush=True)
    print(f"  Metadata     : {out_dir / 'metadata.json'}", flush=True)
    print(flush=True)
    info("PLY colour key:")
    info("  Grey   = IGNORE   — drone body/legs (< 0.70 m)")
    info("  Red    = OBSTACLE — imminent danger (0.70 – 2.00 m)")
    info("  Yellow = CAUTION  — slow down zone  (2.00 – 3.50 m)")
    info("  Green  = CLEAR    — safe            (> 3.50 m)")
    print(flush=True)
    info("Open viewer: drag the output folder into meshview.html")
    info("The PLY appears under ☁️ Point Cloud in the file selector")
    info("Use 📏 Measure tool in viewer to verify zone distances are correct")


if __name__ == "__main__":
    main()
