#!/usr/bin/env python3
"""Live LiDAR + IMX219 camera simultaneous capture → textured mesh.

CAPTURE PHASE (live):
  Starts LiDAR + Point-LIO SLAM, subscribes to /Odometry and /cloud_registered,
  opens the IMX219 via OpenCV/V4L2, and captures frames at regular intervals
  while recording the matching SLAM pose for each frame.

PROCESS PHASE (triggered by Ctrl+C):
  Saves accumulated point cloud as binary PCD and trajectory.json, then runs
  perspective texture projection in the open3d_env Python 3.11 subprocess:
    cloud clean → Poisson mesh → perspective projection → textured OBJ

Prerequisites:
  1. IMX219 camera connected to Pi CSI port and visible as /dev/video0
     (or pass --device N)
  2. Unitree L1 LiDAR on /dev/ttyUSB0 (or pass --port /dev/ttyUSBX)
  3. ROS 2 Jazzy + px4_msgs workspace sourced:
       source /opt/ros/jazzy/setup.bash
       source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash
  4. ~/open3d_env/ Python 3.11 with open3d + opencv-python-headless

Optional Pixhawk telemetry (read-only, --with-mavlink):
  Pixhawk 6X on /dev/ttyACM0 via USB serial (MAVLink)

Run (from unitree_drone_mapper/):
  # Full capture + post-process
  python3 -m tests.test_texture_live

  # Custom interval and camera device
  python3 -m tests.test_texture_live --interval 5 --device 0

  # Skip SLAM launch (already running), no RViz
  python3 -m tests.test_texture_live --no-slam --no-rviz

  # Pipeline test with dummy camera frames (no physical camera needed)
  python3 -m tests.test_texture_live --no-camera

  # Only capture, skip post-processing (run pipeline separately later)
  python3 -m tests.test_texture_live --no-process
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# ── paths ──────────────────────────────────────────────────────────────────────
_THIS_DIR  = Path(__file__).resolve().parent
MAPPER_DIR = _THIS_DIR.parent
PROJ_ROOT  = MAPPER_DIR.parent

ROS_SETUP   = "/opt/ros/jazzy/setup.bash"
WS_SETUP    = str(PROJ_ROOT / "RPI5/ros2_ws/install/setup.bash")
LAUNCH_FILE = str(PROJ_ROOT / "RPI5/ros2_ws/src/point_lio_ros2/launch/"
                              "combined_lidar_mapping.launch.py")
OPEN3D_PY   = str(Path.home() / "open3d_env/bin/python3")
CONFIG_YAML = str(MAPPER_DIR / "config.yaml")
CAL_YAML    = str(MAPPER_DIR / "config/camera_calibration.yaml")

sys.path.insert(0, str(MAPPER_DIR))

# ── defaults ───────────────────────────────────────────────────────────────────
LIDAR_PORT   = "/dev/ttyUSB0"
CAMERA_DEV   = 0          # /dev/video0
INTERVAL_SEC = 10.0       # seconds between captures
SLAM_TIMEOUT = 25.0       # seconds to wait for first /Odometry message
CAM_WIDTH    = 1920
CAM_HEIGHT   = 1080


# ══════════════════════════════════════════════════════════════════════════════
#  PointCloud2 parse — vectorized, no open3d
# ══════════════════════════════════════════════════════════════════════════════

def parse_cloud2(msg) -> np.ndarray:
    """Return (N,3) float64 XYZ array from a ROS 2 PointCloud2 message."""
    offs = {f.name: f.offset for f in msg.fields}
    step = msg.point_step
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = len(data) // step
    if n == 0:
        return np.zeros((0, 3), dtype=np.float64)
    raw = data.reshape(n, step)

    def col(off: int) -> np.ndarray:
        # Slice 4 bytes per row, reinterpret as float32
        return raw[:, off:off + 4].copy().view(np.float32).reshape(n)

    pts = np.column_stack([
        col(offs["x"]), col(offs["y"]), col(offs["z"])
    ]).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


# ══════════════════════════════════════════════════════════════════════════════
#  Binary PCD writer — no open3d dependency
# ══════════════════════════════════════════════════════════════════════════════

def save_pcd_binary(points: np.ndarray, filepath: str) -> None:
    """Write (N,3) float64 array to a binary PCD v0.7 file."""
    pts32 = points.astype(np.float32)
    n = len(pts32)
    Path(filepath).parent.mkdir(parents=True, exist_ok=True)
    with open(filepath, "wb") as f:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {n}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {n}\n"
            "DATA binary\n"
        )
        f.write(header.encode("ascii"))
        f.write(pts32.tobytes())
    print(f"  Saved {n:,} pts → {filepath}")


# ══════════════════════════════════════════════════════════════════════════════
#  ROS subscriber node (system Python3, ROS sourced)
# ══════════════════════════════════════════════════════════════════════════════

class TextureCaptureNode:
    """Thread-safe ROS 2 node — subscribes /Odometry + /cloud_registered."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2

        rclpy.init()
        self._node = Node("texture_capture")
        self._lock  = threading.Lock()
        self._pose: dict | None = None
        self._chunks: list[np.ndarray] = []

        # BEST_EFFORT matches Point-LIO's C++ publisher QoS (sensor data profile).
        # Default RELIABLE would silently receive nothing — most common ROS 2 gotcha.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Point-LIO publishes odometry on /aft_mapped_to_init (nav_msgs/Odometry)
        # Confirmed from laserMapping.cpp — there is no /Odometry topic.
        self._node.create_subscription(Odometry, "/aft_mapped_to_init",
                                       self._odom_cb, sensor_qos)
        self._node.create_subscription(PointCloud2, "/cloud_registered",
                                       self._cloud_cb, sensor_qos)

        def _spin_safe():
            try:
                rclpy.spin(self._node)
            except Exception:
                pass  # Normal on rclpy.shutdown() — not an error

        self._spin = threading.Thread(target=_spin_safe, daemon=True)
        self._spin.start()
        self._node.get_logger().info("TextureCaptureNode ready")

    def _odom_cb(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            # pipeline expects position=[x,y,z]  orientation=[w,x,y,z]
            self._pose = {
                "timestamp": ts,
                "position":    [p.x, p.y, p.z],
                "orientation": [q.w, q.x, q.y, q.z],
            }

    def _cloud_cb(self, msg) -> None:
        pts = parse_cloud2(msg)
        if len(pts):
            with self._lock:
                self._chunks.append(pts)

    def get_pose(self) -> dict | None:
        with self._lock:
            return dict(self._pose) if self._pose else None

    def get_cloud(self) -> np.ndarray:
        with self._lock:
            return np.vstack(self._chunks) if self._chunks else np.zeros((0, 3))

    def cloud_size(self) -> int:
        with self._lock:
            return sum(len(c) for c in self._chunks)

    def shutdown(self) -> None:
        import rclpy
        rclpy.shutdown()
        self._spin.join(timeout=3.0)


# ══════════════════════════════════════════════════════════════════════════════
#  SLAM subprocess helpers
# ══════════════════════════════════════════════════════════════════════════════

def start_slam(lidar_port: str, rviz: bool) -> subprocess.Popen:
    if not os.path.isfile(LAUNCH_FILE):
        print(f"  [FAIL] Launch file not found: {LAUNCH_FILE}")
        print("         Build the workspace first: colcon build in RPI5/ros2_ws")
        return None
    rviz_flag = "true" if rviz else "false"
    cmd = (
        f"source {ROS_SETUP} && source {WS_SETUP} && "
        f"ros2 launch {LAUNCH_FILE} rviz:={rviz_flag} port:={lidar_port}"
    )
    # No stdout pipe — SLAM output goes to the terminal so errors are visible
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        preexec_fn=os.setsid,
    )
    print(f"  SLAM launched  PID {proc.pid}")
    return proc


def kill_proc(proc: subprocess.Popen | None, label: str = "process") -> None:
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=8)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        print(f"  {label} stopped")


# ══════════════════════════════════════════════════════════════════════════════
#  IMX219 camera helpers (OpenCV / V4L2)
# ══════════════════════════════════════════════════════════════════════════════

def open_camera(device: int, width: int, height: int):
    """Open IMX219 via V4L2 and return VideoCapture, or None on failure."""
    try:
        import cv2
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        if not cap.isOpened():
            # fallback: auto-detect backend
            cap = cv2.VideoCapture(device)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                print(f"  IMX219 opened  /dev/video{device}  {w}×{h}")
                return cap
        print(f"  [WARN] Camera /dev/video{device} opened but no frame returned")
        cap.release()
    except ImportError:
        print("  [WARN] cv2 not available in this Python — install opencv-python")
    except Exception as e:
        print(f"  [WARN] Camera open error: {e}")
    return None


def capture_frame(cap, images_dir: str, index: int) -> str | None:
    """Capture one JPEG frame from the IMX219. Returns local path or None."""
    import cv2
    ret, frame = cap.read()
    if not ret or frame is None:
        print("  [WARN] Camera read failed")
        return None
    filepath = os.path.join(images_dir, f"photo_{index:04d}.jpg")
    cv2.imwrite(filepath, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    h, w = frame.shape[:2]
    size_kb = os.path.getsize(filepath) // 1024
    print(f"  Frame {index:04d} → {filepath}  ({w}×{h}, {size_kb} KB)")
    return filepath


def make_dummy_frame(images_dir: str, index: int,
                     width: int = CAM_WIDTH, height: int = CAM_HEIGHT) -> str:
    """Write a gray dummy JPEG for pipeline testing (no camera needed)."""
    import cv2
    filepath = os.path.join(images_dir, f"photo_{index:04d}.jpg")
    img = np.full((height, width, 3), 128, dtype=np.uint8)
    # Add a simple pattern so the texture is non-uniform and easier to verify
    for y in range(0, height, 80):
        img[y:y + 2, :] = 180
    for x in range(0, width, 80):
        img[:, x:x + 2] = 200
    cv2.imwrite(filepath, img, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(f"  Dummy frame {index:04d} → {filepath}  ({width}×{height})")
    return filepath


# ══════════════════════════════════════════════════════════════════════════════
#  Perspective texture pipeline — runs inside open3d_env Python 3.11
# ══════════════════════════════════════════════════════════════════════════════

_PIPELINE_SCRIPT = r"""
# Perspective texture pipeline for IMX219 (pinhole camera)
# Executed in open3d_env Python 3.11 via subprocess
import sys, json, os
import numpy as np
import cv2
import open3d as o3d
from pathlib import Path

MAPPER_DIR   = sys.argv[1]
SESSION_DIR  = sys.argv[2]
CAL_YAML     = sys.argv[3]

sys.path.insert(0, MAPPER_DIR)
import yaml

# ── load calibration ──────────────────────────────────────────────────────────
with open(CAL_YAML) as f:
    cal = yaml.safe_load(f)

cam  = cal["camera"]
W, H = cam["image_width"], cam["image_height"]
fx   = float(cam["fx"])
fy   = float(cam["fy"])
cx   = float(cam["cx"])
cy   = float(cam["cy"])

ext = cal["extrinsic"]
t   = np.array(ext["translation"], dtype=np.float64)
rpy = ext.get("rotation_rpy", [0, 0, 0])

# Build LiDAR→camera extrinsic 4×4
from utils.transforms import euler_to_rotation_matrix, build_transform_matrix
R_ext = euler_to_rotation_matrix(*rpy)
T_lidar_cam = build_transform_matrix(t, R_ext)   # LiDAR → camera

print(f"  Camera: {W}×{H}  fx={fx:.1f} fy={fy:.1f}  cx={cx:.1f} cy={cy:.1f}")

# ── paths ─────────────────────────────────────────────────────────────────────
proc_dir   = Path(SESSION_DIR) / "processed"
raw_dir    = Path(SESSION_DIR) / "raw"
images_dir = raw_dir / "images"
meta_dir   = raw_dir / "metadata"
model_dir  = proc_dir / "model_textured"
model_dir.mkdir(parents=True, exist_ok=True)

# ── load raw cloud ────────────────────────────────────────────────────────────
cloud_path = proc_dir / "cloud_raw.pcd"
if not cloud_path.exists():
    print(f"  [FAIL] cloud_raw.pcd not found: {cloud_path}")
    sys.exit(1)

print(f"  Loading cloud: {cloud_path}")
pcd = o3d.io.read_point_cloud(str(cloud_path))
print(f"  Raw cloud: {len(pcd.points):,} pts")

# ── Step 1: clean ─────────────────────────────────────────────────────────────
print("  Step 1/4: Cleaning cloud...")
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
pcd = pcd.voxel_down_sample(voxel_size=0.02)
print(f"  Clean: {len(pcd.points):,} pts")
o3d.io.write_point_cloud(str(proc_dir / "cloud_clean.pcd"), pcd)

# ── Step 2: mesh ──────────────────────────────────────────────────────────────
print("  Step 2/4: Estimating normals + Poisson mesh...")
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    radius=0.1, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(k=10)

mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd, depth=9, width=0, scale=1.1, linear_fit=False)
# Crop low-density artefacts
dens_arr = np.asarray(densities, dtype=np.float64)
thr = np.percentile(dens_arr, 5)
keep = dens_arr > thr
mesh = mesh.select_by_index(np.where(keep)[0])
mesh.compute_vertex_normals()
print(f"  Mesh: {len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris")
o3d.io.write_triangle_mesh(str(proc_dir / "mesh.ply"), mesh)

# ── Step 3: perspective texture projection ────────────────────────────────────
print("  Step 3/4: Perspective texture projection...")

traj_path = meta_dir / "trajectory.json"
if not traj_path.exists():
    print("  [WARN] No trajectory.json — saving untextured mesh")
    o3d.io.write_triangle_mesh(str(model_dir / "model.obj"), mesh)
    sys.exit(0)

with open(traj_path) as f:
    traj = json.load(f)["poses"]

image_files = sorted(images_dir.glob("*.jpg")) + sorted(images_dir.glob("*.png"))
image_files = [str(p) for p in image_files]

if not image_files or not traj:
    print("  [WARN] No images or poses — saving untextured mesh")
    o3d.io.write_triangle_mesh(str(model_dir / "model.obj"), mesh)
    sys.exit(0)

# Match poses 1:1 with images (trajectory has exactly one entry per photo)
n_pairs = min(len(traj), len(image_files))
print(f"  Projecting {n_pairs} image(s) onto {len(mesh.vertices):,} vertices...")

verts  = np.asarray(mesh.vertices, dtype=np.float64)          # (V,3)
colors = np.zeros((len(verts), 3), dtype=np.float64)
weights = np.zeros(len(verts), dtype=np.float64)

from utils.transforms import quaternion_to_rotation_matrix, build_transform_matrix

for i in range(n_pairs):
    pose  = traj[i]
    img   = cv2.imread(image_files[i])
    if img is None:
        continue
    img   = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    ih, iw = img.shape[:2]

    # Build world→LiDAR 4×4 from SLAM pose
    pos = np.array(pose["position"], dtype=np.float64)
    q   = np.array(pose["orientation"], dtype=np.float64)  # [w,x,y,z]
    R_world_lidar = quaternion_to_rotation_matrix(q)       # 3×3
    T_world_lidar = build_transform_matrix(pos, R_world_lidar)

    # Camera in world frame: T_world_lidar @ T_lidar_cam
    T_world_cam = T_world_lidar @ T_lidar_cam
    cam_pos = T_world_cam[:3, 3]      # camera origin in world
    R_cam   = T_world_cam[:3, :3]     # rotation world→cam columns

    # Transform all vertices into camera frame
    v_rel = verts - cam_pos                       # (V,3)
    v_cam = (R_cam.T @ v_rel.T).T                # (V,3) in camera frame

    # Keep only points in front of camera
    in_front = v_cam[:, 2] > 0.1
    Z = np.where(in_front, v_cam[:, 2], 1.0)

    # Perspective projection
    u = fx * v_cam[:, 0] / Z + cx
    v_proj = fy * v_cam[:, 1] / Z + cy

    in_frame = (
        in_front &
        (u >= 0) & (u < iw - 1) &
        (v_proj >= 0) & (v_proj < ih - 1)
    )
    idx = np.where(in_frame)[0]
    if not len(idx):
        print(f"    Frame {i}: 0 visible vertices (check extrinsic calibration)")
        continue

    # Bilinear sampling — distance-weighted contribution
    ui = u[idx].astype(int)
    vi = v_proj[idx].astype(int)
    sampled = img[vi, ui]                     # (K,3)
    dist = np.linalg.norm(v_cam[idx], axis=1)
    w    = 1.0 / np.maximum(dist, 0.01)

    colors[idx]  += sampled * w[:, np.newaxis]
    weights[idx] += w

    visible = np.count_nonzero(in_frame)
    print(f"    Frame {i:03d}: {visible:,} vertices projected, "
          f"cam=({cam_pos[0]:.2f},{cam_pos[1]:.2f},{cam_pos[2]:.2f})")

# Normalise accumulated colors
mask = weights > 0
if mask.any():
    colors[mask] /= weights[mask, np.newaxis]
colors = np.clip(colors, 0.0, 1.0)
colors[~mask] = 0.5   # gray for unseen vertices
mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

# ── Step 4: save ──────────────────────────────────────────────────────────────
print("  Step 4/4: Saving textured mesh...")
obj_path = str(model_dir / "model.obj")
ply_path = str(model_dir / "model.ply")
o3d.io.write_triangle_mesh(obj_path, mesh)
o3d.io.write_triangle_mesh(ply_path, mesh)
print(f"  OBJ: {obj_path}")
print(f"  PLY: {ply_path}")
print("  Pipeline complete.")
"""


def run_pipeline(session_dir: str) -> bool:
    """Run perspective texture pipeline in open3d_env Python 3.11 subprocess."""
    if not Path(OPEN3D_PY).exists():
        print(f"\n  [WARN] open3d_env not found at {OPEN3D_PY}")
        print("         Install: python3.11 -m venv ~/open3d_env && "
              "~/open3d_env/bin/pip install open3d opencv-python-headless")
        return False

    print("\n" + "=" * 60)
    print("Post-processing  (open3d_env Python 3.11)")
    print("=" * 60)

    result = subprocess.run(
        [OPEN3D_PY, "-c", _PIPELINE_SCRIPT,
         str(MAPPER_DIR), str(session_dir), CAL_YAML],
        cwd=str(MAPPER_DIR),
    )
    return result.returncode == 0


def open_result(session_dir: str) -> None:
    model_dir = Path(session_dir) / "processed/model_textured"
    models = list(model_dir.glob("*.obj")) + list(model_dir.glob("*.ply"))
    if not models:
        return
    path = str(models[0])
    for cc in ["/usr/bin/cloudcompare", "/usr/local/bin/cloudcompare",
               "/usr/bin/CloudCompare"]:
        if os.path.exists(cc):
            subprocess.Popen([cc, path])
            print(f"\n  Opened in CloudCompare: {path}")
            return
    print(f"\n  Textured model saved to: {path}")
    print(f"  View with: cloudcompare {path}")


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    ap = argparse.ArgumentParser(
        description="IMX219 + LiDAR simultaneous capture → textured mesh"
    )
    ap.add_argument("--interval", type=float, default=INTERVAL_SEC,
                    help="Seconds between captures (default %(default)s)")
    ap.add_argument("--port", default=LIDAR_PORT,
                    help="LiDAR serial port (default %(default)s)")
    ap.add_argument("--device", type=int, default=CAMERA_DEV,
                    help="Camera V4L2 device index (default %(default)s = /dev/video0)")
    ap.add_argument("--width",  type=int, default=CAM_WIDTH)
    ap.add_argument("--height", type=int, default=CAM_HEIGHT)
    ap.add_argument("--no-slam",    action="store_true",
                    help="Skip SLAM launch (assume it is already running)")
    ap.add_argument("--no-rviz",    action="store_true",
                    help="Suppress RViz")
    ap.add_argument("--no-camera",  action="store_true",
                    help="Use dummy gray frames instead of a real camera")
    ap.add_argument("--no-process", action="store_true",
                    help="Skip post-processing — save data only")
    args = ap.parse_args()

    # ── session dir ───────────────────────────────────────────────────────────
    ts          = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = Path(MAPPER_DIR) / "data/flights" / f"texture_{ts}"
    images_dir  = session_dir / "raw/images"
    meta_dir    = session_dir / "raw/metadata"
    proc_dir    = session_dir / "processed"
    for d in [images_dir, meta_dir, proc_dir]:
        d.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("IMX219 + LiDAR  ——  Live Texture Capture Test")
    print("=" * 60)
    print(f"Session  : {session_dir}")
    print(f"Interval : {args.interval}s")
    print(f"Camera   : {'dummy' if args.no_camera else f'/dev/video{args.device}'}")
    print(f"LiDAR    : {args.port}")
    print()

    # ── check rclpy ───────────────────────────────────────────────────────────
    try:
        __import__("rclpy")   # check only — actual imports deferred to TextureCaptureNode
    except ImportError:
        print("[FAIL] rclpy not found.  Source ROS 2 first:")
        print("  source /opt/ros/jazzy/setup.bash")
        print("  source ~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash")
        sys.exit(1)

    # ── start SLAM ────────────────────────────────────────────────────────────
    slam_proc = None
    if not args.no_slam:
        print("[1/5] Starting SLAM...")
        slam_proc = start_slam(args.port, rviz=not args.no_rviz)
        print(f"  Waiting up to {SLAM_TIMEOUT:.0f}s for /Odometry...")
    else:
        print("[1/5] SLAM launch skipped (--no-slam)")

    # ── ROS node ──────────────────────────────────────────────────────────────
    print("[2/5] Starting ROS subscriber node...")
    ros = TextureCaptureNode()

    deadline = time.time() + SLAM_TIMEOUT
    while time.time() < deadline:
        if ros.get_pose() is not None:
            break
        time.sleep(0.5)
        sys.stdout.write(".")
        sys.stdout.flush()
    print()

    if ros.get_pose() is None:
        print("  [WARN] No pose from /aft_mapped_to_init yet — SLAM may still be initialising.")
        if not args.no_slam:
            print("  Press Enter to continue anyway, or Ctrl+C to abort.")
            try:
                input()
            except KeyboardInterrupt:
                ros.shutdown()
                kill_proc(slam_proc, "SLAM")
                sys.exit(0)

    # ── camera ────────────────────────────────────────────────────────────────
    cap = None
    if not args.no_camera:
        print("[3/5] Opening IMX219 camera...")
        cap = open_camera(args.device, args.width, args.height)
        if cap is None:
            print("  [WARN] Camera not available — falling back to dummy frames.")
            print("         Connect IMX219 to CSI port, or use --no-camera.")
    else:
        print("[3/5] Camera skipped (--no-camera) — using dummy frames")

    # ── capture loop ──────────────────────────────────────────────────────────
    print("\n[4/5] Capturing  (Ctrl+C to stop + process)")
    print("-" * 60)

    poses_at_capture: list[dict] = []
    images_captured:  list[str]  = []
    photo_idx   = 0
    _stop       = threading.Event()

    orig_sigint = signal.getsignal(signal.SIGINT)
    def _on_ctrl_c(*_):
        _stop.set()
    signal.signal(signal.SIGINT, _on_ctrl_c)

    next_capture = time.time() + args.interval

    try:
        while not _stop.is_set():
            now   = time.time()
            pose  = ros.get_pose()
            n_pts = ros.cloud_size()

            pose_str = (
                f"({pose['position'][0]:.2f}, "
                f"{pose['position'][1]:.2f}, "
                f"{pose['position'][2]:.2f})"
                if pose else "no pose yet"
            )
            sys.stdout.write(
                f"\r  {n_pts:>8,} pts | pose: {pose_str:<36} | "
                f"next photo in {max(0, next_capture - now):.1f}s  "
            )
            sys.stdout.flush()

            if now >= next_capture:
                next_capture = now + args.interval
                print()   # break status line

                if pose is None:
                    print("  [SKIP] No SLAM pose — skipping this capture")
                else:
                    # Capture image
                    if cap is not None:
                        img_path = capture_frame(cap, str(images_dir), photo_idx)
                    else:
                        img_path = make_dummy_frame(
                            str(images_dir), photo_idx, args.width, args.height
                        )

                    if img_path:
                        poses_at_capture.append(pose)
                        images_captured.append(img_path)
                        photo_idx += 1

            time.sleep(0.2)

    except KeyboardInterrupt:
        pass

    signal.signal(signal.SIGINT, orig_sigint)

    print(f"\n\nCapture stopped — {photo_idx} frame(s), {ros.cloud_size():,} cloud pts")

    # ── save data ─────────────────────────────────────────────────────────────
    print("\n[5/5] Saving session data...")

    # Trajectory: one pose per captured image (1:1 match, no subsampling needed)
    traj_path = str(meta_dir / "trajectory.json")
    with open(traj_path, "w") as f:
        json.dump({"poses": poses_at_capture}, f, indent=2)
    print(f"  Trajectory: {len(poses_at_capture)} poses → {traj_path}")

    # Point cloud
    cloud = ros.get_cloud()
    if len(cloud):
        save_pcd_binary(cloud, str(proc_dir / "cloud_raw.pcd"))
    else:
        print("  [WARN] No cloud data received — meshing will fail")

    # Image manifest
    with open(str(meta_dir / "images.json"), "w") as f:
        json.dump(
            [{"index": i, "path": p, "pose": poses_at_capture[i]}
             for i, p in enumerate(images_captured)],
            f, indent=2
        )

    # ── shutdown ──────────────────────────────────────────────────────────────
    if cap is not None:
        cap.release()
    ros.shutdown()
    kill_proc(slam_proc, "SLAM")

    # ── post-processing ───────────────────────────────────────────────────────
    if args.no_process:
        print(f"\n  Session: {session_dir}")
        print("  Run pipeline later:")
        print(f"    {OPEN3D_PY} -c \"$(cat tests/test_texture_live.py | grep -A999 _PIPELINE_SCRIPT)\"")
        print(f"  Or rerun with: python3 -m tests.test_texture_live --no-slam --no-camera")
        return

    if photo_idx == 0 or not len(cloud):
        print("\n  Nothing to process (0 frames or no cloud).")
        return

    ok = run_pipeline(str(session_dir))
    if ok:
        open_result(str(session_dir))
    else:
        print(f"\n  Pipeline failed — raw data preserved at: {session_dir}")

    print("\nDone.")


if __name__ == "__main__":
    main()
