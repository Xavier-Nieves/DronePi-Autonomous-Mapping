#!/home/dronepi/open3d_env/bin/python3
"""Live scan-to-CloudCompare test.

Launches Point-LIO + LiDAR, accumulates the registered point cloud in real
time, then on Ctrl+C:
  1. Cleans the cloud (outlier removal + voxel downsample)
  2. Estimates normals
  3. Runs Poisson meshing
  4. Saves  scan_raw.pcd / scan_clean.pcd / model.ply / model.obj
  5. Opens CloudCompare (if available) to view the result

Run from the unitree_drone_mapper/ directory:
    python3 -m tests.test_cloudcompare
    python3 -m tests.test_cloudcompare --no-rviz
    python3 -m tests.test_cloudcompare --no-slam        # accumulate only (SLAM already running)
    python3 -m tests.test_cloudcompare --depth 8        # Poisson depth (8=fast, 9=balanced, 10=quality)
    python3 -m tests.test_cloudcompare --no-cloudcompare # skip launching CloudCompare

Requirements:
    pip3 install open3d
    sudo apt install cloudcompare  (or snap install cloudcompare)
"""

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

# ── ROS / workspace paths ──────────────────────────────────────────────────────
ROS_SETUP  = "/opt/ros/jazzy/setup.bash"
WS_SETUP   = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/install/setup.bash"
)
LAUNCH_FILE = os.path.expanduser(
    "~/unitree_lidar_project/RPI5/ros2_ws/src/point_lio_ros2/launch/"
    "combined_lidar_mapping.launch.py"
)
LIDAR_PORT  = "/dev/ttyUSB0"

# ── output ─────────────────────────────────────────────────────────────────────
DATA_ROOT = os.path.expanduser(
    "~/unitree_lidar_project/unitree_drone_mapper/data"
)

# ── CloudCompare binary candidates ─────────────────────────────────────────────
CC_CANDIDATES = [
    "cloudcompare",
    "CloudCompare",
    "/snap/bin/cloudcompare",
    "/usr/bin/cloudcompare",
    "/usr/local/bin/cloudcompare",
]


# ══════════════════════════════════════════════════════════════════════════════
#  Environment helpers
# ══════════════════════════════════════════════════════════════════════════════

def _load_ros_env() -> dict:
    """Source ROS + workspace setup files and return resulting env vars."""
    cmd = f"source {ROS_SETUP} && source {WS_SETUP} && env"
    result = subprocess.run(
        ["bash", "-c", cmd],
        capture_output=True, text=True, timeout=15,
    )
    env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v
    return env


def _apply_ros_env():
    """Inject ROS environment into current process."""
    print("  Sourcing ROS 2 environment...")
    env = _load_ros_env()
    if not env:
        print("  [WARN] Could not load ROS environment — is ROS 2 Jazzy installed?")
        return False
    os.environ.update(env)
    return True


# ══════════════════════════════════════════════════════════════════════════════
#  CloudCompare helpers
# ══════════════════════════════════════════════════════════════════════════════

def _find_cloudcompare() -> str | None:
    """Return the first CloudCompare binary found on this system."""
    for candidate in CC_CANDIDATES:
        result = subprocess.run(
            ["which", candidate], capture_output=True, text=True
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    return None


def _open_cloudcompare(filepath: str, cc_bin: str):
    """Launch CloudCompare to open a file (non-blocking)."""
    if not os.path.isfile(filepath):
        print(f"  [WARN] File not found, cannot open in CloudCompare: {filepath}")
        return

    print(f"\nOpening CloudCompare with: {os.path.basename(filepath)}")
    try:
        subprocess.Popen(
            [cc_bin, "-O", filepath],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as e:
        print(f"  [WARN] Could not launch CloudCompare: {e}")


# ══════════════════════════════════════════════════════════════════════════════
#  ROS 2 cloud accumulator node
# ══════════════════════════════════════════════════════════════════════════════

class CloudAccumulatorNode:
    """Subscribes to /cloud_registered and accumulates XYZ points."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
        import sensor_msgs_py.point_cloud2 as pc2

        self._rclpy = rclpy
        self._pc2   = pc2

        rclpy.init()

        class _InnerNode(Node):
            def __init__(inner_self):
                super().__init__("cc_cloud_accumulator")
                inner_self.all_points  = []
                inner_self.frame_count = 0
                inner_self.sub = inner_self.create_subscription(
                    PointCloud2,
                    "/cloud_registered",
                    inner_self._cb,
                    10,
                )

            def _cb(inner_self, msg):
                pts = [
                    [p[0], p[1], p[2]]
                    for p in pc2.read_points(
                        msg, skip_nans=True, field_names=("x", "y", "z", "intensity")
                    )
                ]
                if pts:
                    inner_self.all_points.extend(pts)
                    inner_self.frame_count += 1
                    if inner_self.frame_count % 10 == 0:
                        print(
                            f"  [cloud] frames={inner_self.frame_count:>4}  "
                            f"points={len(inner_self.all_points):>8,}",
                            flush=True,
                        )

        self._node = _InnerNode()
        self._stop  = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def start(self):
        self._thread.start()

    def _spin(self):
        while not self._stop.is_set() and self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3)
        if self._rclpy.ok():
            self._rclpy.shutdown()

    @property
    def points(self):
        return list(self._node.all_points)

    @property
    def frame_count(self):
        return self._node.frame_count


# ══════════════════════════════════════════════════════════════════════════════
#  Open3D processing pipeline
# ══════════════════════════════════════════════════════════════════════════════

def process_and_save(points: list, out_dir: str, poisson_depth: int) -> dict:
    """
    Full pipeline: raw points -> clean cloud -> mesh -> save files.
    Returns dict with output file paths.
    """
    import numpy as np
    import open3d as o3d

    os.makedirs(out_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    pts_np = np.array(points, dtype=np.float64)  # float32 segfaults on aarch64 Open3D 0.18
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_np)
    print(f"\n  Raw cloud: {len(pcd.points):,} points")

    # ── Save raw PCD ──────────────────────────────────────────────────────────
    raw_path = os.path.join(out_dir, f"scan_raw_{ts}.pcd")
    o3d.io.write_point_cloud(raw_path, pcd)
    print(f"  Saved raw PCD  -> {raw_path}")

    # ── Step 1: Outlier removal ───────────────────────────────────────────────
    print("\n  [1/4] Removing outliers...")
    pcd_clean, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    removed = len(pcd.points) - len(pcd_clean.points)
    print(f"        Removed {removed:,} outliers — {len(pcd_clean.points):,} remain")

    # ── Step 2: Downsample if large ───────────────────────────────────────────
    if len(pcd_clean.points) > 500_000:
        print("  [2/4] Downsampling (cloud > 500k pts)...")
        pcd_clean = pcd_clean.voxel_down_sample(voxel_size=0.02)
        print(f"        Downsampled to {len(pcd_clean.points):,} points")
    else:
        print(f"  [2/4] No downsampling needed ({len(pcd_clean.points):,} pts)")

    # ── Step 3: Normal estimation ─────────────────────────────────────────────
    print("  [3/4] Estimating normals...")
    pcd_clean.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    pcd_clean.orient_normals_consistent_tangent_plane(k=15)
    print("        Normals computed and oriented")

    # ── Save clean PCD ────────────────────────────────────────────────────────
    clean_path = os.path.join(out_dir, f"scan_clean_{ts}.pcd")
    o3d.io.write_point_cloud(clean_path, pcd_clean)
    print(f"  Saved clean PCD -> {clean_path}")

    # ── Step 4: Poisson meshing ───────────────────────────────────────────────
    print(f"\n  [4/4] Poisson reconstruction (depth={poisson_depth}) ...")
    print("        This can take several minutes on a Pi 5 — please wait.")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd_clean,
        depth=poisson_depth,
        width=0,
        scale=1.1,
        linear_fit=False,
    )
    import numpy as np
    # Remove low-density artefacts
    mask = np.asarray(densities) < np.quantile(np.asarray(densities), 0.01)
    mesh.remove_vertices_by_mask(mask)

    # Keep largest connected component only
    tri_clusters, cluster_sizes, _ = mesh.cluster_connected_triangles()
    tri_clusters  = np.asarray(tri_clusters)
    cluster_sizes = np.asarray(cluster_sizes)
    largest = cluster_sizes.argmax()
    mesh.remove_triangles_by_mask(tri_clusters != largest)

    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    print(f"        Mesh: {len(mesh.vertices):,} vertices / {len(mesh.triangles):,} triangles")

    # ── Save mesh ─────────────────────────────────────────────────────────────
    ply_path = os.path.join(out_dir, f"model_{ts}.ply")
    obj_path = os.path.join(out_dir, f"model_{ts}.obj")
    o3d.io.write_triangle_mesh(ply_path, mesh)
    o3d.io.write_triangle_mesh(obj_path, mesh, write_ascii=True)
    ply_mb = os.path.getsize(ply_path) / 1_048_576
    obj_mb = os.path.getsize(obj_path) / 1_048_576
    print(f"  Saved PLY      -> {ply_path}  ({ply_mb:.1f} MB)")
    print(f"  Saved OBJ      -> {obj_path}  ({obj_mb:.1f} MB)")

    return {
        "raw_pcd":   raw_path,
        "clean_pcd": clean_path,
        "ply":       ply_path,
        "obj":       obj_path,
    }


# ══════════════════════════════════════════════════════════════════════════════
#  Prerequisites
# ══════════════════════════════════════════════════════════════════════════════

def check_prerequisites(port: str, skip_slam: bool) -> bool:
    ok = True

    if not skip_slam:
        if os.path.exists(port):
            print(f"  [OK]   LiDAR port {port}")
        else:
            print(f"  [FAIL] LiDAR port {port} not found")
            ok = False

        if os.path.isfile(WS_SETUP):
            print(f"  [OK]   ROS 2 workspace built")
        else:
            print(f"  [FAIL] Workspace not built: {WS_SETUP}")
            ok = False

        if os.path.isfile(LAUNCH_FILE):
            print(f"  [OK]   Launch file found")
        else:
            print(f"  [FAIL] Launch file missing: {LAUNCH_FILE}")
            ok = False

    try:
        import open3d  # noqa: F401
        print(f"  [OK]   open3d available")
    except ImportError:
        print(f"  [FAIL] open3d not installed — run: pip3 install open3d")
        ok = False

    return ok


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Scan with Point-LIO, process cloud, open in CloudCompare"
    )
    parser.add_argument("--no-rviz",         action="store_true",
                        help="Skip RViz during SLAM")
    parser.add_argument("--no-slam",         action="store_true",
                        help="Skip launching SLAM (assume it is already running)")
    parser.add_argument("--no-cloudcompare", action="store_true",
                        help="Skip opening CloudCompare at the end")
    parser.add_argument("--port",   default=LIDAR_PORT,
                        help=f"LiDAR serial port (default: {LIDAR_PORT})")
    parser.add_argument("--depth",  type=int, default=9,
                        help="Poisson depth: 8=fast, 9=balanced, 10=quality (default: 9)")
    parser.add_argument("--out",    default=None,
                        help="Output directory (default: data/cc_test_TIMESTAMP)")
    args = parser.parse_args()

    print("=" * 60)
    print("  Point-LIO -> Open3D -> CloudCompare Test")
    print("=" * 60)

    # ── prerequisites ─────────────────────────────────────────────────────────
    print("\nChecking prerequisites...")
    if not check_prerequisites(args.port, args.no_slam):
        sys.exit(1)

    cc_bin = None
    if not args.no_cloudcompare:
        cc_bin = _find_cloudcompare()
        if cc_bin:
            print(f"  [OK]   CloudCompare found: {cc_bin}")
        else:
            print("  [WARN] CloudCompare not found — files will be saved but not opened.")
            print("         Install with: sudo apt install cloudcompare")

    # ── ROS environment ───────────────────────────────────────────────────────
    if not args.no_slam:
        print()
        _apply_ros_env()

    # ── output dir ────────────────────────────────────────────────────────────
    session_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or os.path.join(DATA_ROOT, f"cc_test_{session_ts}")
    os.makedirs(out_dir, exist_ok=True)
    print(f"\n  Output directory: {out_dir}")

    # ── launch SLAM stack ─────────────────────────────────────────────────────
    slam_proc = None
    if not args.no_slam:
        rviz_flag = "false" if args.no_rviz else "true"
        launch_cmd = (
            f"source {ROS_SETUP} && "
            f"source {WS_SETUP} && "
            f"ros2 launch {LAUNCH_FILE} "
            f"rviz:={rviz_flag} "
            f"port:={args.port}"
        )
        print("\nStarting LiDAR + Point-LIO" + (" + RViz" if not args.no_rviz else "") + "...")
        slam_proc = subprocess.Popen(
            ["bash", "-c", launch_cmd],
            preexec_fn=os.setsid,
        )
        print("  Waiting 8 s for SLAM to initialise...")
        time.sleep(8)

    # ── start cloud accumulator ───────────────────────────────────────────────
    print("\nStarting cloud accumulator on /cloud_registered ...")
    accumulator = CloudAccumulatorNode()
    accumulator.start()

    print("\nScanning — move the LiDAR around your area.")
    print("Press Ctrl+C when done to process the cloud.\n")

    # ── wait for Ctrl+C ───────────────────────────────────────────────────────
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\nCtrl+C received — stopping scan...")

    # ── stop accumulator ──────────────────────────────────────────────────────
    points = accumulator.points
    frames = accumulator.frame_count
    accumulator.stop()

    print(f"\nCollected {len(points):,} points from {frames} frames.")

    if not points:
        print("\n[ERROR] No points collected.")
        print("  Is Point-LIO running?  Check: ros2 topic hz /cloud_registered")
        if slam_proc:
            os.killpg(os.getpgid(slam_proc.pid), signal.SIGINT)
        sys.exit(1)

    # ── stop SLAM stack ───────────────────────────────────────────────────────
    if slam_proc and slam_proc.poll() is None:
        print("\nStopping SLAM stack (Point-LIO will save its own PCD if enabled)...")
        try:
            os.killpg(os.getpgid(slam_proc.pid), signal.SIGINT)
            slam_proc.wait(timeout=10)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(slam_proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    # ── process cloud ─────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  Processing cloud with Open3D")
    print("=" * 60)

    output_files = process_and_save(points, out_dir, poisson_depth=args.depth)

    # ── summary ───────────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  Done!")
    print("=" * 60)
    print(f"\n  Frames  : {frames}")
    print(f"  Points  : {len(points):,}")
    print(f"  Out dir : {out_dir}")
    print("\n  Files saved:")
    for label, path in [
        ("Raw PCD  ", output_files["raw_pcd"]),
        ("Clean PCD", output_files["clean_pcd"]),
        ("PLY mesh ", output_files["ply"]),
        ("OBJ mesh ", output_files["obj"]),
    ]:
        print(f"    {label} -> {os.path.basename(path)}")

    # ── open CloudCompare ─────────────────────────────────────────────────────
    if not args.no_cloudcompare:
        if cc_bin:
            # Open clean PCD — best for exploring the raw geometry
            _open_cloudcompare(output_files["clean_pcd"], cc_bin)
        else:
            print("\n  CloudCompare not found on this system.")
            print("  To view the files, copy them to a machine with CloudCompare and open:")
            print(f"    {output_files['clean_pcd']}")
            print(f"    {output_files['ply']}")

    print()


if __name__ == "__main__":
    main()
