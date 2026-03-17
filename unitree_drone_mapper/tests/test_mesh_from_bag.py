#!/usr/bin/env python3
"""Mesh reconstruction from a recorded ROS 2 bag.

Reads /cloud_registered from an MCAP rosbag, accumulates the point cloud,
runs Poisson surface reconstruction via pymeshlab, and saves outputs as .ply files.

Dependencies: numpy, scipy, trimesh, pymeshlab, rosbags
Install:
    python3 -m pip install pymeshlab trimesh rosbags --break-system-packages

Usage:
    python3 test_mesh_from_bag.py --bag /mnt/ssd/rosbags/lidar_test_03
    python3 test_mesh_from_bag.py --bag /mnt/ssd/rosbags/lidar_test_03 --max-frames 200
    python3 test_mesh_from_bag.py --bag /mnt/ssd/rosbags/lidar_test_03 --no-mesh

Outputs (saved inside the bag directory):
    combined_cloud.ply   — accumulated downsampled point cloud
    mesh_poisson.ply     — Poisson reconstructed mesh (unless --no-mesh)
    zbuffer_test.ply     — point cloud coloured by occlusion visibility
"""

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial import KDTree


# ── dependency check ──────────────────────────────────────────────────────────

def _require(pkg: str, pip_name: str | None = None):
    import importlib.util
    if importlib.util.find_spec(pkg) is None:
        name = pip_name or pkg
        print(f"  [FAIL] Missing: '{name}'")
        print(f"         pip install {name} --break-system-packages")
        sys.exit(1)

for _pkg in ("numpy", "scipy", "trimesh", "pymeshlab", "rosbags"):
    _require(_pkg)

import trimesh
import pymeshlab
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


# ── constants ─────────────────────────────────────────────────────────────────

CLOUD_TOPIC    = "/cloud_registered"
MIN_VOXEL_SIZE = 0.02    # metres
MAX_VOXEL_SIZE = 0.20    # metres
NN_SAMPLE_SIZE = 2000    # points sampled for adaptive voxel estimation
POISSON_DEPTH  = 9       # screened Poisson octree depth


# ── point cloud extraction ────────────────────────────────────────────────────

def _parse_pointcloud2(msg) -> np.ndarray | None:
    """Extract XYZ from a sensor_msgs/PointCloud2 message -> (N,3) float64."""
    if msg.width * msg.height == 0:
        return None

    fields = {f.name: f.offset for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return None

    point_step = msg.point_step
    n_points   = msg.width * msg.height
    data       = bytes(msg.data)
    x_off      = fields["x"]

    # Fast path — x,y,z tightly packed at start of each point
    if x_off == 0 and fields.get("y") == 4 and fields.get("z") == 8:
        stride = point_step // 4
        raw = np.frombuffer(data, dtype=np.float32).reshape(-1, stride)
        pts = raw[:, :3].astype(np.float64)
    else:
        fmt = "<fff" if not msg.is_bigendian else ">fff"
        pts = np.empty((n_points, 3), dtype=np.float64)
        for i in range(n_points):
            base = i * point_step + x_off
            pts[i] = struct.unpack_from(fmt, data, base)

    valid = np.isfinite(pts).all(axis=1)
    return pts[valid] if valid.any() else None


def extract_cloud(bag_path: Path, max_frames: int | None) -> np.ndarray:
    """Read /cloud_registered frames from an MCAP bag -> (N,3) float64."""
    typestore = get_typestore(Stores.ROS2_JAZZY)
    clouds    = []
    n_frames  = 0

    print(f"\n[1/5] Extracting point cloud from bag...")
    print(f"      Topic : {CLOUD_TOPIC}")
    print(f"      Bag   : {bag_path}")

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == CLOUD_TOPIC]
        if not connections:
            print(f"  [FAIL] Topic '{CLOUD_TOPIC}' not found.")
            print(f"         Available: {[c.topic for c in reader.connections]}")
            sys.exit(1)

        total = sum(c.msgcount for c in connections)
        print(f"      Frames available : {total}")
        if max_frames:
            print(f"      Limiting to      : {max_frames} frames")

        for conn, _ts, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            pts = _parse_pointcloud2(msg)
            if pts is not None:
                clouds.append(pts)
                n_frames += 1
                print(f"\r      Loaded {n_frames}/{total} frames", end="", flush=True)
            if max_frames and n_frames >= max_frames:
                break

    print()
    if not clouds:
        print("  [FAIL] No valid point cloud data extracted.")
        sys.exit(1)

    combined = np.vstack(clouds)
    print(f"      Raw points : {len(combined):,}")
    return combined


# ── adaptive voxel downsampling ───────────────────────────────────────────────

def adaptive_voxel_size(pts: np.ndarray) -> float:
    """Estimate voxel size = 2x mean nearest-neighbour distance, clamped."""
    n = min(NN_SAMPLE_SIZE, len(pts))
    idx = np.random.choice(len(pts), n, replace=False)
    sample = pts[idx]
    tree = KDTree(sample)
    dists, _ = tree.query(sample, k=2)   # k=2: [self, nearest neighbour]
    mean_nn = float(dists[:, 1].mean())
    voxel = float(np.clip(mean_nn * 2.0, MIN_VOXEL_SIZE, MAX_VOXEL_SIZE))
    print(f"      Mean NN distance : {mean_nn:.4f} m")
    print(f"      Adaptive voxel   : {voxel:.4f} m  "
          f"(clamped to [{MIN_VOXEL_SIZE}, {MAX_VOXEL_SIZE}])")
    return voxel


def voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample by keeping one point per voxel cell."""
    voxel_ids = np.floor(pts / voxel_size).astype(np.int64)
    offset = voxel_ids.min(axis=0)
    shifted = voxel_ids - offset
    dims = shifted.max(axis=0) + 1
    keys = (shifted[:, 0] * dims[1] * dims[2]
            + shifted[:, 1] * dims[2]
            + shifted[:, 2])
    _, unique_idx = np.unique(keys, return_index=True)
    return pts[unique_idx]


def statistical_outlier_removal(pts: np.ndarray,
                                 k: int = 20,
                                 std_ratio: float = 2.0) -> np.ndarray:
    """Remove points whose mean k-NN distance exceeds mean + std_ratio * std."""
    tree = KDTree(pts)
    dists, _ = tree.query(pts, k=k + 1)
    mean_dists = dists[:, 1:].mean(axis=1)
    threshold = mean_dists.mean() + std_ratio * mean_dists.std()
    return pts[mean_dists <= threshold]


# ── z-buffer occlusion check ──────────────────────────────────────────────────

def zbuffer_occlusion(
    pts: np.ndarray,
    viewpoint: np.ndarray,
    fov_h_deg: float = 120.0,
    fov_v_deg: float = 90.0,
    depth_tolerance: float = 0.05,
    buf_w: int = 512,
    buf_h: int = 384,
) -> tuple[np.ndarray, np.ndarray]:
    """Classify each point as visible or occluded from a given viewpoint.

    Projects points into a virtual frustum, builds a depth buffer, and marks
    a point occluded if another point is closer on the same ray by more than
    depth_tolerance metres.

    Returns:
        visible_mask  : (N,) bool
        occluded_mask : (N,) bool
    """
    dirs = pts - viewpoint
    dist = np.linalg.norm(dirs, axis=1)
    valid = dist > 1e-6

    dirs_norm = np.zeros_like(dirs)
    dirs_norm[valid] = dirs[valid] / dist[valid, None]

    az = np.degrees(np.arctan2(dirs_norm[:, 1], dirs_norm[:, 0]))
    el = np.degrees(np.arcsin(np.clip(dirs_norm[:, 2], -1.0, 1.0)))

    in_frustum = (
        (np.abs(az) <= fov_h_deg / 2.0) &
        (np.abs(el) <= fov_v_deg / 2.0) &
        valid
    )

    depth_buf   = np.full((buf_h, buf_w), np.inf)
    frustum_idx = np.where(in_frustum)[0]
    u = np.clip(
        ((az[in_frustum] / (fov_h_deg / 2.0) + 1.0) / 2.0 * (buf_w - 1)).astype(int),
        0, buf_w - 1
    )
    v = np.clip(
        ((el[in_frustum] / (fov_v_deg / 2.0) + 1.0) / 2.0 * (buf_h - 1)).astype(int),
        0, buf_h - 1
    )
    d = dist[in_frustum]

    # Fill depth buffer with minimum depth per pixel
    order = np.argsort(d)
    for i in order:
        if d[i] < depth_buf[v[i], u[i]]:
            depth_buf[v[i], u[i]] = d[i]

    visible_mask  = np.zeros(len(pts), dtype=bool)
    occluded_mask = np.zeros(len(pts), dtype=bool)
    for i, gi in enumerate(frustum_idx):
        if d[i] <= depth_buf[v[i], u[i]] + depth_tolerance:
            visible_mask[gi]  = True
        else:
            occluded_mask[gi] = True

    return visible_mask, occluded_mask


# ── PLY I/O ───────────────────────────────────────────────────────────────────

def save_cloud_ply(pts: np.ndarray, path: Path, colors: np.ndarray | None = None):
    """Save a point cloud as PLY via trimesh."""
    cloud = trimesh.PointCloud(vertices=pts, colors=colors)
    cloud.export(str(path))


# ── Poisson reconstruction ────────────────────────────────────────────────────

def reconstruct_mesh(pts: np.ndarray, out_path: Path):
    """Screened Poisson surface reconstruction via pymeshlab."""
    print(f"\n[4/5] Poisson reconstruction (depth={POISSON_DEPTH})...")
    t0 = time.time()

    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(vertex_matrix=pts), "cloud")

    print(f"      Estimating normals...")
    ms.compute_normal_for_point_clouds(
        k=30,
        smoothiter=0,
        flipflag=False,
    )

    print(f"      Reconstructing surface...")
    ms.generate_surface_reconstruction_screened_poisson(
        depth=POISSON_DEPTH,
        fulldepth=5,
        cgdepth=0,
        scale=1.1,
        samplespernode=1.5,
        pointweight=4.0,
        iters=8,
        confidence=False,
        preclean=False,
    )

    elapsed = time.time() - t0
    mesh = ms.current_mesh()
    print(f"      Vertices : {mesh.vertex_number():,}  "
          f"Faces : {mesh.face_number():,}  ({elapsed:.1f}s)")

    # Prune low-quality vertices (exterior spurious surface)
    ms.compute_selection_by_condition_per_vertex(condselect="(q < 0.1)")
    ms.meshing_remove_selected_vertices()
    mesh = ms.current_mesh()
    print(f"      After pruning — Vertices : {mesh.vertex_number():,}  "
          f"Faces : {mesh.face_number():,}")

    ms.save_current_mesh(str(out_path))
    print(f"      Saved -> {out_path}")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    global POISSON_DEPTH
    parser = argparse.ArgumentParser(
        description="Reconstruct mesh from ROS 2 MCAP bag (pymeshlab, no Open3D)."
    )
    parser.add_argument("--bag", required=True,
                        help="Path to rosbag directory")
    parser.add_argument("--max-frames", type=int, default=None,
                        help="Limit frames loaded from /cloud_registered (default: all)")
    parser.add_argument("--no-mesh", action="store_true",
                        help="Skip Poisson reconstruction — point cloud only")
    parser.add_argument("--poisson-depth", type=int, default=POISSON_DEPTH,
                        help=f"Poisson octree depth (default: {POISSON_DEPTH})")
    args = parser.parse_args()

    POISSON_DEPTH = args.poisson_depth

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"[FAIL] Bag not found: {bag_path}")
        sys.exit(1)

    print("=" * 55)
    print("  Mesh Reconstruction from Rosbag")
    print("  (pymeshlab + trimesh + numpy/scipy)")
    print("=" * 55)

    # 1 — Extract
    raw_pts = extract_cloud(bag_path, args.max_frames)

    # 2 — Downsample
    print(f"\n[2/5] Downsampling...")
    voxel = adaptive_voxel_size(raw_pts)
    pts   = voxel_downsample(raw_pts, voxel)
    print(f"      After voxel downsample  : {len(pts):,} points")
    pts   = statistical_outlier_removal(pts)
    print(f"      After outlier removal   : {len(pts):,} points")

    cloud_path = bag_path / "combined_cloud.ply"
    save_cloud_ply(pts, cloud_path)
    print(f"      Saved -> {cloud_path}")

    # 3 — Z-buffer
    print(f"\n[3/5] Z-buffer occlusion check...")
    centroid  = pts.mean(axis=0)
    viewpoint = centroid + np.array([0.0, 0.0, 2.0])
    print(f"      Viewpoint : {viewpoint.round(3)}  (2m above centroid)")

    vis_mask, occ_mask = zbuffer_occlusion(pts, viewpoint)
    n_vis = vis_mask.sum()
    n_occ = occ_mask.sum()
    n_out = len(pts) - n_vis - n_occ
    print(f"      Visible         : {n_vis:,}")
    print(f"      Occluded        : {n_occ:,}")
    print(f"      Outside frustum : {n_out:,}")

    colours = np.full((len(pts), 4), [128, 128, 128, 255], dtype=np.uint8)
    colours[vis_mask] = [0,   255, 0,   255]
    colours[occ_mask] = [255, 0,   0,   255]
    zbuf_path = bag_path / "zbuffer_test.ply"
    save_cloud_ply(pts, zbuf_path, colours)
    print(f"      Saved -> {zbuf_path}")
    print(f"      Green=visible  Red=occluded  Grey=outside frustum")

    # 4 — Mesh
    if args.no_mesh:
        print(f"\n[4/5] Skipping Poisson reconstruction (--no-mesh)")
    else:
        mesh_path = bag_path / "mesh_poisson.ply"
        reconstruct_mesh(pts, mesh_path)

    # 5 — Summary
    print(f"\n[5/5] Complete.")
    print(f"=" * 55)
    print(f"  Output directory : {bag_path}")
    print(f"  combined_cloud.ply  — downsampled point cloud")
    print(f"  zbuffer_test.ply    — occlusion visibility map")
    if not args.no_mesh:
        print(f"  mesh_poisson.ply    — Poisson reconstructed mesh")
    print(f"=" * 55)
    print(f"\nTo inspect in CloudCompare:")
    print(f"  cloudcompare {bag_path}/mesh_poisson.ply &")


if __name__ == "__main__":
    main()
