#!/usr/bin/env python3
"""
postprocess_mesh.py — Post-flight mesh pipeline for DronePi.

Generates TWO output files per session for the dual-mode browser viewer:

  <session_id>_cloud.ply  — raw accumulated point cloud, dynamic density cap,
                            no voxel downsampling (preserves all geometry)
  <session_id>_mesh.ply   — Poisson reconstructed surface, voxel-downsampled
                            input, auto depth selection based on point count

Both files are copied to /mnt/ssd/maps/ and the browser viewer lets you toggle
between them without reloading.

latest.json references BOTH filenames so the viewer loads both on arrival.

Manual usage:
    python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260317_091400

Skip mesh (point cloud only — fast, camera offline):
    python3 postprocess_mesh.py --bag ... --no-mesh

Force Poisson depth (overrides auto-select):
    python3 postprocess_mesh.py --bag ... --poisson-depth 10

Non-interactive (systemd / file watcher):
    python3 postprocess_mesh.py --bag ... --auto

────────────────────────────────────────────────────────
Point cloud dynamic cap formula:
    cap = min(duration_seconds x 1500, 500_000)

    2.5 min  ->  225k pts
    5   min  ->  450k pts
    10+ min  ->  500k pts (ceiling)

Poisson depth auto-selection (points after voxel downsample):
    < 30k    -> depth 8  (smooth, fast ~2min on Pi5)
    30-100k  -> depth 9  (balanced ~5min)
    100-300k -> depth 10 (sharp edges ~15min)
    300k+    -> depth 11 (maximum ~25min)
────────────────────────────────────────────────────────

Dependencies: numpy, scipy, trimesh, pymeshlab, rosbags, pyyaml
    pip install pymeshlab trimesh rosbags pyyaml --break-system-packages
"""

import argparse
import json
import shutil
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

# ── dependency check ──────────────────────────────────────────────────────────

def _require(pkg: str, pip_name: str | None = None):
    import importlib.util
    if importlib.util.find_spec(pkg) is None:
        name = pip_name or pkg
        print(f"  [FAIL] Missing: '{name}'")
        print(f"         pip install {name} --break-system-packages")
        sys.exit(1)

for _pkg, _pip in [("numpy", None), ("scipy", None), ("trimesh", None),
                   ("pymeshlab", None), ("rosbags", None), ("yaml", "pyyaml")]:
    _require(_pkg, _pip)

import yaml
import trimesh
import pymeshlab
from scipy.spatial import KDTree
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# ── constants ─────────────────────────────────────────────────────────────────

CLOUD_TOPIC  = "/cloud_registered"
MAPS_DIR     = Path("/mnt/ssd/maps")
NN_SAMPLE    = 2000

# Point cloud path — dynamic cap, uniform subsample, no voxel grid
CLOUD_CAP_RATE = 1500      # points retained per second of flight
CLOUD_CAP_MAX  = 500_000   # hard ceiling

# Mesh path — voxel downsampling for Poisson
MESH_MIN_VOXEL = 0.02      # 2cm
MESH_MAX_VOXEL = 0.10      # 10cm

# ── Poisson depth selection ───────────────────────────────────────────────────
#
# Poisson surface reconstruction works by fitting an implicit indicator
# function to an octree built from the input point cloud. The octree depth
# controls the resolution of that grid -- higher depth means smaller cells,
# finer surface detail, but also exponentially more memory and CPU time.
#
# TWO failure modes exist and must both be avoided:
#
#   1. Too few points for the requested depth (under-sampling):
#      At depth 11 the octree cells are roughly 0.5-1 cm. If the input
#      only has 10k points, most cells at that resolution are empty. The
#      algorithm has no geometric data to fit there, so it interpolates
#      across voids and hallucinates surface. The result looks worse than
#      depth 8 on the same sparse data -- more depth with insufficient
#      point density produces MORE artifacts, not less.
#
#   2. Too many points at high depth (over-loading the Pi 5):
#      Depth 11 with 300k+ points can consume 8-12 GB RAM and take
#      20-30 minutes on the Pi 5 at 3.0 GHz. Depth 12 risks an OOM
#      crash that loses all reconstruction progress. Depth 10 is the
#      practical ceiling for this hardware at capstone scope.
#
# CORRECTED TWO-AXIS RULE:
#   The right response to too many points is a LARGER voxel size (reduce
#   input), not a higher depth. More points fed into the same depth does
#   not improve quality beyond a saturation point -- it only slows the run.
#
# Depth table (points after voxel downsample + outlier removal):
#
#   n_pts < 10k    -> abort mesh, warn (output is meaningless at any depth)
#   10k  - 30k     -> depth 7  (large cells fill sparse gaps, smooth output)
#   30k  - 80k     -> depth 8  (moderate density, clean smooth result)
#   80k  - 200k    -> depth 9  (good density, balanced quality, ~5 min Pi 5)
#   200k - 400k    -> depth 10 (high density, sharp edges, ~15 min Pi 5)
#   400k+          -> depth 10 (hard cap -- do not go to 11 on Pi 5)
#                               if this fires, MESH_MAX_VOXEL is too small;
#                               the voxel pass should have thinned the cloud
#                               further before reaching this function.
#
# Depth 11 is intentionally excluded. It requires 400k+ well-distributed
# points AND a machine with 16+ GB free RAM at the time of the call. The
# Pi 5 has 16 GB total but is also running ROS 2, the HTTP server, and the
# Foxglove bridge -- free RAM post-flight is typically 10-12 GB. Not worth
# the risk for a capstone demo where depth 10 already looks excellent.

POISSON_MIN_POINTS = 10_000   # below this, skip mesh and warn

# (minimum point count, depth value)
# Evaluated top-to-bottom; first row where n_pts >= threshold wins.
POISSON_DEPTH_MAP = [
    (200_000, 10),   # high density  -> depth 10, hard ceiling for Pi 5
    ( 80_000,  9),   # good density  -> depth 9, balanced
    ( 30_000,  8),   # moderate      -> depth 8, smooth
    ( 10_000,  7),   # sparse        -> depth 7, large cells fill gaps
]

# ── metadata.yaml parser ──────────────────────────────────────────────────────

def parse_bag_metadata(bag_path: Path) -> dict:
    meta_path = bag_path / "metadata.yaml"
    if not meta_path.exists():
        print("  [WARN] metadata.yaml not found — duration/timestamp unknown")
        return {}
    with open(meta_path) as f:
        raw = yaml.safe_load(f)
    info         = raw.get("rosbag2_bagfile_information", {})
    duration_ns  = info.get("duration", {}).get("nanoseconds", 0)
    duration_s   = duration_ns / 1e9
    duration_str = f"{int(duration_s//60):02d}m {int(duration_s%60):02d}s"
    start_ns     = info.get("starting_time", {}).get("nanoseconds_since_epoch", 0)
    dt           = datetime.fromtimestamp(start_ns / 1e9, tz=timezone.utc)
    ts_iso       = dt.strftime("%Y-%m-%d %H:%M")
    cloud_frames = 0
    for t in info.get("topics_with_message_count", []):
        if t.get("topic_metadata", {}).get("name") == CLOUD_TOPIC:
            cloud_frames = t.get("message_count", 0)
            break
    return {
        "timestamp_iso":  ts_iso,
        "duration_str":   duration_str,
        "duration_s":     round(duration_s, 1),
        "cloud_frames":   cloud_frames,
        "total_messages": info.get("message_count", 0),
        "storage":        info.get("storage_identifier", "unknown"),
        "ros_distro":     info.get("ros_distro", "unknown"),
    }

# ── point cloud extraction ────────────────────────────────────────────────────

def _parse_pointcloud2(msg) -> np.ndarray | None:
    if msg.width * msg.height == 0:
        return None
    fields = {f.name: f.offset for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return None
    point_step = msg.point_step
    n_points   = msg.width * msg.height
    data       = bytes(msg.data)
    x_off      = fields["x"]
    if x_off == 0 and fields.get("y") == 4 and fields.get("z") == 8:
        stride = point_step // 4
        raw = np.frombuffer(data, dtype=np.float32).reshape(-1, stride)
        pts = raw[:, :3].astype(np.float64)
    else:
        fmt = "<fff" if not msg.is_bigendian else ">fff"
        pts = np.empty((n_points, 3), dtype=np.float64)
        for i in range(n_points):
            pts[i] = struct.unpack_from(fmt, data, i * point_step + x_off)
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid] if valid.any() else None


def extract_cloud(bag_path: Path, max_frames: int | None) -> np.ndarray:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    clouds, n_frames = [], 0
    print(f"\n[1/4] Extracting point cloud...")
    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == CLOUD_TOPIC]
        if not connections:
            print(f"  [FAIL] Topic '{CLOUD_TOPIC}' not found.")
            print(f"         Available: {[c.topic for c in reader.connections]}")
            sys.exit(1)
        total = sum(c.msgcount for c in connections)
        print(f"      Frames available : {total:,}  |  Max: {max_frames or 'all'}")
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
        print("  [FAIL] No valid point cloud data.")
        sys.exit(1)
    combined = np.vstack(clouds)
    print(f"      Raw points total : {len(combined):,}")
    return combined

# ── cloud path: dynamic cap ───────────────────────────────────────────────────

def dynamic_cap(pts: np.ndarray, duration_s: float) -> np.ndarray:
    """
    Uniform subsample to dynamic cap.
    cap = min(duration_s x CLOUD_CAP_RATE, CLOUD_CAP_MAX), floor 10k.
    Preserves spatial distribution — no voxel grid bias.
    """
    cap = int(min(duration_s * CLOUD_CAP_RATE, CLOUD_CAP_MAX))
    cap = max(cap, 10_000)
    if len(pts) <= cap:
        print(f"      Cloud: kept all {len(pts):,} points (under cap of {cap:,})")
        return pts
    idx = np.sort(np.random.choice(len(pts), cap, replace=False))
    print(f"      Cloud cap : {cap:,}  ({duration_s:.0f}s x {CLOUD_CAP_RATE})")
    print(f"      Subsampled: {len(pts):,} -> {cap:,} points")
    return pts[idx]

# ── mesh path: voxel + outlier removal ───────────────────────────────────────

def adaptive_voxel_size(pts: np.ndarray) -> float:
    n        = min(NN_SAMPLE, len(pts))
    idx      = np.random.choice(len(pts), n, replace=False)
    tree     = KDTree(pts[idx])
    dists, _ = tree.query(pts[idx], k=2)
    mean_nn  = float(dists[:, 1].mean())
    voxel    = float(np.clip(mean_nn * 2.0, MESH_MIN_VOXEL, MESH_MAX_VOXEL))
    print(f"      Mean NN: {mean_nn:.4f}m  ->  voxel: {voxel:.4f}m")
    return voxel


def voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    voxel_ids = np.floor(pts / voxel_size).astype(np.int64)
    offset    = voxel_ids.min(axis=0)
    shifted   = voxel_ids - offset
    dims      = shifted.max(axis=0) + 1
    keys      = (shifted[:, 0] * dims[1] * dims[2]
                 + shifted[:, 1] * dims[2]
                 + shifted[:, 2])
    _, unique_idx = np.unique(keys, return_index=True)
    return pts[unique_idx]


def statistical_outlier_removal(pts: np.ndarray,
                                 k: int = 20,
                                 std_ratio: float = 2.0) -> np.ndarray:
    tree       = KDTree(pts)
    dists, _   = tree.query(pts, k=k + 1)
    mean_dists = dists[:, 1:].mean(axis=1)
    threshold  = mean_dists.mean() + std_ratio * mean_dists.std()
    return pts[mean_dists <= threshold]


def auto_poisson_depth(n_pts: int, override: int | None):
    # Select Poisson octree depth from point count, or apply manual override.
    #
    # Returns None if n_pts is below POISSON_MIN_POINTS -- the caller must
    # skip reconstruction entirely in that case. Running Poisson on fewer
    # than 10k points produces a mesh that is geometrically meaningless
    # regardless of depth: the octree has too many empty cells and the
    # algorithm fills them by interpolating across voids, creating large
    # spurious surface patches that bear no relation to the actual scan.
    #
    # Manual override bypasses the minimum check to allow expert use, but
    # a warning is printed so the operator is aware of the risk.
    if override is not None:
        print(f'      Poisson depth : {override} (manual override)')
        if n_pts < POISSON_MIN_POINTS:
            print(f'      [WARN] Only {n_pts:,} points -- manual override accepted '
                  f'but output quality will be poor (min recommended: '
                  f'{POISSON_MIN_POINTS:,})')
        return override

    if n_pts < POISSON_MIN_POINTS:
        print(f'      [SKIP] {n_pts:,} points is below the minimum of '
              f'{POISSON_MIN_POINTS:,} for meaningful Poisson reconstruction.')
        print('             Mesh output would contain mostly hallucinated surface.')
        print('             Skipping mesh -- collect a longer scan or use --no-mesh.')
        return None

    for threshold, depth in POISSON_DEPTH_MAP:
        if n_pts >= threshold:
            print(f'      Poisson depth : {depth}  '
                  f'(auto -- {n_pts:,} pts >= {threshold:,} threshold)')
            return depth

    # Fallback -- should not be reached given POISSON_MIN_POINTS guard
    return 7

# ── z-buffer occlusion ────────────────────────────────────────────────────────

def zbuffer_occlusion(pts, viewpoint,
                      fov_h_deg=120.0, fov_v_deg=90.0,
                      depth_tolerance=0.05, buf_w=512, buf_h=384):
    dirs      = pts - viewpoint
    dist      = np.linalg.norm(dirs, axis=1)
    valid     = dist > 1e-6
    dirs_norm = np.zeros_like(dirs)
    dirs_norm[valid] = dirs[valid] / dist[valid, None]
    az = np.degrees(np.arctan2(dirs_norm[:, 1], dirs_norm[:, 0]))
    el = np.degrees(np.arcsin(np.clip(dirs_norm[:, 2], -1.0, 1.0)))
    in_frustum = (np.abs(az) <= fov_h_deg/2.0) & (np.abs(el) <= fov_v_deg/2.0) & valid
    depth_buf   = np.full((buf_h, buf_w), np.inf)
    frustum_idx = np.where(in_frustum)[0]
    u = np.clip(((az[in_frustum]/(fov_h_deg/2.0)+1.0)/2.0*(buf_w-1)).astype(int), 0, buf_w-1)
    v = np.clip(((el[in_frustum]/(fov_v_deg/2.0)+1.0)/2.0*(buf_h-1)).astype(int), 0, buf_h-1)
    d = dist[in_frustum]
    for i in np.argsort(d):
        if d[i] < depth_buf[v[i], u[i]]:
            depth_buf[v[i], u[i]] = d[i]
    vis_mask = np.zeros(len(pts), dtype=bool)
    occ_mask = np.zeros(len(pts), dtype=bool)
    for i, gi in enumerate(frustum_idx):
        if d[i] <= depth_buf[v[i], u[i]] + depth_tolerance:
            vis_mask[gi] = True
        else:
            occ_mask[gi] = True
    return vis_mask, occ_mask

# ── Poisson reconstruction ────────────────────────────────────────────────────

def reconstruct_mesh(pts: np.ndarray, out_path: Path, depth: int) -> int:
    print(f"  Poisson reconstruction (depth={depth})...")
    t0 = time.time()
    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(vertex_matrix=pts), "cloud")
    print("      Estimating normals...")
    ms.compute_normal_for_point_clouds(k=30, smoothiter=0, flipflag=False)
    print("      Reconstructing surface...")
    ms.generate_surface_reconstruction_screened_poisson(
        depth=depth, fulldepth=5, cgdepth=0, scale=1.1,
        samplespernode=1.5, pointweight=4.0, iters=8,
        confidence=False, preclean=False,
    )
    elapsed = time.time() - t0
    mesh = ms.current_mesh()
    print(f"      Vertices: {mesh.vertex_number():,}  Faces: {mesh.face_number():,}  ({elapsed:.1f}s)")
    ms.compute_selection_by_condition_per_vertex(condselect="(q < 0.1)")
    ms.meshing_remove_selected_vertices()
    mesh = ms.current_mesh()
    print(f"      After pruning: Vertices {mesh.vertex_number():,}  Faces {mesh.face_number():,}")
    # Strip white RGB padding — viewer applies viridis ramp instead
    ms.save_current_mesh(str(out_path), save_vertex_color=False)
    print(f"      Saved -> {out_path}")
    return mesh.face_number()

# ── output writers ────────────────────────────────────────────────────────────

def write_metadata_json(bag_path, session_id, bag_meta,
                        cloud_count, mesh_face_count,
                        cloud_filename, mesh_filename):
    payload = {
        "id":           session_id,
        "timestamp":    bag_meta.get("timestamp_iso", "unknown"),
        "duration":     bag_meta.get("duration_str",  "unknown"),
        "duration_s":   bag_meta.get("duration_s",    None),
        "point_count":  cloud_count,
        "face_count":   mesh_face_count,
        "cloud_ply":    cloud_filename,
        "mesh_ply":     mesh_filename,
        "ply_file":     mesh_filename or cloud_filename,
        "status":       "complete" if mesh_face_count else "partial",
        "cloud_frames": bag_meta.get("cloud_frames", 0),
        "ros_distro":   bag_meta.get("ros_distro",   "jazzy"),
        "processed_at": datetime.now().strftime("%Y-%m-%d %H:%M"),
    }
    out = bag_path / "metadata.json"
    with open(out, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"      metadata.json -> {out}")
    return payload


def write_latest_json(maps_dir, session_id, bag_meta,
                      cloud_filename, mesh_filename, cloud_count):
    """
    latest.json carries both filenames.
    Viewer defaults to cloud_ply on load; toggle switches to mesh_ply.
    mesh_ply is null when --no-mesh was used.
    """
    payload = {
        "flight_name": session_id,
        "timestamp":   bag_meta.get("timestamp_iso", "unknown"),
        "point_count": cloud_count,
        "filename":    cloud_filename,    # viewer compat (primary)
        "cloud_ply":   cloud_filename,
        "mesh_ply":    mesh_filename,     # null if --no-mesh
    }
    out = maps_dir / "latest.json"
    with open(out, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"      latest.json  -> {out}")

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Post-flight pipeline: bag -> cloud PLY + mesh PLY -> browser."
    )
    parser.add_argument("--bag",           required=True)
    parser.add_argument("--max-frames",    type=int, default=None)
    parser.add_argument("--no-mesh",       action="store_true",
                        help="Skip Poisson -- cloud only (fast, camera offline)")
    parser.add_argument("--poisson-depth", type=int, default=None,
                        help="Override auto Poisson depth")
    parser.add_argument("--maps-dir",      default=str(MAPS_DIR))
    parser.add_argument("--auto",          action="store_true")
    args = parser.parse_args()

    bag_path   = Path(args.bag)
    maps_dir   = Path(args.maps_dir)
    session_id = bag_path.name

    if not bag_path.exists():
        print(f"[FAIL] Bag not found: {bag_path}"); sys.exit(1)
    if not maps_dir.exists():
        print(f"[FAIL] Maps dir not found: {maps_dir}"); sys.exit(1)

    t_start = time.time()
    print("=" * 60)
    print("  DronePi Post-Flight Mesh Pipeline")
    print(f"  Session : {session_id}")
    print(f"  Mode    : {'cloud only' if args.no_mesh else 'cloud + mesh'}")
    print("=" * 60)

    # [0] metadata
    print(f"\n[0/4] Reading bag metadata...")
    bag_meta   = parse_bag_metadata(bag_path)
    duration_s = bag_meta.get("duration_s", 120.0)
    if bag_meta:
        print(f"      Start    : {bag_meta['timestamp_iso']}")
        print(f"      Duration : {bag_meta['duration_str']}")
        print(f"      Frames   : {bag_meta['cloud_frames']:,}")

    # [1] extract
    raw_pts = extract_cloud(bag_path, args.max_frames)

    # [2] cloud path
    print(f"\n[2/4] Building viewer point cloud...")
    cloud_pts      = dynamic_cap(raw_pts, duration_s)
    cloud_filename = f"{session_id}_cloud.ply"
    cloud_path     = bag_path / "combined_cloud.ply"
    trimesh.PointCloud(vertices=cloud_pts).export(str(cloud_path))
    print(f"      Saved -> {cloud_path}")

    # z-buffer debug
    vp = cloud_pts.mean(axis=0) + np.array([0.0, 0.0, 2.0])
    vis, occ = zbuffer_occlusion(cloud_pts, vp)
    cols = np.full((len(cloud_pts), 4), [128,128,128,255], dtype=np.uint8)
    cols[vis] = [0,255,0,255]; cols[occ] = [255,0,0,255]
    trimesh.PointCloud(vertices=cloud_pts, colors=cols).export(
        str(bag_path / "zbuffer_test.ply"))

    # [3] mesh path
    mesh_filename   = None
    mesh_face_count = None

    if args.no_mesh:
        print(f"\n[3/4] Skipping Poisson (--no-mesh)")
    else:
        print(f"\n[3/4] Building Poisson mesh...")
        voxel    = adaptive_voxel_size(raw_pts)
        mesh_pts = voxel_downsample(raw_pts, voxel)
        print(f"      After voxel downsample : {len(mesh_pts):,}")
        mesh_pts = statistical_outlier_removal(mesh_pts)
        print(f"      After outlier removal  : {len(mesh_pts):,}")

        # auto_poisson_depth returns None when point count is too low.
        # In that case mesh reconstruction is skipped entirely -- the
        # cloud PLY is still served and the mesh toggle stays disabled.
        depth = auto_poisson_depth(len(mesh_pts), args.poisson_depth)
        if depth is None:
            print(f"      Mesh generation aborted -- cloud view will still load.")
        else:
            mesh_path       = bag_path / "mesh_poisson.ply"
            mesh_filename   = f"{session_id}_mesh.ply"
            mesh_face_count = reconstruct_mesh(mesh_pts, mesh_path, depth)

    # [4] publish
    print(f"\n[4/4] Publishing to HTTP server...")
    shutil.copy2(cloud_path, maps_dir / cloud_filename)
    print(f"      Cloud -> {maps_dir / cloud_filename}")
    if mesh_filename:
        shutil.copy2(bag_path / "mesh_poisson.ply", maps_dir / mesh_filename)
        print(f"      Mesh  -> {maps_dir / mesh_filename}")

    write_metadata_json(bag_path, session_id, bag_meta,
                        len(cloud_pts), mesh_face_count,
                        cloud_filename, mesh_filename)
    write_latest_json(maps_dir, session_id, bag_meta,
                      cloud_filename, mesh_filename, len(cloud_pts))

    elapsed = time.time() - t_start
    print(f"\n{'='*60}")
    print(f"  Done in {elapsed:.1f}s")
    print(f"  Cloud : {len(cloud_pts):,} points  ({cloud_filename})")
    if mesh_face_count:
        print(f"  Mesh  : {mesh_face_count:,} faces  ({mesh_filename})")
    print(f"  Viewer: http://10.42.0.1:8080/meshview.html")
    print(f"  Auto-loads in browser within 10s")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
