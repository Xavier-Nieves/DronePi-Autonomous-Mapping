#!/usr/bin/env python3
"""
postprocess_mesh.py — Post-flight mesh pipeline orchestrator.

Calls each mesh_tools module in sequence to produce a complete
3D mesh from a ROS 2 LiDAR bag.

Pipeline
--------
1. BagReader          Extract /cloud_registered point cloud from bag
2. MLSSmoother        Remove IMU noise via Moving Least Squares
3. GroundClassifier   SMRF classification → ground / non-ground
4. DTMBuilder         Delaunay 2.5D terrain mesh from ground points
5. DSMBuilder         Ball Pivoting mesh from non-ground points
6. MeshMerger         Combine DTM + DSM into single output mesh
7. Publisher          Save PLYs, metadata.json, latest.json, log flight

Optional Poisson path (--use-poisson):
    Skips classification, DTM, DSM.
    Runs Poisson reconstruction on the full point cloud instead.
    Useful for comparison or complete 3D object scans.

Usage
-----
  python3 postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260319_230358

  # Skip MLS (faster, less accurate)
  python3 postprocess_mesh.py --bag /path/to/bag --no-mls

  # Skip classification (cloud only, no mesh)
  python3 postprocess_mesh.py --bag /path/to/bag --no-mesh

  # Use legacy Poisson instead of DTM+DSM
  python3 postprocess_mesh.py --bag /path/to/bag --use-poisson

  # Fine terrain grid
  python3 postprocess_mesh.py --bag /path/to/bag --grid-res 0.05

  # Called by run_postflight.py (watchdog → auto mode)
  python3 postprocess_mesh.py --bag /path/to/bag --auto
"""

import argparse
import sys
import time
from pathlib import Path

# ── mesh_tools modules ────────────────────────────────────────────────────────
# Each module is a self-contained class with one responsibility.
# See mesh_tools/<module>.py for full documentation.

sys.path.insert(0, str(Path(__file__).parent))

from mesh_tools.bag_reader        import BagReader
from mesh_tools.mls_smoother      import MLSSmoother
from mesh_tools.ground_classifier import GroundClassifier
from mesh_tools.dtm_builder       import DTMBuilder
from mesh_tools.dsm_builder       import DSMBuilder
from mesh_tools.mesh_merger       import MeshMerger
from mesh_tools.publisher         import Publisher

# ── config ────────────────────────────────────────────────────────────────────

MAPS_DIR      = Path("/mnt/ssd/maps")
ROSBAG_DIR    = Path("/mnt/ssd/rosbags")

# Dynamic point cap: duration_s × rate, ceiling 500k
CLOUD_CAP_RATE = 1500
CLOUD_CAP_MAX  = 500_000


# ── Poisson (optional legacy path) ────────────────────────────────────────────

def run_poisson(pts, depth: int | None = None):
    """
    Optional Poisson surface reconstruction.
    Used only when --use-poisson flag is passed.
    Kept for comparison and complete 3D object scans.
    """
    import open3d as o3d
    import numpy as np

    # Auto depth from point count
    if depth is None:
        n = len(pts)
        if n < 10_000:
            print("  [Poisson] Too few points -- skipping")
            return None
        elif n < 30_000:  depth = 7
        elif n < 80_000:  depth = 8
        elif n < 200_000: depth = 9
        else:             depth = 10

    print(f"  [Poisson] depth={depth}  pts={len(pts):,}")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(float))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(100)

    mesh, densities = \
        o3d.geometry.TriangleMesh\
        .create_from_point_cloud_poisson(pcd, depth=depth)

    import numpy as np
    dens   = np.asarray(densities)
    thresh = np.percentile(dens, 5)
    mesh.remove_vertices_by_mask(dens < thresh)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    print(f"  [Poisson] {len(np.asarray(mesh.triangles)):,} faces")
    return mesh


# ── dynamic cap ───────────────────────────────────────────────────────────────

def apply_cap(pts, duration_s: float):
    """Subsample point cloud to dynamic cap based on scan duration."""
    import numpy as np
    cap = int(min(duration_s * CLOUD_CAP_RATE, CLOUD_CAP_MAX))
    cap = max(cap, 10_000)
    if len(pts) <= cap:
        print(f"  [Cap] {len(pts):,} pts (under cap of {cap:,})")
        return pts
    idx = np.random.choice(len(pts), cap, replace=False)
    print(f"  [Cap] {len(pts):,} → {cap:,} pts  "
          f"({duration_s:.0f}s × {CLOUD_CAP_RATE})")
    return pts[idx]


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="DronePi post-flight mesh pipeline (modular).")
    parser.add_argument("--bag",
                        required=True,
                        help="Path to bag directory")
    parser.add_argument("--maps-dir",
                        default=str(MAPS_DIR),
                        help="Maps output directory (default: /mnt/ssd/maps)")
    parser.add_argument("--no-mls",
                        action="store_true",
                        help="Skip MLS smoothing (faster)")
    parser.add_argument("--no-mesh",
                        action="store_true",
                        help="Cloud only -- skip all meshing")
    parser.add_argument("--use-poisson",
                        action="store_true",
                        help="Use legacy Poisson instead of DTM+DSM")
    parser.add_argument("--poisson-depth",
                        type=int, default=None,
                        help="Poisson depth override (only with --use-poisson)")
    parser.add_argument("--grid-res",
                        type=float, default=0.10,
                        help="DTM grid resolution in metres (default: 0.10)")
    parser.add_argument("--mls-radius",
                        type=float, default=0.05,
                        help="MLS search radius in metres (default: 0.05)")
    parser.add_argument("--bpa-radius",
                        type=float, default=None,
                        help="BPA ball radius (default: auto from density)")
    parser.add_argument("--max-frames",
                        type=int, default=None,
                        help="Max bag frames to read (default: all)")
    parser.add_argument("--auto",
                        action="store_true",
                        help="Non-interactive mode (called by watchdog)")
    args = parser.parse_args()

    bag_path  = Path(args.bag)
    maps_dir  = Path(args.maps_dir)
    session_id = bag_path.name

    if not bag_path.exists():
        print(f"[FAIL] Bag not found: {bag_path}")
        sys.exit(1)
    if not maps_dir.exists():
        maps_dir.mkdir(parents=True, exist_ok=True)

    t_start = time.time()

    print("=" * 60)
    print("  DronePi Post-Flight Mesh Pipeline")
    print(f"  Session  : {session_id}")
    mode = "cloud only" if args.no_mesh \
        else "Poisson" if args.use_poisson \
        else "DTM + DSM (BPA + Delaunay)"
    print(f"  Mode     : {mode}")
    print(f"  MLS      : {'disabled' if args.no_mls else f'radius={args.mls_radius}m'}")
    if not args.use_poisson and not args.no_mesh:
        print(f"  Grid res : {args.grid_res}m")
    print("=" * 60)

    # ── [1] Extract point cloud ───────────────────────────────────────────────
    print(f"\n[1/6] Extracting point cloud from bag...")
    reader = BagReader(bag_path, max_frames=args.max_frames)
    pts    = reader.extract()
    meta   = reader.metadata

    # Apply dynamic cap
    pts = apply_cap(pts, meta.get("duration_s", 120.0))

    # ── [2] MLS smoothing ─────────────────────────────────────────────────────
    if args.no_mls:
        print(f"\n[2/6] MLS skipped (--no-mls)")
    else:
        print(f"\n[2/6] MLS smoothing...")
        smoother = MLSSmoother(radius=args.mls_radius)
        pts = smoother.smooth(pts)

    # ── [3-5] Meshing ─────────────────────────────────────────────────────────
    final_mesh = None

    if args.no_mesh:
        print(f"\n[3-5/6] Meshing skipped (--no-mesh)")

    elif args.use_poisson:
        print(f"\n[3-5/6] Running Poisson reconstruction...")
        raw_mesh   = run_poisson(pts, depth=args.poisson_depth)
        merger     = MeshMerger()
        final_mesh = merger.wrap_poisson(raw_mesh)

    else:
        # DTM + DSM pipeline

        # [3] Classify
        print(f"\n[3/6] Ground classification (SMRF)...")
        classifier  = GroundClassifier()
        ground_pts, nonground_pts = classifier.classify(pts)

        # [4] Build DTM
        print(f"\n[4/6] Building DTM (Delaunay 2.5D)...")
        dtm_builder = DTMBuilder(grid_res=args.grid_res)
        dtm_mesh    = dtm_builder.build(ground_pts)

        # [5] Build DSM
        print(f"\n[5/6] Building DSM (Ball Pivoting)...")
        dsm_builder = DSMBuilder(radius=args.bpa_radius)
        dsm_mesh    = dsm_builder.build(nonground_pts)

        # Merge
        print(f"\n[5b] Merging DTM + DSM...")
        merger     = MeshMerger()
        final_mesh = merger.merge(dtm_mesh, dsm_mesh)

    # ── [6] Publish ───────────────────────────────────────────────────────────
    elapsed = time.time() - t_start
    print(f"\n[6/6] Publishing outputs...")
    pub = Publisher(maps_dir=maps_dir)
    pub.publish(
        mesh       = final_mesh,
        cloud_pts  = pts,
        session_id = session_id,
        bag_path   = bag_path,
        bag_meta   = meta,
        elapsed_s  = elapsed,
    )

    # ── Summary ───────────────────────────────────────────────────────────────
    total = time.time() - t_start
    print(f"\n{'='*60}")
    print(f"  Done in {total:.1f}s")
    print(f"  Cloud : {len(pts):,} points")
    if final_mesh is not None:
        print(f"  Mesh  : {len(final_mesh.faces):,} faces  (mode: {mode})")
    print(f"  Viewer: http://10.42.0.1:8080/meshview.html")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
