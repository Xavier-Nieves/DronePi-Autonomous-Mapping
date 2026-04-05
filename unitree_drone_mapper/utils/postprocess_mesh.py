#!/home/dronepi/miniforge3/envs/dronepi/bin/python
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

LED / buzzer integration
------------------------
At each pipeline stage transition this script writes
/tmp/postflight_status.json via PostflightIPC so led_service.py can
show accurate LED feedback:

  Pipeline stage running → POSTFLIGHT_RUNNING (yellow 1 Hz blink)
  Pipeline done          → POSTFLIGHT_DONE    (green + yellow solid, held 5 s)
  Pipeline failed        → POSTFLIGHT_FAILED  (red 3 Hz blink)

The buzzer tones (TUNE_POSTPROCESS_ACTIVE and TUNE_POSTPROCESS_DONE) are
played by PostflightMonitor in watchdog_core/postflight.py — this script
does not directly touch the buzzer.

Debug mode (--debug):
    Saves intermediate PLY files at each stage to debug/ subfolder.

Usage
-----
  python postprocess_mesh.py --bag /mnt/ssd/rosbags/scan_20260319_230358
  python postprocess_mesh.py --bag /path/to/bag --fast --debug
  python postprocess_mesh.py --bag /path/to/bag --no-mls
"""

import argparse
import shutil
import sys
import time
import traceback
from pathlib import Path

import numpy as np

# ── Module imports ────────────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))

from mesh_tools.bag_reader        import BagReader
from mesh_tools.mls_smoother      import MLSSmoother
from mesh_tools.ground_classifier import GroundClassifier
from mesh_tools.dtm_builder       import DTMBuilder
from mesh_tools.dsm_builder       import DSMBuilder
from mesh_tools.mesh_merger       import MeshMerger
from mesh_tools.publisher         import Publisher

from debugger_tools               import DebugSaver, PipelineStage, write_failure_status
from postflight_ipc               import PostflightIPC

import sys as _sys
_sys.path.insert(0, str(Path(__file__).parents[2]))
from unitree_drone_mapper.config_loader import load_config as _load_config

# ── Config ────────────────────────────────────────────────────────────────────
_cfg = _load_config()

_MNT_MAPS   = Path(_cfg["paths"]["maps_dir"])
_MNT_ROSBAG = Path(_cfg["paths"]["rosbags_dir"])

MAPS_DIR   = _MNT_MAPS   if _MNT_MAPS.exists()   else None
ROSBAG_DIR = _MNT_ROSBAG if _MNT_ROSBAG.exists() else None

CLOUD_CAP_RATE      = _cfg["mesh"]["cloud_cap_rate"]
CLOUD_CAP_MAX       = _cfg["mesh"]["cloud_cap_max"]
CLOUD_CAP_RATE_FAST = _cfg["mesh"]["cloud_cap_rate_fast"]
CLOUD_CAP_MAX_FAST  = _cfg["mesh"]["cloud_cap_max_fast"]


# ── Poisson (optional legacy path) ────────────────────────────────────────────

def run_poisson(pts, depth: int = None):
    """Optional Poisson surface reconstruction (used with --use-poisson)."""
    import open3d as o3d

    if depth is None:
        n = len(pts)
        if   n < 10_000:  print("  [Poisson] Too few points -- skipping"); return None
        elif n < 30_000:  depth = 7
        elif n < 80_000:  depth = 8
        elif n < 200_000: depth = 9
        else:             depth = 10

    print(f"  [Poisson] depth={depth}  pts={len(pts):,}")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(float))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(100)

    mesh, densities = (
        o3d.geometry.TriangleMesh
        .create_from_point_cloud_poisson(pcd, depth=depth)
    )

    dens   = np.asarray(densities)
    thresh = np.percentile(dens, 5)
    mesh.remove_vertices_by_mask(dens < thresh)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    print(f"  [Poisson] {len(np.asarray(mesh.triangles)):,} faces")
    return mesh


# ── Dynamic cap ───────────────────────────────────────────────────────────────

def apply_cap(pts, duration_s: float, fast: bool = False):
    """Subsample point cloud to dynamic cap based on scan duration."""
    cap_rate = CLOUD_CAP_RATE_FAST if fast else CLOUD_CAP_RATE
    cap_max  = CLOUD_CAP_MAX_FAST  if fast else CLOUD_CAP_MAX
    cap      = int(min(duration_s * cap_rate, cap_max))
    cap      = max(cap, 10_000)

    if len(pts) <= cap:
        print(f"  [Cap] {len(pts):,} pts (under cap of {cap:,})")
        return pts

    idx      = np.random.choice(len(pts), cap, replace=False)
    mode_str = "FAST " if fast else ""
    print(f"  [Cap] {mode_str}{len(pts):,} → {cap:,} pts  ({duration_s:.0f}s × {cap_rate})")
    return pts[idx]


# ── Path resolution ───────────────────────────────────────────────────────────

def resolve_maps_dir(args_maps_dir: str, bag_path: Path) -> Path:
    """
    Resolve the maps output directory with cross-platform fallback.

    Priority:
      1. Explicit --maps-dir argument
      2. /mnt/ssd/maps if mounted (RPi)
      3. Bag directory itself (Windows / local development)
    """
    if args_maps_dir and args_maps_dir != "auto":
        return Path(args_maps_dir)
    if MAPS_DIR is not None:
        return MAPS_DIR
    fallback = bag_path if bag_path.is_dir() else bag_path.parent
    print(f"  [INFO] /mnt/ssd/maps not found, using bag directory for outputs")
    return fallback


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    default_maps = str(MAPS_DIR) if MAPS_DIR else "auto"

    parser = argparse.ArgumentParser(description="DronePi post-flight mesh pipeline.")
    parser.add_argument("--bag",              required=True,       help="Path to bag directory")
    parser.add_argument("--maps-dir",         default=default_maps,help="Maps output directory")
    parser.add_argument("--fast",             action="store_true", help="Fast mode: skip MLS, aggressive downsampling")
    parser.add_argument("--debug",            action="store_true", help="Save intermediate PLY files to debug/")
    parser.add_argument("--no-mls",           action="store_true", help="Skip MLS smoothing")
    parser.add_argument("--no-mesh",          action="store_true", help="Cloud only — skip all meshing")
    parser.add_argument("--use-poisson",      action="store_true", help="Use legacy Poisson instead of DTM+DSM")
    parser.add_argument("--poisson-depth",    type=int,  default=None)
    parser.add_argument("--grid-res",         type=float, default=0.10)
    parser.add_argument("--mls-radius",       type=float, default=0.05)
    parser.add_argument("--bpa-radius",       type=float, default=None)
    parser.add_argument("--max-bpa-pts",      type=int,  default=None)
    parser.add_argument("--ground-height",    type=float, default=None)
    parser.add_argument("--ground-threshold", type=float, default=0.5)
    parser.add_argument("--ground-cell-size", type=float, default=1.0)
    parser.add_argument("--max-frames",       type=int,  default=None)
    parser.add_argument("--auto",             action="store_true", help="Non-interactive mode")
    parser.add_argument("--min-points",       type=int,  default=None)
    args = parser.parse_args()

    bag_path   = Path(args.bag)
    session_id = bag_path.name
    skip_mls   = args.no_mls or args.fast

    if not bag_path.exists():
        print(f"[FAIL] Bag not found: {bag_path}")
        sys.exit(1)

    maps_dir = resolve_maps_dir(args.maps_dir, bag_path)
    if not maps_dir.exists():
        maps_dir.mkdir(parents=True, exist_ok=True)

    debug   = DebugSaver(bag_path, enabled=args.debug)
    pf_ipc  = PostflightIPC()

    # Signal to LED service that the pipeline is now running.
    # Stage name matches PipelineStage.INIT so the LED transitions to
    # POSTFLIGHT_RUNNING (yellow 1 Hz blink) immediately.
    pf_ipc.update(PipelineStage.INIT.value)

    current_stage = PipelineStage.INIT
    t_start       = time.time()

    bpa_cap  = args.max_bpa_pts or (50_000 if args.fast else 80_000)
    grid_res = args.grid_res * 2 if args.fast else args.grid_res

    print("=" * 60)
    print("  DronePi Post-Flight Mesh Pipeline")
    print(f"  Session  : {session_id}")
    print(f"  Maps dir : {maps_dir}")
    if args.debug: print("  *** DEBUG MODE ***")
    if args.fast:  print("  *** FAST MODE ***")
    mode = ("cloud only" if args.no_mesh
            else "Poisson" if args.use_poisson
            else "DTM + DSM")
    print(f"  Mode     : {mode}")
    print(f"  MLS      : {'disabled' if skip_mls else f'radius={args.mls_radius}m'}")
    if not args.use_poisson and not args.no_mesh:
        print(f"  Grid res : {grid_res}m")
        print(f"  BPA cap  : {bpa_cap:,}")
    print("=" * 60)

    try:
        # ── [1] Extract ───────────────────────────────────────────────────────
        current_stage = PipelineStage.BAG_EXTRACT
        pf_ipc.update(current_stage.value)
        print(f"\n[1/6] Extracting point cloud from bag...")
        reader  = BagReader(bag_path, max_frames=args.max_frames)
        pts_raw = reader.extract()
        meta    = reader.metadata

        _cfg_threshold: int = _cfg.get("mesh", {}).get("min_points_to_process", 50_000)
        min_pts: int = args.min_points if args.min_points is not None else _cfg_threshold

        if len(pts_raw) < min_pts:
            print(
                f"\n[ABORT] Extraction yielded {len(pts_raw):,} points — "
                f"below minimum threshold of {min_pts:,}.\n"
                f"        No output will be written.",
                file=sys.stderr,
            )
            pf_ipc.clear()
            if maps_dir.exists() and not any(maps_dir.iterdir()):
                shutil.rmtree(maps_dir)
            sys.exit(0)

        print(f"  [Guard] {len(pts_raw):,} points ≥ threshold {min_pts:,} — proceeding.")
        debug.save_cloud(pts_raw, "raw")
        debug.print_stats(pts_raw, "Raw Point Cloud")

        # ── [1b] Cap ──────────────────────────────────────────────────────────
        current_stage = PipelineStage.CAP
        pf_ipc.update(current_stage.value)
        pts = apply_cap(pts_raw, meta.get("duration_s", 120.0), fast=args.fast)
        if len(pts) != len(pts_raw):
            debug.save_cloud(pts, "capped")
            debug.print_stats(pts, "After Cap")

        # ── [2] MLS ───────────────────────────────────────────────────────────
        current_stage = PipelineStage.MLS
        pf_ipc.update(current_stage.value)
        if skip_mls:
            reason = "--fast" if args.fast else "--no-mls"
            print(f"\n[2/6] MLS skipped ({reason})")
        else:
            print(f"\n[2/6] MLS smoothing...")
            smoother = MLSSmoother(radius=args.mls_radius)
            pts      = smoother.smooth(pts)
            debug.save_cloud(pts, "mls")
            debug.print_stats(pts, "After MLS")

        final_mesh   = None
        dtm_mesh_out = None
        dsm_mesh_out = None

        if args.no_mesh:
            print(f"\n[3-5/6] Meshing skipped (--no-mesh)")

        elif args.use_poisson:
            print(f"\n[3-5/6] Running Poisson reconstruction...")
            raw_mesh   = run_poisson(pts, depth=args.poisson_depth)
            merger     = MeshMerger()
            final_mesh = merger.wrap_poisson(raw_mesh)

        else:
            # ── [3] Ground classification ─────────────────────────────────────
            current_stage = PipelineStage.GROUND_CLASSIFY
            pf_ipc.update(current_stage.value)
            print(f"\n[3/6] Ground classification...")

            if args.ground_height is not None:
                print(f"  [GroundClassifier] Manual threshold: Z ≤ {args.ground_height}m")
                ground_mask   = pts[:, 2] <= args.ground_height
                ground_pts    = pts[ground_mask]
                nonground_pts = pts[~ground_mask]
                pct = 100.0 * len(ground_pts) / len(pts)
                print(f"  [GroundClassifier] Ground: {len(ground_pts):,} ({pct:.1f}%)  "
                      f"Non-ground: {len(nonground_pts):,} ({100-pct:.1f}%)")
            else:
                classifier    = GroundClassifier(
                    threshold=args.ground_threshold,
                    cell_size=args.ground_cell_size,
                )
                ground_pts, nonground_pts = classifier.classify(pts)

            debug.save_classified(ground_pts, nonground_pts)
            debug.print_stats(ground_pts,    "Ground Points")
            debug.print_stats(nonground_pts, "Non-Ground Points")

            # ── [4] DTM ───────────────────────────────────────────────────────
            current_stage = PipelineStage.DTM
            pf_ipc.update(current_stage.value)
            print(f"\n[4/6] Building DTM (Delaunay 2.5D)...")
            dtm_builder  = DTMBuilder(grid_res=grid_res)
            dtm_mesh     = dtm_builder.build(ground_pts)
            dtm_mesh_out = dtm_mesh
            debug.save_mesh(dtm_mesh, "dtm", is_open3d=False)

            # ── [5] DSM ───────────────────────────────────────────────────────
            current_stage = PipelineStage.DSM
            pf_ipc.update(current_stage.value)
            print(f"\n[5/6] Building DSM (Ball Pivoting)...")
            dsm_builder  = DSMBuilder(
                radius=args.bpa_radius, fast=args.fast, max_pts=args.max_bpa_pts)
            dsm_mesh     = dsm_builder.build(nonground_pts)
            dsm_mesh_out = dsm_mesh
            debug.save_mesh(dsm_mesh, "dsm", is_open3d=True)

            # ── [5b] Merge ────────────────────────────────────────────────────
            current_stage = PipelineStage.MERGE
            pf_ipc.update(current_stage.value)
            print(f"\n[5b] Merging DTM + DSM...")
            merger     = MeshMerger()
            final_mesh = merger.merge(dtm_mesh, dsm_mesh)

        # ── [6] Publish ───────────────────────────────────────────────────────
        current_stage = PipelineStage.PUBLISH
        pf_ipc.update(current_stage.value)
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
            dtm_mesh   = dtm_mesh_out,
            dsm_mesh   = dsm_mesh_out,
        )

        # Signal completion — LED transitions to POSTFLIGHT_DONE
        pf_ipc.done()

    except Exception as exc:
        print(f"\n[FAIL] Pipeline crashed at stage '{current_stage}': {exc}", file=sys.stderr)
        traceback.print_exc()
        # Signal failure — LED transitions to POSTFLIGHT_FAILED
        pf_ipc.failed(current_stage.value)
        write_failure_status(bag_path, current_stage.value, exc, session_id)
        sys.exit(1)

    debug.summarize()

    total = time.time() - t_start
    print(f"\n{'='*60}")
    print(f"  Done in {total:.1f}s")
    print(f"  Cloud : {len(pts):,} points")
    if final_mesh is not None:
        print(f"  Mesh  : {len(final_mesh.faces):,} faces")
    if MAPS_DIR is not None:
        print(f"  Viewer: http://10.42.0.1:8080/meshview.html")
    else:
        print(f"  Output: {maps_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
