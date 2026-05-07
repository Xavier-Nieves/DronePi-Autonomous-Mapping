#!/home/dronepi/miniforge3/envs/dronepi/bin/python
"""
postprocess_mesh.py — Post-flight mesh pipeline orchestrator.

Calls each mesh_tools / texture_tools module in sequence to produce a
complete textured 3D mesh from a ROS 2 LiDAR + camera bag.

Pipeline
--------
1. BagReader             Extract /cloud_registered point cloud from bag
   1b. HealthLogExtractor  Extract /rpi/health → health_log.csv (non-fatal)
2. MLSSmoother           Remove IMU noise via Moving Least Squares
3. GroundClassifier      SMRF classification → ground / non-ground
4. DTMBuilder            Delaunay 2.5D terrain mesh from ground points
5. DSMBuilder            Ball Pivoting mesh from non-ground points
6. MeshMerger            Combine DTM + DSM into single output mesh
7. TextureProjectionStage Project IMX477 frames onto mesh vertices (optional)
8. Publisher             Save PLYs, metadata.json, latest.json, log flight

Stage 7 is skipped when:
  - --no-texture flag is passed
  - --no-mesh flag is passed (no mesh to project onto)
  - /arducam/image_raw topic is absent from the bag
  - camera.enabled is false in config.yaml

Stage timings
-------------
Wall-clock time is measured at each stage boundary and accumulated in
stage_times dict. Passed to Publisher.publish() → written to metadata.json
as stage_timings_s + bottleneck_stage. Skipped stages record 0.0 so the
dict always has all 8 keys and is safe to read without key guards.

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
  python postprocess_mesh.py --bag /path/to/bag --no-texture
  python postprocess_mesh.py --bag /path/to/bag --texture-frames 50
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

from mesh_tools.bag_reader              import BagReader
from mesh_tools.mls_smoother            import MLSSmoother
from mesh_tools.ground_classifier       import GroundClassifier
from mesh_tools.dtm_builder             import DTMBuilder
from mesh_tools.dsm_builder             import DSMBuilder
from mesh_tools.mesh_merger             import MeshMerger
from mesh_tools.publisher               import Publisher
from health_log_extractor               import HealthLogExtractor

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

# ── Texture config (from config.yaml camera block, with safe defaults) ────────
_cam_cfg          = _cfg.get("camera", {})
_CAMERA_ENABLED   = _cam_cfg.get("enabled", True)   # default True — camera is present
_IMAGE_TOPIC      = _cam_cfg.get("image_topic",  "/arducam/image_raw")
_POSE_TOPIC       = _cfg.get("slam", {}).get("pose_topic", "/aft_mapped_to_init")
_CALIB_PATH       = (
    Path(__file__).resolve().parents[2]
    / "unitree_drone_mapper"
    / "config"
    / "camera_calibration.yaml"
)


# ── Poisson (optional legacy path) ────────────────────────────────────────────

def run_poisson(pts, depth: int = None):
    """Optional Poisson surface reconstruction (used with --use-poisson)."""
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
        )
        depth = depth or 8
        mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=depth
        )
        print(f"  [Poisson] {len(np.asarray(mesh.triangles)):,} triangles  depth={depth}")
        return mesh
    except Exception as exc:
        print(f"  [Poisson] FAILED: {exc}", file=sys.stderr)
        return None


# ── Point cloud cap ───────────────────────────────────────────────────────────

def apply_cap(pts: np.ndarray, duration_s: float, fast: bool = False) -> np.ndarray:
    """Downsample point cloud to a tractable size for mesh generation."""
    rate = CLOUD_CAP_RATE_FAST if fast else CLOUD_CAP_RATE
    cap  = CLOUD_CAP_MAX_FAST  if fast else CLOUD_CAP_MAX
    # Rate-based cap: keep at most rate * duration_s points
    rate_cap = int(rate * max(duration_s, 1.0))
    target   = min(cap, rate_cap)
    if len(pts) <= target:
        return pts
    idx = np.random.choice(len(pts), target, replace=False)
    pts = pts[idx]
    print(f"  [Cap] Downsampled to {len(pts):,} points (target={target:,})")
    return pts


# ── Maps dir resolver ─────────────────────────────────────────────────────────

def resolve_maps_dir(maps_arg: str, bag_path: Path) -> Path:
    """Resolve the maps output directory from CLI arg or config."""
    if maps_arg and maps_arg != "auto":
        return Path(maps_arg)
    if MAPS_DIR is not None:
        return MAPS_DIR / bag_path.name
    return bag_path / "maps_output"


# ── Texture stage helper ──────────────────────────────────────────────────────

def _should_run_texture(args, bag_path: Path, has_mesh: bool):
    """Return (should_run: bool, reason: str)."""
    if args.no_texture:
        return False, "--no-texture flag"
    if not has_mesh:
        return False, "--no-mesh flag (no mesh to project onto)"
    if not _CAMERA_ENABLED:
        return False, "camera.enabled=false in config.yaml"
    if not _CALIB_PATH.exists():
        return False, f"calibration file not found: {_CALIB_PATH}"
    # Check bag has image topic
    try:
        from rosbags.rosbag2 import Reader
        with Reader(bag_path) as r:
            topics = {c.topic for c in r.connections}
        if _IMAGE_TOPIC not in topics:
            return False, f"topic '{_IMAGE_TOPIC}' not found in bag"
    except Exception:
        pass  # Let the stage attempt and fail gracefully
    return True, ""


def run_texture_stage(mesh, bag_path: Path, max_frames, fast: bool) -> bool:
    """Run TextureProjectionStage. Returns True on success."""
    try:
        from texture_tools.texture_stage import TextureProjectionStage
        capped = 30 if fast and max_frames is None else max_frames
        stage = TextureProjectionStage(
            calibration_path = str(_CALIB_PATH),
            use_distortion   = False,
            angle_weighting  = False,
            max_frames       = capped,
            save_output      = True,
        )
        stage.project(
            mesh        = mesh,
            bag_path    = bag_path,
            pose_topic  = _POSE_TOPIC,
            image_topic = _IMAGE_TOPIC,
        )
        return True
    except ImportError as exc:
        print(
            f"  [Texture] SKIPPED — TextureProjectionStage not importable: {exc}\n"
            f"            Install texture_tools dependencies to enable.",
            file=sys.stderr,
        )
        return False

    except Exception as exc:
        print(
            f"  [Texture] FAILED — {exc}\n"
            f"           Continuing with untextured mesh.",
            file=sys.stderr,
        )
        traceback.print_exc()
        return False


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    default_maps = str(MAPS_DIR) if MAPS_DIR else "auto"

    parser = argparse.ArgumentParser(description="DronePi post-flight mesh pipeline.")
    parser.add_argument("--bag",              required=True,        help="Path to bag directory")
    parser.add_argument("--maps-dir",         default=default_maps, help="Maps output directory")
    parser.add_argument("--fast",             action="store_true",  help="Fast mode: skip MLS, aggressive downsampling, cap texture frames")
    parser.add_argument("--debug",            action="store_true",  help="Save intermediate PLY files to debug/")
    parser.add_argument("--no-mls",           action="store_true",  help="Skip MLS smoothing")
    parser.add_argument("--no-mesh",          action="store_true",  help="Cloud only — skip all meshing")
    parser.add_argument("--no-texture",       action="store_true",  help="Skip Stage 7 texture projection")
    parser.add_argument("--use-poisson",      action="store_true",  help="Use legacy Poisson instead of DTM+DSM")
    parser.add_argument("--poisson-depth",    type=int,  default=None)
    parser.add_argument("--grid-res",         type=float, default=0.10)
    parser.add_argument("--mls-radius",       type=float, default=0.05)
    parser.add_argument("--bpa-radius",       type=float, default=None)
    parser.add_argument("--max-bpa-pts",      type=int,  default=None)
    parser.add_argument("--ground-height",    type=float, default=None)
    parser.add_argument("--ground-threshold", type=float, default=0.5)
    parser.add_argument("--ground-cell-size", type=float, default=1.0)
    parser.add_argument("--max-frames",       type=int,  default=None,
                        help="Max LiDAR frames to read from bag")
    parser.add_argument("--texture-frames",   type=int,  default=None,
                        help="Max camera frames for texture projection (default: all, fast: 30)")
    parser.add_argument("--auto",             action="store_true",  help="Non-interactive mode")
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
    pf_ipc.update(PipelineStage.INIT.value)

    current_stage = PipelineStage.INIT
    t_start       = time.time()

    # ── Stage timing accumulator ──────────────────────────────────────────────
    # All 8 keys are pre-populated with 0.0 so skipped stages are never absent
    # from metadata.json. t_stage is reset to time.time() after each stage
    # completes so each delta is the exclusive wall-clock for that stage.
    stage_times: dict = {
        "BagReader":        0.0,
        "MLSSmoother":      0.0,
        "GroundClassifier": 0.0,
        "DTMBuilder":       0.0,
        "DSMBuilder":       0.0,
        "MeshMerger":       0.0,
        "TextureProjection":0.0,
        "Publisher":        0.0,
    }
    t_stage = time.time()   # reset after each stage

    bpa_cap  = args.max_bpa_pts or (50_000 if args.fast else 80_000)
    grid_res = args.grid_res * 2 if args.fast else args.grid_res

    # Determine whether texture stage will run (pre-flight check, fast path)
    # Full validation happens inside _should_run_texture after mesh is built.
    has_mesh     = not args.no_mesh
    texture_mode = "skipped (--no-texture)" if args.no_texture else (
                   "skipped (--no-mesh)"    if not has_mesh    else
                   "enabled (fast: 30 frames max)" if args.fast else
                   f"enabled ({args.texture_frames or 'all'} frames)")

    print("=" * 60)
    print("  DronePi Post-Flight Mesh Pipeline")
    print(f"  Session  : {session_id}")
    print(f"  Maps dir : {maps_dir}")
    if args.debug:   print("  *** DEBUG MODE ***")
    if args.fast:    print("  *** FAST MODE ***")
    mode = ("cloud only" if args.no_mesh
            else "Poisson" if args.use_poisson
            else "DTM + DSM")
    print(f"  Mode     : {mode}")
    print(f"  MLS      : {'disabled' if skip_mls else f'radius={args.mls_radius}m'}")
    if not args.use_poisson and not args.no_mesh:
        print(f"  Grid res : {grid_res}m")
        print(f"  BPA cap  : {bpa_cap:,}")
    print(f"  Texture  : {texture_mode}")
    print("=" * 60)

    try:
        # ── [1] Extract ───────────────────────────────────────────────────────
        current_stage = PipelineStage.BAG_EXTRACT
        pf_ipc.update(current_stage.value)
        print(f"\n[1/7] Extracting point cloud from bag...")
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

        # Record BagReader wall-clock, reset timer
        stage_times["BagReader"] = round(time.time() - t_stage, 1)
        t_stage = time.time()

        # ── [1b] Health log extraction (non-fatal, not a named pipeline stage)
        print(f"\n[1b] Extracting health log from bag...")
        try:
            extractor = HealthLogExtractor()
            n_rows = extractor.extract(
                bag_path    = bag_path,
                output_path = bag_path / "health_log.csv",
            )
            if n_rows:
                print(f"  [Health] {n_rows} rows → health_log.csv")
            else:
                print(f"  [Health] No /rpi/health data in bag — health_log.csv not written")
        except Exception as exc:
            print(f"  [Health] Extraction failed (non-fatal): {exc}")
        # Health extraction time is intentionally NOT charged to any stage_times key
        t_stage = time.time()

        # ── [1c] Cap ──────────────────────────────────────────────────────────
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
            print(f"\n[2/7] MLS skipped ({reason})")
            # stage_times["MLSSmoother"] stays 0.0
        else:
            print(f"\n[2/7] MLS smoothing...")
            smoother = MLSSmoother(radius=args.mls_radius)
            pts      = smoother.smooth(pts)
            debug.save_cloud(pts, "mls")
            debug.print_stats(pts, "After MLS")

        stage_times["MLSSmoother"] = round(time.time() - t_stage, 1)
        t_stage = time.time()

        final_mesh   = None
        dtm_mesh_out = None
        dsm_mesh_out = None

        if args.no_mesh:
            print(f"\n[3-5/7] Meshing skipped (--no-mesh)")
            # All mesh stage_times stay 0.0

        elif args.use_poisson:
            print(f"\n[3-5/7] Running Poisson reconstruction...")
            raw_mesh   = run_poisson(pts, depth=args.poisson_depth)
            merger     = MeshMerger()
            final_mesh = merger.wrap_poisson(raw_mesh)
            # Charge all mesh stages to MeshMerger for simplicity
            stage_times["MeshMerger"] = round(time.time() - t_stage, 1)
            t_stage = time.time()

        else:
            # ── [3] Ground classification ─────────────────────────────────────
            current_stage = PipelineStage.GROUND_CLASSIFY
            pf_ipc.update(current_stage.value)
            print(f"\n[3/7] Ground classification...")

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

            stage_times["GroundClassifier"] = round(time.time() - t_stage, 1)
            t_stage = time.time()

            # ── [4] DTM ───────────────────────────────────────────────────────
            current_stage = PipelineStage.DTM
            pf_ipc.update(current_stage.value)
            print(f"\n[4/7] Building DTM (Delaunay 2.5D)...")
            dtm_builder  = DTMBuilder(grid_res=grid_res)
            dtm_mesh     = dtm_builder.build(ground_pts)
            dtm_mesh_out = dtm_mesh
            debug.save_mesh(dtm_mesh, "dtm", is_open3d=False)

            stage_times["DTMBuilder"] = round(time.time() - t_stage, 1)
            t_stage = time.time()

            # ── [5] DSM ───────────────────────────────────────────────────────
            current_stage = PipelineStage.DSM
            pf_ipc.update(current_stage.value)
            print(f"\n[5/7] Building DSM (Ball Pivoting)...")
            dsm_builder  = DSMBuilder(
                radius=args.bpa_radius, fast=args.fast, max_pts=args.max_bpa_pts)
            dsm_mesh     = dsm_builder.build(nonground_pts)
            dsm_mesh_out = dsm_mesh
            debug.save_mesh(dsm_mesh, "dsm", is_open3d=True)

            stage_times["DSMBuilder"] = round(time.time() - t_stage, 1)
            t_stage = time.time()

            # ── [5b] Merge ────────────────────────────────────────────────────
            current_stage = PipelineStage.MERGE
            pf_ipc.update(current_stage.value)
            print(f"\n[5b/7] Merging DTM + DSM...")
            merger     = MeshMerger()
            final_mesh = merger.merge(dtm_mesh, dsm_mesh)

            stage_times["MeshMerger"] = round(time.time() - t_stage, 1)
            t_stage = time.time()

        # ── [6] Texture projection ────────────────────────────────────────────
        # Runs only when a mesh was produced and all preconditions are met.
        # Failure here is non-fatal — the grey mesh is still published.
        texture_ok = False
        run_tex, tex_reason = _should_run_texture(args, bag_path, has_mesh=(final_mesh is not None))

        if run_tex:
            current_stage = PipelineStage.TEXTURE
            pf_ipc.update(current_stage.value)
            print(f"\n[6/7] Texture projection...")
            print(f"  [Texture] Calibration : {_CALIB_PATH.name}")
            print(f"  [Texture] Image topic : {_IMAGE_TOPIC}")
            print(f"  [Texture] Pose topic  : {_POSE_TOPIC}")

            texture_ok = run_texture_stage(
                mesh        = final_mesh,
                bag_path    = bag_path,
                max_frames  = args.texture_frames,
                fast        = args.fast,
            )

            if texture_ok:
                print(f"  [Texture] Stage 6 complete — mesh has vertex colours")
            else:
                print(f"  [Texture] Stage 6 failed — publishing grey mesh as fallback")
        else:
            print(f"\n[6/7] Texture projection skipped — {tex_reason}")

        stage_times["TextureProjection"] = round(time.time() - t_stage, 1)
        t_stage = time.time()

        # ── [7] Publish ───────────────────────────────────────────────────────
        current_stage = PipelineStage.PUBLISH
        pf_ipc.update(current_stage.value)
        elapsed = time.time() - t_start
        print(f"\n[7/7] Publishing outputs...")
        pub = Publisher(maps_dir=maps_dir)
        pub.publish(
            mesh          = final_mesh,
            cloud_pts     = pts,
            session_id    = session_id,
            bag_path      = bag_path,
            bag_meta      = meta,
            elapsed_s     = elapsed,
            dtm_mesh      = dtm_mesh_out,
            dsm_mesh      = dsm_mesh_out,
            stage_timings = stage_times,   # ← new
        )

        # Record Publisher wall-clock (time from stage_times reset to end of publish)
        stage_times["Publisher"] = round(time.time() - t_stage, 1)

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
    print(f"  Cloud   : {len(pts):,} points")
    if final_mesh is not None:
        print(f"  Mesh    : {len(final_mesh.faces):,} faces")
    if texture_ok:
        print(f"  Texture : applied ({args.texture_frames or 'all'} frames processed)")
    elif run_tex:
        print(f"  Texture : FAILED — grey mesh published as fallback")
    else:
        print(f"  Texture : skipped ({tex_reason})")
    # Print stage timing summary
    print(f"\n  Stage timings:")
    for stage, t in stage_times.items():
        if t > 0.0:
            marker = " ◀ bottleneck" if t == max(v for v in stage_times.values() if v > 0) else ""
            print(f"    {stage:<20s}: {t:>6.1f}s{marker}")
    if MAPS_DIR is not None:
        print(f"\n  Viewer  : http://10.42.0.1:8080/meshview.html")
    else:
        print(f"\n  Output  : {maps_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
