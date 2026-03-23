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

Debug mode (--debug):
    Saves intermediate PLY files at each stage:
    - debug_1_raw.ply           Raw extracted points
    - debug_2_capped.ply        After dynamic cap
    - debug_3_mls.ply           After MLS smoothing
    - debug_4_ground.ply        Classified ground points (BLUE)
    - debug_4_nonground.ply     Classified non-ground points (RED)
    - debug_4_classified.ply    Combined with colors
    - debug_5_dtm.ply           DTM mesh
    - debug_6_dsm.ply           DSM mesh
    - mesh_final.ply            Final merged mesh

Usage
-----
  # Debug mode — saves all intermediate files
  python postprocess_mesh.py --bag C:\\path\\to\\bag --fast --debug

  # Standard run
  python postprocess_mesh.py --bag C:\\path\\to\\bag --fast
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# ── mesh_tools modules ────────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))

from mesh_tools.bag_reader        import BagReader
from mesh_tools.mls_smoother      import MLSSmoother
from mesh_tools.ground_classifier import GroundClassifier
from mesh_tools.dtm_builder       import DTMBuilder
from mesh_tools.dsm_builder       import DSMBuilder
from mesh_tools.mesh_merger       import MeshMerger
from mesh_tools.publisher         import Publisher

# ── config ────────────────────────────────────────────────────────────────────

_MNT_MAPS   = Path("/mnt/ssd/maps")
_MNT_ROSBAG = Path("/mnt/ssd/rosbags")

MAPS_DIR   = _MNT_MAPS   if _MNT_MAPS.exists()   else None
ROSBAG_DIR = _MNT_ROSBAG if _MNT_ROSBAG.exists() else None

CLOUD_CAP_RATE      = 1500
CLOUD_CAP_MAX       = 500_000
CLOUD_CAP_RATE_FAST = 800
CLOUD_CAP_MAX_FAST  = 150_000


# ── Debug helpers ─────────────────────────────────────────────────────────────

class DebugSaver:
    """
    Saves intermediate point clouds and meshes for debugging.
    Each file is color-coded to help visualize the pipeline stages.
    """
    
    def __init__(self, output_dir: Path, enabled: bool = False):
        self.output_dir = output_dir
        self.enabled = enabled
        self.step = 0
        
        if enabled:
            self.debug_dir = output_dir / "debug"
            self.debug_dir.mkdir(exist_ok=True)
            print(f"\n  [DEBUG] Saving intermediate files to: {self.debug_dir}")
    
    def save_cloud(self, pts: np.ndarray, name: str, color: tuple = None):
        """Save point cloud as PLY with optional color."""
        if not self.enabled:
            return
        
        self.step += 1
        filename = f"debug_{self.step}_{name}.ply"
        filepath = self.debug_dir / filename
        
        try:
            import trimesh
            
            if color is not None:
                # Create colored point cloud
                colors = np.tile(color, (len(pts), 1))
                cloud = trimesh.PointCloud(vertices=pts, colors=colors)
            else:
                # Use height-based coloring (viridis-like)
                cloud = trimesh.PointCloud(vertices=pts)
                colors = self._height_colors(pts)
                cloud.colors = colors
            
            cloud.export(str(filepath))
            print(f"  [DEBUG] Saved: {filename}  ({len(pts):,} pts)")
            
        except Exception as e:
            print(f"  [DEBUG] Failed to save {filename}: {e}")
    
    def save_mesh(self, mesh, name: str, is_open3d: bool = False):
        """Save mesh as PLY."""
        if not self.enabled or mesh is None:
            return
        
        self.step += 1
        filename = f"debug_{self.step}_{name}.ply"
        filepath = self.debug_dir / filename
        
        try:
            if is_open3d:
                import open3d as o3d
                o3d.io.write_triangle_mesh(str(filepath), mesh)
                faces = len(np.asarray(mesh.triangles))
            else:
                mesh.export(str(filepath))
                faces = len(mesh.faces)
            
            print(f"  [DEBUG] Saved: {filename}  ({faces:,} faces)")
            
        except Exception as e:
            print(f"  [DEBUG] Failed to save {filename}: {e}")
    
    def save_classified(self, ground_pts: np.ndarray, nonground_pts: np.ndarray):
        """Save ground and non-ground as separate colored clouds."""
        if not self.enabled:
            return
        
        self.step += 1
        
        try:
            import trimesh
            
            # Ground = blue/purple
            ground_colors = np.tile([100, 100, 255, 255], (len(ground_pts), 1))
            ground_cloud = trimesh.PointCloud(vertices=ground_pts, colors=ground_colors)
            ground_file = self.debug_dir / f"debug_{self.step}_ground.ply"
            ground_cloud.export(str(ground_file))
            print(f"  [DEBUG] Saved: debug_{self.step}_ground.ply  "
                  f"({len(ground_pts):,} pts, BLUE)")
            
            # Non-ground = red/orange
            nonground_colors = np.tile([255, 100, 50, 255], (len(nonground_pts), 1))
            nonground_cloud = trimesh.PointCloud(vertices=nonground_pts, colors=nonground_colors)
            nonground_file = self.debug_dir / f"debug_{self.step}_nonground.ply"
            nonground_cloud.export(str(nonground_file))
            print(f"  [DEBUG] Saved: debug_{self.step}_nonground.ply  "
                  f"({len(nonground_pts):,} pts, RED)")
            
            # Combined with colors
            combined_pts = np.vstack([ground_pts, nonground_pts])
            combined_colors = np.vstack([ground_colors, nonground_colors])
            combined_cloud = trimesh.PointCloud(vertices=combined_pts, colors=combined_colors)
            combined_file = self.debug_dir / f"debug_{self.step}_classified.ply"
            combined_cloud.export(str(combined_file))
            print(f"  [DEBUG] Saved: debug_{self.step}_classified.ply  "
                  f"(combined, BLUE=ground, RED=objects)")
            
        except Exception as e:
            print(f"  [DEBUG] Failed to save classified clouds: {e}")
    
    def print_stats(self, pts: np.ndarray, label: str):
        """Print point cloud statistics."""
        if not self.enabled:
            return
        
        x_min, y_min, z_min = pts.min(axis=0)
        x_max, y_max, z_max = pts.max(axis=0)
        x_range = x_max - x_min
        y_range = y_max - y_min
        z_range = z_max - z_min
        
        print(f"\n  [DEBUG] {label} Statistics:")
        print(f"          Points: {len(pts):,}")
        print(f"          X range: {x_min:.2f} → {x_max:.2f}  ({x_range:.2f}m)")
        print(f"          Y range: {y_min:.2f} → {y_max:.2f}  ({y_range:.2f}m)")
        print(f"          Z range: {z_min:.2f} → {z_max:.2f}  ({z_range:.2f}m)")
        print(f"          Centroid: ({pts[:,0].mean():.2f}, {pts[:,1].mean():.2f}, {pts[:,2].mean():.2f})")
    
    def _height_colors(self, pts: np.ndarray) -> np.ndarray:
        """Generate viridis-like colors based on Z height."""
        z = pts[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)
        
        # Simple viridis approximation
        colors = np.zeros((len(pts), 4), dtype=np.uint8)
        colors[:, 0] = (68 + 180 * z_norm).astype(np.uint8)   # R
        colors[:, 1] = (1 + 220 * z_norm).astype(np.uint8)    # G
        colors[:, 2] = (84 + 80 * (1 - z_norm)).astype(np.uint8)  # B
        colors[:, 3] = 255  # A
        
        return colors
    
    def summarize(self):
        """Print final debug summary."""
        if not self.enabled:
            return
        
        print(f"\n  [DEBUG] ══════════════════════════════════════════════════")
        print(f"  [DEBUG] Debug files saved to: {self.debug_dir}")
        print(f"  [DEBUG] Open these in your viewer to see each pipeline stage:")
        print(f"  [DEBUG]   1. debug_*_raw.ply        — Raw extracted points")
        print(f"  [DEBUG]   2. debug_*_capped.ply     — After downsampling")
        print(f"  [DEBUG]   3. debug_*_mls.ply        — After noise removal")
        print(f"  [DEBUG]   4. debug_*_classified.ply — Ground(BLUE) + Objects(RED)")
        print(f"  [DEBUG]   5. debug_*_dtm.ply        — Terrain mesh")
        print(f"  [DEBUG]   6. debug_*_dsm.ply        — Object mesh")
        print(f"  [DEBUG] ══════════════════════════════════════════════════")


# ── Poisson (optional legacy path) ────────────────────────────────────────────

def run_poisson(pts, depth: int | None = None):
    """Optional Poisson surface reconstruction."""
    import open3d as o3d

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
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(100)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth)

    dens   = np.asarray(densities)
    thresh = np.percentile(dens, 5)
    mesh.remove_vertices_by_mask(dens < thresh)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    print(f"  [Poisson] {len(np.asarray(mesh.triangles)):,} faces")
    return mesh


# ── dynamic cap ───────────────────────────────────────────────────────────────

def apply_cap(pts, duration_s: float, fast: bool = False):
    """Subsample point cloud to dynamic cap based on scan duration."""
    cap_rate = CLOUD_CAP_RATE_FAST if fast else CLOUD_CAP_RATE
    cap_max  = CLOUD_CAP_MAX_FAST  if fast else CLOUD_CAP_MAX

    cap = int(min(duration_s * cap_rate, cap_max))
    cap = max(cap, 10_000)

    if len(pts) <= cap:
        print(f"  [Cap] {len(pts):,} pts (under cap of {cap:,})")
        return pts

    idx = np.random.choice(len(pts), cap, replace=False)
    mode_str = "FAST " if fast else ""
    print(f"  [Cap] {mode_str}{len(pts):,} → {cap:,} pts  "
          f"({duration_s:.0f}s × {cap_rate})")
    return pts[idx]


# ── path resolution ───────────────────────────────────────────────────────────

def resolve_maps_dir(args_maps_dir: str, bag_path: Path) -> Path:
    """Resolve the maps output directory with cross-platform fallback."""
    if args_maps_dir and args_maps_dir != "auto":
        maps_dir = Path(args_maps_dir)
        if maps_dir.exists() or _MNT_MAPS.exists():
            return maps_dir
        return maps_dir

    if MAPS_DIR is not None:
        return MAPS_DIR

    fallback = bag_path if bag_path.is_dir() else bag_path.parent
    print(f"  [INFO] /mnt/ssd/maps not found, using bag directory for outputs")
    return fallback


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    default_maps = str(MAPS_DIR) if MAPS_DIR else "auto"

    parser = argparse.ArgumentParser(
        description="DronePi post-flight mesh pipeline (modular).")
    parser.add_argument("--bag", required=True,
                        help="Path to bag directory")
    parser.add_argument("--maps-dir", default=default_maps,
                        help="Maps output directory")
    parser.add_argument("--fast", action="store_true",
                        help="Fast mode: downsamples aggressively, fewer BPA scales")
    parser.add_argument("--debug", action="store_true",
                        help="Debug mode: save intermediate PLY files at each stage")
    parser.add_argument("--no-mls", action="store_true",
                        help="Skip MLS smoothing")
    parser.add_argument("--no-mesh", action="store_true",
                        help="Cloud only -- skip all meshing")
    parser.add_argument("--use-poisson", action="store_true",
                        help="Use legacy Poisson instead of DTM+DSM")
    parser.add_argument("--poisson-depth", type=int, default=None,
                        help="Poisson depth override")
    parser.add_argument("--grid-res", type=float, default=0.10,
                        help="DTM grid resolution in metres (default: 0.10)")
    parser.add_argument("--mls-radius", type=float, default=0.05,
                        help="MLS search radius in metres (default: 0.05)")
    parser.add_argument("--bpa-radius", type=float, default=None,
                        help="BPA ball radius (default: auto, clamped 0.03-0.50m)")
    parser.add_argument("--max-bpa-pts", type=int, default=None,
                        help="Max points for BPA (default: 80k, 50k in fast mode)")
    parser.add_argument("--ground-height", type=float, default=None,
                        help="Manual ground height threshold (overrides classification)")
    parser.add_argument("--ground-threshold", type=float, default=0.5,
                        help="Height above ground surface to classify as ground (default: 0.5m)")
    parser.add_argument("--ground-cell-size", type=float, default=1.0,
                        help="Grid cell size for ground detection (default: 1.0m)")
    parser.add_argument("--max-frames", type=int, default=None,
                        help="Max bag frames to read")
    parser.add_argument("--auto", action="store_true",
                        help="Non-interactive mode (called by watchdog)")
    args = parser.parse_args()

    bag_path   = Path(args.bag)
    session_id = bag_path.name
    skip_mls   = args.no_mls

    if not bag_path.exists():
        print(f"[FAIL] Bag not found: {bag_path}")
        sys.exit(1)

    maps_dir = resolve_maps_dir(args.maps_dir, bag_path)
    if not maps_dir.exists():
        maps_dir.mkdir(parents=True, exist_ok=True)

    # Initialize debug saver
    debug = DebugSaver(bag_path, enabled=args.debug)

    t_start = time.time()

    # ── Banner ────────────────────────────────────────────────────────────────
    bpa_cap_str = (f"{args.max_bpa_pts:,}" if args.max_bpa_pts 
                   else "50k" if args.fast else "80k")
    
    print("=" * 60)
    print("  DronePi Post-Flight Mesh Pipeline")
    print(f"  Session  : {session_id}")
    print(f"  Maps dir : {maps_dir}")
    if args.debug:
        print(f"  *** DEBUG MODE — saving intermediate files ***")
    if args.fast:
        print(f"  *** FAST MODE ***")
    mode = ("cloud only" if args.no_mesh 
            else "Poisson" if args.use_poisson 
            else "DTM + DSM")
    print(f"  Mode     : {mode}")
    print(f"  MLS      : {'disabled' if skip_mls else f'radius={args.mls_radius}m'}")
    if not args.use_poisson and not args.no_mesh:
        grid_display = args.grid_res * 2 if args.fast else args.grid_res
        print(f"  Grid res : {grid_display}m")
        print(f"  BPA cap  : {bpa_cap_str}")
        if args.bpa_radius:
            print(f"  BPA radius: {args.bpa_radius}m (fixed)")
        if args.ground_height is not None:
            print(f"  Ground Z : {args.ground_height}m (manual)")
    print("=" * 60)

    # ══════════════════════════════════════════════════════════════════════════
    # [1] EXTRACT POINT CLOUD
    # ══════════════════════════════════════════════════════════════════════════
    print(f"\n[1/6] Extracting point cloud from bag...")
    reader = BagReader(bag_path, max_frames=args.max_frames)
    pts_raw = reader.extract()
    meta = reader.metadata

    debug.save_cloud(pts_raw, "raw")
    debug.print_stats(pts_raw, "Raw Point Cloud")

    # ══════════════════════════════════════════════════════════════════════════
    # [1b] APPLY CAP
    # ══════════════════════════════════════════════════════════════════════════
    pts = apply_cap(pts_raw, meta.get("duration_s", 120.0), fast=args.fast)
    
    if len(pts) != len(pts_raw):
        debug.save_cloud(pts, "capped")
        debug.print_stats(pts, "After Cap")

    # ══════════════════════════════════════════════════════════════════════════
    # [2] MLS SMOOTHING
    # ══════════════════════════════════════════════════════════════════════════
    if skip_mls:
        print(f"\n[2/6] MLS skipped (--no-mls)")
    else:
        print(f"\n[2/6] MLS smoothing...")
        smoother = MLSSmoother(radius=args.mls_radius)
        pts_before_mls = len(pts)
        pts = smoother.smooth(pts)
        
        debug.save_cloud(pts, "mls")
        debug.print_stats(pts, "After MLS")

    # ══════════════════════════════════════════════════════════════════════════
    # [3-5] MESHING
    # ══════════════════════════════════════════════════════════════════════════
    final_mesh = None
    dtm_mesh_out = None
    dsm_mesh_out = None

    if args.no_mesh:
        print(f"\n[3-5/6] Meshing skipped (--no-mesh)")

    elif args.use_poisson:
        print(f"\n[3-5/6] Running Poisson reconstruction...")
        raw_mesh = run_poisson(pts, depth=args.poisson_depth)
        merger = MeshMerger()
        final_mesh = merger.wrap_poisson(raw_mesh)

    else:
        # ══════════════════════════════════════════════════════════════════════
        # [3] GROUND CLASSIFICATION
        # ══════════════════════════════════════════════════════════════════════
        print(f"\n[3/6] Ground classification...")
        
        if args.ground_height is not None:
            # Manual height threshold
            print(f"  [GroundClassifier] Using manual height threshold: Z ≤ {args.ground_height}m")
            ground_mask = pts[:, 2] <= args.ground_height
            ground_pts = pts[ground_mask]
            nonground_pts = pts[~ground_mask]
            pct = 100.0 * len(ground_pts) / len(pts)
            print(f"  [GroundClassifier] Ground: {len(ground_pts):,} ({pct:.1f}%)  "
                  f"Non-ground: {len(nonground_pts):,} ({100-pct:.1f}%)")
        else:
            # Automatic classification (tries SMRF → PMF → local)
            classifier = GroundClassifier(
                threshold=args.ground_threshold,
                cell_size=args.ground_cell_size,
            )
            ground_pts, nonground_pts = classifier.classify(pts)
        
        debug.save_classified(ground_pts, nonground_pts)
        debug.print_stats(ground_pts, "Ground Points")
        debug.print_stats(nonground_pts, "Non-Ground Points")

        # ══════════════════════════════════════════════════════════════════════
        # [4] BUILD DTM
        # ══════════════════════════════════════════════════════════════════════
        grid_res = args.grid_res * 2 if args.fast else args.grid_res
        print(f"\n[4/6] Building DTM (Delaunay 2.5D)...")
        dtm_builder = DTMBuilder(grid_res=grid_res)
        dtm_mesh = dtm_builder.build(ground_pts)
        dtm_mesh_out = dtm_mesh
        
        debug.save_mesh(dtm_mesh, "dtm", is_open3d=False)

        # ══════════════════════════════════════════════════════════════════════
        # [5] BUILD DSM
        # ══════════════════════════════════════════════════════════════════════
        print(f"\n[5/6] Building DSM (Ball Pivoting)...")
        dsm_builder = DSMBuilder(
            radius=args.bpa_radius,
            fast=args.fast,
            max_pts=args.max_bpa_pts,
        )
        dsm_mesh = dsm_builder.build(nonground_pts)
        dsm_mesh_out = dsm_mesh
        
        debug.save_mesh(dsm_mesh, "dsm", is_open3d=True)

        # ══════════════════════════════════════════════════════════════════════
        # [5b] MERGE
        # ══════════════════════════════════════════════════════════════════════
        print(f"\n[5b] Merging DTM + DSM...")
        merger = MeshMerger()
        final_mesh = merger.merge(dtm_mesh, dsm_mesh)

    # ══════════════════════════════════════════════════════════════════════════
    # [6] PUBLISH
    # ══════════════════════════════════════════════════════════════════════════
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

    # ══════════════════════════════════════════════════════════════════════════
    # SUMMARY
    # ══════════════════════════════════════════════════════════════════════════
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
