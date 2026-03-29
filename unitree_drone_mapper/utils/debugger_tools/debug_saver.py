"""
debug_saver.py — Debug output handler for intermediate pipeline files.

Saves point clouds and meshes at each pipeline stage for inspection.
Each file is color-coded to help visualize the pipeline stages.

Output files (saved to debug/ subfolder):
    debug_1_raw.ply           Raw extracted points (height-colored)
    debug_2_capped.ply        After dynamic cap (height-colored)
    debug_3_mls.ply           After MLS smoothing (height-colored)
    debug_4_ground.ply        Classified ground points (BLUE)
    debug_4_nonground.ply     Classified non-ground points (RED)
    debug_4_classified.ply    Combined ground + non-ground
    debug_5_dtm.ply           DTM mesh
    debug_6_dsm.ply           DSM mesh

Usage:
    from debugger_tools import DebugSaver
    
    debug = DebugSaver(bag_path, enabled=args.debug)
    
    debug.save_cloud(pts_raw, "raw")
    debug.print_stats(pts_raw, "Raw Point Cloud")
    
    debug.save_cloud(pts, "capped")
    debug.save_cloud(pts, "mls")
    
    debug.save_classified(ground_pts, nonground_pts)
    
    debug.save_mesh(dtm_mesh, "dtm")
    debug.save_mesh(dsm_mesh, "dsm", is_open3d=True)
    
    debug.summarize()
"""

import numpy as np
from pathlib import Path
from typing import Optional, Tuple, List


class DebugSaver:
    """
    Saves intermediate point clouds and meshes for debugging.
    
    When enabled, creates a debug/ subfolder in the output directory
    and saves PLY files at each pipeline stage with descriptive names.
    
    Color coding:
        - Point clouds: Height-based viridis coloring (purple → yellow)
        - Ground points: Blue (RGB: 100, 100, 255)
        - Non-ground points: Red/Orange (RGB: 255, 100, 50)
    
    Parameters
    ----------
    output_dir : Path
        Directory where debug/ subfolder will be created
    enabled : bool
        If False, all save methods become no-ops (zero overhead)
    """
    
    def __init__(self, output_dir: Path, enabled: bool = False):
        self.output_dir = Path(output_dir)
        self.enabled = enabled
        self.step = 0
        self.saved_files: List[Tuple[str, int, str]] = []
        
        if enabled:
            self.debug_dir = self.output_dir / "debug"
            self.debug_dir.mkdir(exist_ok=True)
            print(f"\n  [DEBUG] Saving intermediate files to: {self.debug_dir}")
        else:
            self.debug_dir = None
    
    def save_cloud(
        self,
        pts: np.ndarray,
        name: str,
        color: Optional[Tuple[int, int, int, int]] = None
    ) -> None:
        """
        Save point cloud as PLY with optional color.
        
        Parameters
        ----------
        pts : np.ndarray
            Nx3 point array
        name : str
            Filename suffix (e.g., "raw", "mls", "capped")
        color : tuple, optional
            RGBA color tuple (0-255). If None, uses height-based coloring.
        """
        if not self.enabled:
            return
        
        self.step += 1
        filename = f"debug_{self.step}_{name}.ply"
        filepath = self.debug_dir / filename
        
        try:
            import trimesh
            
            if color is not None:
                colors = np.tile(color, (len(pts), 1))
                cloud = trimesh.PointCloud(vertices=pts, colors=colors)
            else:
                cloud = trimesh.PointCloud(vertices=pts)
                colors = self._height_colors(pts)
                cloud.colors = colors
            
            cloud.export(str(filepath))
            self.saved_files.append((filename, len(pts), "cloud"))
            print(f"  [DEBUG] Saved: {filename}  ({len(pts):,} pts)")
            
        except Exception as e:
            print(f"  [DEBUG] Failed to save {filename}: {e}")
    
    def save_mesh(
        self,
        mesh,
        name: str,
        is_open3d: bool = False
    ) -> None:
        """
        Save mesh as PLY.
        
        Parameters
        ----------
        mesh : trimesh.Trimesh or open3d.geometry.TriangleMesh
            Mesh to save
        name : str
            Filename suffix (e.g., "dtm", "dsm")
        is_open3d : bool
            True if mesh is an Open3D TriangleMesh, False for trimesh
        """
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
            
            self.saved_files.append((filename, faces, "mesh"))
            print(f"  [DEBUG] Saved: {filename}  ({faces:,} faces)")
            
        except Exception as e:
            print(f"  [DEBUG] Failed to save {filename}: {e}")
    
    def save_classified(
        self,
        ground_pts: np.ndarray,
        nonground_pts: np.ndarray
    ) -> None:
        """
        Save ground and non-ground as separate colored clouds.
        
        Creates three files:
            - debug_N_ground.ply      (BLUE points)
            - debug_N_nonground.ply   (RED points)
            - debug_N_classified.ply  (combined with both colors)
        
        Parameters
        ----------
        ground_pts : np.ndarray
            Mx3 ground point array
        nonground_pts : np.ndarray
            Kx3 non-ground point array
        """
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
            self.saved_files.append((f"debug_{self.step}_ground.ply", len(ground_pts), "cloud"))
            print(f"  [DEBUG] Saved: debug_{self.step}_ground.ply  "
                  f"({len(ground_pts):,} pts, BLUE)")
            
            # Non-ground = red/orange
            nonground_colors = np.tile([255, 100, 50, 255], (len(nonground_pts), 1))
            nonground_cloud = trimesh.PointCloud(vertices=nonground_pts, colors=nonground_colors)
            nonground_file = self.debug_dir / f"debug_{self.step}_nonground.ply"
            nonground_cloud.export(str(nonground_file))
            self.saved_files.append((f"debug_{self.step}_nonground.ply", len(nonground_pts), "cloud"))
            print(f"  [DEBUG] Saved: debug_{self.step}_nonground.ply  "
                  f"({len(nonground_pts):,} pts, RED)")
            
            # Combined with colors
            combined_pts = np.vstack([ground_pts, nonground_pts])
            combined_colors = np.vstack([ground_colors, nonground_colors])
            combined_cloud = trimesh.PointCloud(vertices=combined_pts, colors=combined_colors)
            combined_file = self.debug_dir / f"debug_{self.step}_classified.ply"
            combined_cloud.export(str(combined_file))
            self.saved_files.append((f"debug_{self.step}_classified.ply", len(combined_pts), "cloud"))
            print(f"  [DEBUG] Saved: debug_{self.step}_classified.ply  "
                  f"(combined, BLUE=ground, RED=objects)")
            
        except Exception as e:
            print(f"  [DEBUG] Failed to save classified clouds: {e}")
    
    def print_stats(self, pts: np.ndarray, label: str) -> None:
        """
        Print point cloud statistics.
        
        Parameters
        ----------
        pts : np.ndarray
            Nx3 point array
        label : str
            Description label (e.g., "Raw Point Cloud", "After MLS")
        """
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
    
    def summarize(self) -> None:
        """Print summary of all saved debug files."""
        if not self.enabled or not self.saved_files:
            return
        
        print(f"\n  [DEBUG] Summary - {len(self.saved_files)} files saved to {self.debug_dir}:")
        for filename, count, ftype in self.saved_files:
            unit = "pts" if ftype == "cloud" else "faces"
            print(f"          {filename}: {count:,} {unit}")
    
    def _height_colors(self, pts: np.ndarray) -> np.ndarray:
        """
        Generate viridis-like colors based on Z height.
        
        Maps Z values to a purple → teal → yellow gradient.
        """
        z = pts[:, 2]
        z_min, z_max = z.min(), z.max()
        z_range = z_max - z_min
        if z_range < 1e-8:
            z_range = 1.0
        z_norm = (z - z_min) / z_range
        
        # Viridis approximation: purple → teal → yellow
        colors = np.zeros((len(pts), 4), dtype=np.uint8)
        colors[:, 0] = (68 + 180 * z_norm).astype(np.uint8)   # R: 68 → 248
        colors[:, 1] = (1 + 220 * z_norm).astype(np.uint8)    # G: 1 → 221
        colors[:, 2] = (84 - 50 * z_norm).astype(np.uint8)    # B: 84 → 34
        colors[:, 3] = 255  # Alpha
        
        return colors
