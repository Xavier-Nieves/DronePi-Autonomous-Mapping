"""
ground_classifier.py — Ground/non-ground classification via PDAL SMRF or PMF fallback.

Separates the point cloud into two arrays:
  ground_pts    (class 2)  → bare earth, feed to DTMBuilder
  nonground_pts (class!=2) → buildings, trees, objects, feed to DSMBuilder

Algorithms:
  1. SMRF (Simple Morphological Filter) via PDAL - preferred
  2. PMF (Progressive Morphological Filter) via scipy - fallback

No points are deleted -- classification is additive metadata.
The two output arrays together contain all input points.
"""

import time
import numpy as np


class GroundClassifier:
    """
    Ground/non-ground classifier with PDAL SMRF and PMF fallback.

    Parameters
    ----------
    method : str
        'auto' (default), 'smrf', 'pmf', or 'local_minimum'
    slope : float
        Maximum ground slope for SMRF (degrees per metre).
    window : float
        Maximum morphological window size in metres.
    threshold : float
        Maximum height above fitted ground surface (metres)
        for a point to still be classified as ground.
    cell_size : float
        Grid cell size for PMF/local_minimum methods.
    """

    def __init__(self,
                 method:     str   = 'auto',
                 slope:      float = 0.3,
                 window:     float = 18.0,
                 threshold:  float = 0.5,
                 scalar:     float = 1.25,
                 cell_size:  float = 1.0):
        self.method    = method
        self.slope     = slope
        self.window    = window
        self.threshold = threshold
        self.scalar    = scalar
        self.cell_size = cell_size

    def classify(self,
                 pts: np.ndarray
                 ) -> tuple[np.ndarray, np.ndarray]:
        """
        Classify points into ground and non-ground.

        Parameters
        ----------
        pts : Nx3 float array

        Returns
        -------
        ground_pts    : Mx3 float array  (DTM source)
        nonground_pts : Kx3 float array  (DSM source)
        """
        t0 = time.time()
        method = self.method
        
        print(f"  [GroundClassifier] Method={method}  threshold={self.threshold}m  pts={len(pts):,}")

        ground, nonground, used_method = None, None, None

        # Try SMRF first if auto or smrf
        if method in ('auto', 'smrf'):
            try:
                ground, nonground = self._classify_smrf(pts)
                used_method = 'SMRF'
            except Exception as e:
                print(f"  [GroundClassifier] SMRF unavailable: {e}")
                if method == 'smrf':
                    raise

        # Try PMF if auto and SMRF failed, or if pmf requested
        if ground is None and method in ('auto', 'pmf'):
            try:
                ground, nonground = self._classify_pmf(pts)
                used_method = 'PMF'
            except Exception as e:
                print(f"  [GroundClassifier] PMF failed: {e}")
                if method == 'pmf':
                    raise

        # Local minimum fallback
        if ground is None and method in ('auto', 'local_minimum'):
            ground, nonground = self._classify_local_minimum(pts)
            used_method = 'local_minimum'

        # Final fallback
        if ground is None:
            ground, nonground = self._classify_percentile(pts)
            used_method = 'percentile'

        pct = 100.0 * len(ground) / max(len(pts), 1)
        elapsed = time.time() - t0
        print(f"  [GroundClassifier] Used: {used_method}")
        print(f"  [GroundClassifier] Ground: {len(ground):,} ({pct:.1f}%)  "
              f"Non-ground: {len(nonground):,} ({100-pct:.1f}%)  ({elapsed:.1f}s)")
        return ground, nonground

    # ── PDAL SMRF ─────────────────────────────────────────────────────────────

    def _classify_smrf(self,
                       pts: np.ndarray
                       ) -> tuple[np.ndarray, np.ndarray]:
        """Run SMRF filter via PDAL (updated API for PDAL 3.x)."""
        import pdal
        import json

        print(f"  [GroundClassifier] Using PDAL SMRF (slope={self.slope}, window={self.window}m)")

        # PDAL expects structured array with named fields
        structured = np.zeros(len(pts), dtype=[
            ("X", np.float64),
            ("Y", np.float64),
            ("Z", np.float64),
        ])
        structured["X"] = pts[:, 0].astype(np.float64)
        structured["Y"] = pts[:, 1].astype(np.float64)
        structured["Z"] = pts[:, 2].astype(np.float64)

        pipeline_def = {
            "pipeline": [
                {
                    "type":      "filters.smrf",
                    "slope":     self.slope,
                    "window":    self.window,
                    "threshold": self.threshold,
                    "scalar":    self.scalar,
                }
            ]
        }

        # PDAL 3.x API: pass arrays to Pipeline constructor or use execute()
        pipeline = pdal.Pipeline(json.dumps(pipeline_def), arrays=[structured])
        pipeline.execute()

        result = pipeline.arrays[0]
        classes = result["Classification"]

        ground_mask = classes == 2
        nonground_mask = ~ground_mask

        return pts[ground_mask], pts[nonground_mask]

    # ── PMF (Progressive Morphological Filter) ────────────────────────────────

    def _classify_pmf(self,
                      pts: np.ndarray
                      ) -> tuple[np.ndarray, np.ndarray]:
        """Progressive Morphological Filter using scipy."""
        from scipy import ndimage
        
        print(f"  [GroundClassifier] Using Progressive Morphological Filter (PMF)")
        print(f"  [GroundClassifier] Cell size: {self.cell_size}m, Window: {self.window}m")
        
        # Create grid
        x_min, y_min = pts[:, 0].min(), pts[:, 1].min()
        x_max, y_max = pts[:, 0].max(), pts[:, 1].max()
        
        cell = self.cell_size
        nx = max(1, int(np.ceil((x_max - x_min) / cell)))
        ny = max(1, int(np.ceil((y_max - y_min) / cell)))
        
        # Initialize minimum elevation grid
        z_min_grid = np.full((ny, nx), np.inf)
        
        # Grid indices for each point
        ix = np.clip(((pts[:, 0] - x_min) / cell).astype(int), 0, nx - 1)
        iy = np.clip(((pts[:, 1] - y_min) / cell).astype(int), 0, ny - 1)
        
        # Find minimum Z in each cell
        for i in range(len(pts)):
            if pts[i, 2] < z_min_grid[iy[i], ix[i]]:
                z_min_grid[iy[i], ix[i]] = pts[i, 2]
        
        # Replace inf with nearest valid value
        mask = np.isinf(z_min_grid)
        if mask.any():
            from scipy.ndimage import distance_transform_edt
            _, nearest_idx = distance_transform_edt(mask, return_indices=True)
            z_min_grid[mask] = z_min_grid[nearest_idx[0][mask], nearest_idx[1][mask]]
        
        # Progressive morphological opening
        max_window_cells = max(3, int(self.window / cell))
        ground_surface = z_min_grid.copy()
        
        for w in range(3, max_window_cells + 1, 2):
            opened = ndimage.grey_opening(ground_surface, size=(w, w))
            # Only update where the difference is within slope tolerance
            diff = ground_surface - opened
            slope_limit = self.slope * cell * (w - 1) / 2
            update_mask = diff <= slope_limit
            ground_surface = np.where(update_mask, opened, ground_surface)
        
        # Classify points
        ground_z = ground_surface[iy, ix]
        ground_mask = (pts[:, 2] - ground_z) <= self.threshold
        
        return pts[ground_mask], pts[~ground_mask]

    # ── Local Minimum ─────────────────────────────────────────────────────────

    def _classify_local_minimum(self,
                                pts: np.ndarray
                                ) -> tuple[np.ndarray, np.ndarray]:
        """Simple local minimum ground detection."""
        print(f"  [GroundClassifier] Using local minimum method")
        
        cell = self.cell_size
        x_min, y_min = pts[:, 0].min(), pts[:, 1].min()
        x_max, y_max = pts[:, 0].max(), pts[:, 1].max()
        
        nx = max(1, int(np.ceil((x_max - x_min) / cell)))
        ny = max(1, int(np.ceil((y_max - y_min) / cell)))
        
        z_min_grid = np.full((ny, nx), np.inf)
        
        ix = np.clip(((pts[:, 0] - x_min) / cell).astype(int), 0, nx - 1)
        iy = np.clip(((pts[:, 1] - y_min) / cell).astype(int), 0, ny - 1)
        
        for i in range(len(pts)):
            if pts[i, 2] < z_min_grid[iy[i], ix[i]]:
                z_min_grid[iy[i], ix[i]] = pts[i, 2]
        
        # Handle empty cells
        mask = np.isinf(z_min_grid)
        if mask.any():
            valid_z = z_min_grid[~mask].mean() if (~mask).any() else 0
            z_min_grid[mask] = valid_z
        
        ground_z = z_min_grid[iy, ix]
        ground_mask = (pts[:, 2] - ground_z) <= self.threshold
        
        return pts[ground_mask], pts[~ground_mask]

    # ── Percentile fallback ───────────────────────────────────────────────────

    def _classify_percentile(self,
                             pts: np.ndarray
                             ) -> tuple[np.ndarray, np.ndarray]:
        """Simple Z-percentile fallback."""
        print("  [GroundClassifier] Using Z-percentile fallback")
        z_thresh = np.percentile(pts[:, 2], 20)
        ground_mask = pts[:, 2] <= z_thresh
        return pts[ground_mask], pts[~ground_mask]
