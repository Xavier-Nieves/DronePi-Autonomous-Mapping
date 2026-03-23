"""
ground_classifier.py — Ground/non-ground classification for point clouds.

Separates the point cloud into two arrays:
  ground_pts    → bare earth, feed to DTMBuilder
  nonground_pts → buildings, trees, objects, feed to DSMBuilder

Methods available:
  1. PDAL SMRF (best) - Morphological filter, requires pdal package
  2. Progressive Morphological Filter (PMF) - Good fallback using scipy
  3. Local minimum analysis - Simple but effective fallback

The classifier tries methods in order until one succeeds.

Dependencies: 
  - pdal (optional, best results)
  - numpy (required)
  - scipy (required for PMF)
"""

import time
import numpy as np


class GroundClassifier:
    """
    Ground/non-ground classifier with multiple algorithm options.

    Parameters
    ----------
    method : str
        Classification method: 'auto', 'smrf', 'pmf', 'local'
        'auto' tries methods in order: smrf → pmf → local
    
    slope : float
        Maximum ground slope (degrees per metre). Default 0.3
    
    window : float
        Maximum window size in metres. Default 18.0
    
    threshold : float
        Height threshold above ground surface (metres). Default 0.5
    
    cell_size : float
        Grid cell size for ground estimation (metres). Default 1.0

    Example
    -------
        classifier = GroundClassifier(method='auto')
        ground_pts, nonground_pts = classifier.classify(pts)
    """

    def __init__(self,
                 method:     str   = 'auto',
                 slope:      float = 0.3,
                 window:     float = 18.0,
                 threshold:  float = 0.5,
                 cell_size:  float = 1.0):
        self.method    = method.lower()
        self.slope     = slope
        self.window    = window
        self.threshold = threshold
        self.cell_size = cell_size

    def classify(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Classify points into ground and non-ground.

        Returns
        -------
        ground_pts    : Mx3 float array
        nonground_pts : Kx3 float array
        """
        t0 = time.time()
        print(f"  [GroundClassifier] Method={self.method}  threshold={self.threshold}m  pts={len(pts):,}")

        ground, nonground = None, None
        method_used = None

        if self.method == 'auto':
            # Try methods in order of quality
            methods = [
                ('smrf', self._classify_pdal),
                ('pmf', self._classify_pmf),
                ('local', self._classify_local_minimum),
            ]
            
            for name, func in methods:
                try:
                    ground, nonground = func(pts)
                    method_used = name
                    break
                except Exception as e:
                    print(f"  [GroundClassifier] {name.upper()} unavailable: {e}")
                    continue
        else:
            method_map = {
                'smrf': self._classify_pdal,
                'pmf': self._classify_pmf,
                'local': self._classify_local_minimum,
            }
            
            if self.method in method_map:
                try:
                    ground, nonground = method_map[self.method](pts)
                    method_used = self.method
                except Exception as e:
                    print(f"  [GroundClassifier] {self.method.upper()} failed: {e}")
                    ground, nonground = self._classify_local_minimum(pts)
                    method_used = 'local'

        if ground is None:
            ground, nonground = self._classify_local_minimum(pts)
            method_used = 'local'

        pct = 100.0 * len(ground) / max(len(pts), 1)
        elapsed = time.time() - t0
        print(f"  [GroundClassifier] Used: {method_used.upper()}")
        print(f"  [GroundClassifier] Ground: {len(ground):,} ({pct:.1f}%)  "
              f"Non-ground: {len(nonground):,} ({100-pct:.1f}%)  ({elapsed:.1f}s)")
        
        return ground, nonground

    # ══════════════════════════════════════════════════════════════════════════
    # METHOD 1: PDAL SMRF (Best quality)
    # ══════════════════════════════════════════════════════════════════════════
    def _classify_pdal(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Run SMRF filter via PDAL."""
        import pdal
        import json

        print(f"  [GroundClassifier] Using PDAL SMRF (slope={self.slope}, window={self.window}m)")
        
        structured = np.zeros(len(pts), dtype=[
            ("X", np.float64), ("Y", np.float64), ("Z", np.float64),
        ])
        structured["X"] = pts[:, 0]
        structured["Y"] = pts[:, 1]
        structured["Z"] = pts[:, 2]

        pipeline_def = json.dumps({
            "pipeline": [{
                "type": "filters.smrf",
                "slope": self.slope,
                "window": self.window,
                "threshold": self.threshold,
                "scalar": 1.25,
            }]
        })

        pipeline = pdal.Pipeline(pipeline_def)
        pipeline.execute_with_array(structured)

        result = pipeline.arrays[0]
        ground_mask = result["Classification"] == 2
        return pts[ground_mask], pts[~ground_mask]

    # ══════════════════════════════════════════════════════════════════════════
    # METHOD 2: Progressive Morphological Filter (PMF)
    # ══════════════════════════════════════════════════════════════════════════
    def _classify_pmf(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Progressive Morphological Filter using scipy.
        
        Algorithm:
        1. Create elevation grid from point cloud
        2. Apply morphological opening with increasing window sizes
        3. Points close to the opened surface are ground
        """
        from scipy import ndimage
        
        print(f"  [GroundClassifier] Using Progressive Morphological Filter (PMF)")
        print(f"  [GroundClassifier] Cell size: {self.cell_size}m, Window: {self.window}m")
        
        # Create grid
        x_min, y_min = pts[:, 0].min(), pts[:, 1].min()
        x_max, y_max = pts[:, 0].max(), pts[:, 1].max()
        
        cell_size = self.cell_size
        nx = int(np.ceil((x_max - x_min) / cell_size)) + 1
        ny = int(np.ceil((y_max - y_min) / cell_size)) + 1
        
        # Compute cell indices
        xi = np.clip(((pts[:, 0] - x_min) / cell_size).astype(int), 0, nx - 1)
        yi = np.clip(((pts[:, 1] - y_min) / cell_size).astype(int), 0, ny - 1)
        
        # Create minimum elevation grid (use 10th percentile for robustness)
        min_grid = np.full((ny, nx), np.inf)
        cell_id = yi * nx + xi
        
        for cid in np.unique(cell_id):
            mask = cell_id == cid
            cy, cx = divmod(cid, nx)
            # Use 10th percentile to handle noise
            min_grid[cy, cx] = np.percentile(pts[mask, 2], 10)
        
        # Fill empty cells
        empty_mask = np.isinf(min_grid)
        if np.any(empty_mask):
            # Fill with nearest valid value
            valid_grid = np.where(empty_mask, np.nan, min_grid)
            indices = ndimage.distance_transform_edt(
                empty_mask, return_distances=False, return_indices=True)
            min_grid = min_grid[tuple(indices)]
        
        # Progressive morphological opening
        ground_surface = min_grid.copy()
        max_window_cells = max(3, int(self.window / cell_size))
        
        # Use exponentially increasing window sizes
        window_sizes = [3]
        w = 3
        while w < max_window_cells:
            w = min(w * 2 + 1, max_window_cells)
            window_sizes.append(w)
        
        for window_size in window_sizes:
            # Morphological opening
            struct = np.ones((window_size, window_size))
            opened = ndimage.grey_opening(ground_surface, footprint=struct)
            
            # Elevation threshold based on slope and window size
            elev_threshold = self.slope * window_size * cell_size
            
            # Update surface where change is within threshold
            diff = ground_surface - opened
            ground_surface = np.where(diff <= elev_threshold, opened, ground_surface)
        
        # Light smoothing of final surface
        ground_surface = ndimage.uniform_filter(ground_surface, size=3, mode='nearest')
        
        # Classify points
        ground_z = ground_surface[yi, xi]
        height_above = pts[:, 2] - ground_z
        
        # Ground = within threshold above (and slightly below for noise)
        ground_mask = (height_above >= -0.2) & (height_above <= self.threshold)
        
        return pts[ground_mask], pts[~ground_mask]

    # ══════════════════════════════════════════════════════════════════════════
    # METHOD 3: Local Minimum Analysis
    # ══════════════════════════════════════════════════════════════════════════
    def _classify_local_minimum(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Local minimum-based classification.
        
        Algorithm:
        1. Remove outliers
        2. Grid the XY space
        3. Find local minimum Z in each cell
        4. Smooth to create ground surface estimate
        5. Classify based on height above surface
        """
        from scipy import ndimage
        
        print(f"  [GroundClassifier] Using Local Minimum Analysis")
        
        # Step 1: Remove Z outliers
        z = pts[:, 2]
        q1, q3 = np.percentile(z, [5, 95])
        iqr = q3 - q1
        z_lower, z_upper = q1 - 2.0 * iqr, q3 + 2.0 * iqr
        
        valid_mask = (z >= z_lower) & (z <= z_upper)
        n_outliers = np.sum(~valid_mask)
        if n_outliers > 0:
            pct = 100 * n_outliers / len(pts)
            print(f"  [GroundClassifier] Filtered {n_outliers:,} outliers ({pct:.1f}%)")
        
        # Step 2: Create grid
        x_min, y_min = pts[:, 0].min(), pts[:, 1].min()
        x_max, y_max = pts[:, 0].max(), pts[:, 1].max()
        
        cell_size = self.cell_size
        nx = int(np.ceil((x_max - x_min) / cell_size)) + 1
        ny = int(np.ceil((y_max - y_min) / cell_size)) + 1
        
        xi = np.clip(((pts[:, 0] - x_min) / cell_size).astype(int), 0, nx - 1)
        yi = np.clip(((pts[:, 1] - y_min) / cell_size).astype(int), 0, ny - 1)
        
        # Step 3: Find minimum Z in each cell
        min_grid = np.full((ny, nx), np.nan)
        cell_id = yi * nx + xi
        
        for cid in np.unique(cell_id):
            mask = (cell_id == cid) & valid_mask
            if np.any(mask):
                cy, cx = divmod(cid, nx)
                min_grid[cy, cx] = np.percentile(pts[mask, 2], 5)
        
        # Step 4: Fill gaps and smooth
        nan_mask = np.isnan(min_grid)
        if np.any(nan_mask):
            indices = ndimage.distance_transform_edt(
                nan_mask, return_distances=False, return_indices=True)
            min_grid = min_grid[tuple(indices)]
        
        # Morphological opening to remove small bumps (objects)
        struct = np.ones((5, 5))
        ground_surface = ndimage.grey_opening(min_grid, footprint=struct)
        
        # Smooth
        ground_surface = ndimage.uniform_filter(ground_surface, size=3, mode='nearest')
        
        # Step 5: Classify
        ground_z = ground_surface[yi, xi]
        height_above = pts[:, 2] - ground_z
        
        ground_mask = (height_above >= -0.2) & (height_above <= self.threshold)
        
        print(f"  [GroundClassifier] Grid: {nx}×{ny} cells @ {cell_size}m")
        
        return pts[ground_mask], pts[~ground_mask]
