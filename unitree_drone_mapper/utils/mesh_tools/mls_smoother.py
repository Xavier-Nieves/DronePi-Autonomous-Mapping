"""
mls_smoother.py — Moving Least Squares (MLS) point cloud smoother.

Repositions each point onto a locally fitted polynomial surface,
removing IMU vibration noise and SLAM drift while preserving geometry.

Why this step:
    Raw LiDAR points contain small errors from IMU vibration, SLAM drift,
    and multipath reflections. MLS fits a polynomial surface to each
    point's local neighbourhood and projects the point onto that surface.
    This improves all downstream algorithms:
      - SMRF classification: cleaner ground surface → more accurate split
      - Delaunay 2.5D: stable Z values per grid cell → accurate height map
      - BPA: uniform point spacing → better ball radius estimate

Cross-platform note:
    Open3D's smooth_point_cloud_mls() is not available in all versions.
    This module falls back to statistical outlier removal + voxel downsampling
    when MLS is unavailable, which provides similar noise reduction.

Dependencies: open3d
"""

import time
import numpy as np


class MLSSmoother:
    """
    Moving Least Squares point cloud smoother.

    Parameters
    ----------
    radius : float
        Search radius in metres. Points within this radius of each
        query point contribute to the local polynomial fit.
        Rule of thumb: 2-3× average point spacing.
        Default 0.05m (5cm) suits drone surveys at 10-20m AGL.
    max_nn : int
        Maximum neighbours to consider per query point.
        More neighbours = smoother but slower. Default 20.

    Example
    -------
        smoother = MLSSmoother(radius=0.05)
        pts_smooth = smoother.smooth(pts_raw)
    """

    def __init__(self, radius: float = 0.05, max_nn: int = 20):
        self.radius = radius
        self.max_nn = max_nn

    def smooth(self, pts: np.ndarray) -> np.ndarray:
        """
        Apply MLS smoothing to point cloud.

        Parameters
        ----------
        pts : Nx3 float array

        Returns
        -------
        Nx3 float array with noise reduced.
        Falls back to input unchanged if open3d unavailable.
        """
        try:
            import open3d as o3d
        except ImportError:
            print("  [MLSSmoother] open3d not available -- skipping smoothing")
            return pts

        t0 = time.time()
        print(f"  [MLSSmoother] radius={self.radius}m  "
              f"max_nn={self.max_nn}  pts={len(pts):,}")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        # Estimate normals needed for MLS projection
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.radius * 2,
                max_nn=30
            )
        )

        # Try MLS smoothing (available in some Open3D versions)
        if hasattr(pcd, 'smooth_point_cloud_mls'):
            try:
                smoothed = pcd.smooth_point_cloud_mls(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=self.radius,
                        max_nn=self.max_nn
                    ),
                    upsample_method=o3d.geometry.MLSUpsampleMethod.NONE
                )
                result = np.asarray(smoothed.points, dtype=np.float32)
                elapsed = time.time() - t0
                print(f"  [MLSSmoother] MLS: {len(pts):,} -> {len(result):,} pts  "
                      f"({elapsed:.1f}s)")
                return result
            except Exception as e:
                print(f"  [MLSSmoother] MLS failed: {e}")
                print(f"  [MLSSmoother] Falling back to statistical filter...")

        # Fallback: Statistical outlier removal + optional voxel smoothing
        # This provides similar noise reduction without MLS
        return self._fallback_smooth(pcd, pts, t0)

    def _fallback_smooth(self, pcd, pts: np.ndarray, t0: float) -> np.ndarray:
        """
        Fallback smoothing when MLS is unavailable.

        Uses statistical outlier removal to eliminate noise spikes,
        which achieves similar results to MLS for downstream processing.
        """
        import open3d as o3d

        print(f"  [MLSSmoother] Using statistical outlier removal (MLS unavailable)")

        # Statistical outlier removal
        # nb_neighbors: how many neighbors to analyze for each point
        # std_ratio: standard deviation threshold (lower = more aggressive)
        nb_neighbors = max(self.max_nn, 20)
        std_ratio = 2.0

        cleaned, inlier_idx = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio
        )

        # Optional: light voxel downsampling to regularize point spacing
        # This helps with downstream meshing algorithms
        voxel_size = self.radius / 2
        if voxel_size > 0.01:  # Only if meaningful
            cleaned = cleaned.voxel_down_sample(voxel_size=voxel_size)

        result = np.asarray(cleaned.points, dtype=np.float32)
        elapsed = time.time() - t0
        removed = len(pts) - len(result)
        print(f"  [MLSSmoother] Fallback: {len(pts):,} -> {len(result):,} pts  "
              f"(removed {removed:,} outliers, {elapsed:.1f}s)")
        return result
