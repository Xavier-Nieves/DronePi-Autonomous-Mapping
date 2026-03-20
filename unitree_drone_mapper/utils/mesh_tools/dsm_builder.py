"""
dsm_builder.py — Ball Pivoting Algorithm (BPA) mesh from non-ground points.

Takes classified non-ground points (buildings, trees, objects) and produces
a surface mesh that correctly preserves gaps between structures.

Algorithm: Ball Pivoting Algorithm (Bernardini et al. 1999, IEEE TVCG)
    A virtual ball of radius ρ rolls across the point cloud surface.
    A triangle forms only when the ball simultaneously touches three points
    without any other point falling inside the ball.
    The ball pivots around each edge, forming adjacent triangles.
    If the ball cannot reach between two points (gap too large),
    no triangle forms -- the gap is preserved in the mesh.

Why BPA for non-ground (not Delaunay):
    Buildings and trees are fully 3D structures with vertical walls,
    overhangs, and complex geometry. Delaunay 2.5D only allows one Z
    value per XY position -- it cannot represent a vertical wall.
    BPA works in full 3D space and handles arbitrary geometry.

Why BPA over Poisson for non-ground:
    Poisson fills gaps by solving a global function -- it connects the
    left wall of building A to the right wall of building B through the
    air gap between them. BPA only connects points the ball can touch,
    so the gap between buildings remains a gap in the mesh.

Multi-scale radii:
    Three radii are used (r, 2r, 4r) to handle both fine detail (small r)
    and larger flat surfaces like rooftops (large r).

Dependencies: open3d, scipy
"""

import time
import numpy as np


class DSMBuilder:
    """
    Ball Pivoting Algorithm mesh builder for non-ground objects.

    Parameters
    ----------
    radius : float or None
        Ball radius in metres. None = auto-estimate from point density.
        Auto uses 2.5× the median nearest-neighbour distance.
        Manual override useful when density is known.

    radius_scale : tuple of float
        Multipliers applied to base radius for multi-scale BPA.
        Default (1.0, 2.0, 4.0) handles detail + large surfaces.

    nn_sample : int
        Number of points sampled for nearest-neighbour estimation.
        Higher = more accurate radius estimate, slightly slower.

    Example
    -------
        builder = DSMBuilder()                     # auto radius
        builder = DSMBuilder(radius=0.05)          # 5cm fixed radius
        dsm_mesh = builder.build(nonground_pts)
        # dsm_mesh is an open3d TriangleMesh object
    """

    def __init__(self,
                 radius:       float | None  = None,
                 radius_scale: tuple         = (1.0, 2.0, 4.0),
                 nn_sample:    int           = 1000):
        self.radius       = radius
        self.radius_scale = radius_scale
        self.nn_sample    = nn_sample

    def build(self, nonground_pts: np.ndarray):
        """
        Build BPA surface mesh from non-ground points.

        Parameters
        ----------
        nonground_pts : Nx3 float array (classified non-ground points)

        Returns
        -------
        open3d.geometry.TriangleMesh
        Returns None if fewer than 10 non-ground points.
        """
        try:
            import open3d as o3d
        except ImportError:
            print("  [DSMBuilder] open3d not available -- skipping DSM")
            return None

        if len(nonground_pts) < 10:
            print(f"  [DSMBuilder] Too few non-ground points "
                  f"({len(nonground_pts)}) -- skipping DSM")
            return None

        t0 = time.time()

        # ── estimate radius ───────────────────────────────────────────────────
        radius = self.radius or self._estimate_radius(nonground_pts)
        radii  = [radius * s for s in self.radius_scale]
        print(f"  [DSMBuilder] BPA  radius={radius:.4f}m  "
              f"radii={[f'{r:.4f}' for r in radii]}  "
              f"pts={len(nonground_pts):,}")

        # ── build open3d point cloud ──────────────────────────────────────────
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(
            nonground_pts.astype(np.float64))

        # Estimate normals -- required by BPA
        # Use largest radius × 2 as the normal search radius
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radii[-1] * 2,
                max_nn=30
            )
        )
        # Orient normals consistently across surface
        pcd.orient_normals_consistent_tangent_plane(100)

        # ── run BPA ───────────────────────────────────────────────────────────
        mesh = o3d.geometry.TriangleMesh\
            .create_from_point_cloud_ball_pivoting(
                pcd, o3d.utility.DoubleVector(radii)
            )

        # Clean up degenerate geometry
        mesh.remove_degenerate_triangles()
        mesh.remove_unreferenced_vertices()

        faces   = len(np.asarray(mesh.triangles))
        elapsed = time.time() - t0
        print(f"  [DSMBuilder] DSM: {faces:,} triangles  ({elapsed:.1f}s)")
        return mesh

    # ── radius estimation ─────────────────────────────────────────────────────

    def _estimate_radius(self, pts: np.ndarray) -> float:
        """
        Estimate BPA radius from median nearest-neighbour distance.

        Samples nn_sample points and finds each point's closest neighbour.
        The median of those distances × 2.5 gives a stable radius that
        connects adjacent points without bridging across gaps.
        """
        from scipy.spatial import KDTree
        n_sample = min(self.nn_sample, len(pts))
        idx      = np.random.choice(len(pts), n_sample, replace=False)
        sample   = pts[idx]
        tree     = KDTree(sample)
        dists, _ = tree.query(sample, k=2)  # k=2: point itself + nearest
        avg_nn   = float(np.median(dists[:, 1]))
        radius   = avg_nn * 2.5
        print(f"  [DSMBuilder] Auto radius: median NN={avg_nn:.4f}m  "
              f"→ radius={radius:.4f}m")
        return radius
