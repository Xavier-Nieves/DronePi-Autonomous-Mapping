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

import sys
import threading
import time
import numpy as np


# ── Performance limits ────────────────────────────────────────────────────────
# BPA is O(n²) in practice. These caps keep runtime reasonable.
DEFAULT_MAX_PTS      = 80_000   # Standard mode: ~30-60s on Pi 5
DEFAULT_MAX_PTS_FAST = 50_000   # Fast mode: ~15-30s on laptop
MIN_RADIUS           = 0.03    # Floor to prevent micro-radius from drift
MAX_RADIUS           = 0.50    # Ceiling to prevent mega-radius from drift


# ── BPA progress spinner ─────────────────────────────────────────────────────
# Open3D BPA runs entirely in C++ with no Python callbacks.
# We run a spinner thread showing elapsed time and a pseudo-progress bar
# estimated from empirical timing: ~1s per 5k points for normals,
# ~2s per 5k points for BPA itself on Pi 5.

class _BPAProgress:
    """
    Background thread that prints an animated progress bar while BPA runs.
    Estimated total time is based on point count and hardware benchmarks.
    Call .start() before BPA, .stop() after.
    """
    BAR_WIDTH = 30
    # Empirical seconds-per-1k-points on Pi 5 (conservative estimate)
    SEC_PER_1K_NORMALS = 0.08
    SEC_PER_1K_BPA     = 0.25

    def __init__(self, n_pts: int, stage: str = "normals"):
        self.n_pts    = n_pts
        self.stage    = stage
        self._stop    = threading.Event()
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self.t_start  = None

    @property
    def _estimate_s(self) -> float:
        rate = (self.SEC_PER_1K_NORMALS if self.stage == "normals"
                else self.SEC_PER_1K_BPA)
        return max(5.0, (self.n_pts / 1000) * rate)

    def start(self, stage: str):
        self.stage   = stage
        self.t_start = time.time()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)
        # Clear the spinner line
        sys.stdout.write("\r" + " " * 80 + "\r")
        sys.stdout.flush()

    def _run(self):
        est = self._estimate_s
        while not self._stop.is_set():
            elapsed  = time.time() - self.t_start
            fraction = min(elapsed / est, 0.99)
            filled   = int(self.BAR_WIDTH * fraction)
            bar      = "█" * filled + "░" * (self.BAR_WIDTH - filled)
            pct      = int(fraction * 100)
            eta      = max(0, est - elapsed)
            label    = "normals" if self.stage == "normals" else "BPA"
            msg = (
                f"\r  [DSMBuilder] {label}  [{bar}] {pct:3d}%  "
                f"{elapsed:.0f}s elapsed  ~{eta:.0f}s remaining   "
            )
            sys.stdout.write(msg)
            sys.stdout.flush()
            self._stop.wait(0.5)


class DSMBuilder:
    """
    Ball Pivoting Algorithm mesh builder for non-ground objects.

    Parameters
    ----------
    radius : float or None
        Ball radius in metres. None = auto-estimate from point density.
        Auto uses 2.5× the median nearest-neighbour distance.
        Manual override useful when density is known.
        Clamped to [0.03, 0.50] to handle SLAM drift artifacts.

    radius_scale : tuple of float
        Multipliers applied to base radius for multi-scale BPA.
        Default (1.0, 2.0, 4.0) handles detail + large surfaces.

    nn_sample : int
        Number of points sampled for nearest-neighbour estimation.
        Higher = more accurate radius estimate, slightly slower.

    fast : bool
        Fast mode: single radius, lower point cap. For laptops.

    max_pts : int or None
        Hard cap on points before BPA. Points are uniformly subsampled
        if exceeded. None = use default (80k standard, 50k fast).
        Critical for preventing BPA hangs on large clouds.

    Example
    -------
        builder = DSMBuilder()                     # auto radius, 80k cap
        builder = DSMBuilder(radius=0.05)          # 5cm fixed radius
        builder = DSMBuilder(fast=True)            # laptop mode: 50k cap
        builder = DSMBuilder(max_pts=30_000)       # explicit cap
        dsm_mesh = builder.build(nonground_pts)
        # dsm_mesh is an open3d TriangleMesh object
    """

    def __init__(self,
                 radius:       float | None  = None,
                 radius_scale: tuple         = (1.0, 2.0, 4.0),
                 nn_sample:    int           = 1000,
                 fast:         bool          = False,
                 max_pts:      int | None    = None):
        self.radius       = radius
        # fast mode: single radius only -- quicker but less surface detail
        self.radius_scale = (1.0,) if fast else radius_scale
        self.nn_sample    = nn_sample
        self.fast         = fast
        # Set max_pts with sensible defaults
        if max_pts is not None:
            self.max_pts = max_pts
        else:
            self.max_pts = DEFAULT_MAX_PTS_FAST if fast else DEFAULT_MAX_PTS

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

        # ── apply max_pts cap ─────────────────────────────────────────────────
        pts = self._apply_cap(nonground_pts)
        n_pts = len(pts)

        # ── estimate radius ───────────────────────────────────────────────────
        radius = self.radius or self._estimate_radius(pts)
        # Clamp radius to sane bounds (prevents SLAM drift artifacts)
        radius = max(MIN_RADIUS, min(radius, MAX_RADIUS))
        
        radii  = [radius * s for s in self.radius_scale]
        print(f"  [DSMBuilder] BPA  radius={radius:.4f}m  "
              f"radii={[f'{r:.4f}' for r in radii]}  "
              f"pts={n_pts:,}")

        spinner = _BPAProgress(n_pts)

        # ── build open3d point cloud ──────────────────────────────────────────
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))

        # Estimate normals -- required by BPA
        spinner.start('normals')
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radii[-1] * 2,
                max_nn=30
            )
        )
        # Orient normals consistently across surface
        pcd.orient_normals_consistent_tangent_plane(100)
        spinner.stop()
        t_normals = time.time() - t0
        print(f"  [DSMBuilder] Normals done in {t_normals:.1f}s")

        # ── run BPA ───────────────────────────────────────────────────────────
        spinner.start('bpa')
        mesh = o3d.geometry.TriangleMesh\
            .create_from_point_cloud_ball_pivoting(
                pcd, o3d.utility.DoubleVector(radii)
            )
        spinner.stop()

        # Clean up degenerate geometry
        mesh.remove_degenerate_triangles()
        mesh.remove_unreferenced_vertices()

        faces   = len(np.asarray(mesh.triangles))
        elapsed = time.time() - t0
        print(f"  [DSMBuilder] DSM: {faces:,} triangles  ({elapsed:.1f}s)")
        return mesh

    # ── point cap ─────────────────────────────────────────────────────────────

    def _apply_cap(self, pts: np.ndarray) -> np.ndarray:
        """
        Subsample points if exceeding max_pts cap.
        Uses uniform random sampling to preserve spatial distribution.
        """
        if len(pts) <= self.max_pts:
            return pts
        
        mode_str = "FAST " if self.fast else ""
        print(f"  [DSMBuilder] {mode_str}Downsampling {len(pts):,} → "
              f"{self.max_pts:,} pts (BPA cap)")
        
        idx = np.random.choice(len(pts), self.max_pts, replace=False)
        return pts[idx]

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
              f"→ radius={radius:.4f}m", end="")
        
        # Warn if radius was clamped
        clamped = max(MIN_RADIUS, min(radius, MAX_RADIUS))
        if clamped != radius:
            print(f"  (clamped to {clamped:.4f}m)")
        else:
            print()  # newline
        
        return radius
