"""
dtm_builder.py — Delaunay 2.5D terrain mesh (DTM) from ground points.

Takes classified ground points and produces a TIN (Triangulated Irregular
Network) height-field mesh -- the industry standard for drone terrain output.

Algorithm: Delaunay 2.5D (height field triangulation)
    1. Project ground points onto XY plane
    2. Bin into grid cells of size grid_res × grid_res metres
    3. Take median Z per cell (robust to noise within cell)
    4. Build 2D Delaunay triangulation of cell centres
    5. Lift each vertex back to its median Z value
    6. Remove triangles spanning coverage gaps (edge length filter)

Why Delaunay for ground (not BPA):
    Ground is a height field -- every XY position has exactly one Z value.
    Delaunay 2.5D is mathematically designed for height fields. It cannot
    produce a dome because there is no Z interpolation across open space.
    BPA would work too, but Delaunay is faster and more stable for flat terrain.

Why not Poisson for ground:
    Poisson fills gaps by solving a global indicator function -- it will
    interpolate across buildings, connect far walls, produce dome artefacts.
    Delaunay only connects points within grid_res × 5 distance.

Dependencies: scipy, trimesh
"""

import time
import numpy as np


class DTMBuilder:
    """
    Delaunay 2.5D terrain mesh builder.

    Parameters
    ----------
    grid_res : float
        XY grid cell size in metres.
        Smaller = more detail, larger file, slower.
        0.10m (10cm) is a good default for drone surveys.
        Typical professional values: 0.05m - 0.25m

    max_edge_factor : float
        Triangles with any edge longer than grid_res × max_edge_factor
        are removed. Prevents bridging across coverage gaps.
        Default 5.0 = remove triangles spanning more than 50cm at 0.10m grid.

    Example
    -------
        builder = DTMBuilder(grid_res=0.10)
        dtm_mesh = builder.build(ground_pts)
        # dtm_mesh is a trimesh.Trimesh object
    """

    def __init__(self,
                 grid_res:        float = 0.10,
                 max_edge_factor: float = 5.0):
        self.grid_res        = grid_res
        self.max_edge_factor = max_edge_factor

    def build(self, ground_pts: np.ndarray):
        """
        Build Delaunay 2.5D terrain mesh from ground points.

        Parameters
        ----------
        ground_pts : Nx3 float array (classified ground points)

        Returns
        -------
        trimesh.Trimesh with terrain mesh.
        Returns None if fewer than 4 ground points.
        """
        import trimesh
        from scipy.spatial import Delaunay

        if len(ground_pts) < 4:
            print(f"  [DTMBuilder] Too few ground points ({len(ground_pts)}) "
                  f"-- skipping DTM")
            return None

        t0 = time.time()
        print(f"  [DTMBuilder] Delaunay 2.5D  grid={self.grid_res}m  "
              f"pts={len(ground_pts):,}")

        # ── Step 1: grid the XY plane ─────────────────────────────────────────
        x_min = ground_pts[:, 0].min()
        y_min = ground_pts[:, 1].min()
        x_max = ground_pts[:, 0].max()
        y_max = ground_pts[:, 1].max()

        x_cells = int(np.ceil((x_max - x_min) / self.grid_res)) + 1
        y_cells = int(np.ceil((y_max - y_min) / self.grid_res)) + 1
        print(f"  [DTMBuilder] Grid: {x_cells} × {y_cells} cells  "
              f"({x_max-x_min:.1f}m × {y_max-y_min:.1f}m)")

        # ── Step 2: bin points into grid cells ────────────────────────────────
        xi = np.clip(
            ((ground_pts[:, 0] - x_min) / self.grid_res).astype(int),
            0, x_cells - 1
        )
        yi = np.clip(
            ((ground_pts[:, 1] - y_min) / self.grid_res).astype(int),
            0, y_cells - 1
        )

        # ── Step 3: median Z per cell ─────────────────────────────────────────
        # Using a dict is straightforward and memory-efficient
        height_map: dict = {}
        for i in range(len(ground_pts)):
            key = (xi[i], yi[i])
            if key not in height_map:
                height_map[key] = []
            height_map[key].append(ground_pts[i, 2])

        # ── Step 4: build vertex array ────────────────────────────────────────
        vertices = np.array([
            [x_min + gx * self.grid_res,
             y_min + gy * self.grid_res,
             float(np.median(zvals))]
            for (gx, gy), zvals in height_map.items()
        ], dtype=np.float64)

        print(f"  [DTMBuilder] Grid vertices: {len(vertices):,}")

        # ── Step 5: 2D Delaunay triangulation ─────────────────────────────────
        tri   = Delaunay(vertices[:, :2])
        faces = tri.simplices

        # ── Step 6: remove long-edge triangles (gap bridging) ─────────────────
        max_edge   = self.grid_res * self.max_edge_factor
        good_faces = []
        for f in faces:
            v0, v1, v2 = vertices[f[0]], vertices[f[1]], vertices[f[2]]
            e0 = np.linalg.norm(v0[:2] - v1[:2])
            e1 = np.linalg.norm(v1[:2] - v2[:2])
            e2 = np.linalg.norm(v2[:2] - v0[:2])
            if max(e0, e1, e2) <= max_edge:
                good_faces.append(f)

        faces   = np.array(good_faces, dtype=np.int32)
        elapsed = time.time() - t0

        mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
        mesh.fix_normals()

        print(f"  [DTMBuilder] DTM: {len(faces):,} triangles  ({elapsed:.1f}s)")
        return mesh
