"""
mesh_merger.py — Combine DTM and DSM into a single unified mesh.

Takes the terrain mesh (DTMBuilder output) and the object mesh
(DSMBuilder output) and concatenates them into one PLY file.

Why merge rather than keep separate:
    The browser viewer serves one mesh file per session. The merged
    mesh is the complete scene -- terrain as the base with buildings
    and objects sitting on top of it, exactly as in the real world.
    Classification was the processing step, not the final output.

Merge strategy:
    trimesh.util.concatenate() combines vertex arrays and face arrays,
    re-indexing faces from the DSM mesh to account for the offset
    introduced by the DTM vertices. The result is a single valid
    trimesh.Trimesh with no duplicate vertices between the two meshes
    (they occupy different Z space so no spatial overlap occurs).

Optional Poisson path:
    When --use-poisson is passed to the orchestrator, this module
    receives a Poisson mesh instead of DTM/DSM and simply saves it
    directly without merging.

Dependencies: trimesh, open3d (for DSM conversion), numpy
"""

import time
import numpy as np
from pathlib import Path


class MeshMerger:
    """
    Combines DTM and DSM meshes into a single output mesh.

    Example
    -------
        merger   = MeshMerger()
        final    = merger.merge(dtm_mesh, dsm_mesh)
        # final is a trimesh.Trimesh ready for export

        # Poisson path
        final    = merger.wrap_poisson(poisson_mesh)
    """

    def merge(self, dtm_mesh, dsm_mesh):
        """
        Merge DTM (trimesh) and DSM (open3d) into one trimesh.Trimesh.

        Handles the case where either mesh is None (one algorithm
        produced no output due to insufficient points).

        Parameters
        ----------
        dtm_mesh : trimesh.Trimesh or None
        dsm_mesh : open3d.geometry.TriangleMesh or None

        Returns
        -------
        trimesh.Trimesh — merged mesh ready for export.
        Returns None if both inputs are None.
        """
        import trimesh

        t0     = time.time()
        meshes = []

        # ── process DTM ───────────────────────────────────────────────────────
        if dtm_mesh is not None:
            dtm_faces = len(dtm_mesh.faces)
            meshes.append(dtm_mesh)
            print(f"  [MeshMerger] DTM: {dtm_faces:,} faces")
        else:
            print(f"  [MeshMerger] DTM: not available (skipped)")

        # ── process DSM (convert open3d → trimesh) ────────────────────────────
        if dsm_mesh is not None:
            dsm_trimesh = self._o3d_to_trimesh(dsm_mesh)
            dsm_faces   = len(dsm_trimesh.faces)
            meshes.append(dsm_trimesh)
            print(f"  [MeshMerger] DSM: {dsm_faces:,} faces")
        else:
            print(f"  [MeshMerger] DSM: not available (skipped)")

        if not meshes:
            print(f"  [MeshMerger] Both DTM and DSM are None -- no mesh output")
            return None

        # ── merge ─────────────────────────────────────────────────────────────
        if len(meshes) == 1:
            merged = meshes[0]
        else:
            merged = trimesh.util.concatenate(meshes)

        total_faces = len(merged.faces)
        elapsed     = time.time() - t0
        print(f"  [MeshMerger] Merged: {total_faces:,} total faces  "
              f"({elapsed:.1f}s)")
        return merged

    def wrap_poisson(self, poisson_mesh):
        """
        Convert an open3d Poisson mesh to trimesh for the publisher.
        Used when --use-poisson flag is passed.

        Parameters
        ----------
        poisson_mesh : open3d.geometry.TriangleMesh

        Returns
        -------
        trimesh.Trimesh
        """
        if poisson_mesh is None:
            return None
        tm = self._o3d_to_trimesh(poisson_mesh)
        print(f"  [MeshMerger] Poisson mesh: {len(tm.faces):,} faces")
        return tm

    # ── conversion helper ─────────────────────────────────────────────────────

    def _o3d_to_trimesh(self, o3d_mesh):
        """Convert open3d TriangleMesh to trimesh.Trimesh."""
        import trimesh
        vertices = np.asarray(o3d_mesh.vertices)
        faces    = np.asarray(o3d_mesh.triangles)
        tm       = trimesh.Trimesh(vertices=vertices, faces=faces)
        tm.fix_normals()
        return tm
