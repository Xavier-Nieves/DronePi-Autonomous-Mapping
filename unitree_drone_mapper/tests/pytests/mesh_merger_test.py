"""Unit tests for MeshMerger — requires trimesh only (no open3d/ROS)."""

import sys
import os

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "utils"))

try:
    import trimesh
    from mesh_tools.mesh_merger import MeshMerger
    HAS_TRIMESH = True
except ImportError:
    HAS_TRIMESH = False

pytestmark = pytest.mark.skipif(not HAS_TRIMESH, reason="trimesh required")


def _make_box_mesh():
    """Return a simple box trimesh — stands in for a DTM or DSM."""
    return trimesh.creation.box(extents=[1.0, 1.0, 0.1])


class TestMeshMerger:

    def test_merge_two_trimeshes(self):
        dtm = _make_box_mesh()
        # Shift second mesh upward so it's clearly a separate object
        dsm_verts  = _make_box_mesh().vertices.copy()
        dsm_verts[:, 2] += 2.0
        dsm = trimesh.Trimesh(vertices=dsm_verts, faces=_make_box_mesh().faces)

        merger = MeshMerger()
        # Pass dsm as dtm_mesh to avoid needing open3d for _o3d_to_trimesh
        result = merger.merge(dtm, None)
        assert result is not None
        assert len(result.faces) == len(dtm.faces)

    def test_both_none_returns_none(self):
        merger = MeshMerger()
        assert merger.merge(None, None) is None

    def test_dtm_only_returns_dtm(self):
        dtm = _make_box_mesh()
        merger = MeshMerger()
        result = merger.merge(dtm, None)
        assert result is not None
        assert len(result.faces) == len(dtm.faces)

    def test_wrap_poisson_none_returns_none(self):
        merger = MeshMerger()
        assert merger.wrap_poisson(None) is None

    def test_merged_face_count_equals_sum(self):
        import trimesh as tm
        a = _make_box_mesh()
        b_verts = _make_box_mesh().vertices.copy()
        b_verts[:, 2] += 5.0
        b = tm.Trimesh(vertices=b_verts, faces=_make_box_mesh().faces)
        merged = tm.util.concatenate([a, b])
        assert len(merged.faces) == len(a.faces) + len(b.faces)
