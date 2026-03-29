"""Unit tests for DTMBuilder — requires scipy + trimesh (no open3d/ROS)."""

import sys
import os

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "utils"))

try:
    from mesh_tools.dtm_builder import DTMBuilder
    import trimesh  # noqa: F401
    from scipy.spatial import Delaunay  # noqa: F401
    HAS_DEPS = True
except ImportError:
    HAS_DEPS = False

pytestmark = pytest.mark.skipif(not HAS_DEPS, reason="scipy and trimesh required")


@pytest.fixture
def ground_grid():
    """10×10m regular grid of ground points."""
    rng = np.random.default_rng(7)
    x = np.linspace(0, 10, 40)
    y = np.linspace(0, 10, 40)
    xx, yy = np.meshgrid(x, y)
    zz = rng.normal(0.0, 0.01, xx.shape)
    return np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()]).astype(np.float32)


class TestDTMBuilder:

    def test_build_returns_trimesh(self, ground_grid):
        import trimesh
        builder = DTMBuilder(grid_res=0.5)
        mesh = builder.build(ground_grid)
        assert mesh is not None
        assert isinstance(mesh, trimesh.Trimesh)

    def test_mesh_has_faces(self, ground_grid):
        builder = DTMBuilder(grid_res=0.5)
        mesh = builder.build(ground_grid)
        assert len(mesh.faces) > 0

    def test_mesh_has_vertices(self, ground_grid):
        builder = DTMBuilder(grid_res=0.5)
        mesh = builder.build(ground_grid)
        assert len(mesh.vertices) > 0

    def test_too_few_points_returns_none(self):
        pts = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32)
        builder = DTMBuilder()
        result = builder.build(pts)
        assert result is None

    def test_coarser_grid_fewer_faces(self, ground_grid):
        fine   = DTMBuilder(grid_res=0.25).build(ground_grid)
        coarse = DTMBuilder(grid_res=1.00).build(ground_grid)
        assert len(fine.faces) > len(coarse.faces)

    def test_vertex_z_within_cloud_range(self, ground_grid):
        builder = DTMBuilder(grid_res=0.5)
        mesh = builder.build(ground_grid)
        z_min = ground_grid[:, 2].min()
        z_max = ground_grid[:, 2].max()
        mesh_z = mesh.vertices[:, 2]
        assert mesh_z.min() >= z_min - 1e-4
        assert mesh_z.max() <= z_max + 1e-4
