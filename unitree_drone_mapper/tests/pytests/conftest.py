"""Shared pytest fixtures for mesh_tools unit tests.

These fixtures produce synthetic point clouds that work without
any hardware, ROS, PDAL, or open3d.  All heavy-dep tests that
need open3d are skipped automatically when the library is absent.
"""

import numpy as np
import pytest


@pytest.fixture
def flat_cloud():
    """200×200 flat ground plane at Z≈0 with small Gaussian noise."""
    rng = np.random.default_rng(42)
    x = np.linspace(0, 10, 50)
    y = np.linspace(0, 10, 50)
    xx, yy = np.meshgrid(x, y)
    zz = rng.normal(0.0, 0.02, xx.shape)
    return np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()]).astype(np.float32)


@pytest.fixture
def mixed_cloud():
    """Flat ground at Z≈0 plus a box of objects at Z∈[1, 3]."""
    rng = np.random.default_rng(42)
    x = np.linspace(0, 10, 40)
    y = np.linspace(0, 10, 40)
    xx, yy = np.meshgrid(x, y)
    ground = np.column_stack([
        xx.ravel(), yy.ravel(),
        rng.normal(0.0, 0.02, xx.size),
    ]).astype(np.float32)
    obj_x = rng.uniform(3, 7, 300)
    obj_y = rng.uniform(3, 7, 300)
    obj_z = rng.uniform(1.0, 3.0, 300)
    objects = np.column_stack([obj_x, obj_y, obj_z]).astype(np.float32)
    return np.vstack([ground, objects])


@pytest.fixture
def tiny_cloud():
    """Minimal 5-point cloud — for boundary / too-few-points tests."""
    return np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.5, 0.5, 0.1],
    ], dtype=np.float32)
