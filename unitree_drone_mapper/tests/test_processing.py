#!/home/dronepi/open3d_env/bin/python3
"""Test post-processing pipeline on sample data.

Run: python3 -m tests.test_processing
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_transforms():
    """Test coordinate transform utilities."""
    from utils.transforms import (
        quaternion_to_rotation_matrix,
        rotation_matrix_to_quaternion,
        euler_to_rotation_matrix,
        build_transform_matrix,
        invert_transform,
        apply_transform,
    )

    # Identity quaternion -> identity rotation
    q = np.array([1.0, 0.0, 0.0, 0.0])
    R = quaternion_to_rotation_matrix(q)
    assert np.allclose(R, np.eye(3)), "Identity quaternion failed"

    # Round-trip quaternion -> matrix -> quaternion
    q_test = np.array([0.7071, 0.7071, 0.0, 0.0])
    q_test = q_test / np.linalg.norm(q_test)
    R = quaternion_to_rotation_matrix(q_test)
    q_back = rotation_matrix_to_quaternion(R)
    # Quaternions can be negated and still represent same rotation
    assert np.allclose(q_test, q_back) or np.allclose(q_test, -q_back), \
        "Quaternion round-trip failed"

    # Transform / inverse
    T = build_transform_matrix(np.array([1.0, 2.0, 3.0]), np.eye(3))
    T_inv = invert_transform(T)
    identity = T @ T_inv
    assert np.allclose(identity, np.eye(4)), "Transform inverse failed"

    # Apply transform to points
    points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    T = build_transform_matrix(np.array([10.0, 0.0, 0.0]), np.eye(3))
    transformed = apply_transform(T, points)
    assert np.allclose(transformed[0], [10.0, 0.0, 0.0]), "Point transform failed"

    print("[PASS] transforms")


def test_cloud_cleaner():
    """Test point cloud cleaning."""
    import open3d as o3d
    from mapping.cloud_cleaner import remove_outliers, voxel_downsample

    # Create a simple point cloud with some outliers
    np.random.seed(42)
    main_points = np.random.randn(1000, 3) * 0.1  # Tight cluster
    outliers = np.random.randn(50, 3) * 10.0  # Far away
    all_points = np.vstack([main_points, outliers])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)

    # Outlier removal should remove most outliers
    clean = remove_outliers(pcd, nb_neighbors=20, std_ratio=2.0)
    assert len(clean.points) < len(pcd.points), "Outlier removal did nothing"
    assert len(clean.points) > 800, "Too many points removed"

    # Downsample
    down = voxel_downsample(clean, voxel_size=0.05)
    assert len(down.points) <= len(clean.points), "Downsample increased points"

    print("[PASS] cloud_cleaner")


def test_mesh_generator():
    """Test mesh generation from point cloud."""
    import open3d as o3d
    from mapping.mesh_generator import estimate_normals, poisson_reconstruction

    # Create a sphere point cloud (should mesh well)
    pcd = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=20)
    pcd = pcd.sample_points_poisson_disk(number_of_points=5000)

    # Estimate normals
    pcd = estimate_normals(pcd)
    assert pcd.has_normals(), "Normal estimation failed"

    # Generate mesh
    mesh = poisson_reconstruction(pcd, depth=6)
    assert len(mesh.vertices) > 0, "No vertices in mesh"
    assert len(mesh.triangles) > 0, "No triangles in mesh"

    print("[PASS] mesh_generator")


def test_trajectory_extractor():
    """Test trajectory operations."""
    from mapping.trajectory_extractor import subsample_trajectory, interpolate_pose_at_time

    poses = [
        {"timestamp": 0.0, "position": [0, 0, 0], "orientation": [1, 0, 0, 0]},
        {"timestamp": 0.1, "position": [0.01, 0, 0], "orientation": [1, 0, 0, 0]},
        {"timestamp": 1.0, "position": [1, 0, 0], "orientation": [1, 0, 0, 0]},
        {"timestamp": 2.0, "position": [2, 0, 0], "orientation": [1, 0, 0, 0]},
    ]

    # Subsample should skip the close second pose
    subsampled = subsample_trajectory(poses, min_distance=0.5)
    assert len(subsampled) < len(poses), "Subsample didn't reduce poses"

    # Interpolation
    interp = interpolate_pose_at_time(poses, 1.5)
    assert abs(interp["position"][0] - 1.5) < 0.01, "Interpolation failed"

    print("[PASS] trajectory_extractor")


if __name__ == "__main__":
    print("Running post-processing tests...\n")

    test_transforms()
    test_trajectory_extractor()

    # These need Open3D
    try:
        import open3d
        test_cloud_cleaner()
        test_mesh_generator()
    except ImportError:
        print("[SKIP] cloud_cleaner and mesh_generator (Open3D not installed)")

    print("\nAll tests passed!")
