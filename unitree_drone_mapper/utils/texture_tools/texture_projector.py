"""
texture_projector.py — Per-vertex colour accumulation with occlusion culling.

Iterates over all camera frames recorded during a flight, projects each
mesh vertex into the camera image using the interpolated SLAM pose, and
accumulates the sampled pixel colour into a weighted per-vertex buffer.
After all frames have been processed, dividing accumulated colour by
accumulated weight yields the final per-vertex RGB colour.

Algorithm per frame
-------------------
  1. Retrieve the drone pose at the frame timestamp via PoseInterpolator.
  2. Derive the camera world pose via CameraModel.get_camera_world_pose().
  3. Transform all mesh vertices from the world frame into the camera frame
     via CameraModel.transform_points_to_camera().
  4. Project each vertex to a pixel coordinate via CameraModel.project_point().
  5. Reject vertices that are behind the camera or outside the image.
  6. Apply Z-buffer occlusion culling using Open3D's RaycastingScene:
     cast a ray from the camera origin toward each candidate vertex and
     check whether the ray reaches the vertex depth without hitting
     intervening geometry first.
  7. Sample the image at the projected pixel (bilinear interpolation).
  8. Accumulate:
       colour_accum[i]  += sampled_colour * weight
       weight_accum[i]  += weight

  Weight is 1.0 per visible vertex per frame by default.  Optionally
  scale by cos(angle of incidence) for higher-quality blending.

Multi-frame accumulation
------------------------
  Running the projection over all frames (rather than using a single
  best frame) fills gaps from occlusion and smooths colour noise from
  JPEG compression and varying illumination across the flight path.

Untextured vertices
-------------------
  Vertices that receive zero weight across all frames are coloured grey
  (RGB 128, 128, 128) as a neutral placeholder so the mesh is always
  fully coloured in the browser viewer.

Dependencies
------------
  open3d ≥ 0.19.0 (ARM64 conda-forge wheel)
  numpy, opencv-python
  camera_model.CameraModel
  pose_interpolator.PoseInterpolator
"""

from __future__ import annotations

import time
from typing import Optional

import cv2
import numpy as np


# Grey placeholder for vertices that receive no colour from any frame
_UNTEXTURED_GREY = np.array([128, 128, 128], dtype=np.uint8)

# Minimum accumulated weight for a vertex to be considered textured
_WEIGHT_THRESHOLD = 0.01


class TextureProjector:
    """
    Accumulates per-vertex colour across camera frames with occlusion culling.

    Parameters
    ----------
    mesh : open3d.geometry.TriangleMesh
        The LiDAR mesh produced by the post-processing pipeline.
        Vertices are read once at construction and never modified here —
        only the colour buffer is written.

    camera : CameraModel
        Loaded camera model providing projection and pose transform methods.

    use_distortion : bool
        If True, apply lens distortion correction to projected pixel
        coordinates before colour sampling.  Costs ~2ms per frame.
        Default False — distortion is small at the survey altitudes used.

    angle_weighting : bool
        If True, weight each colour sample by cos(angle of incidence)
        between the camera ray and the vertex normal.  Produces smoother
        colour blending at grazing angles.  Requires vertex normals.
        Default False.
    """

    def __init__(
        self,
        mesh,
        camera,
        use_distortion:  bool = False,
        angle_weighting: bool = False,
    ) -> None:
        try:
            import open3d as o3d
        except ImportError as exc:
            raise ImportError(
                "[TextureProjector] open3d is required.\n"
                "  conda install -c conda-forge open3d"
            ) from exc

        self._camera          = camera
        self._use_distortion  = use_distortion
        self._angle_weighting = angle_weighting

        # Extract vertex array once — shape (N, 3) float64
        self._vertices = np.asarray(mesh.vertices, dtype=np.float64)
        n = len(self._vertices)

        # Per-vertex accumulation buffers
        self._colour_accum = np.zeros((n, 3), dtype=np.float64)
        self._weight_accum = np.zeros(n,      dtype=np.float64)

        # Pre-compute vertex normals if angle weighting is requested
        self._normals: Optional[np.ndarray] = None
        if angle_weighting:
            if not mesh.has_vertex_normals():
                mesh.compute_vertex_normals()
            self._normals = np.asarray(mesh.vertex_normals, dtype=np.float64)

        # Build Open3D raycasting scene for occlusion culling
        # t.geometry requires the tensor geometry API (Open3D ≥ 0.13)
        scene = o3d.t.geometry.RaycastingScene()
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        scene.add_triangles(mesh_t)
        self._scene = scene

        self._frames_processed = 0
        print(
            f"  [TextureProjector] Initialised  "
            f"vertices={n:,}  "
            f"distortion={use_distortion}  "
            f"angle_weighting={angle_weighting}"
        )

    # ── Public API ─────────────────────────────────────────────────────────────

    def project_frame(
        self,
        image: np.ndarray,
        camera_world_pose: np.ndarray,
    ) -> int:
        """
        Project one camera frame onto the mesh and accumulate colour.

        Parameters
        ----------
        image : np.ndarray, shape (H, W, 3), dtype uint8
            BGR camera frame (OpenCV convention).
        camera_world_pose : np.ndarray, shape (4, 4)
            Camera pose in the world frame, as returned by
            CameraModel.get_camera_world_pose().

        Returns
        -------
        int
            Number of vertices that received colour from this frame.
        """
        # Step 1 — Transform all vertices into the camera frame
        pts_cam = self._camera.transform_points_to_camera(
            self._vertices, camera_world_pose
        )

        # Step 2 — Reject points behind the camera (Z <= 0)
        in_front = pts_cam[:, 2] > 0.0
        if not np.any(in_front):
            return 0

        # Step 3 — Project to pixel coordinates
        # Vectorised: compute u/v for all in-front vertices at once
        z   = pts_cam[in_front, 2]
        u_f = self._camera.fx * (pts_cam[in_front, 0] / z) + self._camera.cx
        v_f = self._camera.fy * (pts_cam[in_front, 1] / z) + self._camera.cy

        # Step 4 — Reject points outside image bounds
        h, w = image.shape[:2]
        in_bounds = (
            (u_f >= 0) & (u_f < w - 1) &
            (v_f >= 0) & (v_f < h - 1)
        )

        # Indices into the original vertex array that are candidates
        front_indices   = np.where(in_front)[0]
        candidate_mask  = in_bounds
        candidate_idx   = front_indices[candidate_mask]

        if len(candidate_idx) == 0:
            return 0

        u_c = u_f[candidate_mask]
        v_c = v_f[candidate_mask]
        z_c = z[candidate_mask]

        # Step 5 — Optional distortion correction
        if self._use_distortion:
            u_c, v_c = self._correct_distortion_batch(u_c, v_c)

        # Step 6 — Occlusion culling
        camera_origin = camera_world_pose[:3, 3]
        visible_mask  = self._get_visible_vertices(
            candidate_idx, camera_origin, z_c
        )
        visible_idx = candidate_idx[visible_mask]
        u_v         = u_c[visible_mask]
        v_v         = v_c[visible_mask]

        if len(visible_idx) == 0:
            return 0

        # Step 7 — Sample image at projected coordinates (bilinear)
        colours = self._sample_bilinear(image, u_v, v_v)

        # Step 8 — Compute per-vertex weights
        if self._angle_weighting and self._normals is not None:
            weights = self._compute_angle_weights(
                visible_idx, camera_origin
            )
        else:
            weights = np.ones(len(visible_idx), dtype=np.float64)

        # Step 9 — Accumulate
        # np.add.at handles repeated indices safely (each vertex can only
        # be visible once per frame, so repeats shouldn't occur, but this
        # is defensive)
        np.add.at(self._colour_accum, visible_idx, colours * weights[:, None])
        np.add.at(self._weight_accum, visible_idx, weights)

        self._frames_processed += 1
        return len(visible_idx)

    def get_current_mesh(self):
        """
        Return the mesh with current per-vertex colours applied.

        Vertices with insufficient weight are coloured grey.
        Call at any point during processing for a preview, or call
        finalize() after all frames to get the final result.

        Returns
        -------
        open3d.geometry.TriangleMesh
            A new mesh object with vertex_colors populated.
        """
        import open3d as o3d

        textured     = self._weight_accum >= _WEIGHT_THRESHOLD
        final_colour = np.zeros((len(self._vertices), 3), dtype=np.float64)

        # Normalise textured vertices
        w = np.maximum(self._weight_accum[textured], _WEIGHT_THRESHOLD)
        final_colour[textured] = (
            self._colour_accum[textured] / w[:, None]
        ) / 255.0

        # Grey for untextured vertices
        final_colour[~textured] = _UNTEXTURED_GREY / 255.0

        # Rebuild mesh with colours
        # We cannot modify the raycasting scene mesh in-place; return a
        # fresh TriangleMesh with the same geometry.
        mesh_out = o3d.geometry.TriangleMesh()
        mesh_out.vertices       = o3d.utility.Vector3dVector(self._vertices)
        mesh_out.vertex_colors  = o3d.utility.Vector3dVector(
            np.clip(final_colour, 0.0, 1.0)
        )
        # Triangles are not stored on this object — caller re-uses the
        # original mesh topology if needed, or TextureProjectionStage
        # transfers the colours onto the original mesh.
        return mesh_out

    def finalize(self):
        """
        Return the final textured mesh after all frames have been processed.

        Equivalent to get_current_mesh() but also prints a summary.

        Returns
        -------
        open3d.geometry.TriangleMesh
        """
        n_textured   = int(np.sum(self._weight_accum >= _WEIGHT_THRESHOLD))
        n_total      = len(self._vertices)
        coverage_pct = 100.0 * n_textured / max(n_total, 1)

        print(
            f"  [TextureProjector] Finalised  "
            f"frames={self._frames_processed}  "
            f"textured={n_textured:,}/{n_total:,} "
            f"({coverage_pct:.1f}%)"
        )

        if coverage_pct < 20.0:
            print(
                "  [TextureProjector] WARNING: coverage < 20% — "
                "check extrinsic calibration in config/camera_calibration.yaml. "
                "Expected rotation_rpy=[pi, 0, pi] for a nadir camera "
                "(roll=pi flips optical axis down, yaw=pi matches 180 deg mount rotation)."
            )

        return self.get_current_mesh()

    # ── Private helpers ────────────────────────────────────────────────────────

    def _get_visible_vertices(
        self,
        vertex_indices: np.ndarray,
        camera_origin:  np.ndarray,
        depths_cam:     np.ndarray,
        tolerance_m:    float = 0.05,
    ) -> np.ndarray:
        """
        Z-buffer occlusion test via Open3D RaycastingScene.

        For each candidate vertex, cast a ray from the camera origin toward
        the vertex.  The vertex is visible if the first mesh intersection
        along the ray occurs at approximately the vertex depth (within
        tolerance_m).

        Parameters
        ----------
        vertex_indices : np.ndarray, shape (M,)
            Indices into self._vertices for the candidate set.
        camera_origin : np.ndarray, shape (3,)
            Camera position in the world frame.
        depths_cam : np.ndarray, shape (M,)
            Depth of each candidate vertex in the camera frame (Z_c).
        tolerance_m : float
            Depth tolerance in metres.  Vertices within this distance of
            the first ray intersection are considered visible.

        Returns
        -------
        np.ndarray, shape (M,), dtype bool
            True for visible vertices.

        Implementation note — Open3D tensor API
        ----------------------------------------
        RaycastingScene.cast_rays() accepts an (M, 6) float32 tensor
        [ox, oy, oz, dx, dy, dz] where (ox,oy,oz) is the ray origin and
        (dx,dy,dz) is the unit direction vector.  It returns a dict with
        't_hit' — the distance to the first intersection along each ray.
        """
        import open3d as o3d

        world_pts = self._vertices[vertex_indices]

        # Direction vectors from camera to each vertex (world frame)
        dirs = world_pts - camera_origin[None, :]
        dist_world = np.linalg.norm(dirs, axis=1, keepdims=True)

        # Avoid division by zero for degenerate cases
        valid = (dist_world[:, 0] > 1e-6)
        dirs_unit = np.where(
            valid[:, None],
            dirs / np.maximum(dist_world, 1e-6),
            np.array([[0.0, 0.0, 1.0]])
        )

        # Build (M, 6) ray tensor
        origins   = np.tile(camera_origin.astype(np.float32), (len(world_pts), 1))
        rays_np   = np.hstack([origins, dirs_unit.astype(np.float32)])
        rays_t    = o3d.core.Tensor(rays_np, dtype=o3d.core.Dtype.Float32)

        result    = self._scene.cast_rays(rays_t)
        t_hit     = result["t_hit"].numpy()

        # Vertex is visible if the ray hits at approximately the vertex
        # distance (within tolerance_m).  Invalid rays are marked occluded.
        visible = valid & (t_hit >= dist_world[:, 0] - tolerance_m)
        return visible

    def _sample_bilinear(
        self,
        image: np.ndarray,
        u:     np.ndarray,
        v:     np.ndarray,
    ) -> np.ndarray:
        """
        Sample pixel colours from image at sub-pixel positions using
        bilinear interpolation.

        OpenCV's remap() is used for efficiency on large vertex sets.
        Input u, v are float arrays; output is uint8 BGR colours.

        Returns
        -------
        np.ndarray, shape (M, 3), dtype float64
            BGR colour values in [0, 255].
        """
        u_f32 = u.astype(np.float32).reshape(1, -1)
        v_f32 = v.astype(np.float32).reshape(1, -1)

        sampled = cv2.remap(
            image, u_f32, v_f32,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE,
        )
        # sampled shape: (1, M, 3)
        return sampled[0].astype(np.float64)

    def _correct_distortion_batch(
        self,
        u: np.ndarray,
        v: np.ndarray,
    ):
        """Apply lens distortion correction to a batch of pixel coordinates."""
        import cv2

        K = np.array([
            [self._camera.fx, 0.0,              self._camera.cx],
            [0.0,             self._camera.fy,  self._camera.cy],
            [0.0,             0.0,              1.0            ],
        ], dtype=np.float64)

        pts = np.stack([u, v], axis=1).reshape(-1, 1, 2).astype(np.float64)
        corrected = cv2.undistortPoints(pts, K, self._camera.dist_coeffs, P=K)
        u_c = corrected[:, 0, 0]
        v_c = corrected[:, 0, 1]
        return u_c, v_c

    def _compute_angle_weights(
        self,
        vertex_indices: np.ndarray,
        camera_origin:  np.ndarray,
    ) -> np.ndarray:
        """
        Compute cos(angle of incidence) weight for each visible vertex.

        Weight = max(0, dot(vertex_normal, unit_ray_to_camera)).
        A vertex facing directly toward the camera gets weight 1.0;
        a grazing vertex approaches 0.
        """
        world_pts = self._vertices[vertex_indices]
        normals   = self._normals[vertex_indices]

        ray_to_cam = camera_origin[None, :] - world_pts
        norms      = np.linalg.norm(ray_to_cam, axis=1, keepdims=True)
        ray_unit   = ray_to_cam / np.maximum(norms, 1e-9)

        weights = np.einsum("ij,ij->i", normals, ray_unit)
        return np.maximum(weights, 0.0)
