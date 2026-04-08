#!/usr/bin/env python3
"""
test_texture_projection.py — Synthetic scene validation for the texture pipeline.

Validates the complete texture projection chain:
    CameraModel → PoseInterpolator → TextureProjector

WITHOUT requiring any hardware, ROS bag, or live camera.
A synthetic 10×10m ground-plane mesh and a procedurally generated
checkerboard image are used in place of real flight data.

Expected result
---------------
  The output PLY (test_texture_output/textured_ground.ply) should show
  a black-and-white checkerboard pattern projected onto the flat plane.
  Open it in the local meshview or CloudCompare to inspect visually.

  If the plane appears solid grey → extrinsic rotation is wrong.
  If the pattern is shifted/warped → intrinsics need adjustment.
  If only a small fraction of vertices are coloured → projection FOV
    does not cover the mesh; check the synthetic camera pose.

Failure modes and diagnostics
------------------------------
  Each test prints PASS / FAIL with an explanation.  The final
  test (T5) does a pixel-level colour sanity check on the output PLY
  vertex colours, which is the ground truth for the full chain.

Usage
-----
  conda activate dronepi
  python3 tests/test_texture_projection.py

  # Verbose — prints per-vertex projection statistics
  python3 tests/test_texture_projection.py --verbose

  # Keep output files for visual inspection
  python3 tests/test_texture_projection.py --keep-output

Dependencies
------------
  open3d, numpy, scipy, opencv-python, trimesh
  All present in the dronepi conda environment.

This script is STANDALONE — it does not import from any other project
module except texture_tools/.  It adds the project root to sys.path
so texture_tools/ is importable regardless of working directory.
"""

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

# ── Project root on sys.path so texture_tools/ is importable ─────────────────
# This file lives at: unitree_drone_mapper/tests/test_texture_projection.py
# parents[0] = .../tests/
# parents[1] = .../unitree_drone_mapper/
# parents[2] = .../unitree_lidar_project/   ← project root
_TESTS_DIR   = Path(__file__).resolve().parent
_MAPPER_DIR  = _TESTS_DIR.parent
_UTILS_DIR   = _MAPPER_DIR / "utils"

for _p in [str(_MAPPER_DIR), str(_UTILS_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from texture_tools.camera_model      import CameraModel
from texture_tools.pose_interpolator import PoseInterpolator
from texture_tools.texture_projector import TextureProjector

# ── Output directory ──────────────────────────────────────────────────────────
OUTPUT_DIR = _TESTS_DIR / "test_texture_output"

# ── Synthetic camera intrinsics ───────────────────────────────────────────────
# These approximate the IMX477 at a moderate focal length.
# We do NOT use the real calibration file here — the test must be
# hermetic and not depend on the config/ directory being present.
SYNTH_FX    = 1200.0
SYNTH_FY    = 1200.0
SYNTH_CX    = 640.0
SYNTH_CY    = 480.0
SYNTH_W     = 1280
SYNTH_H     = 960
SYNTH_DIST  = [0.0, 0.0, 0.0, 0.0, 0.0]   # No distortion in synthetic test

# ── Test constants ────────────────────────────────────────────────────────────
PLANE_SIZE_M    = 10.0   # ground plane extent (metres)
PLANE_SUBDIV    = 40     # subdivisions per axis → 40×40 = 1600 quads
CAMERA_ALT_M    = 6.0    # synthetic drone altitude above ground (metres)
                         # At fx=1200, W=1280: footprint=6.40x4.80m = 30.7% of 10x10m plane
CHECKER_CELLS   = 8      # checkerboard squares per axis
MIN_COVERAGE    = 0.28   # ≥28% of 10x10m plane must be textured
                         # At alt=6m, fx=1200: footprint covers 30.7% → passes with margin

# ── Result tracking ───────────────────────────────────────────────────────────
_results: list[tuple[str, bool, str]] = []


def _record(name: str, passed: bool, detail: str = "") -> None:
    _results.append((name, passed, detail))
    status = "\033[92mPASS\033[0m" if passed else "\033[91mFAIL\033[0m"
    print(f"  [{status}] {name}" + (f"  — {detail}" if detail else ""))


# ══════════════════════════════════════════════════════════════════════════════
# Synthetic scene builders
# ══════════════════════════════════════════════════════════════════════════════

def make_ground_plane_mesh(size: float, subdivisions: int):
    """
    Build a flat ground plane TriangleMesh of size×size metres centred at origin.

    The plane lies in the Z=0 XY plane (world up = +Z in ENU convention).
    Returns an open3d.geometry.TriangleMesh.

    Reference: Open3D geometry primitives
    https://www.open3d.org/docs/latest/python_api/open3d.geometry.TriangleMesh.html
    """
    import open3d as o3d

    step   = size / subdivisions
    verts  = []
    faces  = []

    # Generate vertices on a regular grid
    for i in range(subdivisions + 1):
        for j in range(subdivisions + 1):
            x = -size / 2 + i * step
            y = -size / 2 + j * step
            verts.append([x, y, 0.0])

    # Generate quad faces (two triangles each)
    cols = subdivisions + 1
    for i in range(subdivisions):
        for j in range(subdivisions):
            v00 = i * cols + j
            v10 = (i + 1) * cols + j
            v01 = i * cols + (j + 1)
            v11 = (i + 1) * cols + (j + 1)
            faces.append([v00, v10, v11])
            faces.append([v00, v11, v01])

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(np.array(verts, dtype=np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(np.array(faces, dtype=np.int32))
    mesh.compute_vertex_normals()
    return mesh


def make_checkerboard_image(width: int, height: int, cells: int) -> np.ndarray:
    """
    Generate a black-and-white checkerboard image as a BGR numpy array.

    Used as the synthetic camera frame.  The pattern has `cells` squares
    per axis, alternating between (0,0,0) black and (255,255,255) white.

    Returns np.ndarray shape (height, width, 3), dtype uint8.
    """
    image    = np.zeros((height, width, 3), dtype=np.uint8)
    cell_w   = width  // cells
    cell_h   = height // cells

    for row in range(cells):
        for col in range(cells):
            if (row + col) % 2 == 0:
                colour = 255
            else:
                colour = 0
            y0 = row * cell_h
            y1 = y0 + cell_h
            x0 = col * cell_w
            x1 = x0 + cell_w
            image[y0:y1, x0:x1] = colour

    return image


def make_synthetic_camera_model(tmp_calib_path: Path) -> CameraModel:
    """
    Write a synthetic camera_calibration.yaml and load it through CameraModel.

    This keeps the test hermetic — it does not read the real calibration file
    in config/ and will pass even on a machine without the project configured.

    The extrinsic is the identity rotation with a straight-down offset:
    camera is directly at the drone body origin, facing +Z down
    (pitch = -π/2 in the RPY convention, which points the camera
    toward -Z in the drone frame, i.e. nadir viewing).
    """
    import yaml

    calib = {
        "camera": {
            "image_width":  SYNTH_W,
            "image_height": SYNTH_H,
            "fx": SYNTH_FX,
            "fy": SYNTH_FY,
            "cx": SYNTH_CX,
            "cy": SYNTH_CY,
            "dist_coeffs": SYNTH_DIST,
        },
        "extrinsic": {
            # Matches the real hardware mount validated 2026-04-05 (sketch + ruler):
            #   translation: x=0 (same fwd pos), y=-0.070m (camera right of LiDAR),
            #                z=+0.030m (camera above LiDAR origin — LiDAR hangs lower)
            #   rotation:    roll=π AND yaw=π.
            #                Rz(π) alone leaves camera +Z pointing UP (as manufactured).
            #                Rx(π) flips +Z → –Z so optical axis points DOWN (nadir).
            #                Combined RPY [π, 0, π]:
            #                  R = [[-1,0,0],[0,1,0],[0,0,-1]]
            #                  cam +X → backward, cam +Y → left, cam +Z → DOWN ✓
            "translation": [0.0, -0.070, 0.030],
            "rotation_rpy": [math.pi, 0.0, math.pi],
        },
    }

    tmp_calib_path.parent.mkdir(parents=True, exist_ok=True)
    with open(tmp_calib_path, "w") as f:
        yaml.safe_dump(calib, f)

    return CameraModel(tmp_calib_path)


def make_drone_pose_at_altitude(altitude: float) -> np.ndarray:
    """
    Return a 4×4 drone body pose hovering at the given altitude above the origin.

    The drone is at position (0, 0, altitude) with no rotation (identity
    orientation, i.e. the body frame is aligned with the world ENU frame).

    In ENU: X=East, Y=North, Z=Up.
    """
    T          = np.eye(4, dtype=np.float64)
    T[2, 3]    = altitude
    return T


# ══════════════════════════════════════════════════════════════════════════════
# Individual tests
# ══════════════════════════════════════════════════════════════════════════════

def test_T1_camera_model_loads(tmp_calib: Path) -> CameraModel:
    """T1 — CameraModel loads a synthetic YAML without raising."""
    try:
        cam = make_synthetic_camera_model(tmp_calib)
        _record(
            "T1 CameraModel load",
            True,
            f"fx={cam.fx:.1f}  fy={cam.fy:.1f}  "
            f"{cam.width}×{cam.height}",
        )
        return cam
    except Exception as exc:
        _record("T1 CameraModel load", False, str(exc))
        raise


def test_T2_project_point_below_camera(cam: CameraModel, altitude: float):
    """
    T2 — A point directly below the camera projects to the principal point.

    With RPY=[π,0,π] and translation=[0,-0.070,0.030]:
      T_body_cam translation = R.T @ -t = [0, +0.070, +0.030]
      To place the camera exactly at (0, 0, altitude), the drone body
      must be at (0, -0.070, altitude-0.030).
    """
    from scipy.spatial.transform import Rotation as _R
    _rpy = [math.pi, 0.0, math.pi]
    _t   = np.array([0.0, -0.070, 0.030])
    _Rm  = _R.from_euler("xyz", _rpy).as_matrix()
    _t_body_cam = -_Rm.T @ _t          # translation part of T_body_cam

    drone_pose = np.eye(4, dtype=np.float64)
    drone_pose[:3, 3] = np.array([0.0, 0.0, altitude]) - _t_body_cam

    cam_pose   = cam.get_camera_world_pose(drone_pose)

    world_point = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
    pt_cam      = cam.transform_points_to_camera(world_point, cam_pose)[0]

    uv = cam.project_point(pt_cam)

    if uv is None:
        _record("T2 nadir project", False,
                f"project_point returned None — pt_cam={pt_cam.round(3).tolist()}")
        return

    u, v  = uv
    u_err = abs(u - SYNTH_CX)
    v_err = abs(v - SYNTH_CY)
    ok    = u_err < 5 and v_err < 5
    _record(
        "T2 nadir project",
        ok,
        f"projected to ({u}, {v})  principal=({SYNTH_CX:.0f}, {SYNTH_CY:.0f})  "
        f"err=({u_err:.1f}, {v_err:.1f})  pt_cam_z={pt_cam[2]:.3f}",
    )


def test_T3_pose_interpolator(altitude: float):
    """
    T3 — PoseInterpolator returns a pose within tolerance of ground truth.

    Adds three synthetic poses at t=0, t=1s, t=2s with linearly increasing
    altitude, then queries at t=1.5s and checks that the returned altitude
    is within 1mm of the expected value.

    SLERP between identity rotations is also identity, so the rotation
    part of the result is verified to be the identity matrix.
    """
    interp = PoseInterpolator()

    for i in range(3):
        ts_ns  = i * 1_000_000_000
        pose   = make_drone_pose_at_altitude(float(i) * altitude)
        interp.add_pose(ts_ns, pose)

    query_ns   = 1_500_000_000   # 1.5 s → expected altitude = 1.5 × altitude
    result     = interp.get_pose_at(query_ns)

    if result is None:
        _record("T3 interpolation", False, "get_pose_at returned None")
        return

    expected_z = 1.5 * altitude
    actual_z   = result[2, 3]
    z_err      = abs(actual_z - expected_z)

    R_err      = np.max(np.abs(result[:3, :3] - np.eye(3)))

    ok = z_err < 0.001 and R_err < 1e-10
    _record(
        "T3 interpolation",
        ok,
        f"z_err={z_err:.6f}m  R_err={R_err:.2e}  "
        f"size={interp.size}  span={interp.time_range_s:.1f}s",
    )


def test_T4_pose_interpolator_out_of_range():
    """
    T4 — PoseInterpolator returns None for timestamps outside valid range.

    Requests before the first pose and beyond the extrapolation limit
    must both return None to prevent bad projections from stale data.
    """
    interp = PoseInterpolator()
    interp.add_pose(1_000_000_000, make_drone_pose_at_altitude(5.0))
    interp.add_pose(2_000_000_000, make_drone_pose_at_altitude(5.0))

    before_first = interp.get_pose_at(0)
    way_after    = interp.get_pose_at(10_000_000_000)   # 10s — far beyond limit

    ok = (before_first is None) and (way_after is None)
    _record(
        "T4 OOB returns None",
        ok,
        f"before_first={'None' if before_first is None else 'NOT None'}  "
        f"way_after={'None' if way_after is None else 'NOT None'}",
    )


def test_T5_full_projection(
    cam:      CameraModel,
    altitude: float,
    verbose:  bool,
    keep:     bool,
) -> bool:
    """
    T5 — Full projection: synthetic mesh + checkerboard → vertex colours.

    Verifies that at least MIN_COVERAGE fraction of ground plane vertices
    receive non-grey colour from the synthetic checkerboard image.

    Also verifies that the projected colours are bimodal (black and white)
    which confirms the checkerboard pattern was correctly sampled rather
    than a solid colour or random noise.

    Saves the result to OUTPUT_DIR/textured_ground.ply when keep=True.
    """
    import open3d as o3d

    print("\n  Building synthetic scene...")
    mesh    = make_ground_plane_mesh(PLANE_SIZE_M, PLANE_SUBDIV)
    image   = make_checkerboard_image(SYNTH_W, SYNTH_H, CHECKER_CELLS)
    n_verts = len(np.asarray(mesh.vertices))

    print(f"    mesh: {n_verts:,} vertices, "
          f"image: {SYNTH_W}×{SYNTH_H} checkerboard ({CHECKER_CELLS}×{CHECKER_CELLS})")

    projector  = TextureProjector(mesh=mesh, camera=cam)

    drone_pose = make_drone_pose_at_altitude(altitude)
    cam_pose   = cam.get_camera_world_pose(drone_pose)

    print(f"    camera world position: {cam_pose[:3, 3].round(3).tolist()}")

    t0      = time.time()
    visible = projector.project_frame(image, cam_pose)
    elapsed = time.time() - t0

    if verbose:
        print(f"\n    project_frame: {visible:,} vertices coloured in {elapsed:.3f}s")
        print(f"    camera pose:\n{cam_pose.round(3)}")

    textured_mesh = projector.finalize()
    colours       = np.asarray(textured_mesh.vertex_colors)  # (N, 3) in [0,1]

    # Coverage check
    grey_threshold   = 0.51   # vertex_colors > 0.5 in all channels → grey placeholder
    is_grey          = np.all(np.abs(colours - 0.502) < 0.01, axis=1)
    n_textured       = int(np.sum(~is_grey))
    coverage         = n_textured / max(n_verts, 1)

    if verbose:
        print(f"\n    textured vertices: {n_textured:,}/{n_verts:,} ({coverage:.1%})")

    coverage_ok = coverage >= MIN_COVERAGE
    _record(
        "T5a coverage",
        coverage_ok,
        f"{n_textured:,}/{n_verts:,} = {coverage:.1%}  (need ≥{MIN_COVERAGE:.0%})",
    )

    # Bimodal colour check — textured vertices should be near black or white
    textured_colours = colours[~is_grey]
    if len(textured_colours) > 0:
        # Convert to greyscale intensity (in [0,1])
        intensity        = textured_colours.mean(axis=1)
        # Bimodal: most pixels are either dark (<0.25) or bright (>0.75)
        dark_or_bright   = np.sum((intensity < 0.25) | (intensity > 0.75))
        bimodal_fraction = dark_or_bright / max(len(intensity), 1)
        bimodal_ok       = bimodal_fraction > 0.60
        _record(
            "T5b bimodal colours",
            bimodal_ok,
            f"{bimodal_fraction:.1%} of textured vertices are dark or bright "
            f"(need >60%)",
        )
    else:
        _record("T5b bimodal colours", False, "no textured vertices to check")
        bimodal_ok = False

    # Optionally save the result
    if keep or (coverage_ok and bimodal_ok):
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        # Transfer colours to a full mesh (with triangles) for saving
        out_mesh = o3d.geometry.TriangleMesh()
        out_mesh.vertices       = mesh.vertices
        out_mesh.triangles      = mesh.triangles
        out_mesh.vertex_colors  = textured_mesh.vertex_colors
        out_path = OUTPUT_DIR / "textured_ground.ply"
        o3d.io.write_triangle_mesh(str(out_path), out_mesh)
        print(f"\n  Saved → {out_path}")
        print(f"  Open in meshview or CloudCompare to inspect the checkerboard pattern.")

    return coverage_ok


def test_T6_behind_camera_rejected(cam: CameraModel, altitude: float):
    """
    T6 — Points behind the camera (above it, in +Z world) must not project.

    With yaw=π and both sensors facing down, the camera optical axis points
    toward –Z world.  A point above the drone (world Z > camera Z) is behind
    the optical axis and must return None from project_point().
    """
    drone_pose   = make_drone_pose_at_altitude(altitude)
    cam_pose     = cam.get_camera_world_pose(drone_pose)

    # Point 2m above the drone body — behind the downward-facing camera
    world_above  = np.array([[0.0, 0.0, altitude + 2.0]], dtype=np.float64)
    pt_cam       = cam.transform_points_to_camera(world_above, cam_pose)[0]

    uv = cam.project_point(pt_cam)

    ok = (uv is None) or (pt_cam[2] <= 0)
    _record(
        "T6 behind-camera rejection",
        ok,
        f"pt_cam_z={pt_cam[2]:.3f}  "
        f"uv={'None (correct)' if uv is None else f'{uv} (unexpected)'}",
    )


def test_T7_extrapolation_within_limit():
    """
    T7 — PoseInterpolator returns a result for timestamps just within the
    50ms extrapolation window, and None just beyond it.
    """
    interp = PoseInterpolator()
    interp.add_pose(0,               make_drone_pose_at_altitude(0.0))
    interp.add_pose(1_000_000_000,   make_drone_pose_at_altitude(5.0))

    # 30ms beyond last pose — within the 50ms limit → should succeed
    within = interp.get_pose_at(1_030_000_000)
    # 80ms beyond last pose — beyond the 50ms limit → should return None
    beyond = interp.get_pose_at(1_080_000_000)

    ok = (within is not None) and (beyond is None)
    _record(
        "T7 extrapolation window",
        ok,
        f"30ms={'ok' if within is not None else 'None (FAIL)'}  "
        f"80ms={'None (ok)' if beyond is None else 'got result (FAIL)'}",
    )


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Synthetic texture projection test — no hardware required."
    )
    parser.add_argument("--verbose",     action="store_true",
                        help="Print per-vertex projection statistics.")
    parser.add_argument("--keep-output", action="store_true",
                        help="Always save the output PLY even if tests fail.")
    args = parser.parse_args()

    print("=" * 60)
    print("  Texture Projection — Synthetic Scene Test Suite")
    print("  No hardware, ROS, or bag file required.")
    print("=" * 60)

    # Temporary calibration YAML written inside the test output dir
    tmp_calib = OUTPUT_DIR / "_synthetic_calib.yaml"

    print("\nDependency check...")
    for pkg in ("open3d", "cv2", "scipy", "yaml"):
        try:
            __import__(pkg)
            print(f"  [ok] {pkg}")
        except ImportError:
            print(f"  [MISSING] {pkg} — install with: pip install {pkg.replace('cv2','opencv-python')}")
            sys.exit(1)

    print("\nRunning tests...\n")

    # T1 — CameraModel load
    cam = test_T1_camera_model_loads(tmp_calib)

    # T2 — Nadir projection to principal point
    test_T2_project_point_below_camera(cam, CAMERA_ALT_M)

    # T3 — Linear translation + SLERP rotation interpolation
    test_T3_pose_interpolator(CAMERA_ALT_M)

    # T4 — Out-of-range timestamps return None
    test_T4_pose_interpolator_out_of_range()

    # T6 — Points behind camera are rejected
    test_T6_behind_camera_rejected(cam, CAMERA_ALT_M)

    # T7 — Extrapolation window is respected
    test_T7_extrapolation_within_limit()

    # T5 — Full projection (most expensive, run last)
    print()
    test_T5_full_projection(cam, CAMERA_ALT_M, args.verbose, args.keep_output)

    # Clean up synthetic calibration file
    try:
        tmp_calib.unlink(missing_ok=True)
    except Exception:
        pass

    # Summary
    print("\n" + "=" * 60)
    passed = sum(1 for _, p, _ in _results if p)
    total  = len(_results)
    print(f"  {passed}/{total} tests passed")
    if passed == total:
        print("  All tests passed — texture pipeline chain is functional.")
        print(
            "\n  Next step: test with a real bag file:\n"
            "    python3 tests/test_texture_from_bag.py "
            "--bag /mnt/ssd/rosbags/<session>"
        )
    else:
        print("  One or more tests FAILED — see details above.")
        print(
            "\n  Most likely causes:\n"
            "    T2 fail: extrinsic rotation_rpy wrong in synthetic calib\n"
            "    T5 fail: projection geometry mismatch — check camera pose math\n"
            "    T5b fail: bilinear sampling off — check image axis convention"
        )
    print("=" * 60)

    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
