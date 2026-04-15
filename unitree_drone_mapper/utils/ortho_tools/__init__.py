"""
ortho_tools/ — Post-flight orthomosaic and slippy-map tile pipeline for DronePi.

Produces a navigable 2D tile map from IMX477 flight frames, served by
meshview.html as a pan/zoom reference layer alongside the 3D mesh viewer.

Architecture
------------
This package follows the same modular class pattern as mesh_tools/ and
texture_tools/. Each module is a single class. postprocess_ortho.py is the
orchestrator that calls them in sequence — it is never imported by this package.

Module dependency chain
-----------------------
    frame_ingestor.py
        Reads flight_frames/<session>/ written by camera_capture.py.
        Parses ros_timestamp from each sidecar JSON.
        Calls PoseInterpolator (from texture_tools) to match SLAM pose.
        Output: list[FrameRecord]

    quality_filter.py
        Accepts list[FrameRecord], rejects blurry frames via Laplacian variance.
        Output: filtered list[FrameRecord]

    mosaic_builder.py  — FAST PATH
        Wraps cv2.Stitcher_create(). Produces a single stitched numpy canvas.
        No georeferencing — pixel coordinates only. GPS affine applied after.
        Output: np.ndarray (H, W, 3)

    orthorectifier.py  — FULL PATH
        Ray-casts each frame pixel through CameraModel into the LiDAR DTM.
        Requires dtm.ply from dtm_builder.py (mesh pipeline must run first).
        Output: np.ndarray with proper ground-sample distance, rasterio CRS.

    tile_cutter.py
        Wraps gdal2tiles on either mosaic or orthorectified canvas.
        Outputs tiles/{z}/{x}/{y}.png under the session ortho/ directory.

    publisher.py
        Writes ortho_metadata.json consumed by meshview.html slippy-map tab.
        Fields: session_id, mode, bounds, tile_zoom_range, gps_waypoints.

Pipeline paths
--------------
    Fast (default):
        FrameIngestor → QualityFilter → MosaicBuilder → TileCutter → Publisher

    Full (--full flag, requires mesh pipeline to have completed first):
        FrameIngestor → QualityFilter → Orthorectifier → TileCutter → Publisher

Coordinate conventions
----------------------
    Fast path output: pixel coordinates anchored by GPS affine transform.
                      Not measurement-grade. Suitable for visual reference.
    Full path output: LiDAR DTM-derived ground coordinates. Same SLAM frame
                      as mesh_final.ply. Co-registration with 3D mesh is native.

Dependencies
------------
    opencv-python   — feature matching and stitching (fast path)
    rasterio        — GeoTIFF CRS write (full path)
    gdal2tiles      — tile pyramid generation (both paths)
    texture_tools   — CameraModel, PoseInterpolator (full path, shared)

Install:
    pip install opencv-python rasterio gdal2tiles --break-system-packages
"""

from .frame_ingestor import FrameIngestor, FrameRecord

__all__ = [
    "FrameIngestor",
    "FrameRecord",
]
