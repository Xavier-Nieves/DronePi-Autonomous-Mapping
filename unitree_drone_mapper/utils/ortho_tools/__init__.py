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
        GPS affine applied after stitching via write_world_file().
        Output: np.ndarray (H, W, 3), geo bounds dict

    orthorectifier.py  — FULL PATH (planned)
        Ray-casts each frame pixel through CameraModel into the LiDAR DTM.
        Requires dtm.ply from dtm_builder.py (mesh pipeline must run first).
        Output: np.ndarray with proper ground-sample distance, rasterio CRS.

    tile_cutter.py
        Wraps gdal2tiles on the stitched canvas or orthorectified raster.
        Falls back to Pillow grid splitter when gdal2tiles is unavailable.
        Outputs tiles/{z}/{x}/{y}.png under session ortho/ directory.

    publisher.py
        Writes ortho_metadata.json consumed by meshview.html slippy-map tab.
        Fields: session_id, mode, bounds, tile_url, zoom_min, zoom_max,
                frame_count, gps_count, collage_fallback, processing_s.

Pipeline paths
--------------
    Fast (default):
        FrameIngestor → QualityFilter → MosaicBuilder → TileCutter → OrthoPublisher

    Full (--full flag, requires mesh pipeline to have completed first):
        FrameIngestor → QualityFilter → Orthorectifier → TileCutter → OrthoPublisher
        [Orthorectifier not yet implemented]

Coordinate conventions
----------------------
    Fast path output: pixel coordinates anchored by GPS affine (world file).
                      Not measurement-grade. Suitable for visual reference.
    Full path output: LiDAR DTM-derived ground coordinates. Same SLAM frame
                      as mesh_final.ply. Co-registration with 3D mesh is native.

Dependencies
------------
    opencv-python   — feature matching and stitching (fast path)
    piexif          — GPS EXIF extraction from flight frames
    gdal2tiles      — tile pyramid generation (both paths); Pillow fallback
    Pillow          — tile fallback when gdal2tiles unavailable
    rasterio        — GeoTIFF CRS write (full path, planned)
    texture_tools   — CameraModel, PoseInterpolator (full path, planned)

Install (RPi):
    pip install opencv-python piexif Pillow --break-system-packages
    sudo apt install gdal-bin

Install (Windows/dev):
    pip install opencv-python piexif Pillow gdal2tiles
"""

from .frame_ingestor import FrameIngestor, FrameRecord
from .quality_filter  import QualityFilter
from .mosaic_builder  import MosaicBuilder
from .tile_cutter     import TileCutter
from .publisher       import OrthoPublisher

__all__ = [
    "FrameIngestor",
    "FrameRecord",
    "QualityFilter",
    "MosaicBuilder",
    "TileCutter",
    "OrthoPublisher",
]
