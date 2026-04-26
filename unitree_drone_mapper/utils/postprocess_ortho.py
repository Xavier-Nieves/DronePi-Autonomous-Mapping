#!/usr/bin/env python3
"""
postprocess_ortho.py — Post-flight orthomosaic pipeline orchestrator.

Runs the fast-path ortho pipeline against a completed flight session,
producing a slippy-map tile pyramid viewable in meshview.html.

Pipeline (fast path — default)
------------------------------
1. FrameIngestor   Load flight frames from flight_frames/ or a custom dir
2. QualityFilter   Reject blurry frames via Laplacian variance
3. MosaicBuilder   Stitch frames into a single canvas (cv2.Stitcher)
4. TileCutter      Cut canvas into XYZ PNG tiles (gdal2tiles or Pillow)
5. OrthoPublisher  Write ortho_metadata.json for meshview.html

The full path (--full, requires mesh pipeline to have run first) replaces
MosaicBuilder with Orthorectifier, which ray-casts each frame through the
LiDAR DTM for measurement-grade output.  The full path is not yet
implemented — Orthorectifier is a planned module.

This script is the sole entry point for the ortho pipeline.
It is never imported by any production module.

Usage
-----
  # From utils/ — uses most recent scan_* session under /mnt/ssd/rosbags:
  python3 postprocess_ortho.py

  # Specific session:
  python3 postprocess_ortho.py --session /mnt/ssd/rosbags/scan_20260424_221106

  # Custom frames directory (for ODM test):
  python3 postprocess_ortho.py --frames-dir /path/to/images --session-id scan_test

  # Override maps dir:
  python3 postprocess_ortho.py --maps-dir /mnt/ssd/maps

  # Adjust blur threshold:
  python3 postprocess_ortho.py --blur-threshold 50.0

  # Fast mode (smaller stitcher input, fewer tiles):
  python3 postprocess_ortho.py --fast

Exit codes
----------
  0  Success
  1  No frames found or pipeline stage crashed
"""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

# ── sys.path bootstrap ────────────────────────────────────────────────────────
# This file lives at unitree_drone_mapper/utils/postprocess_ortho.py.
# _UTILS_DIR is therefore this file's own directory.
_UTILS_DIR = Path(__file__).resolve().parent
_REPO_ROOT  = _UTILS_DIR.parent.parent          # repo root
for _p in (_UTILS_DIR, _REPO_ROOT):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

# ── Imports ───────────────────────────────────────────────────────────────────
from ortho_tools.frame_ingestor import FrameIngestor, FrameRecord
from ortho_tools.quality_filter  import QualityFilter
from ortho_tools.mosaic_builder  import MosaicBuilder
from ortho_tools.tile_cutter     import TileCutter
from ortho_tools.publisher       import OrthoPublisher

# ── Defaults ──────────────────────────────────────────────────────────────────
_DEFAULT_ROSBAG_DIR = Path("/mnt/ssd/rosbags")
_DEFAULT_MAPS_DIR   = Path("/mnt/ssd/maps")
_BAG_DIR_RE_STR     = r'^(?:scan|flt|flight)_\d{8}_\d{6}$'

import re
_BAG_DIR_RE = re.compile(_BAG_DIR_RE_STR)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _find_latest_session() -> Path | None:
    if not _DEFAULT_ROSBAG_DIR.exists():
        return None
    candidates = sorted(
        [d for d in _DEFAULT_ROSBAG_DIR.iterdir()
         if d.is_dir() and _BAG_DIR_RE.match(d.name)],
        reverse=True,
    )
    return candidates[0] if candidates else None


def _resolve_dirs(args) -> tuple[Path, Path, str]:
    """
    Return (session_dir, maps_dir, session_id).

    Handles three cases:
      1. --session path provided explicitly.
      2. --frames-dir + --session-id provided (test/ODM path).
      3. Auto-detect latest scan_* under /mnt/ssd/rosbags.
    """
    if args.session:
        session_dir = Path(args.session).resolve()
        session_id  = args.session_id or session_dir.name
    elif args.frames_dir:
        # Standalone frames dir — create a synthetic session dir for output
        session_id  = args.session_id or (
            f"scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        )
        session_dir = (
            Path(args.output_root) / session_id
            if args.output_root
            else _resolve_output_root() / session_id
        )
        session_dir.mkdir(parents=True, exist_ok=True)
    else:
        session_dir = _find_latest_session()
        if session_dir is None:
            print(
                f"[FAIL] No scan_* session found in {_DEFAULT_ROSBAG_DIR}",
                file=sys.stderr,
            )
            sys.exit(1)
        session_id = args.session_id or session_dir.name

    maps_dir = (
        Path(args.maps_dir)
        if args.maps_dir
        else (_DEFAULT_MAPS_DIR if _DEFAULT_MAPS_DIR.exists() else session_dir)
    )

    return session_dir, maps_dir, session_id


def _resolve_output_root() -> Path:
    if _DEFAULT_ROSBAG_DIR.exists():
        return _DEFAULT_ROSBAG_DIR
    return _REPO_ROOT


# ── Pipeline ──────────────────────────────────────────────────────────────────

def run(args) -> int:
    session_dir, maps_dir, session_id = _resolve_dirs(args)
    ortho_dir = session_dir / "ortho"
    ortho_dir.mkdir(parents=True, exist_ok=True)

    # Fast-mode overrides
    max_canvas_px = 2048 if args.fast else 4096
    zoom_max      = 17   if args.fast else 19

    print("=" * 60)
    print("  DronePi Ortho Pipeline")
    print(f"  Session  : {session_id}")
    print(f"  Ortho dir: {ortho_dir}")
    print(f"  Maps dir : {maps_dir}")
    print(f"  Mode     : {'FAST' if args.fast else 'standard'}")
    if args.ortho_tif:
        print(f"  Source   : ODM GeoTIFF (Option A)")
    print("=" * 60)

    t_start = time.time()

    try:
        # ── Option A: ODM GeoTIFF bypass ──────────────────────────────────────
        # Skip frame loading, quality filter, and mosaic entirely.
        # Feed the pre-built georeferenced GeoTIFF directly to TileCutter.
        if args.ortho_tif:
            tif_path = Path(args.ortho_tif).resolve()
            if not tif_path.exists():
                print(f"[FAIL] GeoTIFF not found: {tif_path}", file=sys.stderr)
                return 1

            print(f"\n[ODM] Using GeoTIFF: {tif_path.name}")
            print(f"[ODM] Skipping frame load, quality filter, and mosaic.")

            # Extract GPS bounds from GeoTIFF via osgeo if available,
            # otherwise use None (TileCutter will still cut correctly from
            # the embedded CRS — bounds are for metadata display only).
            geo = _bounds_from_geotiff(tif_path)
            if geo:
                print(
                    f"  [ODM] Bounds: "
                    f"lat [{geo['lat_min']:.6f}, {geo['lat_max']:.6f}]  "
                    f"lon [{geo['lon_min']:.6f}, {geo['lon_max']:.6f}]"
                )
            else:
                print("  [ODM] Could not read bounds from GeoTIFF (osgeo not available).")

            print(f"\n[4/5] Cutting tiles from GeoTIFF (zoom {args.zoom_min}–{zoom_max})...")
            cutter = TileCutter(
                zoom_min=args.zoom_min,
                zoom_max=zoom_max,
                processes=args.processes,
                resampling="average",
            )
            tile_dir, z_min, z_max = cutter.cut(
                raster_path=tif_path,
                output_dir=ortho_dir,
                geo=geo,
            )

            print(f"\n[5/5] Publishing ortho_metadata.json...")
            publisher = OrthoPublisher(maps_dir=maps_dir)
            publisher.publish(
                session_id       = session_id,
                session_dir      = session_dir,
                tile_dir         = tile_dir,
                zoom_min         = z_min,
                zoom_max         = z_max,
                mode             = "odm",
                geo              = geo,
                frame_count      = 0,
                collage_fallback = False,
                processing_s     = time.time() - t_start,
            )

            elapsed = time.time() - t_start
            print(f"\n{'='*60}")
            print(f"  Ortho pipeline complete in {elapsed:.1f}s  [ODM GeoTIFF path]")
            print(f"  Tiles : {tile_dir}")
            print(f"  Meta  : {maps_dir / 'ortho_metadata.json'}")
            print(f"  Viewer: open meshview.html → Ortho tab")
            print(f"{'='*60}")
            return 0

        # ── Option B: GPS affine compositor (production flight path) ──────────

        # [1] Frame ingestion
        print(f"\n[1/5] Loading frames...")
        if args.frames_dir:
            frames_path = Path(args.frames_dir)
            print(f"  [Ingest] Direct frames dir: {frames_path}")
            records = _load_frames_direct(frames_path)
        else:
            ingestor = FrameIngestor(session_dir=session_dir, bag_path=None)
            records  = ingestor.load()

        if not records:
            print(f"[FAIL] No frames found.", file=sys.stderr)
            return 1
        print(f"  [Ingest] {len(records)} frames loaded")

        # [2] Quality filter
        print(f"\n[2/5] Quality filtering (blur threshold={args.blur_threshold})...")
        filt    = QualityFilter(blur_threshold=args.blur_threshold, min_frames=5)
        records = filt.filter(records)
        if not records:
            print(f"[FAIL] No frames passed quality filter.", file=sys.stderr)
            return 1

        # [3] GPS affine mosaic
        print(f"\n[3/5] Building mosaic (GPS affine compositor)...")
        builder = MosaicBuilder(
            max_canvas_px        = max_canvas_px,
            altitude_m           = args.altitude,
            use_stitcher         = args.use_stitcher,
            max_frames           = 30 if args.fast else 60,
            scale                = 0.3 if args.fast else 0.5,
            confidence_threshold = args.stitch_confidence,
        )
        canvas, geo = builder.build(records)

        if canvas is None:
            print(f"[FAIL] Mosaic builder returned no canvas.", file=sys.stderr)
            return 1

        collage_fallback = geo is None

        import cv2
        canvas_path = ortho_dir / "mosaic.jpg"
        cv2.imwrite(str(canvas_path), canvas, [cv2.IMWRITE_JPEG_QUALITY, 90])
        print(f"  [Mosaic] Canvas saved: {canvas_path.name}")

        if geo:
            builder.write_world_file(canvas, geo, canvas_path)

        # [4] Tile cutting
        print(f"\n[4/5] Cutting tiles (zoom {args.zoom_min}–{zoom_max})...")
        cutter = TileCutter(
            zoom_min=args.zoom_min,
            zoom_max=zoom_max,
            processes=args.processes,
            resampling="average",
        )
        tile_dir, z_min, z_max = cutter.cut(
            raster_path=canvas_path,
            output_dir=ortho_dir,
            geo=geo,
        )

        # [5] Publish
        print(f"\n[5/5] Publishing ortho_metadata.json...")
        publisher = OrthoPublisher(maps_dir=maps_dir)
        publisher.publish(
            session_id       = session_id,
            session_dir      = session_dir,
            tile_dir         = tile_dir,
            zoom_min         = z_min,
            zoom_max         = z_max,
            mode             = "fast",
            geo              = geo,
            frame_count      = len(records),
            collage_fallback = collage_fallback,
            processing_s     = time.time() - t_start,
        )

    except Exception as exc:
        print(f"\n[FAIL] Pipeline crashed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 1

    elapsed = time.time() - t_start
    print(f"\n{'='*60}")
    print(f"  Ortho pipeline complete in {elapsed:.1f}s")
    print(f"  Tiles : {tile_dir}")
    print(f"  Meta  : {maps_dir / 'ortho_metadata.json'}")
    print(f"  Viewer: open meshview.html → Ortho tab")
    print(f"{'='*60}")
    return 0


def _bounds_from_geotiff(tif_path: Path) -> dict | None:
    """
    Extract WGS-84 bounding box from a GeoTIFF via osgeo.gdal.

    Returns a geo dict compatible with OrthoPublisher, or None if osgeo
    is unavailable (non-fatal — tiles will still be cut correctly from
    the embedded CRS; bounds are only used for metadata display and
    Leaflet auto-center).
    """
    try:
        from osgeo import gdal, osr
        ds = gdal.Open(str(tif_path))
        if ds is None:
            return None
        gt  = ds.GetGeoTransform()
        w, h = ds.RasterXSize, ds.RasterYSize
        # Corner coordinates in source CRS
        src_srs = osr.SpatialReference()
        src_srs.ImportFromWkt(ds.GetProjection())
        wgs84 = osr.SpatialReference()
        wgs84.SetWellKnownGeogCS("WGS84")
        transform = osr.CoordinateTransformation(src_srs, wgs84)
        corners = [
            (gt[0],              gt[3]),
            (gt[0] + w * gt[1],  gt[3]),
            (gt[0],              gt[3] + h * gt[5]),
            (gt[0] + w * gt[1],  gt[3] + h * gt[5]),
        ]
        wgs_corners = [transform.TransformPoint(x, y) for x, y in corners]
        lats = [c[1] for c in wgs_corners]
        lons = [c[0] for c in wgs_corners]
        return {
            "lat_min":   min(lats), "lat_max": max(lats),
            "lon_min":   min(lons), "lon_max": max(lons),
            "gps_count": 0,
        }
    except Exception:
        return None


def _load_frames_direct(frames_dir: Path) -> list[FrameRecord]:
    """
    Build FrameRecord list directly from a JPEG directory.

    GPS is extracted from EXIF using piexif.  pose_4x4 and ros_ts are None
    (no SLAM trajectory available).  Used by the ODM test path.
    """
    import cv2
    from ortho_tools.frame_ingestor import FrameRecord

    jpegs = sorted(frames_dir.glob("*.jpg")) + sorted(frames_dir.glob("*.JPG"))
    if not jpegs:
        return []

    records = []
    for i, path in enumerate(jpegs):
        image = cv2.imread(str(path))
        if image is None:
            continue
        gps = _read_exif_gps(path)

        # Read sidecar JSON for enu_z (SLAM altitude for GSD calculation)
        import json as _json
        enu_z = None
        json_path = path.with_suffix('.json')
        if json_path.exists():
            try:
                sd = _json.loads(json_path.read_text())
                ez = sd.get('enu', {}).get('z')
                enu_z = float(ez) if ez is not None and float(ez) > 0.5 else None
            except Exception:
                pass

        records.append(FrameRecord(
            image        = image,
            pose_4x4     = None,
            gps          = gps,
            ros_ts       = None,
            waypoint_idx = i,
            path         = path,
            enu_z        = enu_z,
    ))
    return records


def _read_exif_gps(path: Path) -> tuple:
    """Extract (lat, lon, alt) from JPEG EXIF GPS IFD via piexif."""
    try:
        import piexif

        exif = piexif.load(str(path))
        gps  = exif.get("GPS", {})

        def _dms_to_decimal(dms, ref):
            d = dms[0][0] / dms[0][1]
            m = dms[1][0] / dms[1][1]
            s = dms[2][0] / dms[2][1]
            v = d + m / 60 + s / 3600
            if ref in (b"S", b"W"):
                v = -v
            return v

        lat_dms = gps.get(piexif.GPSIFD.GPSLatitude)
        lat_ref = gps.get(piexif.GPSIFD.GPSLatitudeRef)
        lon_dms = gps.get(piexif.GPSIFD.GPSLongitude)
        lon_ref = gps.get(piexif.GPSIFD.GPSLongitudeRef)
        alt_raw = gps.get(piexif.GPSIFD.GPSAltitude)

        if lat_dms and lon_dms:
            lat = _dms_to_decimal(lat_dms, lat_ref)
            lon = _dms_to_decimal(lon_dms, lon_ref)
            alt = (alt_raw[0] / alt_raw[1]) if alt_raw else None
            return (lat, lon, alt)

    except Exception:
        pass

    return (None, None, None)


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="DronePi orthomosaic pipeline — GPS affine compositor + ODM GeoTIFF path.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--session",          default=None,
                        help="Path to session directory (auto-detects latest if omitted)")
    parser.add_argument("--session-id",       default=None,
                        help="Override session ID string")
    parser.add_argument("--frames-dir",       default=None,
                        help="Direct JPEG directory (bypasses FrameIngestor)")
    parser.add_argument("--ortho-tif",        default=None,
                        help="Path to a georeferenced GeoTIFF (Option A — bypasses "
                             "frame loading and mosaic entirely). Example: "
                             "odm_data_aukerman/odm_orthophoto/odm_orthophoto.tif")
    parser.add_argument("--output-root",      default=None,
                        help="Root dir for synthetic session output (used with --frames-dir)")
    parser.add_argument("--maps-dir",         default=None,
                        help=f"Maps directory (default: {_DEFAULT_MAPS_DIR})")
    parser.add_argument("--blur-threshold",   type=float, default=100.0,
                        help="Laplacian variance threshold (default: 100.0)")
    parser.add_argument("--use-stitcher",     action="store_true",
                        help="Attempt cv2.Stitcher before GPS affine compositor "
                             "(disabled by default — fails on nadir survey photos)")
    parser.add_argument("--altitude",         type=float, default=None,
                        help="Manual AGL altitude in metres for GSD calculation. "
                             "Auto-detected from SLAM enu.z or GPS if omitted. "
                             "Example: --altitude 30.0")
    parser.add_argument("--stitch-confidence", type=float, default=0.3,
                        help="cv2.Stitcher confidence threshold (default: 0.3)")
    parser.add_argument("--zoom-min",         type=int, default=14,
                        help="Minimum tile zoom level (default: 14)")
    parser.add_argument("--processes",        type=int, default=4,
                        help="gdal2tiles parallel processes (default: 4)")
    parser.add_argument("--fast",             action="store_true",
                        help="Smaller canvas (2048px), fewer tiles, faster output")
    args = parser.parse_args()

    sys.exit(run(args))


if __name__ == "__main__":
    main()
