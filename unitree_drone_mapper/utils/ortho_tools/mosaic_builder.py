"""
ortho_tools/mosaic_builder.py — GPS-affine orthomosaic compositor with GSD-aware frame sizing.

Primary path (production — Pi after flight)
--------------------------------------------
Projects each FrameRecord onto a shared canvas using its GPS lat/lon
position. Frame footprint on the canvas is computed from actual GSD
(Ground Sample Distance) derived from flight altitude, giving accurate
frame sizes regardless of altitude variation between waypoints.

GSD computation — altitude source priority
------------------------------------------
1. enu.z from sidecar JSON (SLAM Z in ENU frame, Z=0 at takeoff).
   Written by camera_capture.py from the waypoint trigger context.
   Most accurate — Point-LIO EKF2 position, ~0.1 m vertical precision.

2. gps.alt (MSL) relative to median ground elevation of all frames.
   Approximation — GPS vertical accuracy ~5 m CEP, useful when SLAM
   data is absent (test datasets without a bag).

3. --altitude CLI override (manual fallback).

4. Statistical estimate from canvas extent ÷ frame count (original
   behaviour, no altitude data at all).

IMX477 camera constants (Sony IMX477 datasheet)
------------------------------------------------
    Focal length : 4.74 mm  (standard lens shipped with ArduCam IMX477)
    Pixel size   : 1.55 µm  (IMX477 datasheet, Table 2)
    Sensor       : 2028 × 1520 px  (4:3 full-frame mode)

GSD formula
-----------
    GSD (m/px) = altitude_m × pixel_size_m / focal_length_m
    e.g. at 30 m AGL:  30 × 0.00000155 / 0.00474 = 0.00981 m/px ≈ 1 cm/px

Frame footprint on canvas
-------------------------
    width_px_canvas  = sensor_width_px  × GSD / (lon_extent / canvas_width)
    height_px_canvas = sensor_height_px × GSD / (lat_extent / canvas_height)

Dependencies
------------
    opencv-python — cv2.resize, optional cv2.Stitcher_create
    numpy
"""

import logging
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .frame_ingestor import FrameRecord

logger = logging.getLogger(__name__)

# ── Earth & camera constants ──────────────────────────────────────────────────
_EARTH_R         = 6_371_000.0    # WGS-84 mean radius (m)

# IMX477 physical sensor parameters (Sony IMX477 datasheet)
_IMX477_FOCAL_M  = 0.00474        # 4.74 mm focal length
_IMX477_PIXEL_M  = 0.00000155     # 1.55 µm pixel pitch
_IMX477_W_PX     = 2028           # sensor width  (px, full-frame mode)
_IMX477_H_PX     = 1520           # sensor height (px, full-frame mode)


class MosaicBuilder:
    """
    GPS-affine orthomosaic compositor with GSD-aware frame sizing.

    Parameters
    ----------
    max_canvas_px : int
        Target canvas size on the long edge in pixels.  Default 4096.
        Use 2048 for --fast mode on the Pi.
    altitude_m : float or None
        Manual AGL altitude override in metres.  When None, altitude is
        read per-frame from sidecar enu.z (SLAM) or gps.alt (GPS fallback).
    use_stitcher : bool
        If True, attempt cv2.Stitcher before GPS affine.  Default False.
    max_frames : int
        Frame cap for cv2.Stitcher (ignored when use_stitcher=False).
    scale : float
        Resize factor for cv2.Stitcher frames (ignored when use_stitcher=False).
    confidence_threshold : float
        cv2.Stitcher confidence (ignored when use_stitcher=False).
    """

    def __init__(
        self,
        max_canvas_px:        int           = 4096,
        altitude_m:           Optional[float] = None,
        use_stitcher:         bool          = False,
        max_frames:           int           = 60,
        scale:                float         = 0.5,
        confidence_threshold: float         = 0.3,
    ) -> None:
        self.max_canvas_px        = max_canvas_px
        self.altitude_m           = altitude_m
        self.use_stitcher         = use_stitcher
        self.max_frames           = max_frames
        self.scale                = scale
        self.confidence_threshold = confidence_threshold

    # ── Public API ────────────────────────────────────────────────────────────

    def build(
        self,
        records: List[FrameRecord],
    ) -> Tuple[Optional[np.ndarray], Optional[dict]]:
        """
        Composite frames into a single georeferenced canvas.

        Returns
        -------
        canvas : np.ndarray (H, W, 3) BGR or None on total failure.
        geo    : dict with lat_min/lat_max/lon_min/lon_max/gps_count, or None.
        """
        if not records:
            logger.error("[MosaicBuilder] No frames provided.")
            return None, None

        geo = self._compute_gps_bounds(records)

        # Opt-in stitcher
        if self.use_stitcher:
            canvas = self._try_stitcher(records)
            if canvas is not None:
                return canvas, geo

        # GPS affine compositor — primary production path
        if geo is not None:
            gps_count  = geo.get("gps_count",  0)
            slam_count = geo.get("slam_count", 0)
            if slam_count > 0:
                print(
                    f"  [MosaicBuilder] Position source: "
                    f"{gps_count} GPS frames + {slam_count} SLAM-anchored frames"
                )
            canvas = self._gps_composite(records, geo)
            if canvas is not None:
                if geo:
                    print(
                        f"  [MosaicBuilder] GPS bounds: "
                        f"lat [{geo['lat_min']:.6f}, {geo['lat_max']:.6f}]  "
                        f"lon [{geo['lon_min']:.6f}, {geo['lon_max']:.6f}]  "
                        f"({gps_count} GPS, {slam_count} SLAM-anchored)"
                    )
                return canvas, geo

        # Last resort: grid collage
        print("  [MosaicBuilder] GPS composite failed — using grid collage")
        images = [r.image for r in records]
        canvas = self._collage_fallback(images)
        if canvas is not None:
            print(f"  [MosaicBuilder] Collage: {canvas.shape[1]}×{canvas.shape[0]}px")
        return canvas, geo

    def write_world_file(
        self,
        canvas:      np.ndarray,
        geo:         dict,
        output_path: Path,
    ) -> None:
        """Write ESRI world file (.wld) for gdal2tiles georeferencing."""
        h, w       = canvas.shape[:2]
        lon_per_px = (geo["lon_max"] - geo["lon_min"]) / max(w, 1)
        lat_per_px = (geo["lat_min"] - geo["lat_max"]) / max(h, 1)
        wld_path   = output_path.with_suffix(".wld")
        wld_path.write_text(
            f"{lon_per_px:.10f}\n0.0\n0.0\n"
            f"{lat_per_px:.10f}\n"
            f"{geo['lon_min']:.10f}\n"
            f"{geo['lat_max']:.10f}\n"
        )
        logger.info(f"[MosaicBuilder] World file: {wld_path.name}")

    # ── Altitude resolution ───────────────────────────────────────────────────

    def _resolve_altitude(self, records: List[FrameRecord]) -> Optional[float]:
        """
        Resolve AGL altitude in metres using the priority chain.

        Priority
        --------
        1. self.altitude_m  — CLI override, always wins if set
        2. enu.z median     — SLAM Z (ENU frame, Z=0 at takeoff) from sidecar
        3. gps.alt relative — GPS MSL minus estimated ground elevation
        4. None             — fall back to area-based tile sizing
        """
        # 1. CLI override
        if self.altitude_m is not None:
            print(f"  [MosaicBuilder] Altitude: {self.altitude_m:.1f}m (--altitude override)")
            return self.altitude_m

        # 2. SLAM enu.z — stored in FrameRecord.gps as (lat, lon, alt)
        #    but enu.z is a separate field injected by FrameIngestor from sidecar.
        #    Access via record.enu_z attribute if present, else skip.
        enu_z_vals = []
        for r in records:
            enu_z = getattr(r, 'enu_z', None)
            if enu_z is not None and isinstance(enu_z, (int, float)) and enu_z > 0.5:
                enu_z_vals.append(float(enu_z))

        if enu_z_vals:
            alt = float(np.median(enu_z_vals))
            print(
                f"  [MosaicBuilder] Altitude: {alt:.1f}m  "
                f"(SLAM enu.z median, {len(enu_z_vals)} frames)"
            )
            return alt

        # 3. GPS altitude relative — subtract estimated ground elevation
        #    Ground elevation approximated as the minimum GPS alt across all frames
        #    (lowest point is most likely near ground level).
        gps_alts = [
            float(r.gps[2]) for r in records
            if r.gps[2] is not None and float(r.gps[2]) > 0
        ]
        if len(gps_alts) >= 3:
            ground_elev = min(gps_alts)
            median_alt  = float(np.median(gps_alts))
            agl         = median_alt - ground_elev
            if agl > 1.0:
                print(
                    f"  [MosaicBuilder] Altitude: {agl:.1f}m  "
                    f"(GPS MSL relative, ground≈{ground_elev:.0f}m)"
                )
                return agl

        print("  [MosaicBuilder] Altitude: unknown — using area-based tile sizing")
        return None

    def _gsd_m_per_px(self, altitude_m: float) -> float:
        """
        Compute GSD in metres per pixel for the IMX477 at a given altitude.

        GSD = altitude × pixel_pitch / focal_length
        """
        return altitude_m * _IMX477_PIXEL_M / _IMX477_FOCAL_M

    # ── GPS affine compositor ─────────────────────────────────────────────────

    def _gps_composite(
        self,
        records: List[FrameRecord],
        geo:     dict,
    ) -> Optional[np.ndarray]:
        """
        Project frames onto canvas using GPS position, with SLAM ENU fallback.

        Position resolution per frame (Option B):
        1. Frame has GPS → use GPS lat/lon directly
        2. Frame has SLAM pose_4x4 + at least one GPS anchor exists →
           derive lat/lon from ENU offset relative to first GPS fix
        3. Neither → frame skipped
        """
        t0 = time.time()

        lat_min, lat_max = geo["lat_min"], geo["lat_max"]
        lon_min, lon_max = geo["lon_min"], geo["lon_max"]
        lat_mid          = (lat_min + lat_max) / 2.0

        m_per_lat = _EARTH_R * (np.pi / 180.0)
        m_per_lon = _EARTH_R * np.cos(np.radians(lat_mid)) * (np.pi / 180.0)
        height_m  = (lat_max - lat_min) * m_per_lat
        width_m   = (lon_max - lon_min) * m_per_lon

        if height_m < 0.1 or width_m < 0.1:
            logger.warning("[MosaicBuilder] GPS extent too small.")
            return None

        ppm      = self.max_canvas_px / max(height_m, width_m)
        canvas_w = max(1, int(width_m  * ppm))
        canvas_h = max(1, int(height_m * ppm))

        # ── Resolve GPS anchor for SLAM fallback ──────────────────────────────
        # Find first frame with valid GPS — used as the ENU coordinate origin.
        gps_anchor_lat = None
        gps_anchor_lon = None
        anchor_enu     = np.zeros(3)
        slam_used      = 0

        for r in records:
            if r.gps[0] is not None and r.gps[1] is not None:
                gps_anchor_lat = float(r.gps[0])
                gps_anchor_lon = float(r.gps[1])
                if r.pose_4x4 is not None:
                    anchor_enu = r.pose_4x4[:3, 3]
                break

        if gps_anchor_lat is None:
            print("  [MosaicBuilder] No GPS anchor available — cannot composite")
            return None

        # ── Resolve altitude and tile size ────────────────────────────────────
        altitude = self._resolve_altitude(records)

        if altitude is not None and altitude > 1.0:
            gsd_m     = self._gsd_m_per_px(altitude)
            frame_w_m = _IMX477_W_PX * gsd_m
            frame_h_m = _IMX477_H_PX * gsd_m
            tile_w    = max(4, int(frame_w_m * ppm))
            tile_h    = max(4, int(frame_h_m * ppm))
            print(
                f"  [MosaicBuilder] GSD={gsd_m*100:.2f}cm/px  "
                f"footprint={frame_w_m:.1f}×{frame_h_m:.1f}m  "
                f"tile={tile_w}×{tile_h}px"
            )
        else:
            n_frames = max(len(records), 1)
            avg_area = (canvas_w * canvas_h) / n_frames
            side     = max(int(np.sqrt(avg_area) * 1.5), 64)
            tile_w   = int(side * (_IMX477_W_PX / _IMX477_H_PX))
            tile_h   = side

        print(
            f"  [MosaicBuilder] GPS+SLAM composite  "
            f"{canvas_w}×{canvas_h}px  "
            f"extent={width_m:.0f}×{height_m:.0f}m  "
            f"ppm={ppm:.1f}"
        )

        canvas  = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        placed  = 0
        skipped = 0

        for r in records:
            lat, lon = self._resolve_frame_position(
                r, gps_anchor_lat, gps_anchor_lon,
                anchor_enu, m_per_lat, m_per_lon,
            )
            if lat is None or lon is None:
                skipped += 1
                continue

            # Track how many frames used SLAM position
            if r.gps[0] is None:
                slam_used += 1

            cx = int((lon - lon_min) / (lon_max - lon_min) * canvas_w)
            cy = int((lat_max - lat) / (lat_max - lat_min) * canvas_h)

            try:
                tile = cv2.resize(r.image, (tile_w, tile_h),
                                  interpolation=cv2.INTER_AREA)
                tile = cv2.cvtColor(tile, cv2.COLOR_RGB2BGR)
            except Exception:
                skipped += 1
                continue

            x0, y0 = cx - tile_w // 2, cy - tile_h // 2
            x1, y1 = x0 + tile_w,      y0 + tile_h

            sx0 = max(0, -x0);   sx1 = tile_w - max(0, x1 - canvas_w)
            sy0 = max(0, -y0);   sy1 = tile_h - max(0, y1 - canvas_h)
            dx0 = max(0, x0);    dx1 = dx0 + (sx1 - sx0)
            dy0 = max(0, y0);    dy1 = dy0 + (sy1 - sy0)

            if dx1 > dx0 and dy1 > dy0 and sx1 > sx0 and sy1 > sy0:
                canvas[dy0:dy1, dx0:dx1] = tile[sy0:sy1, sx0:sx1]
                placed += 1

        elapsed = time.time() - t0
        slam_note = f"  ({slam_used} via SLAM anchor)" if slam_used > 0 else ""
        print(
            f"  [MosaicBuilder] {placed} frames placed, "
            f"{skipped} skipped{slam_note}  ({elapsed:.1f}s)"
        )
        return canvas if placed > 0 else None

    # ── cv2.Stitcher (opt-in) ─────────────────────────────────────────────────

    def _try_stitcher(self, records: List[FrameRecord]) -> Optional[np.ndarray]:
        frames = self._subsample(records)
        images = self._load_images(frames)
        print(f"  [MosaicBuilder] cv2.Stitcher  {len(images)} frames")
        try:
            stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
            if hasattr(stitcher, 'setConfidenceThresh'):
                stitcher.setConfidenceThresh(self.confidence_threshold)
            elif hasattr(stitcher, 'setPanoConfidenceThresh'):
                stitcher.setPanoConfidenceThresh(self.confidence_threshold)
            status, canvas = stitcher.stitch(images)
            if status == cv2.Stitcher_OK:
                print(f"  [MosaicBuilder] Stitcher OK: {canvas.shape[1]}×{canvas.shape[0]}px")
                return canvas
            status_names = {
                cv2.Stitcher_ERR_NEED_MORE_IMGS:            "NEED_MORE_IMGS",
                cv2.Stitcher_ERR_HOMOGRAPHY_EST_FAIL:       "HOMOGRAPHY_EST_FAIL",
                cv2.Stitcher_ERR_CAMERA_PARAMS_ADJUST_FAIL: "CAMERA_PARAMS_ADJUST_FAIL",
            }
            print(f"  [MosaicBuilder] Stitcher failed: "
                  f"{status_names.get(status, f'code={status}')} — using GPS composite")
            return None
        except Exception as exc:
            print(f"  [MosaicBuilder] Stitcher exception: {exc} — using GPS composite")
            return None

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _subsample(self, records):
        if self.max_frames is None or len(records) <= self.max_frames:
            return records
        indices = np.linspace(0, len(records) - 1, self.max_frames, dtype=int)
        return [records[i] for i in indices]

    def _load_images(self, records):
        images = []
        for r in records:
            img = r.image
            if self.scale != 1.0:
                h, w = img.shape[:2]
                img  = cv2.resize(img, (int(w * self.scale), int(h * self.scale)),
                                  interpolation=cv2.INTER_AREA)
            images.append(img)
        return images

    def _collage_fallback(self, images):
        if not images:
            return None
        n    = len(images)
        cols = int(np.ceil(np.sqrt(n)))
        rows = int(np.ceil(n / cols))
        h, w = images[0].shape[:2]
        resized = [cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA) for img in images]
        blank   = np.zeros((h, w, 3), dtype=np.uint8)
        while len(resized) < rows * cols:
            resized.append(blank)
        return np.vstack([np.hstack(resized[r * cols:(r + 1) * cols]) for r in range(rows)])

    @staticmethod
    def _compute_gps_bounds(records):
        """
        Compute WGS-84 bounding box from all frames.

        For frames with valid GPS, use GPS lat/lon directly.
        For frames with no GPS but a SLAM pose_4x4, derive lat/lon from the
        SLAM ENU translation anchored to the nearest GPS fix (Option B).

        Returns None only if no position information is available at all.
        """
        # First pass — collect frames with GPS and SLAM poses
        gps_frames  = [(i, r) for i, r in enumerate(records)
                       if r.gps[0] is not None and r.gps[1] is not None]
        slam_frames = [(i, r) for i, r in enumerate(records)
                       if r.pose_4x4 is not None]

        if not gps_frames and not slam_frames:
            return None

        lats, lons = [], []

        # Pure GPS frames
        for _, r in gps_frames:
            lats.append(float(r.gps[0]))
            lons.append(float(r.gps[1]))

        # SLAM-only frames anchored to nearest GPS fix
        if slam_frames and gps_frames:
            anchor_idx, anchor_r = gps_frames[0]   # first GPS fix as anchor
            anchor_lat = float(anchor_r.gps[0])
            anchor_lon = float(anchor_r.gps[1])
            anchor_enu = anchor_r.pose_4x4[:3, 3]  # ENU (ex, ey, ez)

            lat_mid    = anchor_lat
            m_per_lat  = _EARTH_R * (np.pi / 180.0)
            m_per_lon  = _EARTH_R * np.cos(np.radians(lat_mid)) * (np.pi / 180.0)

            for _, r in slam_frames:
                if r.gps[0] is not None:
                    continue   # already have GPS for this frame
                t      = r.pose_4x4[:3, 3]
                d_east = float(t[0] - anchor_enu[0])
                d_north= float(t[1] - anchor_enu[1])
                lats.append(anchor_lat + d_north / m_per_lat)
                lons.append(anchor_lon + d_east  / m_per_lon)

        if not lats:
            return None

        return {
            "lat_min":    min(lats), "lat_max": max(lats),
            "lon_min":    min(lons), "lon_max": max(lons),
            "gps_count":  len(gps_frames),
            "slam_count": len([r for _, r in slam_frames if r.gps[0] is None]),
        }

    @staticmethod
    def _resolve_frame_position(
        r,
        gps_anchor_lat: float,
        gps_anchor_lon: float,
        anchor_enu:     np.ndarray,
        m_per_lat:      float,
        m_per_lon:      float,
    ) -> tuple:
        """
        Return (lat, lon) for a single frame using GPS if available,
        otherwise derive from SLAM ENU offset relative to GPS anchor.

        Returns (None, None) if neither GPS nor SLAM pose is available.
        """
        lat, lon, _ = r.gps
        if lat is not None and lon is not None:
            return float(lat), float(lon)

        if r.pose_4x4 is not None and gps_anchor_lat is not None:
            t       = r.pose_4x4[:3, 3]
            d_east  = float(t[0] - anchor_enu[0])
            d_north = float(t[1] - anchor_enu[1])
            return (
                gps_anchor_lat + d_north / m_per_lat,
                gps_anchor_lon + d_east  / m_per_lon,
            )

        return None, None
