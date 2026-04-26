"""
ortho_tools/tile_cutter.py — Slippy-map tile pyramid generator.

Wraps gdal2tiles to cut a stitched or orthorectified raster into XYZ
(TMS/Google-convention) PNG tiles consumable by Leaflet in meshview.html.

Input
-----
A georeferenced raster produced by either:
    - MosaicBuilder (fast path) — JPEG/PNG + world file (.wld)
    - Orthorectifier (full path) — GeoTIFF with embedded CRS

Output
------
    <session_dir>/ortho/tiles/{z}/{x}/{y}.png

The tile directory follows the XYZ (Google Maps) convention, not TMS.
gdal2tiles default is TMS; --xyz flag is passed to match Leaflet's
default coordinate scheme.

gdal2tiles availability
-----------------------
gdal2tiles is part of the GDAL package.  On the RPi:
    sudo apt install gdal-bin

On Windows/dev:
    pip install gdal2tiles  (thin Python wrapper invoking system GDAL)
    or install OSGeo4W / GDAL for Windows.

If gdal2tiles is unavailable, TileCutter falls back to a pure-Python
tile splitter using Pillow that produces valid PNG tiles from a plain
numpy array without any geographic reference.  This fallback is suitable
for development testing and the Aukerman ODM validation run.

Zoom levels
-----------
Default zoom range 14–19 covers drone survey altitudes (20–150m AGL)
with sufficient tile detail.  Zoom 19 produces ~0.3m/pixel tiles which
matches IMX477 GSD at 30m AGL.

Dependencies
------------
    gdal2tiles (system GDAL or gdal2tiles pip package)  — primary
    Pillow                                               — fallback
    numpy
"""

import logging
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_ZOOM_MIN = 14
_DEFAULT_ZOOM_MAX = 19


class TileCutter:
    """
    XYZ tile pyramid generator.

    Parameters
    ----------
    zoom_min : int
        Minimum zoom level to generate.  Default 14.
    zoom_max : int
        Maximum zoom level to generate.  Default 19.
    processes : int
        Number of parallel gdal2tiles worker processes.  Default 4.
    resampling : str
        gdal2tiles resampling algorithm.  Options: average, near, bilinear,
        cubic, cubicspline, lanczos, antialias.  Default "average" — produces
        smoother tiles than nearest-neighbour for aerial imagery.
    """

    def __init__(
        self,
        zoom_min:   int = _DEFAULT_ZOOM_MIN,
        zoom_max:   int = _DEFAULT_ZOOM_MAX,
        processes:  int = 4,
        resampling: str = "average",
    ) -> None:
        self.zoom_min   = zoom_min
        self.zoom_max   = zoom_max
        self.processes  = processes
        self.resampling = resampling

    # ── Public API ────────────────────────────────────────────────────────────

    def cut(
        self,
        raster_path: Path,
        output_dir:  Path,
        geo:         Optional[dict] = None,
    ) -> Tuple[Path, int, int]:
        """
        Generate XYZ tile pyramid from a raster file.

        Parameters
        ----------
        raster_path : Path
            Input raster.  Must be a JPEG, PNG, or GeoTIFF.  If a .wld
            world file exists alongside it, gdal2tiles will use it for
            georeferencing.
        output_dir : Path
            Directory under which the tiles/ subdirectory is created.
            Creates output_dir if it does not exist.
        geo : dict or None
            GPS bounds dict from MosaicBuilder (optional).  Used only for
            the Pillow fallback path when gdal2tiles is unavailable.

        Returns
        -------
        tile_dir : Path — path to the tiles/ directory
        zoom_min : int  — actual minimum zoom generated
        zoom_max : int  — actual maximum zoom generated
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        tile_dir = output_dir / "tiles"

        t0 = time.time()
        print(
            f"  [TileCutter] Cutting tiles  "
            f"zoom={self.zoom_min}-{self.zoom_max}  "
            f"raster={raster_path.name}"
        )

        if self._gdal2tiles_available():
            self._run_gdal2tiles(raster_path, tile_dir)
        else:
            logger.warning(
                "[TileCutter] gdal2tiles not found — using Pillow fallback. "
                "Tiles will not be geographically referenced."
            )
            self._pillow_fallback(raster_path, tile_dir, geo)

        elapsed = time.time() - t0
        tile_count = sum(1 for _ in tile_dir.rglob("*.png")) if tile_dir.exists() else 0
        print(
            f"  [TileCutter] {tile_count:,} tiles written  "
            f"→ {tile_dir}  ({elapsed:.1f}s)"
        )
        return tile_dir, self.zoom_min, self.zoom_max

    # ── Internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _gdal2tiles_available() -> bool:
        """Return True if gdal2tiles is reachable via PATH, Python, or Docker."""
        if shutil.which("gdal2tiles") or shutil.which("gdal2tiles.py"):
            return True
        try:
            import gdal2tiles  # pip install gdal2tiles
            return True
        except ImportError:
            pass
        # Docker fallback — ODM container has gdal2tiles at a known path
        if shutil.which("docker"):
            return True
        return False

    @staticmethod
    def _gdal2tiles_via_docker(raster_path: Path, tile_dir: Path,
                                zoom_str: str, resampling: str,
                                processes: int) -> bool:
        """
        Run gdal2tiles inside the ODM Docker container.

        Maps the raster's parent directory into the container at /datasets.
        Used on Windows dev machines where GDAL is not installed natively
        but the opendronemap/odm image is available from ODM processing.

        Returns True on success, False on failure.
        """
        raster_abs  = raster_path.resolve()
        tile_abs    = tile_dir.resolve()
        host_dir    = raster_abs.parent

        # Both raster and tile output must be under the same mounted directory.
        # If tile_dir is elsewhere, mount raster parent only and write tiles there.
        container_raster = f"/datasets/{raster_abs.name}"
        container_tiles  = f"/datasets/tiles"

        cmd = [
            "docker", "run", "--rm",
            "--entrypoint", "/code/SuperBuild/install/bin/gdal2tiles.py",
            "-v", f"{host_dir}:/datasets",
            "opendronemap/odm",
            "--xyz",
            f"--zoom={zoom_str}",
            f"--resampling={resampling}",
            f"--processes={processes}",
            "--tiledriver=PNG",
            container_raster,
            container_tiles,
        ]

        logger.info(f"[TileCutter] Docker gdal2tiles: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            logger.error(
                f"[TileCutter] Docker gdal2tiles failed:\n{result.stderr}"
            )
            return False

        # Move tiles from host_dir/tiles to actual tile_dir
        docker_tile_output = host_dir / "tiles"
        if docker_tile_output.exists() and docker_tile_output != tile_dir:
            if tile_dir.exists():
                shutil.rmtree(tile_dir)
            shutil.move(str(docker_tile_output), str(tile_dir))

        return True

    def _run_gdal2tiles(self, raster_path: Path, tile_dir: Path) -> None:
        """Invoke gdal2tiles via native binary, Python module, or Docker."""
        zoom_str = f"{self.zoom_min}-{self.zoom_max}"
        args = [
            "--xyz",
            f"--zoom={zoom_str}",
            f"--resampling={self.resampling}",
            f"--processes={self.processes}",
            "--tiledriver=PNG",
            str(raster_path),
            str(tile_dir),
        ]

        # Native binary
        exe = shutil.which("gdal2tiles") or shutil.which("gdal2tiles.py")
        if exe:
            logger.info(f"[TileCutter] Native gdal2tiles: {exe}")
            result = subprocess.run([exe] + args, capture_output=True, text=True)
            if result.returncode == 0:
                return
            logger.error(f"[TileCutter] gdal2tiles failed:\n{result.stderr}")
            raise RuntimeError(f"gdal2tiles failed (exit {result.returncode})")

        # Python module (pip install gdal2tiles)
        try:
            import gdal2tiles as _g2t
            _g2t.generate_tiles(str(raster_path), str(tile_dir), **{
                "zoom":         zoom_str,
                "resampling":   self.resampling,
                "nb_processes": self.processes,
                "xyz":          True,
            })
            return
        except ImportError:
            pass

        # Docker fallback (ODM container)
        if shutil.which("docker"):
            ok = self._gdal2tiles_via_docker(
                raster_path, tile_dir, zoom_str,
                self.resampling, self.processes
            )
            if ok:
                return
            raise RuntimeError("Docker gdal2tiles failed")

        raise RuntimeError("gdal2tiles not available via any method")

    def _pillow_fallback(
        self,
        raster_path: Path,
        tile_dir:    Path,
        geo:         Optional[dict],
    ) -> None:
        """
        Pure-Python tile splitter using Pillow.

        Cuts the raster into 256×256 PNG tiles at zoom_max only, placed
        under tile_dir/<zoom_max>/<x>/<y>.png using a simple pixel-grid
        division (no reprojection).  For development/validation use only.
        """
        try:
            from PIL import Image
        except ImportError:
            raise RuntimeError(
                "[TileCutter] Neither gdal2tiles nor Pillow is available. "
                "Install one: pip install Pillow"
            )

        img = Image.open(str(raster_path))
        w, h = img.size
        tile_px = 256
        z = self.zoom_max

        cols = max(1, (w + tile_px - 1) // tile_px)
        rows = max(1, (h + tile_px - 1) // tile_px)

        logger.info(
            f"[TileCutter] Pillow fallback: {w}×{h}px → "
            f"{cols}×{rows} tiles at zoom {z}"
        )

        for row in range(rows):
            for col in range(cols):
                left   = col * tile_px
                upper  = row * tile_px
                right  = min(left + tile_px, w)
                lower  = min(upper + tile_px, h)
                tile   = img.crop((left, upper, right, lower))

                # Pad to 256×256 if on right/bottom edge
                if tile.size != (tile_px, tile_px):
                    padded = Image.new("RGB", (tile_px, tile_px), (0, 0, 0))
                    padded.paste(tile, (0, 0))
                    tile = padded

                out_path = tile_dir / str(z) / str(col)
                out_path.mkdir(parents=True, exist_ok=True)
                tile.save(str(out_path / f"{row}.png"), "PNG")
