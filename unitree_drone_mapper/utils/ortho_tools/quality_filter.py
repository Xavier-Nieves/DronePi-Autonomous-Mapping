"""
ortho_tools/quality_filter.py — Blur rejection for the ortho pipeline.

Accepts a list[FrameRecord] and returns only frames that pass a Laplacian
variance sharpness test.  Blurry frames (motion blur, out-of-focus) produce
stitching artefacts and tile smearing; removing them before MosaicBuilder
and TileCutter improves output quality at no information cost because the
drone captures overlapping frames at 10 Hz.

Algorithm
---------
Laplacian variance (Pech-Pacheco et al., 2000):
    1.  Convert frame to greyscale.
    2.  Convolve with the Laplacian kernel.
    3.  Compute variance of the result.
    4.  Variance < threshold → frame is blurry → reject.

The Laplacian highlights high-frequency edges.  A sharp image has large
edge response variance; a blurry image has near-zero variance because the
low-pass effect of blur suppresses high frequencies.

Threshold calibration
---------------------
The default threshold of 100.0 is empirically appropriate for IMX477 frames
at full resolution (2028×1520) shot nadir at 20–50m AGL with a 10 Hz shutter.
For the Aukerman dataset (77 JPEG survey images at typical DJI Mavic Air
altitude ~100m), the same threshold applies because the images are already
sharp aerial captures; the filter will reject any genuinely blurry frame
while accepting the rest.

Adjust via the --blur-threshold CLI argument in postprocess_ortho.py.

References
----------
Pech-Pacheco, J. L., Cristóbal, G., Chamorro-Martínez, J., & Fernández-
Valdivia, J. (2000). Diatom autofocusing in brightfield microscopy: a
comparative study. Proc. ICPR 2000.

Dependencies
------------
    opencv-python  — cv2.Laplacian
    numpy
"""

import logging
import time
from typing import List

import cv2
import numpy as np

from .frame_ingestor import FrameRecord

logger = logging.getLogger(__name__)


class QualityFilter:
    """
    Laplacian variance blur-rejection filter.

    Parameters
    ----------
    blur_threshold : float
        Frames with Laplacian variance below this value are rejected.
        Default 100.0.  Set to 0.0 to disable (pass all frames through).
    min_frames : int
        If fewer than min_frames survive the blur filter, the threshold is
        halved and filtering is re-run once.  This prevents an overly
        aggressive threshold from discarding an entire flight that happened
        to be shot in haze.  Default 5.
    """

    def __init__(
        self,
        blur_threshold: float = 100.0,
        min_frames:     int   = 5,
    ) -> None:
        self.blur_threshold = blur_threshold
        self.min_frames     = min_frames

    # ── Public API ────────────────────────────────────────────────────────────

    def filter(self, records: List[FrameRecord]) -> List[FrameRecord]:
        """
        Filter frames by Laplacian variance sharpness.

        Parameters
        ----------
        records : list[FrameRecord]
            Input frames, typically sorted by ros_ts from FrameIngestor.

        Returns
        -------
        list[FrameRecord] — frames that passed the blur threshold, in the
        same order as the input.  If all frames are rejected, returns the
        single sharpest frame so downstream stages always have at least
        one frame to work with.
        """
        if not records:
            logger.warning("[QualityFilter] No frames to filter.")
            return []

        if self.blur_threshold <= 0.0:
            logger.info(
                f"[QualityFilter] Threshold=0 — passing all {len(records)} frames."
            )
            return records

        t0 = time.time()
        scores = [self._laplacian_variance(r.image) for r in records]
        passed = [r for r, s in zip(records, scores) if s >= self.blur_threshold]

        # Adaptive fallback: halve threshold if too many frames were dropped
        threshold_used = self.blur_threshold
        if len(passed) < self.min_frames and len(records) >= self.min_frames:
            fallback_thresh = self.blur_threshold / 2.0
            passed = [r for r, s in zip(records, scores) if s >= fallback_thresh]
            threshold_used  = fallback_thresh
            logger.warning(
                f"[QualityFilter] Threshold {self.blur_threshold:.0f} left only "
                f"{len([r for r,s in zip(records,scores) if s >= self.blur_threshold])} "
                f"frames — halved to {fallback_thresh:.0f}, now {len(passed)} pass."
            )

        # Last-resort: always return at least one frame
        if not passed:
            best_idx = int(np.argmax(scores))
            passed   = [records[best_idx]]
            logger.warning(
                f"[QualityFilter] All frames below threshold — returning "
                f"single sharpest frame (variance={scores[best_idx]:.1f})."
            )

        elapsed  = time.time() - t0
        rejected = len(records) - len(passed)
        logger.info(
            f"[QualityFilter] {len(passed)}/{len(records)} frames passed  "
            f"threshold={threshold_used:.0f}  "
            f"rejected={rejected}  ({elapsed:.2f}s)"
        )
        print(
            f"  [QualityFilter] {len(passed)}/{len(records)} frames accepted  "
            f"(threshold={threshold_used:.0f}, {rejected} rejected in {elapsed:.2f}s)"
        )
        return passed

    # ── Internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _laplacian_variance(image: np.ndarray) -> float:
        """
        Compute Laplacian variance of a BGR image.

        Returns a float.  Higher = sharper.
        """
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(grey, cv2.CV_64F).var())
