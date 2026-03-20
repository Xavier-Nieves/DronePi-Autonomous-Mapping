"""
ground_classifier.py — SMRF ground/non-ground classification via PDAL.

Separates the point cloud into two arrays:
  ground_pts    (class 2)  → bare earth, feed to DTMBuilder
  nonground_pts (class!=2) → buildings, trees, objects, feed to DSMBuilder

Algorithm: SMRF (Simple Morphological Filter)
    Conceptually rolls a sphere underneath the point cloud from below.
    Points the sphere can touch = ground (class 2).
    Points too elevated for the sphere to reach = non-ground.
    Slope threshold prevents steep terrain from being misclassified.

No points are deleted -- classification is additive metadata.
The two output arrays together contain all input points.

Dependencies: pdal (pip install pdal --break-system-packages)
Fallback:     Z-percentile split if PDAL unavailable.
"""

import time
import numpy as np


class GroundClassifier:
    """
    SMRF ground/non-ground classifier using PDAL.

    Parameters
    ----------
    slope : float
        Maximum ground slope in degrees per metre.
        0.15 = gentle campus/field terrain (default)
        0.30 = hilly terrain
        Use lower values for flat areas -- tighter classification.

    window : float
        Maximum morphological window size in metres.
        Should be larger than the widest building in the scene.
        Default 18.0m works for typical campus buildings.

    threshold : float
        Maximum height above the fitted ground surface (metres)
        for a point to still be classified as ground.
        0.5m default: points more than 50cm above fitted surface
        are non-ground.

    scalar : float
        Controls how the morphological window grows across iterations.
        Default 1.25 is the PDAL recommended value.

    Example
    -------
        classifier = GroundClassifier(slope=0.15, window=18.0)
        ground_pts, nonground_pts = classifier.classify(pts)
        print(f"Ground: {len(ground_pts):,}  Non-ground: {len(nonground_pts):,}")
    """

    def __init__(self,
                 slope:     float = 0.15,
                 window:    float = 18.0,
                 threshold: float = 0.5,
                 scalar:    float = 1.25):
        self.slope     = slope
        self.window    = window
        self.threshold = threshold
        self.scalar    = scalar

    def classify(self,
                 pts: np.ndarray
                 ) -> tuple[np.ndarray, np.ndarray]:
        """
        Classify points into ground and non-ground.

        Parameters
        ----------
        pts : Nx3 float array (MLS-smoothed recommended)

        Returns
        -------
        ground_pts    : Mx3 float array  (DTM source)
        nonground_pts : Kx3 float array  (DSM source)
        where M + K == N
        """
        t0 = time.time()
        print(f"  [GroundClassifier] SMRF  slope={self.slope}  "
              f"window={self.window}m  threshold={self.threshold}m  "
              f"pts={len(pts):,}")

        try:
            ground, nonground = self._classify_pdal(pts)
        except Exception as e:
            print(f"  [GroundClassifier] PDAL failed: {e}")
            print(f"  [GroundClassifier] Falling back to Z-percentile split")
            ground, nonground = self._classify_fallback(pts)

        pct = 100.0 * len(ground) / max(len(pts), 1)
        elapsed = time.time() - t0
        print(f"  [GroundClassifier] Ground: {len(ground):,} ({pct:.1f}%)  "
              f"Non-ground: {len(nonground):,} ({100-pct:.1f}%)  "
              f"({elapsed:.1f}s)")
        return ground, nonground

    # ── PDAL SMRF ─────────────────────────────────────────────────────────────

    def _classify_pdal(self,
                       pts: np.ndarray
                       ) -> tuple[np.ndarray, np.ndarray]:
        """Run SMRF filter via PDAL."""
        import pdal
        import json

        # PDAL expects structured array with named fields
        structured      = np.zeros(len(pts), dtype=[
            ("X", np.float64),
            ("Y", np.float64),
            ("Z", np.float64),
        ])
        structured["X"] = pts[:, 0].astype(np.float64)
        structured["Y"] = pts[:, 1].astype(np.float64)
        structured["Z"] = pts[:, 2].astype(np.float64)

        pipeline_def = json.dumps({
            "pipeline": [
                {
                    "type":      "filters.smrf",
                    "slope":     self.slope,
                    "window":    self.window,
                    "threshold": self.threshold,
                    "scalar":    self.scalar,
                }
            ]
        })

        pipeline = pdal.Pipeline(pipeline_def)
        pipeline.execute_with_array(structured)

        result  = pipeline.arrays[0]
        classes = result["Classification"]

        ground_mask    = classes == 2
        nonground_mask = ~ground_mask

        return pts[ground_mask], pts[nonground_mask]

    # ── fallback ──────────────────────────────────────────────────────────────

    def _classify_fallback(self,
                           pts: np.ndarray
                           ) -> tuple[np.ndarray, np.ndarray]:
        """
        Simple Z-percentile fallback when PDAL is unavailable.
        Classifies the lowest 20% of Z values as ground.
        Less accurate than SMRF but works without PDAL.
        """
        print("  [GroundClassifier] Using Z-percentile fallback "
              "(install pdal for better results)")
        z_thresh    = np.percentile(pts[:, 2], 20)
        ground_mask = pts[:, 2] <= z_thresh
        return pts[ground_mask], pts[~ground_mask]
