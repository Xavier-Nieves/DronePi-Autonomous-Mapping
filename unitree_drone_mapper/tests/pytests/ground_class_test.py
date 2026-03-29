"""Unit tests for GroundClassifier — pure-numpy methods only (no PDAL/scipy)."""

import sys
import os

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "utils"))
from mesh_tools.ground_classifier import GroundClassifier


class TestPercentileFallback:
    """Direct tests of the Z-percentile fallback (pure numpy, always available)."""

    def test_output_shapes_sum_to_input(self, flat_cloud):
        clf = GroundClassifier()
        g, ng = clf._classify_percentile(flat_cloud)
        assert len(g) + len(ng) == len(flat_cloud)

    def test_returns_two_nonempty_arrays(self, flat_cloud):
        clf = GroundClassifier()
        g, ng = clf._classify_percentile(flat_cloud)
        assert len(g) > 0
        assert len(ng) > 0

    def test_ground_is_lower_than_nonground(self, mixed_cloud):
        clf = GroundClassifier()
        g, ng = clf._classify_percentile(mixed_cloud)
        # Ground median Z should be strictly below non-ground median Z
        assert np.median(g[:, 2]) < np.median(ng[:, 2])

    def test_no_points_lost(self, mixed_cloud):
        clf = GroundClassifier()
        g, ng = clf._classify_percentile(mixed_cloud)
        combined = np.vstack([g, ng])
        assert combined.shape == mixed_cloud.shape


class TestLocalMinimum:
    """Tests for the local_minimum method (pure numpy, no scipy/PDAL)."""

    def test_classify_local_minimum_splits_mixed(self, mixed_cloud):
        clf = GroundClassifier(method="local_minimum", cell_size=1.0)
        g, ng = clf.classify(mixed_cloud)
        # Ground should not contain the high-Z object points
        assert g[:, 2].max() < 1.0, "Ground array should not contain Z>1 points"

    def test_output_covers_all_input_points(self, mixed_cloud):
        clf = GroundClassifier(method="local_minimum")
        g, ng = clf.classify(mixed_cloud)
        assert len(g) + len(ng) == len(mixed_cloud)

    def test_flat_cloud_mostly_ground(self, flat_cloud):
        clf = GroundClassifier(method="local_minimum", threshold=0.1)
        g, ng = clf.classify(flat_cloud)
        ground_fraction = len(g) / len(flat_cloud)
        # A flat cloud should classify nearly everything as ground
        assert ground_fraction > 0.80, f"Expected >80% ground, got {ground_fraction:.1%}"

    def test_tiny_cloud_does_not_crash(self, tiny_cloud):
        clf = GroundClassifier(method="local_minimum")
        g, ng = clf.classify(tiny_cloud)
        assert len(g) + len(ng) == len(tiny_cloud)


class TestAutoFallback:
    """Verify the SMRF→PMF→local_minimum→percentile chain completes."""

    def test_auto_mode_always_returns_result(self, mixed_cloud):
        """auto mode must produce output regardless of which library is available."""
        clf = GroundClassifier(method="auto")
        g, ng = clf.classify(mixed_cloud)
        assert len(g) + len(ng) == len(mixed_cloud)

    def test_result_dtype_preserved(self, mixed_cloud):
        clf = GroundClassifier(method="local_minimum")
        g, ng = clf.classify(mixed_cloud)
        assert g.dtype == mixed_cloud.dtype or g.dtype == np.float32
