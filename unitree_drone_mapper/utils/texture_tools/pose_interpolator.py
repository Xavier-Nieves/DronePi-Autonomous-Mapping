"""
pose_interpolator.py — SLAM pose history interpolator.

Maintains a rolling buffer of timestamped 4×4 drone body poses published
by Point-LIO on /aft_mapped_to_init and returns the interpolated pose at
any requested timestamp.

Interpolation method
--------------------
  Translation : Linear (LERP).  Translation error from linear interpolation
                is negligible at the pose rates produced by Point-LIO
                (≥ 10 Hz) and the camera trigger intervals used in this
                project (0.5–2 Hz).

  Rotation    : Spherical linear interpolation (SLERP) via
                scipy.spatial.transform.Slerp.  SLERP is used instead of
                naïve matrix interpolation because linearly blending
                rotation matrices does not produce valid rotation matrices
                (their determinant deviates from 1.0 and they are no longer
                orthogonal at intermediate steps), which would introduce
                geometric distortion in the projected texture.

Extrapolation
-------------
  The interpolator allows extrapolation up to EXTRAPOLATION_LIMIT_NS
  (50 ms) beyond the last received pose.  This accommodates software-timed
  camera captures whose timestamps may lag slightly behind the last SLAM
  message.  Extrapolation beyond this limit returns None to prevent bad
  projections from stale data.

Thread safety
-------------
  add_pose() and get_pose_at() both acquire a threading.Lock so the
  interpolator can be called from a ROS callback thread (add_pose) and
  the main processing thread (get_pose_at) concurrently without data races.

Dependencies
------------
  numpy, scipy  — available in the dronepi conda environment.
  No ROS dependency — the class works identically in offline bag replay
  and in standalone unit tests.
"""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


# Maximum extrapolation beyond the last known pose (nanoseconds).
# 50 ms matches the PoseInterpolator tolerance documented in
# docs/texture_pipeline.md section 6.
EXTRAPOLATION_LIMIT_NS: int = 50_000_000   # 50 ms


class PoseInterpolator:
    """
    Rolling buffer of SLAM poses with SLERP interpolation.

    Parameters
    ----------
    max_history : int
        Maximum number of poses to retain.  Older poses are discarded
        FIFO when the buffer is full.  At 250 Hz SLAM rate, 5000 entries
        cover 20 seconds — sufficient for a full camera trigger cycle.

    extrapolation_limit_ns : int
        Maximum nanoseconds to extrapolate beyond the last received pose.
        Requests beyond this limit return None.  Default: 50 000 000 (50 ms).

    Usage
    -----
        interpolator = PoseInterpolator()

        # From the ROS callback (any thread):
        interpolator.add_pose(msg.header.stamp.sec * 1_000_000_000
                              + msg.header.stamp.nanosec, pose_4x4)

        # From the texture projector (main thread):
        pose = interpolator.get_pose_at(camera_timestamp_ns)
        if pose is None:
            # Timestamp out of range — skip this frame
    """

    def __init__(
        self,
        max_history: int = 5000,
        extrapolation_limit_ns: int = EXTRAPOLATION_LIMIT_NS,
    ) -> None:
        self._max_history            = max_history
        self._extrapolation_limit_ns = extrapolation_limit_ns
        self._timestamps: list[int]        = []   # sorted, nanoseconds
        self._poses:      list[np.ndarray] = []   # corresponding 4×4 arrays
        self._lock = threading.Lock()

    # ── Public API ─────────────────────────────────────────────────────────────

    def add_pose(self, timestamp_ns: int, pose_4x4: np.ndarray) -> None:
        """
        Append a new timestamped pose to the rolling buffer.

        Parameters
        ----------
        timestamp_ns : int
            ROS clock timestamp in nanoseconds.  Must be monotonically
            increasing; out-of-order poses are silently dropped to keep
            the buffer sorted.
        pose_4x4 : np.ndarray, shape (4, 4)
            Homogeneous transform of the drone body in the world frame.
        """
        with self._lock:
            # Drop out-of-order poses — binary search would still find the
            # correct bracket but SLERP between out-of-order frames is
            # meaningless and indicates a clock issue worth ignoring.
            if self._timestamps and timestamp_ns <= self._timestamps[-1]:
                return

            self._timestamps.append(timestamp_ns)
            self._poses.append(pose_4x4.copy())

            # Trim oldest entries when the buffer is full
            if len(self._timestamps) > self._max_history:
                self._timestamps.pop(0)
                self._poses.pop(0)

    def get_pose_at(self, query_ts_ns: int) -> Optional[np.ndarray]:
        """
        Return the interpolated 4×4 drone pose at the requested timestamp.

        Performs LERP on translation and SLERP on rotation between the two
        bracketing recorded poses.  Allows limited extrapolation beyond the
        last known pose (see EXTRAPOLATION_LIMIT_NS).

        Parameters
        ----------
        query_ts_ns : int
            Requested timestamp in nanoseconds.

        Returns
        -------
        np.ndarray, shape (4, 4)
            Interpolated homogeneous transform of the drone body in the
            world frame.
        None
            If the buffer contains fewer than 2 poses, or if the requested
            timestamp is before the first recorded pose, or if extrapolation
            beyond the limit is required.
        """
        with self._lock:
            n = len(self._timestamps)
            if n < 2:
                return None

            ts = self._timestamps
            ps = self._poses

            # Before the first recorded pose — no valid bracket
            if query_ts_ns < ts[0]:
                return None

            # Beyond the last recorded pose — allow limited extrapolation
            if query_ts_ns > ts[-1]:
                overshoot = query_ts_ns - ts[-1]
                if overshoot > self._extrapolation_limit_ns:
                    return None
                # Extrapolate by clamping alpha slightly beyond 1.0 using
                # the last two poses as the bracket
                t0, t1 = ts[-2], ts[-1]
                p0, p1 = ps[-2], ps[-1]
                alpha  = (query_ts_ns - t0) / max(t1 - t0, 1)
                return self._interpolate(p0, p1, alpha)

            # Binary search for the left bracket index
            lo, hi = 0, n - 1
            while lo < hi - 1:
                mid = (lo + hi) // 2
                if ts[mid] <= query_ts_ns:
                    lo = mid
                else:
                    hi = mid

            t0, t1 = ts[lo], ts[hi]
            p0, p1 = ps[lo], ps[hi]
            alpha  = (query_ts_ns - t0) / max(t1 - t0, 1)
            return self._interpolate(p0, p1, alpha)

    @property
    def size(self) -> int:
        """Number of poses currently in the buffer."""
        with self._lock:
            return len(self._timestamps)

    @property
    def time_range_s(self) -> float:
        """Duration covered by the buffer in seconds (0.0 if fewer than 2 poses)."""
        with self._lock:
            if len(self._timestamps) < 2:
                return 0.0
            return (self._timestamps[-1] - self._timestamps[0]) / 1e9

    # ── Private helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _interpolate(
        p0: np.ndarray, p1: np.ndarray, alpha: float
    ) -> np.ndarray:
        """
        LERP translation + SLERP rotation between two 4x4 poses.

        alpha = 0.0 -> p0, 1.0 -> p1.  Values outside [0,1] extrapolate.

        Design note — extrapolation
        ---------------------------
        scipy.spatial.transform.Slerp enforces times in [0.0, 1.0] and
        raises ValueError for any value outside that range, including
        modest values like 1.03 from a 30 ms extrapolation window.

        For alpha > 1 we extend manually:
          R_extra = Rotation.from_rotvec(rotvec(R1*R0.inv()) * excess) * R1
        This continues rotating past R1 at the same angular velocity and
        always stays on SO(3).  Symmetric formula for alpha < 0.
        """
        t_interp = (1.0 - alpha) * p0[:3, 3] + alpha * p1[:3, 3]

        R0      = Rotation.from_matrix(p0[:3, :3])
        R1      = Rotation.from_matrix(p1[:3, :3])
        alpha_f = float(alpha)

        if 0.0 <= alpha_f <= 1.0:
            slerp    = Slerp([0.0, 1.0], Rotation.concatenate([R0, R1]))
            R_interp = slerp(alpha_f).as_matrix()
        elif alpha_f > 1.0:
            excess   = alpha_f - 1.0
            R_rel    = R1 * R0.inv()
            R_interp = (Rotation.from_rotvec(R_rel.as_rotvec() * excess) * R1).as_matrix()
        else:
            excess   = -alpha_f
            R_rel    = R0 * R1.inv()
            R_interp = (Rotation.from_rotvec(R_rel.as_rotvec() * excess) * R0).as_matrix()

        result         = np.eye(4, dtype=np.float64)
        result[:3, :3] = R_interp
        result[:3,  3] = t_interp
        return result
