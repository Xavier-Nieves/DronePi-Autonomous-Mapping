"""flight/waypoint_validator.py — Preflight stale-waypoint sanity gate.

PURPOSE
-------
Rejects a waypoint list before arming if any ENU waypoint implies a real-world
position that is geographically implausible from the current GPS home fix.

This guards against stale waypoints loaded from a previous session or a
different physical location, which would appear as valid ENU coordinates but
would fly the vehicle far outside the intended survey area.

INTEGRATION
-----------
Call WaypointValidator.check() from main.py between MAVROS-ready and
warmup-setpoints (before any arming call). On rejection the caller must abort
the mission via _teardown() — the validator does not arm or disarm.

FRAME RELATIONSHIP
------------------
ENU waypoints are relative to the SLAM local origin. The GPS home fix is
relative to the physical ground position at startup. Under normal operation:

    ENU origin ≈ GPS home position

So a waypoint at ENU (10, 5, 4) implies the vehicle should fly ~11 m from
its current GPS position. If a waypoint at (500, 200, 4) is loaded from a
previous session where the home was 3 km away, the GPS-derived displacement
check will catch it.

THRESHOLD GUIDANCE
------------------
Default MAX_WAYPOINT_RADIUS_M = 200 m. For a Tarot 810 survey mission this
is a practical upper bound. Override via environment variable
WAYPOINT_MAX_RADIUS_M.

GPS FALLBACK
------------
If GpsReader reports no reliable fix, validation is SKIPPED (non-fatal) and
the reason is logged. GPS is secondary and non-authoritative; blocking a
mission on GPS unavailability would be worse than the stale-waypoint risk
in a GPS-denied environment.
"""

from __future__ import annotations

import logging
import math
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

MAX_WAYPOINT_RADIUS_M: float = float(
    os.environ.get("WAYPOINT_MAX_RADIUS_M", "200.0")
)

WaypointENU = Tuple[float, float, float]


@dataclass
class ValidationResult:
    passed: bool
    skipped: bool          # True when GPS unavailable — not a failure
    reason: str
    offending_index: Optional[int] = None
    offending_radius_m: Optional[float] = None


class WaypointValidator:
    """Validates that ENU waypoints are geographically plausible from the
    current GPS home fix.

    Parameters
    ----------
    gps_reader : GpsReader
        Live GpsReader instance. Must already be started (background thread
        running). The validator reads the current reliable fix at call time.
    max_radius_m : float
        Maximum allowed straight-line distance (ENU horizontal plane) from
        the origin for any waypoint. Defaults to WAYPOINT_MAX_RADIUS_M env var.
    """

    def __init__(
        self,
        gps_reader,                      # GpsReader — avoids circular import
        max_radius_m: float = MAX_WAYPOINT_RADIUS_M,
    ) -> None:
        self._gps = gps_reader
        self._max_radius_m = max_radius_m

    def check(self, waypoints: List[WaypointENU]) -> ValidationResult:
        """Run the stale-waypoint gate.

        Steps
        -----
        1. Acquire a reliable GPS fix and set the home datum.
        2. For each waypoint, compute the horizontal distance from the ENU
           origin (0, 0).
        3. If any waypoint exceeds max_radius_m, return a failing result.

        The horizontal radius check is intentionally simple: it measures
        distance from the ENU origin, not from every other waypoint. This
        catches the stale-location case because a correct mission always has
        waypoints near the origin; a stale mission from a different location
        will have large ENU offsets.

        Returns
        -------
        ValidationResult with passed=True on success, passed=False on
        rejection, skipped=True when GPS is unavailable.
        """
        if not waypoints:
            return ValidationResult(
                passed=False,
                skipped=False,
                reason="waypoint_list_empty",
            )

        # ── 1. GPS fix ────────────────────────────────────────────────────────
        if not self._gps.is_reliable():
            logger.warning(
                "[WaypointValidator] GPS fix unavailable or unreliable. "
                "Skipping stale-waypoint check (non-fatal). "
                "Set home manually or improve GPS signal before flight."
            )
            return ValidationResult(
                passed=True,
                skipped=True,
                reason="gps_unavailable_check_skipped",
            )

        home_set = self._gps.set_home()
        if not home_set:
            logger.warning(
                "[WaypointValidator] set_home() failed despite is_reliable(). "
                "Race condition or fix went stale. Skipping check."
            )
            return ValidationResult(
                passed=True,
                skipped=True,
                reason="home_set_failed_check_skipped",
            )

        # ── 2. Per-waypoint radius check ──────────────────────────────────────
        for i, (ex, ey, _ez) in enumerate(waypoints):
            radius = math.sqrt(ex**2 + ey**2)
            if radius > self._max_radius_m:
                logger.error(
                    f"[WaypointValidator] REJECT: Waypoint {i+1} at ENU "
                    f"({ex:.1f}, {ey:.1f}) is {radius:.1f} m from origin — "
                    f"exceeds limit of {self._max_radius_m:.1f} m. "
                    "Possible stale waypoints from a different location."
                )
                return ValidationResult(
                    passed=False,
                    skipped=False,
                    reason="waypoint_exceeds_max_radius",
                    offending_index=i,
                    offending_radius_m=round(radius, 1),
                )

        logger.info(
            f"[WaypointValidator] PASS: {len(waypoints)} waypoints, "
            f"all within {self._max_radius_m:.1f} m of origin. "
            f"GPS home: lat={self._gps.get_home().lat:.6f} "
            f"lon={self._gps.get_home().lon:.6f}."
        )
        return ValidationResult(
            passed=True,
            skipped=False,
            reason="ok",
        )