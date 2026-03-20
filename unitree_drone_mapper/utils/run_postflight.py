#!/usr/bin/env python3
"""
run_postflight.py — Post-flight pipeline trigger for DronePi.

Finds the most recent completed bag session and runs postprocess_mesh.py
against it, which generates the point cloud and Poisson mesh (if enough
points), copies both to /mnt/ssd/maps/, writes metadata.json and
latest.json, and triggers the browser viewer to auto-load within 10s.

Mesh decision is delegated entirely to postprocess_mesh.py:
    points >= POISSON_MIN_POINTS (10k) -> cloud + mesh
    points <  POISSON_MIN_POINTS      -> cloud only

Usage
-----
Manual (run after landing):
    python3 run_postflight.py
    python3 run_postflight.py --bag /mnt/ssd/rosbags/scan_20260317_091400
    python3 run_postflight.py --latest   # explicit latest-bag mode

Auto (called by drone_watchdog.py on deactivation):
    python3 run_postflight.py --auto

Options
-------
--bag <path>     Process a specific bag directory (overrides auto-detect)
--latest         Explicitly process the most recent bag (default behaviour)
--auto           Non-interactive mode: no prompts, structured exit codes
                   0 = success
                   1 = no bag found
                   2 = postprocess_mesh.py failed
--maps-dir       Override maps directory (default: /mnt/ssd/maps)
--skip-wait      Skip the bag-close wait (use if bag is already closed)
"""

import argparse
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# ── config ────────────────────────────────────────────────────────────────────

ROSBAG_DIR       = Path("/mnt/ssd/rosbags")
MAPS_DIR         = Path("/mnt/ssd/maps")
POSTPROCESS_SCRIPT = Path(__file__).parent / "postprocess_mesh.py"

# Bag directory name pattern: scan_YYYYMMDD_HHMMSS
BAG_DIR_RE = re.compile(
    r'^(?:scan|flt|flight)_(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})$'
)

# How long to wait for the bag recorder to close the MCAP file cleanly
# after the watchdog sends SIGINT. The watchdog's GRACEFUL_KILL_S is 5s
# so waiting 8s here gives it comfortable margin.
BAG_CLOSE_WAIT_S = 8

# ── helpers ───────────────────────────────────────────────────────────────────

def log(msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def find_latest_bag() -> Path | None:
    """
    Return the most recent scan_* directory in ROSBAG_DIR, or None.

    Sorted by directory name (timestamp-based) descending so the newest
    session is always first regardless of filesystem mtime.
    """
    if not ROSBAG_DIR.exists():
        return None

    candidates = sorted(
        [d for d in ROSBAG_DIR.iterdir()
         if d.is_dir() and BAG_DIR_RE.match(d.name)],
        reverse=True
    )
    return candidates[0] if candidates else None


def bag_is_closed(bag_path: Path) -> bool:
    """
    Return True if the bag has been closed cleanly.

    A ROS 2 MCAP bag is closed when the .mcap file is no longer being
    written to. We check by reading the metadata.yaml which is written
    by ros2 bag record only after the recording completes. If metadata.yaml
    exists the bag is closed.
    """
    return (bag_path / "metadata.yaml").exists()


def wait_for_bag_close(bag_path: Path, timeout: float = BAG_CLOSE_WAIT_S) -> bool:
    """
    Block until the bag recorder closes the session, up to timeout seconds.
    Returns True if closed, False if timed out.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        if bag_is_closed(bag_path):
            return True
        time.sleep(0.5)
    return False


def run_postprocess(bag_path: Path, maps_dir: Path, auto: bool) -> int:
    """
    Run postprocess_mesh.py against the given bag.

    Always runs without --no-mesh so postprocess_mesh.py can decide
    internally whether to generate a mesh based on point count.
    Returns the subprocess exit code.
    """
    cmd = [
        sys.executable,
        str(POSTPROCESS_SCRIPT),
        "--bag",    str(bag_path),
        "--maps-dir", str(maps_dir),
    ]
    if auto:
        cmd.append("--auto")

    log(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd)
    return result.returncode

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Post-flight pipeline trigger — finds latest bag and processes it."
    )
    parser.add_argument("--bag",       default=None,
                        help="Process a specific bag directory")
    parser.add_argument("--latest",    action="store_true",
                        help="Process the most recent bag (default)")
    parser.add_argument("--auto",      action="store_true",
                        help="Non-interactive mode for watchdog integration")
    parser.add_argument("--maps-dir",  default=str(MAPS_DIR),
                        help=f"Maps directory (default: {MAPS_DIR})")
    parser.add_argument("--skip-wait", action="store_true",
                        help="Skip bag-close wait (bag already closed)")
    args = parser.parse_args()

    maps_dir = Path(args.maps_dir)

    if not args.auto:
        print("=" * 55)
        print("  DronePi Post-Flight Pipeline")
        print("=" * 55)

    # ── resolve bag path ──────────────────────────────────────────────────────
    if args.bag:
        bag_path = Path(args.bag)
        if not bag_path.exists():
            log(f"[FAIL] Bag not found: {bag_path}")
            sys.exit(1)
    else:
        bag_path = find_latest_bag()
        if bag_path is None:
            log(f"[FAIL] No bag sessions found in {ROSBAG_DIR}")
            log(f"       Record a flight first with the watchdog active.")
            sys.exit(1)

    log(f"Session  : {bag_path.name}")
    log(f"Bag path : {bag_path}")

    # ── wait for bag to close ─────────────────────────────────────────────────
    if not args.skip_wait:
        if bag_is_closed(bag_path):
            log("Bag is already closed (metadata.yaml present).")
        else:
            log(f"Waiting up to {BAG_CLOSE_WAIT_S}s for bag recorder to close session...")
            if wait_for_bag_close(bag_path):
                log("Bag closed cleanly.")
            else:
                log(f"[WARN] Bag did not close within {BAG_CLOSE_WAIT_S}s.")
                log(f"       metadata.yaml missing -- bag may be incomplete.")
                log(f"       Proceeding anyway -- point cloud may be truncated.")

    # ── validate maps dir ─────────────────────────────────────────────────────
    if not maps_dir.exists():
        log(f"[FAIL] Maps directory not found: {maps_dir}")
        log(f"       Mount SSD first: sudo mount -a")
        sys.exit(1)

    # ── run postprocess_mesh.py ───────────────────────────────────────────────
    log("Starting post-flight mesh pipeline...")
    log("(Mesh will be generated if point count >= 10,000 after downsampling)")

    rc = run_postprocess(bag_path, maps_dir, args.auto)

    if rc == 0:
        log("=" * 55)
        log("Post-flight pipeline complete.")
        log(f"Viewer   : http://10.42.0.1:8080/meshview.html")
        log("Browser auto-loads mesh within 10s.")
        log("=" * 55)
        sys.exit(0)
    else:
        log(f"[FAIL] postprocess_mesh.py exited with code {rc}")
        sys.exit(2)


if __name__ == "__main__":
    main()
