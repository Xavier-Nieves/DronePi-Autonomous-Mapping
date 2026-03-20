#!/usr/bin/env python3
"""
flight_logger.py — Flight history logger for DronePi.

Maintains a human-readable flight history log at:
    ~/unitree_lidar_project/logs/flight_history.log

Each entry records one flight session with all available metadata.
Called automatically by postprocess_mesh.py after each session is processed.
Can also be called manually to view or query the flight history.

Log format (one line per flight):
    Flight NNN | DD Month YYYY | HH:MM:SS | Duration: Xm Ys | Points: N | Mesh: YES/NO | Bag: scan_...

Usage
-----
  # View full flight history
  python3 flight_logger.py

  # View last N flights
  python3 flight_logger.py --last 5

  # View summary stats
  python3 flight_logger.py --summary

  # Called programmatically from postprocess_mesh.py:
  from flight_logger import log_flight
  log_flight(
      bag_name   = "scan_20260319_143215",
      duration_s = 272,
      point_count= 47231,
      mesh_ok    = True,
      notes      = "SLAM bridge active",   # optional
  )
"""

import argparse
import os
from datetime import datetime
from pathlib import Path

# ── config ────────────────────────────────────────────────────────────────────

LOG_DIR      = Path.home() / "unitree_lidar_project" / "logs"
HISTORY_FILE = LOG_DIR / "flight_history.log"
COUNTER_FILE = LOG_DIR / ".flight_counter"   # hidden file tracking flight number

# ── helpers ───────────────────────────────────────────────────────────────────

def _ensure_log_dir():
    LOG_DIR.mkdir(parents=True, exist_ok=True)


def _next_flight_number() -> int:
    """
    Return the next flight number and increment the counter.
    Counter persists across reboots in .flight_counter file.
    """
    _ensure_log_dir()
    if COUNTER_FILE.exists():
        try:
            n = int(COUNTER_FILE.read_text().strip())
        except ValueError:
            n = 0
    else:
        # First flight -- count existing entries to resume correctly
        n = _count_existing_flights()
    n += 1
    COUNTER_FILE.write_text(str(n))
    return n


def _count_existing_flights() -> int:
    """Count existing flight entries so counter resumes correctly after reinstall."""
    if not HISTORY_FILE.exists():
        return 0
    count = 0
    with open(HISTORY_FILE) as f:
        for line in f:
            if line.startswith("Flight "):
                count += 1
    return count


def _format_duration(duration_s: float) -> str:
    """Format seconds as Xm Ys."""
    duration_s = int(duration_s)
    m = duration_s // 60
    s = duration_s % 60
    if m > 0:
        return f"{m}m {s:02d}s"
    return f"{s}s"


def _format_points(n: int) -> str:
    """Format point count with thousands separator."""
    return f"{n:,}"


def _format_date(dt: datetime) -> str:
    """Format date as '19 March 2026'."""
    return dt.strftime("%-d %B %Y")


# ── public API ────────────────────────────────────────────────────────────────

def log_flight(
    bag_name:    str,
    duration_s:  float,
    point_count: int,
    mesh_ok:     bool,
    notes:       str = "",
) -> int:
    """
    Append one flight entry to the history log.

    Parameters
    ----------
    bag_name    : session directory name e.g. 'scan_20260319_143215'
    duration_s  : recording duration in seconds
    point_count : total points in the processed cloud
    mesh_ok     : True if Poisson mesh was generated
    notes       : optional free-text notes (mode, issues, etc.)

    Returns
    -------
    Flight number assigned to this session.
    """
    _ensure_log_dir()

    now        = datetime.now()
    flight_num = _next_flight_number()
    date_str   = _format_date(now)
    time_str   = now.strftime("%H:%M:%S")
    dur_str    = _format_duration(duration_s)
    pts_str    = _format_points(point_count)
    mesh_str   = "YES" if mesh_ok else "NO "
    notes_part = f" | {notes}" if notes else ""

    line = (
        f"Flight {flight_num:03d} | "
        f"{date_str} | "
        f"{time_str} | "
        f"Duration: {dur_str:>8s} | "
        f"Points: {pts_str:>10s} | "
        f"Mesh: {mesh_str} | "
        f"Bag: {bag_name}"
        f"{notes_part}\n"
    )

    with open(HISTORY_FILE, "a") as f:
        f.write(line)

    print(f"[flight_logger] Flight {flight_num:03d} logged → {HISTORY_FILE}")
    return flight_num


def read_history(last_n: int = None) -> list[str]:
    """
    Read flight history entries.

    Parameters
    ----------
    last_n : if set, return only the last N entries

    Returns
    -------
    List of log line strings (without newline).
    """
    if not HISTORY_FILE.exists():
        return []
    with open(HISTORY_FILE) as f:
        lines = [l.rstrip() for l in f if l.startswith("Flight ")]
    if last_n is not None:
        lines = lines[-last_n:]
    return lines


def get_summary() -> dict:
    """
    Return summary statistics across all logged flights.

    Returns
    -------
    dict with keys: total_flights, total_points, mesh_count,
                    no_mesh_count, total_duration_s
    """
    lines      = read_history()
    total_pts  = 0
    mesh_count = 0
    no_mesh    = 0
    total_dur  = 0

    for line in lines:
        # Parse points
        try:
            pts_part = [p for p in line.split("|") if "Points:" in p][0]
            pts = int(pts_part.strip().replace("Points:", "").replace(",", "").strip())
            total_pts += pts
        except (IndexError, ValueError):
            pass

        # Parse mesh
        try:
            mesh_part = [p for p in line.split("|") if "Mesh:" in p][0]
            if "YES" in mesh_part:
                mesh_count += 1
            else:
                no_mesh += 1
        except IndexError:
            pass

        # Parse duration
        try:
            dur_part = [p for p in line.split("|") if "Duration:" in p][0]
            dur_str  = dur_part.strip().replace("Duration:", "").strip()
            # Parse "Xm Ys" or "Ys"
            secs = 0
            if "m" in dur_str:
                parts = dur_str.split("m")
                secs += int(parts[0].strip()) * 60
                secs += int(parts[1].replace("s", "").strip())
            else:
                secs += int(dur_str.replace("s", "").strip())
            total_dur += secs
        except (IndexError, ValueError):
            pass

    return {
        "total_flights":  len(lines),
        "total_points":   total_pts,
        "mesh_count":     mesh_count,
        "no_mesh_count":  no_mesh,
        "total_duration_s": total_dur,
    }


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="View DronePi flight history log.")
    parser.add_argument("--last",    type=int, default=None,
                        help="Show only the last N flights")
    parser.add_argument("--summary", action="store_true",
                        help="Show summary statistics")
    args = parser.parse_args()

    if not HISTORY_FILE.exists():
        print(f"No flight history yet.")
        print(f"Log will be created at: {HISTORY_FILE}")
        print("Entries are added automatically after each flight is processed.")
        return

    if args.summary:
        s = get_summary()
        total_h = s["total_duration_s"] // 3600
        total_m = (s["total_duration_s"] % 3600) // 60
        total_s = s["total_duration_s"] % 60
        print("=" * 55)
        print("  DronePi Flight History — Summary")
        print("=" * 55)
        print(f"  Total flights    : {s['total_flights']}")
        print(f"  Total airtime    : {total_h}h {total_m}m {total_s}s")
        print(f"  Total points     : {s['total_points']:,}")
        print(f"  Meshes generated : {s['mesh_count']}")
        print(f"  Cloud only       : {s['no_mesh_count']}")
        print(f"  Log file         : {HISTORY_FILE}")
        print("=" * 55)
        return

    entries = read_history(args.last)
    if not entries:
        print("No flight entries found.")
        return

    label = f"last {args.last}" if args.last else "all"
    print("=" * 75)
    print(f"  DronePi Flight History ({label} flights)")
    print("=" * 75)
    for entry in entries:
        print(f"  {entry}")
    print("=" * 75)
    print(f"  Log file: {HISTORY_FILE}")
    print("=" * 75)


if __name__ == "__main__":
    main()
