#!/usr/bin/env python3
"""
flight_logger.py — Flight history logger for DronePi.

Maintains a human-readable flight history log at:
    ~/unitree_lidar_project/logs/flight_history.log

Each entry records one flight session with all available metadata,
including failures with stage information from debugger_tools.status.

Log format (one line per flight):
    Flight NNN | DD Month YYYY | HH:MM:SS | Duration: Xm Ys | Points: N | Status: OK/FAILED | Bag: scan_...

Usage
-----
  # View full flight history
  python3 flight_logger.py

  # View last N flights
  python3 flight_logger.py --last 5

  # View summary stats
  python3 flight_logger.py --summary

  # View only failures
  python3 flight_logger.py --failures

  # Called programmatically from postprocess_mesh.py:
  from flight_logger import log_flight, log_failure
  
  # On success:
  log_flight(
      bag_name    = "scan_20260319_143215",
      duration_s  = 272,
      point_count = 47231,
      mesh_faces  = 125000,
      notes       = "DTM+DSM mode",
  )
  
  # On failure (called by debugger_tools.status):
  log_failure(
      bag_name    = "scan_20260319_143215",
      stage       = "dsm",
      error       = "ValueError: Not enough points",
      duration_s  = 45,
      point_count = 12000,
  )
"""

import argparse
from datetime import datetime
from pathlib import Path
from typing import Optional

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
    if duration_s is None:
        return "N/A"
    duration_s = int(duration_s)
    m = duration_s // 60
    s = duration_s % 60
    if m > 0:
        return f"{m}m {s:02d}s"
    return f"{s}s"


def _format_points(n: int) -> str:
    """Format point count with thousands separator."""
    if n is None:
        return "N/A"
    return f"{n:,}"


def _format_date(dt: datetime) -> str:
    """Format date as '19 March 2026'."""
    return dt.strftime("%-d %B %Y")


# ── public API ────────────────────────────────────────────────────────────────

def log_flight(
    bag_name:    str,
    duration_s:  float,
    point_count: int,
    mesh_faces:  Optional[int] = None,
    mesh_ok:     bool = True,
    notes:       str = "",
) -> int:
    """
    Append one successful flight entry to the history log.

    Parameters
    ----------
    bag_name    : session directory name e.g. 'scan_20260319_143215'
    duration_s  : recording duration in seconds
    point_count : total points in the processed cloud
    mesh_faces  : number of faces in final mesh (optional)
    mesh_ok     : True if mesh was generated (default True)
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
    
    # Status field
    if mesh_ok and mesh_faces:
        status_str = f"OK ({mesh_faces:,} faces)"
    elif mesh_ok:
        status_str = "OK"
    else:
        status_str = "CLOUD ONLY"
    
    notes_part = f" | {notes}" if notes else ""

    line = (
        f"Flight {flight_num:03d} | "
        f"{date_str} | "
        f"{time_str} | "
        f"Duration: {dur_str:>8s} | "
        f"Points: {pts_str:>10s} | "
        f"Status: {status_str:<20s} | "
        f"Bag: {bag_name}"
        f"{notes_part}\n"
    )

    with open(HISTORY_FILE, "a") as f:
        f.write(line)

    print(f"[flight_logger] Flight {flight_num:03d} logged → {HISTORY_FILE}")
    return flight_num


def log_failure(
    bag_name:    str,
    stage:       str,
    error:       str,
    duration_s:  Optional[float] = None,
    point_count: Optional[int] = None,
) -> int:
    """
    Append one failed flight entry to the history log.
    
    Called by debugger_tools.status.write_failure_status() when
    the pipeline crashes at any stage.

    Parameters
    ----------
    bag_name    : session directory name e.g. 'scan_20260319_143215'
    stage       : pipeline stage where failure occurred (e.g., 'dsm', 'mls')
    error       : error message (truncated if too long)
    duration_s  : time elapsed before failure (optional)
    point_count : points processed before failure (optional)

    Returns
    -------
    Flight number assigned to this session.
    """
    _ensure_log_dir()

    now        = datetime.now()
    flight_num = _next_flight_number()
    date_str   = _format_date(now)
    time_str   = now.strftime("%H:%M:%S")
    dur_str    = _format_duration(duration_s) if duration_s else "N/A"
    pts_str    = _format_points(point_count) if point_count else "N/A"
    
    # Truncate error message if too long
    error_short = error[:50] + "..." if len(error) > 50 else error
    status_str = f"FAILED @ {stage}"

    line = (
        f"Flight {flight_num:03d} | "
        f"{date_str} | "
        f"{time_str} | "
        f"Duration: {dur_str:>8s} | "
        f"Points: {pts_str:>10s} | "
        f"Status: {status_str:<20s} | "
        f"Bag: {bag_name} | "
        f"Error: {error_short}\n"
    )

    with open(HISTORY_FILE, "a") as f:
        f.write(line)

    print(f"[flight_logger] Flight {flight_num:03d} (FAILED) logged → {HISTORY_FILE}")
    return flight_num


def read_history(last_n: int = None, failures_only: bool = False) -> list[str]:
    """
    Read flight history entries.

    Parameters
    ----------
    last_n : if set, return only the last N entries
    failures_only : if True, return only failed flights

    Returns
    -------
    List of log line strings (without newline).
    """
    if not HISTORY_FILE.exists():
        return []
    with open(HISTORY_FILE) as f:
        lines = [l.rstrip() for l in f if l.startswith("Flight ")]
    
    if failures_only:
        lines = [l for l in lines if "FAILED" in l]
    
    if last_n is not None:
        lines = lines[-last_n:]
    return lines


def get_summary() -> dict:
    """
    Return summary statistics across all logged flights.

    Returns
    -------
    dict with keys: total_flights, total_points, success_count,
                    failed_count, cloud_only_count, total_duration_s
    """
    lines         = read_history()
    total_pts     = 0
    success_count = 0
    failed_count  = 0
    cloud_only    = 0
    total_dur     = 0

    for line in lines:
        # Parse points
        try:
            pts_part = [p for p in line.split("|") if "Points:" in p][0]
            pts_str = pts_part.strip().replace("Points:", "").replace(",", "").strip()
            if pts_str != "N/A":
                total_pts += int(pts_str)
        except (IndexError, ValueError):
            pass

        # Parse status
        try:
            status_part = [p for p in line.split("|") if "Status:" in p][0]
            if "FAILED" in status_part:
                failed_count += 1
            elif "CLOUD ONLY" in status_part:
                cloud_only += 1
            elif "OK" in status_part:
                success_count += 1
        except IndexError:
            pass

        # Parse duration
        try:
            dur_part = [p for p in line.split("|") if "Duration:" in p][0]
            dur_str  = dur_part.strip().replace("Duration:", "").strip()
            if dur_str != "N/A":
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
        "total_flights":    len(lines),
        "total_points":     total_pts,
        "success_count":    success_count,
        "failed_count":     failed_count,
        "cloud_only_count": cloud_only,
        "total_duration_s": total_dur,
    }


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="View DronePi flight history log.")
    parser.add_argument("--last", type=int, default=None,
                        help="Show only the last N flights")
    parser.add_argument("--summary", action="store_true",
                        help="Show summary statistics")
    parser.add_argument("--failures", action="store_true",
                        help="Show only failed flights")
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
        print("=" * 60)
        print("  DronePi Flight History — Summary")
        print("=" * 60)
        print(f"  Total flights    : {s['total_flights']}")
        print(f"  Successful       : {s['success_count']}")
        print(f"  Failed           : {s['failed_count']}")
        print(f"  Cloud only       : {s['cloud_only_count']}")
        print(f"  Total airtime    : {total_h}h {total_m}m {total_s}s")
        print(f"  Total points     : {s['total_points']:,}")
        print(f"  Log file         : {HISTORY_FILE}")
        print("=" * 60)
        return

    entries = read_history(args.last, failures_only=args.failures)
    if not entries:
        if args.failures:
            print("No failed flights found. 🎉")
        else:
            print("No flight entries found.")
        return

    label = "failed" if args.failures else f"last {args.last}" if args.last else "all"
    print("=" * 90)
    print(f"  DronePi Flight History ({label} flights)")
    print("=" * 90)
    for entry in entries:
        print(f"  {entry}")
    print("=" * 90)
    print(f"  Log file: {HISTORY_FILE}")
    print("=" * 90)


if __name__ == "__main__":
    main()
