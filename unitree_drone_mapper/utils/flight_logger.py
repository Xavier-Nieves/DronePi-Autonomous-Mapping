#!/usr/bin/env python3
"""
utils/flight_logger.py — DronePi flight history logger.

DESIGN INTENT
-------------
Every arm event produces a record immediately — crash, clean land, test abort,
or any outcome. Post-processing results are appended later if they run, but
a missing or failed postprocess never prevents a flight from being recorded.

TWO ARTIFACTS PER SESSION
--------------------------
1. flight_history.log  — one human-readable line per flight, append-only.
   Format:
     Flight 012 | 22 April 2026 | 14:32:15 | test_altitude_validation.py |
     TEST | Duration: 2m 18s | END: RC_KILL | Points: N/A | Mesh: N/A |
     Bag: alt_validation_20260422_pid4821

2. sessions/<session_id>/flight_record.json  — structured JSON, written at arm
   time and updated at disarm and postprocess. Safe to read at any point.

FLIGHT TYPES
------------
  TEST          — any script using CONTEXT_TEST (test_*.py scripts)
  AUTONOMOUS    — main.py using CONTEXT_MISSION
  MANUAL_SCAN   — drone_watchdog-managed scan (no mixin)
  UNKNOWN       — fallback if caller does not specify

CALL POINTS
-----------
  SafeFlightMixin.start_safety_monitors()  → open_session()   (arm event)
  SafeFlightMixin._teardown()              → close_session()  (disarm event)
  FlightStack.stop() in drone_watchdog     → open_session() + close_session()
  postprocess_mesh.py                      → append_postprocess()

BACKWARD COMPATIBILITY
----------------------
The existing log_flight() and log_failure() signatures are preserved.
postprocess_mesh.py does not need changes — it calls the same functions
it always has. They now also update the session JSON if one exists.

PUBLIC API SUMMARY
------------------
  open_session(session_id, script_name, flight_type, bag_path, context) -> int
  close_session(session_id, end_reason, duration_s, bag_closed_cleanly)
  append_postprocess(session_id, point_count, mesh_ok, mesh_faces,
                     processing_duration_s, failure_stage, failure_error)
  log_flight(...)     ← backward compat, calls open+close+append internally
  log_failure(...)    ← backward compat
  read_history(last_n) -> list[str]
  get_summary()       -> dict
"""

import argparse
import json
import os
import shutil
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional

# ── Paths ─────────────────────────────────────────────────────────────────────

LOG_DIR        = Path(os.environ.get(
    "DRONEPI_LOG_DIR",
    Path.home() / "unitree_lidar_project" / "logs"
))
HISTORY_FILE   = LOG_DIR / "flight_history.log"
COUNTER_FILE   = LOG_DIR / ".flight_counter"
SESSIONS_DIR   = LOG_DIR / "sessions"

# ── Flight type constants ──────────────────────────────────────────────────────

FLIGHT_TYPE_TEST      = "TEST"
FLIGHT_TYPE_AUTONOMUS = "AUTONOMOUS"
FLIGHT_TYPE_MANUAL    = "MANUAL_SCAN"
FLIGHT_TYPE_UNKNOWN   = "UNKNOWN"

# Map SafeFlightMixin context strings to flight type labels
_CONTEXT_TO_TYPE = {
    "test":    FLIGHT_TYPE_TEST,
    "mission": FLIGHT_TYPE_AUTONOMUS,
}

# ── Thread safety ──────────────────────────────────────────────────────────────
_write_lock = threading.Lock()

# ══════════════════════════════════════════════════════════════════════════════
# INTERNAL HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def _ensure_dirs() -> None:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    SESSIONS_DIR.mkdir(parents=True, exist_ok=True)


def _next_flight_number() -> int:
    _ensure_dirs()
    with _write_lock:
        if COUNTER_FILE.exists():
            try:
                n = int(COUNTER_FILE.read_text().strip())
            except ValueError:
                n = _count_existing_flights()
        else:
            n = _count_existing_flights()
        n += 1
        COUNTER_FILE.write_text(str(n))
    return n


def _count_existing_flights() -> int:
    if not HISTORY_FILE.exists():
        return 0
    count = 0
    try:
        with open(HISTORY_FILE) as f:
            for line in f:
                if line.startswith("Flight "):
                    count += 1
    except OSError:
        pass
    return count


def _fmt_duration(s: Optional[float]) -> str:
    if s is None:
        return "N/A"
    s = int(s)
    m, sec = divmod(s, 60)
    return f"{m}m {sec:02d}s" if m else f"{sec}s"


def _fmt_points(n: Optional[int]) -> str:
    return f"{n:,}" if n is not None else "N/A"


def _fmt_date(dt: datetime) -> str:
    return dt.strftime("%-d %B %Y")


def _session_dir(session_id: str) -> Path:
    d = SESSIONS_DIR / session_id
    d.mkdir(parents=True, exist_ok=True)
    return d


def _session_json_path(session_id: str) -> Path:
    return _session_dir(session_id) / "flight_record.json"


def _read_session(session_id: str) -> dict:
    p = _session_json_path(session_id)
    if p.exists():
        try:
            return json.loads(p.read_text())
        except Exception:
            pass
    return {}


def _write_session(session_id: str, record: dict) -> None:
    p = _session_json_path(session_id)
    try:
        with _write_lock:
            p.write_text(json.dumps(record, indent=2))
    except Exception as exc:
        print(f"[flight_logger] WARNING: could not write session JSON: {exc}")


def _copy_record_to_bag(session_id: str, record: dict) -> None:
    """
    Copy flight_record.json into the rosbag session folder so serve.py can
    serve it via GET /rosbags/<session>/flight_record.json.

    Called from append_postprocess() — at that point all flight AND
    postprocess fields are populated, so the laptop receives a complete
    record in a single fetch.

    Guard conditions:
    - record["bag_path"] must be set (written by open_session or close_session)
    - bag_path must be a real directory (not a bare session ID string from
      the backward-compat log_flight() path)
    - Never raises — copy failure is non-fatal and logged only
    """
    bag_path_str = record.get("bag_path")
    if not bag_path_str:
        return
    bag_dir = Path(bag_path_str)
    if not bag_dir.is_dir():
        # bag_path is a bare session ID string (backward-compat path) — skip
        return
    src = _session_json_path(session_id)
    dst = bag_dir / "flight_record.json"
    try:
        shutil.copy2(str(src), str(dst))
        print(f"[flight_logger] flight_record.json → {dst}")
    except Exception as exc:
        print(f"[flight_logger] WARNING: could not copy record to bag dir: {exc}")


def _append_history_line(line: str) -> None:
    _ensure_dirs()
    with _write_lock:
        with open(HISTORY_FILE, "a") as f:
            f.write(line + "\n")


def _build_history_line(record: dict) -> str:
    """
    Build the human-readable flight_history.log line from a session record.
    All fields are right-padded for columnar alignment.
    """
    n           = record.get("flight_number", 0)
    ts          = record.get("arm_time_iso", "")
    script      = record.get("script_name", "unknown")
    ftype       = record.get("flight_type", FLIGHT_TYPE_UNKNOWN)
    duration    = _fmt_duration(record.get("duration_s"))
    end_reason  = record.get("end_reason", "unknown")
    points      = _fmt_points(record.get("point_count"))
    mesh        = ("YES" if record.get("mesh_generated") else
                   "NO"  if record.get("postprocess_ran") else "N/A")
    bag         = record.get("session_id", "")

    # Parse ISO timestamp for date/time display
    try:
        dt       = datetime.fromisoformat(ts)
        date_str = _fmt_date(dt)
        time_str = dt.strftime("%H:%M:%S")
    except Exception:
        date_str = "unknown date"
        time_str = "??:??:??"

    return (
        f"Flight {n:03d} | "
        f"{date_str} | "
        f"{time_str} | "
        f"{script:<35s} | "
        f"{ftype:<12s} | "
        f"Duration: {duration:>8s} | "
        f"END: {end_reason:<22s} | "
        f"Points: {points:>10s} | "
        f"Mesh: {mesh:<3s} | "
        f"Bag: {bag}"
    )


# ══════════════════════════════════════════════════════════════════════════════
# PUBLIC API — PRIMARY (new)
# ══════════════════════════════════════════════════════════════════════════════

def open_session(
    session_id:   str,
    script_name:  str,
    flight_type:  str,
    bag_path:     Optional[str] = None,
    context:      Optional[str] = None,
    pid:          Optional[int] = None,
) -> int:
    """
    Record the moment of arming. Call this as early as possible — before any
    arming attempt. Returns the assigned flight number.

    Parameters
    ----------
    session_id   : Unique session tag (e.g. "test_alt_20260422_143215_pid4821").
                   Used as the key for sessions/<session_id>/flight_record.json.
    script_name  : Basename of the calling script (__file__).
    flight_type  : FLIGHT_TYPE_TEST, FLIGHT_TYPE_AUTONOMUS, FLIGHT_TYPE_MANUAL,
                   or FLIGHT_TYPE_UNKNOWN.
    bag_path     : Absolute path to the bag directory (may be None if bag not yet started).
    context      : SafeFlightMixin context string ("test" or "mission") — used to
                   override flight_type if flight_type is UNKNOWN.
    pid          : PID of the calling process (optional, for cross-reference).
    """
    if context and flight_type == FLIGHT_TYPE_UNKNOWN:
        flight_type = _CONTEXT_TO_TYPE.get(context, FLIGHT_TYPE_UNKNOWN)

    flight_num = _next_flight_number()
    now        = datetime.now()

    record = {
        "flight_number":   flight_num,
        "session_id":      session_id,
        "script_name":     script_name,
        "flight_type":     flight_type,
        "context":         context,
        "pid":             pid or os.getpid(),
        "arm_time_iso":    now.isoformat(),
        "arm_time_unix":   now.timestamp(),
        "bag_path":        bag_path,
        "end_reason":      None,           # filled by close_session()
        "disarm_time_iso": None,
        "duration_s":      None,
        "bag_closed_cleanly": None,
        "postprocess_ran": False,
        "point_count":     None,
        "mesh_generated":  None,
        "mesh_faces":      None,
        "processing_duration_s": None,
        "failure_stage":   None,
        "failure_error":   None,
    }

    _write_session(session_id, record)
    print(f"[flight_logger] Flight {flight_num:03d} opened — "
          f"{script_name} ({flight_type})  session={session_id}")
    return flight_num


def close_session(
    session_id:        str,
    end_reason:        str,
    duration_s:        Optional[float] = None,
    bag_closed_cleanly: bool = True,
    bag_path:          Optional[str] = None,
) -> None:
    """
    Record the disarm/teardown event. Safe to call even if open_session()
    was never called (creates a minimal record).

    Parameters
    ----------
    session_id         : Must match the session_id passed to open_session().
    end_reason         : How the flight ended. Examples:
                           "normal"              clean land + disarm
                           "RC_KILL_SWITCH"      CH7 fired
                           "PILOT_OVERRIDE"      mode switch in test context
                           "SETPOINT_WATCHDOG_STALE"
                           "ERRATIC_ANGULAR_VELOCITY"
                           "ALTITUDE_DEVIATION"
                           "keyboard_interrupt"
                           "arm_rejected"
                           "offboard_set_failed"
                           "manual_scan_disarm"  watchdog-managed scan
    duration_s         : Seconds from arm to disarm.
    bag_closed_cleanly : True if SIGINT to bag recorder completed within timeout.
    bag_path           : Update bag path if not set at open time.
    """
    record = _read_session(session_id)
    if not record:
        # open_session() was never called — create a minimal stub record
        flight_num   = _next_flight_number()
        record = {
            "flight_number": flight_num,
            "session_id":    session_id,
            "script_name":   "unknown",
            "flight_type":   FLIGHT_TYPE_UNKNOWN,
            "arm_time_iso":  None,
            "arm_time_unix": None,
            "postprocess_ran": False,
        }

    now = datetime.now()

    # Compute duration from stored arm time if not provided
    if duration_s is None and record.get("arm_time_unix"):
        duration_s = now.timestamp() - record["arm_time_unix"]

    record["end_reason"]         = end_reason
    record["disarm_time_iso"]    = now.isoformat()
    record["duration_s"]         = round(duration_s, 1) if duration_s else None
    record["bag_closed_cleanly"] = bag_closed_cleanly
    if bag_path:
        record["bag_path"]       = bag_path

    _write_session(session_id, record)

    # Write the history line now that we have duration + end reason.
    # If postprocess runs later it will update the JSON but NOT re-write
    # the history line (that would cause duplicates). The history line
    # is written once at close time with whatever is known then.
    line = _build_history_line(record)
    _append_history_line(line)
    print(f"[flight_logger] Flight {record['flight_number']:03d} closed — "
          f"reason={end_reason}  duration={_fmt_duration(duration_s)}")


def append_postprocess(
    session_id:            str,
    point_count:           Optional[int]   = None,
    mesh_ok:               bool            = False,
    mesh_faces:            Optional[int]   = None,
    processing_duration_s: Optional[float] = None,
    failure_stage:         Optional[str]   = None,
    failure_error:         Optional[str]   = None,
) -> None:
    """
    Append post-processing results to an existing session record.
    Updates the JSON only — does NOT write another history line.
    Safe to call even if no session JSON exists (silently returns).

    Also copies the completed flight_record.json to the rosbag session folder
    so serve.py can serve it via GET /rosbags/<session>/flight_record.json.
    This is the only copy point — it fires after all fields are populated so
    the laptop always receives a complete record in a single fetch.

    Called by postprocess_mesh.py after processing completes or fails.
    """
    record = _read_session(session_id)
    if not record:
        # No session was opened (e.g. very old bag, or manual CLI reprocess).
        # Silently skip — don't create a phantom record.
        print(f"[flight_logger] append_postprocess: no session record for "
              f"'{session_id}' — skipping JSON update")
        return

    record["postprocess_ran"]        = True
    record["point_count"]            = point_count
    record["mesh_generated"]         = mesh_ok
    record["mesh_faces"]             = mesh_faces
    record["processing_duration_s"]  = (
        round(processing_duration_s, 1) if processing_duration_s else None
    )
    record["failure_stage"]  = failure_stage
    record["failure_error"]  = failure_error

    _write_session(session_id, record)
    status = "success" if not failure_stage else f"FAILED at {failure_stage}"
    print(f"[flight_logger] Flight {record.get('flight_number', '?'):03d} "
          f"postprocess appended — {status}")

    # Copy completed record to rosbag folder for serve.py / ArtifactFetcher.
    # Done here — after all fields written — so the laptop receives one
    # complete file rather than a partial record.
    _copy_record_to_bag(session_id, record)


# ══════════════════════════════════════════════════════════════════════════════
# PUBLIC API — BACKWARD COMPAT (existing callers unchanged)
# ══════════════════════════════════════════════════════════════════════════════

def log_flight(
    bag_name:    str,
    duration_s:  float,
    point_count: int,
    mesh_ok:     bool  = True,
    mesh_faces:  Optional[int] = None,
    notes:       str   = "",
) -> int:
    """
    Backward-compatible entry point. Called by postprocess_mesh.py on success.

    Tries to find an existing session record for bag_name and append postprocess
    results to it. If no record exists (old-style bag or watchdog-only session
    that never called open_session), creates a complete record directly.

    Returns the flight number.
    """
    session_id = bag_name

    # Try to append to an existing session opened by the mixin or watchdog
    existing = _read_session(session_id)
    if existing:
        append_postprocess(
            session_id            = session_id,
            point_count           = point_count,
            mesh_ok               = mesh_ok,
            mesh_faces            = mesh_faces,
            processing_duration_s = None,
        )
        return existing.get("flight_number", 0)

    # No existing session — create a complete record (backward compat path)
    flight_num = open_session(
        session_id  = session_id,
        script_name = "postprocess_mesh.py",
        flight_type = FLIGHT_TYPE_UNKNOWN,
        bag_path    = str(bag_name),
    )
    close_session(
        session_id  = session_id,
        end_reason  = "postprocess_only",
        duration_s  = duration_s,
    )
    append_postprocess(
        session_id  = session_id,
        point_count = point_count,
        mesh_ok     = mesh_ok,
        mesh_faces  = mesh_faces,
    )
    return flight_num


def log_failure(
    bag_name:    str,
    stage:       str,
    error:       str,
    duration_s:  Optional[float] = None,
    point_count: Optional[int]   = None,
) -> int:
    """
    Backward-compatible entry point. Called by debugger_tools/status.py on failure.
    """
    session_id = bag_name
    existing   = _read_session(session_id)

    if existing:
        append_postprocess(
            session_id     = session_id,
            point_count    = point_count,
            mesh_ok        = False,
            failure_stage  = stage,
            failure_error  = error,
        )
        return existing.get("flight_number", 0)

    flight_num = open_session(
        session_id  = session_id,
        script_name = "postprocess_mesh.py",
        flight_type = FLIGHT_TYPE_UNKNOWN,
        bag_path    = str(bag_name),
    )
    close_session(
        session_id  = session_id,
        end_reason  = f"postprocess_failed:{stage}",
        duration_s  = duration_s,
    )
    append_postprocess(
        session_id    = session_id,
        point_count   = point_count,
        mesh_ok       = False,
        failure_stage = stage,
        failure_error = error,
    )
    return flight_num


# ══════════════════════════════════════════════════════════════════════════════
# READ / QUERY API
# ══════════════════════════════════════════════════════════════════════════════

def read_history(last_n: Optional[int] = None) -> list:
    if not HISTORY_FILE.exists():
        return []
    try:
        with open(HISTORY_FILE) as f:
            lines = [l.rstrip() for l in f if l.startswith("Flight ")]
    except OSError:
        return []
    return lines[-last_n:] if last_n else lines


def read_session_record(session_id: str) -> Optional[dict]:
    """Return the full structured record for a session, or None."""
    r = _read_session(session_id)
    return r if r else None


def get_summary() -> dict:
    lines = read_history()
    total = len(lines)
    if not total:
        return {"total": 0}

    by_type: dict = {}
    for line in lines:
        parts = {p.strip() for p in line.split("|")}
        for ftype in (FLIGHT_TYPE_TEST, FLIGHT_TYPE_AUTONOMUS,
                      FLIGHT_TYPE_MANUAL, FLIGHT_TYPE_UNKNOWN):
            if ftype in parts:
                by_type[ftype] = by_type.get(ftype, 0) + 1

    return {
        "total":   total,
        "by_type": by_type,
    }


def list_sessions() -> list:
    """Return list of all session IDs that have a flight_record.json."""
    if not SESSIONS_DIR.exists():
        return []
    return sorted(
        p.parent.name
        for p in SESSIONS_DIR.glob("*/flight_record.json")
    )


# ══════════════════════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════════════════════

def _cli_print(lines: list) -> None:
    if not lines:
        print("No flight records found.")
        return
    for line in lines:
        print(line)


def _cli() -> None:
    parser = argparse.ArgumentParser(
        description="DronePi flight logger — read flight history."
    )
    sub = parser.add_subparsers(dest="cmd")

    p_hist = sub.add_parser("history", help="Print flight history log")
    p_hist.add_argument("--last", type=int, default=None,
                        help="Show last N flights only")

    p_sum  = sub.add_parser("summary", help="Print flight summary counts")
    p_sess = sub.add_parser("sessions", help="List all session IDs")

    p_show = sub.add_parser("show", help="Show structured record for a session")
    p_show.add_argument("session_id")

    args = parser.parse_args()

    if args.cmd == "history":
        _cli_print(read_history(args.last))
    elif args.cmd == "summary":
        s = get_summary()
        print(f"Total flights : {s['total']}")
        for k, v in s.get("by_type", {}).items():
            print(f"  {k:<12s}: {v}")
    elif args.cmd == "sessions":
        for sid in list_sessions():
            print(sid)
    elif args.cmd == "show":
        r = read_session_record(args.session_id)
        if r:
            print(json.dumps(r, indent=2))
        else:
            print(f"No record found for session '{args.session_id}'")
    else:
        parser.print_help()


if __name__ == "__main__":
    _cli()
