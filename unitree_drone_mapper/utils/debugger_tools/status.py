"""
status.py — Pipeline status tracking and failure reporting.

Provides stage tracking so errors report exactly where the pipeline
crashed, and writes metadata.json on failure so the web viewer can
display meaningful error messages instead of showing stale data.

Usage:
    from debugger_tools import PipelineStatus, PipelineStage, write_failure_status
    
    # In main():
    current_stage = PipelineStage.INIT
    
    try:
        current_stage = PipelineStage.BAG_EXTRACT
        pts = reader.extract()
        
        current_stage = PipelineStage.MLS
        pts = smoother.smooth(pts)
        
        # ... rest of pipeline ...
        
    except Exception as exc:
        write_failure_status(bag_path, current_stage.value, exc)
        sys.exit(1)

Failure metadata.json example:
    {
        "id": "scan_20260319_230358",
        "status": "failed",
        "failed_stage": "dsm",
        "error": "ValueError: Not enough points for BPA",
        "processed_at": "2026-03-24 15:30"
    }
"""

import json
import sys
import traceback
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Optional


class PipelineStage(Enum):
    """
    Pipeline execution stages for error tracking.
    
    Used to identify exactly where a failure occurred.
    """
    INIT = "init"
    BAG_EXTRACT = "bag_extract"
    CAP = "cap"
    MLS = "mls"
    GROUND_CLASSIFY = "ground_classify"
    DTM = "dtm"
    DSM = "dsm"
    MERGE = "merge"
    PUBLISH = "publish"
    COMPLETE = "complete"


class PipelineStatus:
    """
    Tracks pipeline execution status.
    
    Can be used as a context manager for automatic failure handling,
    or manually with stage assignment.
    
    Usage (context manager):
        with PipelineStatus(bag_path) as status:
            status.stage = PipelineStage.BAG_EXTRACT
            pts = reader.extract()
            # If exception occurs, automatically writes failure status
    
    Usage (manual):
        status = PipelineStatus(bag_path)
        try:
            status.stage = PipelineStage.BAG_EXTRACT
            pts = reader.extract()
        except Exception as e:
            status.fail(e)
            sys.exit(1)
    
    Parameters
    ----------
    bag_path : Path
        Path to the bag directory (for metadata.json output)
    session_id : str, optional
        Session identifier (defaults to bag_path.name)
    """
    
    def __init__(self, bag_path: Path, session_id: str = None):
        self.bag_path = Path(bag_path)
        self.session_id = session_id or self.bag_path.name
        self.stage = PipelineStage.INIT
        self.start_time = datetime.now()
        self.error: Optional[Exception] = None
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_val is not None:
            self.fail(exc_val)
            return False  # Re-raise the exception
        return False
    
    def fail(self, exc: Exception) -> None:
        """
        Handle pipeline failure.
        
        Prints error message with stage info, prints traceback,
        and writes failure status to metadata.json.
        
        Parameters
        ----------
        exc : Exception
            The exception that caused the failure
        """
        self.error = exc
        
        print(f"\n{'='*60}", file=sys.stderr)
        print(f"[FAIL] Pipeline crashed at stage '{self.stage.value}'", file=sys.stderr)
        print(f"       Error: {type(exc).__name__}: {exc}", file=sys.stderr)
        print(f"{'='*60}", file=sys.stderr)
        traceback.print_exc()
        
        write_failure_status(
            bag_path=self.bag_path,
            stage=self.stage.value,
            exc=exc,
            session_id=self.session_id,
        )
    
    def complete(self) -> None:
        """Mark pipeline as successfully completed."""
        self.stage = PipelineStage.COMPLETE


def write_failure_status(
    bag_path: Path,
    stage: str,
    exc: Exception,
    session_id: str = None,
    duration_s: float = None,
    point_count: int = None,
) -> None:
    """
    Write a metadata.json marking the pipeline as failed.

    Called from the exception handler so the web viewer can display
    a failure notice instead of showing stale or missing results.
    Also logs the failure to flight_history.log via flight_logger.
    
    Parameters
    ----------
    bag_path : Path
        Directory containing the bag file (metadata.json goes here)
    stage : str
        Pipeline stage where failure occurred (e.g., "dsm", "mls")
    exc : Exception
        The exception that caused the failure
    session_id : str, optional
        Session identifier (defaults to bag_path.name)
    duration_s : float, optional
        Time elapsed before failure (for flight log)
    point_count : int, optional
        Points processed before failure (for flight log)
    
    Output
    ------
    1. Writes {bag_path}/metadata.json with failure info
    2. Appends failure entry to flight_history.log
    """
    bag_path = Path(bag_path)
    session_id = session_id or bag_path.name
    error_msg = f"{type(exc).__name__}: {exc}"
    
    # Write metadata.json for web viewer
    payload = {
        "id": session_id,
        "status": "failed",
        "failed_stage": stage,
        "error": error_msg,
        "processed_at": datetime.now().strftime("%Y-%m-%d %H:%M"),
    }
    
    try:
        out = bag_path / "metadata.json"
        with open(out, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"  [Pipeline] Failure status written → {out}")
    except Exception as write_err:
        print(f"  [Pipeline] Could not write failure status: {write_err}",
              file=sys.stderr)
    
    # Log to flight history
    try:
        from flight_logger import log_failure
        log_failure(
            bag_name=session_id,
            stage=stage,
            error=error_msg,
            duration_s=duration_s,
            point_count=point_count,
        )
    except ImportError:
        # flight_logger not available, skip
        pass
    except Exception as log_err:
        print(f"  [Pipeline] Could not log failure: {log_err}",
              file=sys.stderr)
