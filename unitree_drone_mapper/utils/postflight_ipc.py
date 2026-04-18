"""
utils/postflight_ipc.py — Post-flight pipeline status IPC writer.

Writes /tmp/postflight_status.json at each pipeline stage so led_service.py
can display accurate LED feedback throughout the post-flight pipeline rather
than holding a static solid yellow (PROCESSING) from start to finish.

Schema written to /tmp/postflight_status.json:
    {
      "ts":     float,  # Unix timestamp of last write (heartbeat)
      "stage":  str,    # Current pipeline stage name (from PipelineStage enum)
      "failed": bool,   # True once the pipeline has crashed
      "done":   bool,   # True once the pipeline has completed successfully
    }

The LED service (flight/led_service.py) reads this file every poll cycle and
maps the fields to LED states via _derive_state() priorities 11–13:
    pf_failed → POSTFLIGHT_FAILED  (red 3 Hz blink)
    pf_done   → POSTFLIGHT_DONE    (green + yellow solid, held 5 s)
    pf_stage  → POSTFLIGHT_RUNNING (yellow 1 Hz blink)

Usage (called by postprocess_mesh.py)
--------------------------------------
    from postflight_ipc import PostflightIPC

    pf = PostflightIPC()
    pf.update("bag_extract")
    # ... pipeline work ...
    pf.update("mls")
    # ... more work ...
    pf.done()          # on success
    pf.failed("dsm")   # on failure (also called by write_failure_status)

The file is cleared (removed) on clean service shutdown. If led_service.py
reads a stale file from a previous boot it falls through to IDLE because the
file's ts field is too old to be considered live.
"""

import json
import os
import time
from pathlib import Path

_STATUS_FILE     = "/tmp/postflight_status.json"
_STATUS_FILE_TMP = _STATUS_FILE + ".tmp"


class PostflightIPC:
    """
    Writes /tmp/postflight_status.json at each pipeline stage.

    Thread-safe via atomic file rename (write to .tmp then os.replace).
    """

    def __init__(self) -> None:
        self._stage  = ""
        self._failed = False
        self._done   = False

    # ── Public API ─────────────────────────────────────────────────────────────

    def update(self, stage: str) -> None:
        """Advance to the named pipeline stage and write heartbeat."""
        self._stage  = stage
        self._failed = False
        self._done   = False
        self._write()

    def done(self) -> None:
        """Mark pipeline as successfully completed."""
        self._done = True
        self._write()

    def failed(self, stage: str = "") -> None:
        """Mark pipeline as failed, optionally at a specific stage."""
        if stage:
            self._stage = stage
        self._failed = True
        self._write()

    def clear(self) -> None:
        """Remove the status file (call on clean shutdown or at service start)."""
        try:
            Path(_STATUS_FILE).unlink(missing_ok=True)
            Path(_STATUS_FILE_TMP).unlink(missing_ok=True)
        except Exception:
            pass

    # ── Internal ───────────────────────────────────────────────────────────────

    def _write(self) -> None:
        try:
            payload = json.dumps({
                "ts":     time.time(),
                "stage":  self._stage,
                "failed": self._failed,
                "done":   self._done,
            })
            with open(_STATUS_FILE_TMP, "w") as f:
                f.write(payload)
            os.replace(_STATUS_FILE_TMP, _STATUS_FILE)
        except Exception as exc:
            print(f"  [PostflightIPC] Status write failed: {exc}")
            
    def __init__(self) -> None:
        self._stage  = ""
        self._failed = False
        self._done   = False
        self.clear()   # ← ADD: remove stale file from previous boot/crash
