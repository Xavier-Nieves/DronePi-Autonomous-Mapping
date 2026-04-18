"""watchdog_core/postflight.py — Post-flight processing monitor.

Launches run_postflight.py as a subprocess and:
  - Pipes stdout line-by-line to the watchdog log
  - Drives PostflightBuzzerDriver via its poll() method so stage-transition
    tones fire as postprocess_mesh.py advances through the pipeline
  - Plays TUNE_POSTPROCESS_DONE when the subprocess exits with code 0
  - Plays TUNE_ERROR once if it exits non-zero

Interpreter selection
---------------------
postprocess_mesh.py requires open3d, trimesh, scipy, pdal, and rosbags —
all installed in the dronepi conda environment, not in system Python.
The watchdog service runs under /usr/bin/python3 (sys.executable), so
launching the postflight script with sys.executable would fail at the
first import.

Resolution order:
  1. ~/miniforge3/envs/dronepi/bin/python3   (standard Miniforge install)
  2. ~/anaconda3/envs/dronepi/bin/python3    (Anaconda install fallback)
  3. ~/miniconda3/envs/dronepi/bin/python3   (Miniconda install fallback)
  4. sys.executable                           (last resort — may lack packages)

The selected interpreter is logged at launch so failures are diagnosable.

Buzzer → LED coherence
----------------------
The PostflightBuzzerDriver (buzzer.py) reads /tmp/postflight_status.json
written by postprocess_mesh.py's PostflightIPC and fires:
  - TUNE_PF_EXTRACT    when stage = "bag_extract"
  - TUNE_PF_CLASSIFY   when stage = "ground_classify"
  - TUNE_PF_MESH_STAGE when stage = "dtm" or "dsm"
  - background TUNE_POSTPROCESS_ACTIVE tick throughout

These match the LED service's POSTFLIGHT_RUNNING state (yellow 1 Hz blink).
On subprocess exit:
  - exit 0 → TUNE_POSTPROCESS_DONE  ↔  LED POSTFLIGHT_DONE (green+yellow, held 5 s)
  - exit !0 → TUNE_ERROR             ↔  LED POSTFLIGHT_FAILED (red 3 Hz)

Everything runs on daemon threads so the watchdog main loop is never blocked.
poll() is driven by the caller (drone_watchdog.py) from its main loop.
"""

from __future__ import annotations

import subprocess
import sys
import threading
from pathlib import Path

from .buzzer import PostflightBuzzerDriver, TUNE_ERROR, TUNE_POSTPROCESS_DONE
from .logging_utils import log

SCRIPT_DIR        = Path(__file__).parent.parent
POSTFLIGHT_SCRIPT = SCRIPT_DIR.parent / "utils" / "run_postflight.py"

# ── Conda interpreter resolution ──────────────────────────────────────────────
# Checked in priority order. The first path that exists wins.
# sys.executable is the final fallback so the watchdog never hard-crashes
# even on a non-standard install, but a warning is logged so the operator
# knows the pipeline may fail at import time.
_CONDA_CANDIDATES = [
    Path.home() / "miniforge3"  / "envs" / "dronepi" / "bin" / "python3",
    Path.home() / "anaconda3"   / "envs" / "dronepi" / "bin" / "python3",
    Path.home() / "miniconda3"  / "envs" / "dronepi" / "bin" / "python3",
]


def _resolve_python() -> str:
    """Return the path to the dronepi conda Python, falling back to sys.executable."""
    for candidate in _CONDA_CANDIDATES:
        if candidate.exists():
            return str(candidate)
    log(
        "[POSTFLIGHT] WARN — dronepi conda env not found at any standard path. "
        "Falling back to sys.executable. Pipeline may fail if open3d/trimesh/pdal "
        "are not installed in system Python. "
        "Expected: ~/miniforge3/envs/dronepi/bin/python3"
    )
    return sys.executable


class PostflightMonitor:
    """Manages a single postflight subprocess with stage-aware buzzer feedback.

    Parameters
    ----------
    reader : MavrosReader
        Provides play_tune() so all audio goes through MAVROS to the Pixhawk.

    Public interface
    ----------------
    trigger()       Start the postflight subprocess (no-op if already running).
    poll()          Advance PostflightBuzzerDriver — call from watchdog main loop.
    is_active       True while the subprocess is running.
    """

    def __init__(self, reader) -> None:
        self._reader    = reader
        self._pf_buzzer = PostflightBuzzerDriver(
            play_tune_fn=reader.play_tune,
        )
        self._lock   = threading.Lock()
        self._active = False

    @property
    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def poll(self) -> None:
        """Advance the stage-aware buzzer driver.

        Must be called on every watchdog main loop cycle so stage-transition
        tones fire promptly when postprocess_mesh.py advances.
        """
        self._pf_buzzer.poll()

    def trigger(self) -> None:
        """Launch the postflight script. No-op if already running."""
        if not POSTFLIGHT_SCRIPT.exists():
            log(f"[POSTFLIGHT] WARN — Post-flight script not found: {POSTFLIGHT_SCRIPT}")
            return

        with self._lock:
            if self._active:
                log("[POSTFLIGHT] Already running — skipping duplicate trigger")
                return
            self._active = True

        python = _resolve_python()
        log(f"[POSTFLIGHT] Launching post-flight processing...")
        log(f"[POSTFLIGHT] Interpreter: {python}")

        try:
            proc = subprocess.Popen(
                [python, str(POSTFLIGHT_SCRIPT), "--auto", "--skip-wait"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except Exception as exc:
            log(f"[POSTFLIGHT] Launch failed: {exc}")
            self._reader.play_tune(TUNE_ERROR)
            self._pf_buzzer.stop()
            with self._lock:
                self._active = False
            return

        monitor_thread = threading.Thread(
            target=self._monitor, args=(proc,), daemon=True
        )
        monitor_thread.start()

    def _monitor(self, proc: subprocess.Popen) -> None:
        """Pipe subprocess output to the watchdog log, then play finish tone.

        The PostflightBuzzerDriver handles all in-progress audio via poll().
        This method only fires the terminal tone (done or failed) and clears
        the driver once the subprocess has exited.
        """
        try:
            if proc.stdout is not None:
                for line in proc.stdout:
                    stripped = line.rstrip()
                    if stripped:
                        log(f"[POSTFLIGHT] {stripped}")
            proc.wait()
        except Exception as exc:
            log(f"[POSTFLIGHT] Log pipe error: {exc}")
        finally:
            # Stop the background heartbeat tick.
            # Terminal tones (TUNE_POSTPROCESS_DONE / TUNE_ERROR) are handled
            # by PostflightBuzzerDriver.poll() which reads postflight_status.json
            # written by PostflightIPC in postprocess_mesh.py.
            self._pf_buzzer.stop()

            if proc.returncode == 0:
                log("[POSTFLIGHT] Processing complete")
            else:
                log(f"[POSTFLIGHT] Processing failed (exit {proc.returncode})")

            with self._lock:
                self._active = False
