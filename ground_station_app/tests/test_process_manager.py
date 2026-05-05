#!/usr/bin/env python3
"""
tests/test_process_manager.py — Standalone test for ProcessManager.

Tests
-----
1. PID file write / read / remove round-trip.
2. _pid_alive() correctly returns True for the current process and False
   for a recycled/dead PID (PID 1 is init on Linux; unlikely to be alive
   as the current user's process on Windows).
3. is_running() returns False when no PID file exists.
4. register_autostart() / unregister_autostart() run without exception
   on the current platform (Windows: actual schtasks call; others: no-op).
5. spawn_daemon() is NOT called in this test — it would launch a real
   background process. That is covered by test_end_to_end.py.

Run with:
    python tests/test_process_manager.py

Produces a structured log to stdout and exits with code 0 (all pass) or 1.
"""

import logging
import os
import platform
import sys
import tempfile
from pathlib import Path

# Allow running from repo root without installing the package.
sys.path.insert(0, str(Path(__file__).parent.parent))

from ground_station.process_manager import ProcessManager, pid_file_path

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ── Test helpers ──────────────────────────────────────────────────────────────

PASS = "PASS"
FAIL = "FAIL"
results: list[tuple[str, str]] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    status = PASS if condition else FAIL
    results.append((name, status))
    marker = "  [PASS]" if condition else "  [FAIL]"
    msg = f"{marker}  {name}"
    if detail:
        msg += f" — {detail}"
    log.info(msg)


# ── Override PID file to a temp location for tests ────────────────────────────

_tmp_dir = Path(tempfile.mkdtemp(prefix="dronepi_pm_test_"))
os.environ["DRONEPI_PID_FILE"] = str(_tmp_dir / "test_daemon.pid")
os.environ["DRONEPI_LOG_FILE"] = str(_tmp_dir / "test_daemon.log")

# ══════════════════════════════════════════════════════════════════════════════


def test_pid_file_roundtrip() -> None:
    """PID write → read → remove does not corrupt or lose the value."""
    pm = ProcessManager()
    test_pid = 99999

    ProcessManager._write_pid(test_pid, pid_file_path())
    read_back = ProcessManager._read_pid()
    check(
        "pid_file_write_read",
        read_back == test_pid,
        f"wrote={test_pid} read={read_back}",
    )

    ProcessManager._remove_pid()
    after_remove = ProcessManager._read_pid()
    check(
        "pid_file_remove",
        after_remove is None,
        f"after remove: {after_remove}",
    )


def test_pid_alive_self() -> None:
    """Current process PID must report as alive."""
    pm = ProcessManager()
    alive = ProcessManager._pid_alive(os.getpid())
    check("pid_alive_self", alive, f"own pid={os.getpid()}")


def test_pid_alive_dead() -> None:
    """PID 0 should never be alive for a regular user process."""
    # PID 0 is the idle/swapper process on POSIX and is not accessible;
    # on Windows it is the System Idle Process. Either way, os.kill(0, 0)
    # on POSIX raises PermissionError (our _pid_alive returns True for EPERM)
    # so we use a large unlikely PID instead.
    pm = ProcessManager()
    unlikely_pid = 2_000_000
    # If by some miracle this PID exists, the test is indeterminate — skip.
    if platform.system() != "Windows":
        try:
            os.kill(unlikely_pid, 0)
            log.warning(f"PID {unlikely_pid} unexpectedly alive — skipping dead-pid test.")
            check("pid_alive_dead", True, "skipped — PID collision")
            return
        except ProcessLookupError:
            pass
        except PermissionError:
            log.warning(f"PID {unlikely_pid} exists (EPERM) — skipping.")
            check("pid_alive_dead", True, "skipped — PID collision")
            return

    alive = ProcessManager._pid_alive(unlikely_pid)
    check("pid_alive_dead", not alive, f"pid={unlikely_pid} alive={alive}")


def test_is_running_no_pid_file() -> None:
    """is_running() must return False when no PID file exists."""
    ProcessManager._remove_pid()  # ensure clean state
    pm = ProcessManager()
    running = pm.is_running()
    check("is_running_no_pid_file", not running)


def test_is_running_stale_pid() -> None:
    """is_running() must return False for a PID that no longer exists."""
    stale_pid = 2_000_001
    ProcessManager._write_pid(stale_pid, pid_file_path())
    pm = ProcessManager()

    if platform.system() != "Windows":
        try:
            os.kill(stale_pid, 0)
            log.warning(f"Stale PID {stale_pid} is alive — skipping stale test.")
            ProcessManager._remove_pid()
            check("is_running_stale_pid", True, "skipped — PID collision")
            return
        except ProcessLookupError:
            pass
        except PermissionError:
            log.warning(f"PID {stale_pid} exists (EPERM) — skipping.")
            ProcessManager._remove_pid()
            check("is_running_stale_pid", True, "skipped — PID collision")
            return

    running = pm.is_running()
    ProcessManager._remove_pid()
    check("is_running_stale_pid", not running, f"stale pid={stale_pid}")


def test_autostart_registration() -> None:
    """
    register_autostart / unregister_autostart complete without exception.

    On Windows: makes real schtasks.exe calls (requires no admin rights for
    user-scoped tasks). The task is unregistered immediately after.
    On non-Windows: confirmed no-ops that return True.
    """
    pm = ProcessManager()

    reg_result = pm.register_autostart()
    check("register_autostart", reg_result)

    unreg_result = pm.unregister_autostart()
    check("unregister_autostart", unreg_result)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    log.info("=" * 60)
    log.info("ProcessManager test suite")
    log.info(f"Platform: {platform.system()} {platform.release()}")
    log.info(f"Temp dir: {_tmp_dir}")
    log.info("=" * 60)

    test_pid_file_roundtrip()
    test_pid_alive_self()
    test_pid_alive_dead()
    test_is_running_no_pid_file()
    test_is_running_stale_pid()
    test_autostart_registration()

    log.info("=" * 60)
    passed = sum(1 for _, s in results if s == PASS)
    failed = sum(1 for _, s in results if s == FAIL)
    log.info(f"Results: {passed} passed, {failed} failed")

    # Clean up temp dir
    import shutil
    shutil.rmtree(_tmp_dir, ignore_errors=True)

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
