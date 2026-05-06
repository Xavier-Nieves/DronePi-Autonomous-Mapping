"""
ground_station/process_manager.py — Cross-platform daemon process management.

Responsibilities
----------------
- Spawn the FlightReportDaemon as a detached background process.
- Track its PID via a file at DRONEPI_PID_FILE.
- Detect whether the daemon is alive (poll PID without blocking).
- Terminate the daemon cleanly (SIGTERM → 5 s grace → SIGKILL on POSIX;
  TerminateProcess on Windows).
- Register / unregister a Windows Task Scheduler task for cross-reboot
  persistence using schtasks.exe with an XML task definition.
- On Linux/macOS the equivalent is a ~/.config/systemd/user unit (out of
  scope for this build; the project's primary dev target is Windows laptop).

Design constraints (inherited from parent project)
---------------------------------------------------
- No main() in this module; it is imported by cli.py only.
- All tunable paths come from environment variables with documented defaults.
- All operations are wrapped in try/except; failures are non-fatal to the
  caller (the CLI reports the error and continues).

References
----------
- Python subprocess docs: https://docs.python.org/3/library/subprocess.html
- Windows Task Scheduler XML schema:
  https://learn.microsoft.com/en-us/windows/win32/taskschd/task-scheduler-schema
- os.kill / signal docs: https://docs.python.org/3/library/os.html#os.kill
- psutil is intentionally NOT used to avoid an extra dependency; PID polling
  is done via os.kill(pid, 0) on POSIX and OpenProcess on Windows through
  subprocess.
"""

import logging
import os
import platform
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
from pathlib import Path
from typing import Optional

log = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────

TASK_NAME = "DronePiGroundStation"

# ── Path helpers ──────────────────────────────────────────────────────────────

def _default_state_dir() -> Path:
    return Path.home() / ".dronepi-ground"


def pid_file_path() -> Path:
    return Path(os.environ.get(
        "DRONEPI_PID_FILE",
        _default_state_dir() / "daemon.pid",
    ))


def log_file_path() -> Path:
    return Path(os.environ.get(
        "DRONEPI_LOG_FILE",
        _default_state_dir() / "daemon.log",
    ))


# ══════════════════════════════════════════════════════════════════════════════
# ProcessManager
# ══════════════════════════════════════════════════════════════════════════════

class ProcessManager:
    """
    Manages the lifetime of the FlightReportDaemon background process.

    All public methods are safe to call when the daemon is not running.
    All failures are caught internally and reported via logging; callers
    receive a bool or Optional[int] indicating success/failure.
    """

    # ── Spawn ─────────────────────────────────────────────────────────────────

    def spawn_daemon(self) -> Optional[int]:
        """
        Launch 'ground-station _daemon-worker' as a fully detached process.

        The worker subcommand is an internal Typer command defined in cli.py
        that runs FlightReportDaemon.run_forever() and nothing else.

        Returns the PID of the spawned process, or None on failure.

        Detachment strategy
        -------------------
        Windows: CREATE_NEW_PROCESS_GROUP + DETACHED_PROCESS flags via
                 subprocess.CREATE_NEW_PROCESS_GROUP | 0x00000008.
                 The child has no console and survives terminal close.
        POSIX:   start_new_session=True puts the child in a new session,
                 detaching it from the parent's controlling terminal.

        References
        ----------
        - CREATE_NEW_PROCESS_GROUP:
          https://learn.microsoft.com/en-us/windows/win32/procthread/process-creation-flags
        - subprocess.Popen creationflags:
          https://docs.python.org/3/library/subprocess.html#subprocess.Popen
        """
        pid_path = pid_file_path()
        log_path = log_file_path()

        pid_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        cmd = [sys.executable, "-m", "ground_station.cli", "_daemon-worker"]

        try:
            log_fh = open(log_path, "a")  # noqa: SIM115 — kept open for child

            if platform.system() == "Windows":
                CREATE_NO_WINDOW = 0x08000000
                proc = subprocess.Popen(
                    cmd,
                    stdout=log_fh,
                    stderr=log_fh,
                    creationflags=CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP,
                    close_fds=True,
                )
            else:
                proc = subprocess.Popen(
                    cmd,
                    stdout=log_fh,
                    stderr=log_fh,
                    start_new_session=True,
                    close_fds=True,
                )

            self._write_pid(proc.pid, pid_path)
            log.info(f"[PM] Daemon spawned (pid={proc.pid}), log={log_path}")
            return proc.pid

        except Exception as exc:
            log.error(f"[PM] Failed to spawn daemon: {exc}")
            return None

    # ── Status ────────────────────────────────────────────────────────────────

    def is_running(self) -> bool:
        """
        Return True if a daemon process with the recorded PID is alive.

        Detection method
        ----------------
        POSIX: os.kill(pid, 0) raises ProcessLookupError if the PID does not
               exist, OSError(EPERM) if it exists but we lack permission.
               Either way a process is present on EPERM; absent on ESRCH.
        Windows: subprocess.run(['tasklist', '/FI', 'PID eq <pid>']) and
                 check output contains the PID. os.kill is not reliable on
                 Windows for existence checks (it may raise on any error).

        References: https://docs.python.org/3/library/os.html#os.kill
        """
        pid = self._read_pid()
        if pid is None:
            return False
        return self._pid_alive(pid)

    def daemon_pid(self) -> Optional[int]:
        """Return the recorded daemon PID, or None if not running."""
        pid = self._read_pid()
        if pid is not None and self._pid_alive(pid):
            return pid
        return None

    # ── Stop ─────────────────────────────────────────────────────────────────

    def stop_daemon(self, grace_period_s: float = 5.0) -> bool:
        """
        Terminate the daemon process.

        Sequence
        --------
        1. Read PID from file.
        2. Send SIGTERM (POSIX) or CTRL_BREAK_EVENT (Windows).
        3. Poll for up to grace_period_s for clean exit.
        4. If still alive, send SIGKILL (POSIX) or TerminateProcess (Windows).
        5. Remove PID file.

        Returns True if the process is no longer running after this call.

        References
        ----------
        - CTRL_BREAK_EVENT: https://docs.python.org/3/library/signal.html
        - os.kill signals on Windows:
          https://docs.python.org/3/library/os.html#os.kill
        """
        pid = self._read_pid()
        if pid is None:
            log.info("[PM] No PID file found; daemon not running.")
            return True

        if not self._pid_alive(pid):
            log.info(f"[PM] PID {pid} already dead; cleaning up.")
            self._remove_pid()
            return True

        # ── Graceful terminate ─────────────────────────────────────────────
        try:
            if platform.system() == "Windows":
                os.kill(pid, signal.CTRL_BREAK_EVENT)
            else:
                os.kill(pid, signal.SIGTERM)
            log.info(f"[PM] Sent termination signal to pid={pid}")
        except Exception as exc:
            log.warning(f"[PM] Could not send signal to pid={pid}: {exc}")

        # ── Poll for clean exit ────────────────────────────────────────────
        deadline = time.monotonic() + grace_period_s
        while time.monotonic() < deadline:
            if not self._pid_alive(pid):
                log.info(f"[PM] pid={pid} exited cleanly.")
                self._remove_pid()
                return True
            time.sleep(0.25)

        # ── Force kill ─────────────────────────────────────────────────────
        log.warning(f"[PM] pid={pid} did not exit within {grace_period_s}s — force killing.")
        try:
            if platform.system() == "Windows":
                subprocess.run(
                    ["taskkill", "/F", "/PID", str(pid)],
                    capture_output=True,
                    timeout=5,
                )
            else:
                os.kill(pid, signal.SIGKILL)
        except Exception as exc:
            log.error(f"[PM] Force kill failed: {exc}")

        self._remove_pid()
        return not self._pid_alive(pid)

    # ── Windows Task Scheduler ────────────────────────────────────────────────

    def register_autostart(self) -> bool:
        """
        Register a Task Scheduler task (Windows) for cross-reboot persistence.

        The task runs 'ground-station _daemon-worker' at user logon with no
        terminal window. It is created under the current user's account and
        requires no elevated privileges.

        Implementation: writes an XML task definition to a temp file and
        invokes 'schtasks.exe /Create /XML <file> /TN <name>'.

        On non-Windows platforms this method logs a no-op and returns True.

        References
        ----------
        - schtasks /Create: https://learn.microsoft.com/en-us/windows-server/
          administration/windows-commands/schtasks-create
        - Task Scheduler XML schema:
          https://learn.microsoft.com/en-us/windows/win32/taskschd/
          task-scheduler-schema
        """
        if platform.system() != "Windows":
            log.info("[PM] register_autostart: no-op on non-Windows platform.")
            return True

        exe = sys.executable
        # Use pythonw.exe (no console) if available, fall back to python.exe
        pythonw = Path(exe).parent / "pythonw.exe"
        if not pythonw.exists():
            pythonw = Path(exe)
        exe = str(pythonw)

        # Point to the .pyw launcher — same as double-clicking the desktop icon
        from ground_station.cli import _app_dir
        launcher = str(_app_dir() / "DronePi Ground Station.pyw")
        worker_arg = f'"{launcher}"'
        working_dir = str(_app_dir())

        # Task Scheduler XML — LogonTrigger fires when current user logs on.
        # ExecutionTimeLimit PT0S = no time limit.
        # MultipleInstancesPolicy IgnoreNew = second click is a no-op.
        xml = textwrap.dedent(f"""\
            <?xml version="1.0" encoding="UTF-16"?>
            <Task version="1.2"
                  xmlns="http://schemas.microsoft.com/windows/2004/02/mit/task">
              <RegistrationInfo>
                <Description>DronePi ground station report daemon</Description>
              </RegistrationInfo>
              <Triggers>
                <LogonTrigger>
                  <Enabled>true</Enabled>
                  <UserId>{os.environ.get('USERNAME', os.environ.get('USER', ''))}</UserId>
                </LogonTrigger>
              </Triggers>
              <Principals>
                <Principal id="Author">
                  <LogonType>InteractiveToken</LogonType>
                  <RunLevel>LeastPrivilege</RunLevel>
                </Principal>
              </Principals>
              <Settings>
                <MultipleInstancesPolicy>IgnoreNew</MultipleInstancesPolicy>
                <DisallowStartIfOnBatteries>false</DisallowStartIfOnBatteries>
                <StopIfGoingOnBatteries>false</StopIfGoingOnBatteries>
                <ExecutionTimeLimit>PT0S</ExecutionTimeLimit>
                <Priority>7</Priority>
              </Settings>
              <Actions>
                <Exec>
                  <Command>"{exe}"</Command>
                  <Arguments>{worker_arg}</Arguments>
                  <WorkingDirectory>{working_dir}</WorkingDirectory>
                </Exec>
              </Actions>
            </Task>
        """)

        try:
            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".xml", encoding="utf-16", delete=False
            ) as f:
                f.write(xml)
                tmp_path = f.name

            result = subprocess.run(
                [
                    "schtasks", "/Create",
                    "/XML", tmp_path,
                    "/TN", TASK_NAME,
                    "/F",  # overwrite if already exists
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )
            os.unlink(tmp_path)

            if result.returncode == 0:
                log.info(f"[PM] Task Scheduler task '{TASK_NAME}' registered.")
                return True
            else:
                log.error(
                    f"[PM] schtasks /Create failed (rc={result.returncode}): "
                    f"{result.stderr.strip()}"
                )
                return False

        except Exception as exc:
            log.error(f"[PM] register_autostart failed: {exc}")
            return False

    def unregister_autostart(self) -> bool:
        """Remove the Task Scheduler task. No-op on non-Windows."""
        if platform.system() != "Windows":
            log.info("[PM] unregister_autostart: no-op on non-Windows platform.")
            return True
        try:
            result = subprocess.run(
                ["schtasks", "/Delete", "/TN", TASK_NAME, "/F"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode == 0:
                log.info(f"[PM] Task '{TASK_NAME}' removed.")
                return True
            log.warning(f"[PM] schtasks /Delete: {result.stderr.strip()}")
            return False
        except Exception as exc:
            log.error(f"[PM] unregister_autostart failed: {exc}")
            return False

    # ── Internal helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _write_pid(pid: int, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(str(pid), encoding="utf-8")

    @staticmethod
    def _read_pid() -> Optional[int]:
        path = pid_file_path()
        try:
            return int(path.read_text(encoding="utf-8").strip())
        except (FileNotFoundError, ValueError):
            return None

    @staticmethod
    def _remove_pid() -> None:
        try:
            pid_file_path().unlink(missing_ok=True)
        except Exception:
            pass

    @staticmethod
    def _pid_alive(pid: int) -> bool:
        """
        Non-blocking check: return True if a process with this PID exists.
        Never raises; returns False on any unexpected error.
        """
        try:
            if platform.system() == "Windows":
                result = subprocess.run(
                    ["tasklist", "/FI", f"PID eq {pid}", "/NH"],
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                return str(pid) in result.stdout
            else:
                os.kill(pid, 0)
                return True
        except ProcessLookupError:
            return False
        except PermissionError:
            # Process exists but we can't signal it (different user).
            # Treat as alive to avoid phantom restarts.
            return True
        except Exception:
            return False
