"""Manage ROS 2 launch processes via QProcess with graceful shutdown."""

import os
import signal
import time
from typing import Callable

import psutil

from PyQt6.QtCore import QProcess, QObject, pyqtSignal, QTimer

from utils.paths import get_workspace_root


_WS_ROOT = get_workspace_root()
ROS_SETUP = "source /opt/ros/jazzy/setup.bash"
WS_SETUP = f"source {_WS_ROOT / 'install' / 'setup.bash'}"
SETUP_CMD = f"{ROS_SETUP} && {WS_SETUP}"

# Graceful stop: SIGINT (ros2 launch Ctrl+C) -> SIGTERM -> SIGKILL
_STOP_POLL_MS = 400
_STOP_INT_TICKS = 25   # ~10 s after SIGINT
_STOP_TERM_TICKS = 40  # ~6 s after SIGTERM
_STOP_KILL_TICKS = 55  # ~6 s after SIGKILL attempt


class LaunchProcess(QObject):
    """Wrapper around QProcess for a single ros2 launch command."""

    started = pyqtSignal()
    finished = pyqtSignal(int)
    output_line = pyqtSignal(str)
    stop_completed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._proc = QProcess(self)
        self._proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self._proc.readyReadStandardOutput.connect(self._on_output)
        self._proc.started.connect(self.started)
        self._proc.finished.connect(self._on_finished)

        self._stop_timer: QTimer | None = None
        self._stop_ticks = 0
        self._stopping = False

    def start(self, launch_cmd: str):
        self._stopping = False
        self._stop_ticks = 0
        full_cmd = f"{SETUP_CMD} && {launch_cmd}"
        self._proc.setProcessEnvironment(self._proc.processEnvironment())
        self._proc.start("setsid", ["bash", "-c", full_cmd])

    def stop(self):
        """Request graceful shutdown (non-blocking). Emits stop_completed when done."""
        if self._proc.state() != QProcess.ProcessState.Running:
            self.stop_completed.emit()
            return

        if self._stopping:
            return

        self._stopping = True
        self._stop_ticks = 0
        self._signal_process_group(signal.SIGINT)

        if self._stop_timer is None:
            self._stop_timer = QTimer(self)
            self._stop_timer.timeout.connect(self._on_stop_tick)
        self._stop_timer.start(_STOP_POLL_MS)

    def is_running(self) -> bool:
        return self._proc.state() == QProcess.ProcessState.Running

    def _signal_process_group(self, sig: signal.Signals):
        pid = self._proc.processId()
        if not pid:
            return
        try:
            os.killpg(os.getpgid(pid), sig)
        except (ProcessLookupError, PermissionError):
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass

    def _on_stop_tick(self):
        if self._proc.state() != QProcess.ProcessState.Running:
            self._finish_stop()
            return

        self._stop_ticks += 1

        if self._stop_ticks == _STOP_INT_TICKS:
            self._signal_process_group(signal.SIGTERM)
        elif self._stop_ticks == _STOP_TERM_TICKS:
            self._signal_process_group(signal.SIGKILL)
            self._proc.kill()
        elif self._stop_ticks >= _STOP_KILL_TICKS:
            if self._proc.state() == QProcess.ProcessState.Running:
                self._proc.kill()
            self._proc.waitForFinished(2000)
            self._finish_stop()

    def _finish_stop(self):
        if self._stop_timer and self._stop_timer.isActive():
            self._stop_timer.stop()
        self._stopping = False
        self._stop_ticks = 0
        self.stop_completed.emit()

    def _on_output(self):
        data = self._proc.readAllStandardOutput().data().decode(errors="replace")
        for line in data.splitlines():
            self.output_line.emit(line)

    def _on_finished(self, exit_code, _status):
        self._finish_stop()
        self.finished.emit(exit_code)


# Project-scoped ROS processes to reap after launch trees exit.
_WS_MARKER = str(_WS_ROOT.resolve()).lower()
_ROS_CMD_MARKERS = (
    "ros-args",
    "ros2 launch",
    "ros2 run",
    "/opt/ros/jazzy",
    "fleet_manager",
    "fleet_adapter",
    "metric_logger",
    "dkr_controller",
    "idkr_controller",
    "rmf_traffic",
    "rmf_fleet",
    "gz sim",
    "ruby $(which gz)",
    "rviz2",
    "gazebo",
    "rmf_demos",
    "rmf_demos_gz",
    "rmf_demos_maps",
)


def _cmdline_str(proc: psutil.Process) -> str:
    try:
        parts = proc.cmdline()
        return " ".join(parts) if parts else ""
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return ""


def _is_project_ros_process(proc: psutil.Process, own_pid: int) -> bool:
    if proc.pid == own_pid or proc.pid == os.getpid():
        return False
    cmd = _cmdline_str(proc).lower()
    if _WS_MARKER not in cmd:
        return False
    if "dashboard/main.py" in cmd or "dashboard/.venv" in cmd:
        return False
    return any(m in cmd for m in _ROS_CMD_MARKERS)


def find_stray_ros_processes() -> list[psutil.Process]:
    """Return project-related ROS processes still running."""
    own = os.getpid()
    found: list[psutil.Process] = []
    seen: set[int] = set()
    for proc in psutil.process_iter(["pid", "name"]):
        try:
            if proc.pid in seen:
                continue
            if not _is_project_ros_process(proc, own):
                continue
            seen.add(proc.pid)
            found.append(proc)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return found


def _cleanup_pass(log: Callable[[str], None] | None, sigkill: bool) -> list[int]:
    def _log(msg: str):
        if log:
            log(msg)

    targets = find_stray_ros_processes()
    if not targets:
        return []

    pids = sorted({p.pid for p in targets})
    for proc in targets:
        try:
            if sigkill:
                _log(f"[UI]   SIGKILL pid={proc.pid}")
                proc.kill()
            else:
                _log(f"[UI]   SIGTERM pid={proc.pid} {_cmdline_str(proc)[:72]}")
                proc.terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return pids


def cleanup_stray_ros_processes(
    log: Callable[[str], None] | None = None,
    grace_sec: float = 2.0,
) -> list[int]:
    """SIGTERM then SIGKILL leftover workspace ROS processes (blocking)."""
    def _log(msg: str):
        if log:
            log(msg)

    time.sleep(0.3)
    targets = find_stray_ros_processes()
    if not targets:
        _log("[UI] Temizlik: kalan ROS süreci yok.")
        return []

    pids = sorted({p.pid for p in targets})
    _log(f"[UI] Temizlik: {len(pids)} süreç bulundu (ros-args / launch)...")
    _cleanup_pass(log, sigkill=False)
    time.sleep(grace_sec)
    still = find_stray_ros_processes()
    if still:
        _log(f"[UI] Temizlik: {len(still)} proses hala acik, SIGKILL...")
        _cleanup_pass(log, sigkill=True)
        time.sleep(0.5)

    remaining = [p.pid for p in find_stray_ros_processes()]
    if remaining:
        _log(f"[UI] Temizlik uyarısı: hâlâ çalışan pid'ler: {remaining}")
    else:
        _log("[UI] Temizlik tamamlandı.")
    return pids


class RosCleanupWorker(QObject):
    """Non-blocking cleanup via QTimer (keeps UI responsive)."""

    finished = pyqtSignal()

    def __init__(self, log: Callable[[str], None] | None = None, parent=None):
        super().__init__(parent)
        self._log_fn = log
        self._pids: list[int] = []
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._phase_kill)

    def start(self):
        def _log(msg: str):
            if self._log_fn:
                self._log_fn(msg)

        targets = find_stray_ros_processes()
        if not targets:
            _log("[UI] Temizlik: kalan ROS süreci yok.")
            self.finished.emit()
            return

        self._pids = sorted({p.pid for p in targets})
        _log(f"[UI] Temizlik: {len(self._pids)} süreç bulundu (ros-args / launch)...")
        _cleanup_pass(_log, sigkill=False)
        self._timer.start(2000)

    def _phase_kill(self):
        def _log(msg: str):
            if self._log_fn:
                self._log_fn(msg)

        still = find_stray_ros_processes()
        if still:
            _log(f"[UI] Temizlik: {len(still)} süreç hâlâ açık, SIGKILL...")
            _cleanup_pass(_log, sigkill=True)

        remaining = [p.pid for p in find_stray_ros_processes()]
        if remaining:
            _log(f"[UI] Temizlik uyarısı: hâlâ çalışan pid'ler: {remaining}")
        else:
            _log("[UI] Temizlik tamamlandı.")
        self.finished.emit()
