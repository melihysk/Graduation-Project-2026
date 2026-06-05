"""Position Gazebo, RViz and Dashboard windows."""

import subprocess
import shutil
import re
import time
from typing import TYPE_CHECKING, Callable

from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import Qt, QTimer

if TYPE_CHECKING:
    from PyQt6.QtWidgets import QMainWindow


DASHBOARD_TITLE_HINTS = [
    "Trafik Yönetimi Paneli",
    "Trafik Yönetimi",
    "AGV Trafik Yönetimi Paneli",
]
# Order matters: screenshot shows "Gazebo Sim"
GZ_TITLE_HINTS = ["Gazebo Sim", "Gazebo", "gz sim", "Harmonic", "gz-sim"]
RVIZ_TITLE_HINTS = ["RViz2", "RViz", "rviz2"]


def _has_wmctrl() -> bool:
    return shutil.which("wmctrl") is not None


def _run(cmd: list[str], timeout: float = 5) -> str:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except Exception:
        return ""


def _screen_layout() -> tuple[int, int, int, int, int, int, int, int]:
    screen = QApplication.primaryScreen()
    if not screen:
        return 0, 0, 1920, 1080, 768, 1152, 540, 540

    geom = screen.availableGeometry()
    ox, oy = geom.x(), geom.y()
    sw, sh = geom.width(), geom.height()

    dw = max(520, int(sw * 0.38))
    if dw >= sw - 400:
        dw = int(sw * 0.35)
    rw = sw - dw
    rh_top = sh // 2
    rh_bot = sh - rh_top
    return ox, oy, sw, sh, dw, rw, rh_top, rh_bot


def _position_dashboard_qt(window: QWidget | None, ox: int, oy: int, dw: int, sh: int) -> bool:
    if window is None:
        return False

    state = window.windowState()
    state &= ~Qt.WindowState.WindowMaximized
    state &= ~Qt.WindowState.WindowFullScreen
    window.setWindowState(state)
    window.showNormal()

    old_min = window.minimumSize()
    window.setMinimumSize(min(480, dw), min(400, sh))
    window.move(ox, oy)
    window.resize(dw, sh)
    window.setGeometry(ox, oy, dw, sh)
    window.setMinimumSize(old_min)

    window.raise_()
    window.activateWindow()
    return True


# --------------- GNOME Shell D-Bus ---------------

def _gnome_eval(js_code: str) -> tuple[bool, str]:
    out = _run([
        "gdbus", "call", "--session",
        "--dest", "org.gnome.Shell",
        "--object-path", "/org/gnome/Shell",
        "--method", "org.gnome.Shell.Eval",
        js_code,
    ])
    if not out:
        return False, ""
    m = re.match(r"\(true,\s*'(.*)'\)", out, re.DOTALL)
    if m:
        return True, m.group(1)
    return False, out


def _gnome_list_windows() -> list[str]:
    ok, result = _gnome_eval(
        'global.get_window_actors().map(a => a.meta_window.get_title()).join("\\n")'
    )
    if ok and result:
        return [t for t in result.split("\\n") if t]
    return []


def _gnome_move_window(title_fragment: str, x: int, y: int, w: int, h: int) -> bool:
    frag = title_fragment.replace("\\", "\\\\").replace("'", "\\'")
    js = f"""
    (function() {{
        let actors = global.get_window_actors();
        for (let a of actors) {{
            let mw = a.meta_window;
            if (!mw || mw.minimized) continue;
            let t = mw.get_title() || '';
            if (t.toLowerCase().includes('{frag.lower()}')) {{
                mw.unmaximize(3);
                mw.move_resize(true, {x}, {y}, {w}, {h});
                return 'moved:' + t;
            }}
        }}
        return 'not_found';
    }})()
    """.strip()
    ok, result = _gnome_eval(js)
    return ok and result.startswith("moved:")


def _has_gnome_shell() -> bool:
    ok, _ = _gnome_eval('"ok"')
    return ok


# --------------- wmctrl backend ---------------

def _wmctrl_list() -> list[str]:
    out = _run(["wmctrl", "-l"])
    return out.splitlines() if out else []


def _wmctrl_find(title_fragment: str) -> str | None:
    frag = title_fragment.lower()
    for line in _wmctrl_list():
        low = line.lower()
        if frag in low:
            parts = line.split()
            if parts:
                return parts[0]
    return None


def _wmctrl_move(win_id: str, x: int, y: int, w: int, h: int):
    _run(["wmctrl", "-i", "-r", win_id, "-b", "remove,maximized_vert,maximized_horz"])
    _run(["wmctrl", "-i", "-r", win_id, "-e", f"0,{x},{y},{w},{h}"])


def _move_external(title_hints: list[str], x: int, y: int, w: int, h: int) -> bool:
    moved = False
    if _has_gnome_shell():
        for hint in title_hints:
            if _gnome_move_window(hint, x, y, w, h):
                moved = True
                break
    if _has_wmctrl():
        for hint in title_hints:
            wid = _wmctrl_find(hint)
            if wid:
                _wmctrl_move(wid, x, y, w, h)
                moved = True
                break
    return moved


def list_windows() -> list[str]:
    titles = []
    if _has_gnome_shell():
        titles.extend(_gnome_list_windows())
    for line in _wmctrl_list():
        parts = line.split(None, 3)
        if len(parts) == 4 and parts[3] not in titles:
            titles.append(parts[3])
    return titles


def arrange_windows(
    dashboard_window: "QMainWindow | QWidget | None" = None,
    retries: int = 3,
) -> dict:
    """Arrange dashboard (left), Gazebo (top-right), RViz (bottom-right)."""
    result = {
        "dashboard": False,
        "gazebo": False,
        "rviz": False,
        "backend": "none",
    }

    ox, oy, _sw, sh, dw, rw, rh_top, rh_bot = _screen_layout()
    backends = []
    if _has_gnome_shell():
        backends.append("gnome-shell")
    if _has_wmctrl():
        backends.append("wmctrl")
    result["backend"] = "+".join(backends) if backends else "none"

    gz_x, gz_y = ox + dw, oy
    rv_x, rv_y = ox + dw, oy + rh_top

    for attempt in range(retries):
        if attempt > 0:
            time.sleep(0.35)

        result["dashboard"] = _position_dashboard_qt(dashboard_window, ox, oy, dw, sh)
        if not result["dashboard"]:
            for hint in DASHBOARD_TITLE_HINTS:
                if _move_external([hint], ox, oy, dw, sh):
                    result["dashboard"] = True
                    break

        if _move_external(GZ_TITLE_HINTS, gz_x, gz_y, rw, rh_top):
            result["gazebo"] = True
        if _move_external(RVIZ_TITLE_HINTS, rv_x, rv_y, rw, rh_bot):
            result["rviz"] = True

        if result["dashboard"] and result["gazebo"] and result["rviz"]:
            break

    if result["backend"] == "none":
        result["error"] = "Ne GNOME Shell ne de wmctrl kullanilabilir"

    result["visible_windows"] = list_windows()
    result["layout"] = {
        "origin": (ox, oy),
        "dashboard": (ox, oy, dw, sh),
        "gazebo": (gz_x, gz_y, rw, rh_top),
        "rviz": (rv_x, rv_y, rw, rh_bot),
    }
    return result


def arrange_windows_async(
    dashboard_window: "QMainWindow | QWidget | None",
    on_done: Callable[[dict], None] | None = None,
    delay_ms: int = 0,
):
    """Non-blocking arrange (retries in QTimer) for use from GUI thread."""

    def _run_once(remaining: int):
        res = arrange_windows(dashboard_window, retries=1)
        if remaining > 1:
            QTimer.singleShot(400, lambda: _run_once(remaining - 1))
        elif on_done:
            on_done(res)

    if delay_ms > 0:
        QTimer.singleShot(delay_ms, lambda: _run_once(3))
    else:
        _run_once(3)
