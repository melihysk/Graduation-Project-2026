"""Gazebo Harmonic simülasyon saati: duraklat / devam ettir."""

import re
import shutil
import subprocess
from typing import Callable

# warehouse_starter.world içindeki dünya adı
DEFAULT_WORLD_NAMES = ("sim_world", "warehouse_starter", "default")


def _gz_bin() -> str | None:
    return shutil.which("gz")


def _discover_world_names() -> list[str]:
    """Çalışan Gazebo'dan /world/<ad>/control servislerini bul."""
    gz = _gz_bin()
    if not gz:
        return []
    try:
        r = subprocess.run(
            [gz, "service", "-l"],
            capture_output=True,
            text=True,
            timeout=5,
        )
        out = (r.stdout or "") + (r.stderr or "")
    except Exception:
        return []

    names = []
    for m in re.finditer(r"/world/([^/\s]+)/control", out):
        names.append(m.group(1))
    return names


def set_simulation_paused(
    pause: bool,
    log: Callable[[str], None] | None = None,
) -> tuple[bool, str | None]:
    """Gazebo sim saatini duraklat veya devam ettir. (başarı, kullanılan dünya adı)"""
    gz = _gz_bin()
    if not gz:
        if log:
            log("[UI] gz komutu bulunamadı — Gazebo çalışıyor mu?")
        return False, None

    worlds = list(_discover_world_names())
    for w in DEFAULT_WORLD_NAMES:
        if w not in worlds:
            worlds.append(w)

    req = "pause: true" if pause else "pause: false"
    action = "duraklatıldı" if pause else "devam ettirildi"

    for world in worlds:
        cmd = [
            gz,
            "service",
            "-s",
            f"/world/{world}/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            req,
        ]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=6)
            combined = (r.stdout or "") + (r.stderr or "")
            if r.returncode == 0 and ("true" in combined.lower() or not combined.strip()):
                if log:
                    log(f"[UI] Gazebo simülasyon saati {action} (dünya: {world})")
                return True, world
        except subprocess.TimeoutExpired:
            continue
        except Exception:
            continue

    if log:
        log("[UI] Gazebo duraklatma başarısız — simülasyon açık mı?")
    return False, None
