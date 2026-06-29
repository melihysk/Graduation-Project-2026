"""Workspace root discovery — independent of folder name or location."""

from pathlib import Path

_workspace_root: Path | None = None


def _is_workspace_root(path: Path) -> bool:
    return (
        (path / "src" / "metric_logger").is_dir()
        and (path / "dashboard").is_dir()
    ) or (
        (path / "install" / "setup.bash").is_file()
        and (path / "src").is_dir()
    )


def find_workspace_root(start: Path | None = None) -> Path:
    """Walk up from *start* until a ROS workspace layout is found."""
    if start is None:
        start = Path(__file__).resolve()
    elif start.is_file():
        start = start.parent

    for candidate in (start, *start.parents):
        if _is_workspace_root(candidate):
            return candidate

    raise RuntimeError(
        "Workspace root not found. Expected a directory containing "
        "src/metric_logger and dashboard/ (or install/setup.bash and src/)."
    )


def get_workspace_root() -> Path:
    global _workspace_root
    if _workspace_root is None:
        _workspace_root = find_workspace_root()
    return _workspace_root
