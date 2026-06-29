"""Load scenario YAML files."""

from pathlib import Path
from dataclasses import dataclass

import yaml

from utils.paths import get_workspace_root


SCENARIOS_DIR = (
    get_workspace_root() / "src" / "metric_logger" / "config" / "scenarios"
)


@dataclass
class TaskDef:
    description: str
    pickup_place: str
    dropoff_place: str


@dataclass
class ScenarioDef:
    scenario_id: str
    description: str
    expected_duration_sec: int
    tasks: list[TaskDef]
    file_path: Path


def load_scenario(scenario_id: str) -> ScenarioDef | None:
    path = SCENARIOS_DIR / f"scenario_{scenario_id}.yaml"
    if not path.exists():
        return None

    data = yaml.safe_load(path.read_text())
    sc = data.get("scenario", {})
    raw_tasks = data.get("tasks", [])

    tasks = []
    for t in raw_tasks:
        tasks.append(TaskDef(
            description=t.get("description", ""),
            pickup_place=t.get("pickup", {}).get("place", ""),
            dropoff_place=t.get("dropoff", {}).get("place", ""),
        ))

    return ScenarioDef(
        scenario_id=sc.get("id", scenario_id),
        description=sc.get("description", ""),
        expected_duration_sec=sc.get("expected_duration_sec", 300),
        tasks=tasks,
        file_path=path,
    )


def load_all_scenarios() -> dict[str, ScenarioDef]:
    result = {}
    for sid in ["normal", "bottleneck", "high_density"]:
        s = load_scenario(sid)
        if s:
            result[sid] = s
    return result
