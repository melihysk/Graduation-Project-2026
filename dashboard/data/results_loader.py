"""Load experiment results from results/ JSON files."""

import json
from pathlib import Path
from dataclasses import dataclass, field


RESULTS_ROOT = Path.home() / "Desktop" / "graduation_project" / "results"

MODES = ["rmf", "dkr", "idkr"]
MODE_LABELS = {"rmf": "Open-RMF", "dkr": "DKR", "idkr": "IDKR"}
SCENARIOS = ["normal", "bottleneck", "high_density"]
SCENARIO_LABELS = {
    "normal": "Normal",
    "bottleneck": "Dar Boğaz",
    "high_density": "Yoğun Trafik",
}

SUMMARY_METRICS = [
    ("throughput_per_min", "Verim (görev/dk)", True),
    ("avg_completion_time_sec", "Ort. Tamamlanma (sn)", False),
    ("deadlock_count", "Kilitlenme", False),
    ("conflict_count", "Çakışma", False),
    ("near_miss_count", "Yakın Kaçınma", False),
    ("total_energy_wh", "Enerji (Wh)", False),
    ("wait_time_variance", "Bekleme Varyansı", False),
]


@dataclass
class RunResult:
    mode: str
    scenario: str
    run_id: int
    raw: dict = field(default_factory=dict)

    @property
    def summary(self) -> dict:
        return self.raw.get("summary", {})

    @property
    def task_metrics(self) -> dict:
        return self.raw.get("task_metrics", {})

    @property
    def robot_metrics(self) -> dict:
        return self.raw.get("robot_metrics", {})

    @property
    def conflict_metrics(self) -> dict:
        return self.raw.get("conflict_metrics", {})

    @property
    def energy_metrics(self) -> dict:
        return self.raw.get("energy_metrics", {})

    @property
    def per_robot(self) -> dict:
        return self.robot_metrics.get("per_robot", {})


class ResultsLoader:
    """Scans results/ directory and loads JSON run files."""

    def __init__(self, root: Path | None = None):
        self._root = root or RESULTS_ROOT
        self._cache: dict[tuple[str, str, int], RunResult] = {}
        self.reload()

    def reload(self):
        self._cache.clear()
        if not self._root.exists():
            return
        for mode in MODES:
            for scenario in SCENARIOS:
                d = self._root / mode / scenario
                if not d.is_dir():
                    continue
                for f in sorted(d.glob("run_*.json")):
                    try:
                        run_id = int(f.stem.split("_")[1])
                        data = json.loads(f.read_text())
                        self._cache[(mode, scenario, run_id)] = RunResult(
                            mode=mode, scenario=scenario, run_id=run_id, raw=data
                        )
                    except (ValueError, json.JSONDecodeError, IndexError):
                        continue

    def get(self, mode: str, scenario: str, run_id: int) -> RunResult | None:
        return self._cache.get((mode, scenario, run_id))

    def get_latest(self, mode: str, scenario: str) -> RunResult | None:
        runs = self.list_runs(mode, scenario)
        return runs[-1] if runs else None

    def list_runs(self, mode: str, scenario: str) -> list[RunResult]:
        return sorted(
            [r for k, r in self._cache.items() if k[0] == mode and k[1] == scenario],
            key=lambda r: r.run_id,
        )

    def all_results(self) -> list[RunResult]:
        return list(self._cache.values())

    def has_data(self) -> bool:
        return len(self._cache) > 0

    def available_run_ids(self, mode: str, scenario: str) -> list[int]:
        return [r.run_id for r in self.list_runs(mode, scenario)]

    def next_run_id(self, mode: str, scenario: str) -> int:
        ids = self.available_run_ids(mode, scenario)
        return max(ids) + 1 if ids else 1
