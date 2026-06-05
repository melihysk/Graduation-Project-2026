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
    "bottleneck": "Dar Koridor",
    "high_density": "Yoğun Trafik",
}

SUMMARY_METRICS = [
    ("throughput_per_min", "Verim (görev/dk)", True),
    ("avg_completion_time_sec", "Ort. Tamamlanma (sn)", False),
    ("deadlock_count", "Kilitlenme", False),
    ("conflict_count", "Trafik Çatışması", False),
    ("unresolved_conflicts", "Çözülmemiş Çatışma", False),
    ("avg_resolution_time_sec", "Ort. Çözüm Süresi (sn)", False),
    ("avg_wait_time_sec", "Ort. Bekleme (sn)", False),
    ("wait_time_variance", "Bekleme Varyansı", False),
    ("total_energy_wh", "Enerji (Wh)", False),
]

RADAR_METRICS = [
    "throughput_per_min",
    "avg_completion_time_sec",
    "deadlock_count",
    "conflict_count",
    "avg_resolution_time_sec",
    "total_energy_wh",
]

_METRIC_SOURCES = {
    "avg_resolution_time_sec": ("conflict_metrics", "avg_resolution_time_sec"),
    "unresolved_conflicts": ("conflict_metrics", "unresolved_conflicts"),
    "avg_wait_time_sec": ("robot_metrics", "avg_wait_time_sec"),
}


def metric_value(run: "RunResult", key: str) -> float:
    if key in run.summary:
        return float(run.summary.get(key, 0))
    section, field_name = _METRIC_SOURCES.get(key, (None, None))
    if section:
        return float(getattr(run, section).get(field_name, 0))
    return 0.0


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

    def get_averages(self, mode: str, scenario: str) -> tuple[dict[str, float], int]:
        runs = self.list_runs(mode, scenario)
        if not runs:
            return {}, 0
        totals = {key: 0.0 for key, _, _ in SUMMARY_METRICS}
        for run in runs:
            for key, _, _ in SUMMARY_METRICS:
                totals[key] += metric_value(run, key)
        count = len(runs)
        return {key: totals[key] / count for key in totals}, count

    def get_metric_average(
        self, mode: str, scenarios: list[str], metric_key: str
    ) -> float:
        values = []
        for scenario in scenarios:
            avgs, count = self.get_averages(mode, scenario)
            if count > 0:
                values.append(avgs.get(metric_key, 0))
        return sum(values) / len(values) if values else 0.0

    def all_results(self) -> list[RunResult]:
        return list(self._cache.values())

    def has_data(self) -> bool:
        return len(self._cache) > 0

    def available_run_ids(self, mode: str, scenario: str) -> list[int]:
        return [r.run_id for r in self.list_runs(mode, scenario)]

    def next_run_id(self, mode: str, scenario: str) -> int:
        ids = self.available_run_ids(mode, scenario)
        return max(ids) + 1 if ids else 1
