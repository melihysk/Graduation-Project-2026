"""Comparison tab: grouped bar charts, radar chart, per-robot breakdown, heatmap."""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QScrollArea, QFrame, QSplitter,
)
from PyQt6.QtCore import Qt

from data.results_loader import (
    ResultsLoader, MODES, SCENARIOS, MODE_LABELS, SCENARIO_LABELS, SUMMARY_METRICS,
)
from widgets.chart_widgets import GroupedBarChart, RadarChart, StackedBarChart, HeatmapTable


class ComparisonTab(QWidget):
    def __init__(self, loader: ResultsLoader, parent=None):
        super().__init__(parent)
        self._loader = loader

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        self._main_layout = QVBoxLayout(container)
        self._main_layout.setContentsMargins(24, 20, 24, 20)
        self._main_layout.setSpacing(16)

        title = QLabel("Algoritma Karşılaştırması")
        title.setProperty("class", "sectionTitle")
        self._main_layout.addWidget(title)

        subtitle = QLabel("DKR · IDKR · Open-RMF — tüm metrikler")
        subtitle.setProperty("class", "sectionSubtitle")
        self._main_layout.addWidget(subtitle)

        # Filters row
        filter_row = QHBoxLayout()
        filter_row.setSpacing(12)

        filter_row.addWidget(QLabel("Senaryo:"))
        self._scenario_combo = QComboBox()
        self._scenario_combo.addItem("Tümü", "all")
        for s in SCENARIOS:
            self._scenario_combo.addItem(SCENARIO_LABELS[s], s)
        self._scenario_combo.currentIndexChanged.connect(self._on_filter_change)
        filter_row.addWidget(self._scenario_combo)

        filter_row.addWidget(QLabel("Metrik:"))
        self._metric_combo = QComboBox()
        for key, label, _ in SUMMARY_METRICS:
            self._metric_combo.addItem(label, key)
        self._metric_combo.currentIndexChanged.connect(self._on_filter_change)
        filter_row.addWidget(self._metric_combo)

        filter_row.addWidget(QLabel("Robot detayı:"))
        self._robot_mode_combo = QComboBox()
        for mode in MODES:
            self._robot_mode_combo.addItem(MODE_LABELS[mode], mode)
        self._robot_mode_combo.currentIndexChanged.connect(self._on_filter_change)
        filter_row.addWidget(self._robot_mode_combo)

        filter_row.addStretch()
        self._main_layout.addLayout(filter_row)

        # Charts
        self._bar_chart = GroupedBarChart(figsize=(9, 3.5))
        self._main_layout.addWidget(self._bar_chart)

        charts_row = QHBoxLayout()
        charts_row.setSpacing(12)

        self._radar_chart = RadarChart(figsize=(5, 4.5))
        charts_row.addWidget(self._radar_chart)

        self._stacked_chart = StackedBarChart(figsize=(6, 4))
        charts_row.addWidget(self._stacked_chart)

        self._main_layout.addLayout(charts_row)

        # Heatmap
        self._heatmap = HeatmapTable(figsize=(8, 3))
        self._main_layout.addWidget(self._heatmap)

        self._main_layout.addStretch()

        scroll.setWidget(container)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    def refresh(self):
        self._loader.reload()
        self._on_filter_change()

    def _on_filter_change(self):
        scenario_filter = self._scenario_combo.currentData()
        metric_key = self._metric_combo.currentData()

        if not metric_key:
            return

        metric_info = None
        for key, label, hib in SUMMARY_METRICS:
            if key == metric_key:
                metric_info = (key, label, hib)
                break
        if not metric_info:
            return

        _, metric_label, higher_is_better = metric_info
        scenarios = SCENARIOS if scenario_filter == "all" else [scenario_filter]

        # Grouped bar chart
        bar_data = {}
        for mode in MODES:
            bar_data[mode] = {}
            for sc in scenarios:
                r = self._loader.get_latest(mode, sc)
                val = r.summary.get(metric_key, 0) if r else 0
                bar_data[mode][sc] = val

        self._bar_chart.plot(bar_data, title=metric_label, ylabel=metric_label,
                            higher_is_better=higher_is_better)

        # Radar chart (use first scenario or average across all)
        target_scenario = scenarios[0] if len(scenarios) == 1 else scenarios[0]
        radar_metrics = []
        radar_labels = []
        radar_data = {}

        normalize_metrics = [
            ("throughput_per_min", "Verim", True),
            ("avg_completion_time_sec", "Tamamlanma", False),
            ("deadlock_count", "Kilitlenme", False),
            ("conflict_count", "Çakışma", False),
            ("near_miss_count", "Yakın Kaçınma", False),
            ("total_energy_wh", "Enerji", False),
        ]

        raw_values = {mode: [] for mode in MODES}
        valid_labels = []
        for mk, ml, hib in normalize_metrics:
            vals = {}
            for mode in MODES:
                r = self._loader.get_latest(mode, target_scenario)
                vals[mode] = r.summary.get(mk, 0) if r else 0

            max_val = max(vals.values()) if vals else 1
            if max_val == 0:
                continue

            valid_labels.append(ml)
            for mode in MODES:
                v = vals[mode] / max_val
                if not hib:
                    v = 1.0 - v
                v = max(0.05, min(1.0, v))
                raw_values[mode].append(v)

        for mode in MODES:
            if raw_values[mode]:
                radar_data[mode] = raw_values[mode]

        sc_label = SCENARIO_LABELS.get(target_scenario, target_scenario)
        self._radar_chart.plot(radar_data, valid_labels,
                              title=f"Çoklu Metrik — {sc_label}")

        # Stacked bar: per-robot time for selected mode in target_scenario
        selected_mode = self._robot_mode_combo.currentData() or MODES[0]
        stacked_data = {}
        r = self._loader.get_latest(selected_mode, target_scenario)
        if r and r.per_robot:
            for rname, rdata in r.per_robot.items():
                stacked_data[rname] = {
                    "moving": rdata.get("moving_time_sec", 0),
                    "waiting": rdata.get("waiting_time_sec", 0),
                    "idle": rdata.get("idle_time_sec", 0),
                    "charging": rdata.get("charging_time_sec", 0),
                }
            self._stacked_chart.plot(stacked_data,
                                    title=f"Robot Zaman Dağılımı — {MODE_LABELS[selected_mode]} / {sc_label}")
        else:
            self._stacked_chart.plot({}, title=f"Robot verisi yok — {MODE_LABELS[selected_mode]} / {sc_label}")

        # Heatmap table
        heatmap_data = {}
        for sc in SCENARIOS:
            heatmap_data[sc] = {}
            for mode in MODES:
                r = self._loader.get_latest(mode, sc)
                heatmap_data[sc][mode] = r.summary.get(metric_key, 0) if r else 0

        self._heatmap.plot(heatmap_data, metric_label, higher_is_better)
