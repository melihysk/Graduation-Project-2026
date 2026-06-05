"""Comparison tab: overview grid, bar chart, and radar chart."""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QComboBox, QScrollArea, QFrame,
)
from PyQt6.QtCore import Qt

from data.results_loader import (
    ResultsLoader, MODES, SCENARIOS, MODE_LABELS, SCENARIO_LABELS,
    SUMMARY_METRICS, RADAR_METRICS,
)
from widgets.chart_widgets import GroupedBarChart, RadarChart
from widgets.metric_card import MetricCard


def _short_label(label: str) -> str:
    return label.split(" (")[0]


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

        title = QLabel("Karşılaştırma")
        title.setProperty("class", "sectionTitle")
        self._main_layout.addWidget(title)

        grid_title = QLabel("Tüm senaryolar — verim özeti")
        grid_title.setProperty("class", "sectionTitle")
        self._main_layout.addWidget(grid_title)

        grid_help = QLabel(
            "DKR, IDKR ve Open-RMF algoritmalarının Normal, Dar Koridor ve Yoğun Trafik "
            "senaryolarındaki karşılaştırması. Her hücre, o algoritma ve senaryo için "
            "kayıtlı tüm simülasyon koşularının ortalama verimini gösterir."
        )
        grid_help.setProperty("class", "sectionSubtitle")
        grid_help.setWordWrap(True)
        self._main_layout.addWidget(grid_help)

        self._grid = QGridLayout()
        self._grid.setSpacing(8)
        self._grid.setColumnStretch(0, 0)
        for j in range(len(MODES)):
            self._grid.setColumnStretch(j + 1, 1)

        for j, mode in enumerate(MODES):
            header = QLabel(MODE_LABELS[mode])
            header.setProperty("class", "cardTitle")
            header.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self._grid.addWidget(header, 0, j + 1)

        for i, scenario in enumerate(SCENARIOS):
            header = QLabel(SCENARIO_LABELS[scenario])
            header.setProperty("class", "cardTitle")
            header.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self._grid.addWidget(header, i + 1, 0)

        self._grid_cards: dict[tuple[str, str], MetricCard] = {}
        for i, scenario in enumerate(SCENARIOS):
            for j, mode in enumerate(MODES):
                card = MetricCard(title="Verim", value="--", label="—")
                card.setMinimumSize(140, 88)
                self._grid_cards[(mode, scenario)] = card
                self._grid.addWidget(card, i + 1, j + 1)

        grid_row = QHBoxLayout()
        grid_row.addStretch(1)
        grid_widget = QWidget()
        grid_widget.setLayout(self._grid)
        grid_row.addWidget(grid_widget)
        grid_row.addStretch(1)
        self._main_layout.addLayout(grid_row)

        divider = QFrame()
        divider.setProperty("class", "separator")
        divider.setFrameShape(QFrame.Shape.HLine)
        divider.setFixedHeight(1)
        self._main_layout.addWidget(divider)

        charts_title = QLabel("Seçili senaryo ve metrik")
        charts_title.setProperty("class", "sectionTitle")
        self._main_layout.addWidget(charts_title)

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

        filter_row.addStretch()
        self._main_layout.addLayout(filter_row)

        self._bar_chart = GroupedBarChart(figsize=(5.5, 3.4))
        self._radar_chart = RadarChart(figsize=(4.5, 3.4))

        charts_row = QHBoxLayout()
        charts_row.setSpacing(16)
        charts_row.addWidget(self._bar_chart, stretch=3)
        charts_row.addWidget(self._radar_chart, stretch=2)
        self._main_layout.addLayout(charts_row)

        self._main_layout.addStretch()

        scroll.setWidget(container)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    def refresh(self):
        self._loader.reload()
        self._refresh_grid()
        self._on_filter_change()

    def _refresh_grid(self):
        for scenario in SCENARIOS:
            throughputs = {}
            for mode in MODES:
                card = self._grid_cards[(mode, scenario)]
                card.set_card_style("neutral")
                avgs, count = self._loader.get_averages(mode, scenario)
                if count:
                    avg_tp = avgs.get("throughput_per_min", 0)
                    throughputs[mode] = avg_tp
                    card.set_value(f"{avg_tp:.2f}")
                    card.set_label(
                        f"{count} koşu · {avgs.get('avg_completion_time_sec', 0):.0f} sn · "
                        f"{avgs.get('deadlock_count', 0):.0f} kilitlenme"
                    )
                else:
                    card.set_value("--")
                    card.set_label("Veri yok")
                    throughputs[mode] = None

            valid = {m: v for m, v in throughputs.items() if v is not None}
            if len(valid) < 2:
                continue

            best_mode = max(valid, key=valid.get)
            worst_mode = min(valid, key=valid.get)
            self._grid_cards[(best_mode, scenario)].set_card_style("best")
            self._grid_cards[(best_mode, scenario)].set_value(
                f"{valid[best_mode]:.2f}", "good"
            )
            if best_mode != worst_mode:
                self._grid_cards[(worst_mode, scenario)].set_card_style("worst")
                self._grid_cards[(worst_mode, scenario)].set_value(
                    f"{valid[worst_mode]:.2f}", "bad"
                )

    def _on_filter_change(self):
        scenario_filter = self._scenario_combo.currentData()
        metric_key = self._metric_combo.currentData()
        if not metric_key:
            return

        metric_info = next(
            ((key, label, hib) for key, label, hib in SUMMARY_METRICS if key == metric_key),
            None,
        )
        if not metric_info:
            return

        _, metric_label, higher_is_better = metric_info
        scenarios = SCENARIOS if scenario_filter == "all" else [scenario_filter]
        sc_title = (
            "Tüm Senaryolar"
            if scenario_filter == "all"
            else SCENARIO_LABELS[scenario_filter]
        )
        bar_data = {}
        for mode in MODES:
            bar_data[mode] = {}
            for sc in scenarios:
                avgs, count = self._loader.get_averages(mode, sc)
                bar_data[mode][sc] = avgs.get(metric_key, 0) if count else 0

        self._bar_chart.plot(
            bar_data,
            title=metric_label,
            ylabel=metric_label,
            higher_is_better=higher_is_better,
        )

        metric_lookup = {key: (label, hib) for key, label, hib in SUMMARY_METRICS}
        radar_labels = []
        radar_data = {mode: [] for mode in MODES}
        for mk in RADAR_METRICS:
            ml, hib = metric_lookup[mk]
            vals = {
                mode: self._loader.get_metric_average(mode, scenarios, mk)
                for mode in MODES
            }
            max_val = max(vals.values()) if vals else 0
            if max_val == 0:
                continue

            radar_labels.append(_short_label(ml))
            for mode in MODES:
                v = vals[mode] / max_val
                if not hib:
                    v = 1.0 - v
                radar_data[mode].append(max(0.05, min(1.0, v)))

        radar_data = {mode: values for mode, values in radar_data.items() if values}
        self._radar_chart.plot(
            radar_data,
            radar_labels,
            title=f"Genel Profil — {sc_title}",
        )
