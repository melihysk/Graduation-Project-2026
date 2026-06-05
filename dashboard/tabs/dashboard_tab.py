"""Dashboard overview tab: 3x3 summary grid + key metrics."""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QScrollArea, QFrame, QSizePolicy,
)
from PyQt6.QtCore import Qt

from data.results_loader import (
    ResultsLoader, MODES, SCENARIOS, MODE_LABELS, SCENARIO_LABELS, SUMMARY_METRICS,
)
from widgets.metric_card import MetricCard


class DashboardTab(QWidget):
    def __init__(self, loader: ResultsLoader, parent=None):
        super().__init__(parent)
        self._loader = loader

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        self._layout = QVBoxLayout(container)
        self._layout.setContentsMargins(24, 20, 24, 20)
        self._layout.setSpacing(16)

        title = QLabel("Özet Panel")
        title.setProperty("class", "sectionTitle")
        self._layout.addWidget(title)

        subtitle = QLabel("Tüm algoritma ve senaryoların özet karşılaştırması")
        subtitle.setProperty("class", "sectionSubtitle")
        self._layout.addWidget(subtitle)

        # Top-level summary cards
        self._summary_layout = QHBoxLayout()
        self._summary_layout.setSpacing(12)
        self._summary_cards: dict[str, MetricCard] = {}
        summary_items = [
            ("total_runs", "Toplam Koşu"),
            ("best_throughput", "En İyi Verim"),
            ("min_deadlocks", "En Az Kilitlenme"),
            ("total_scenarios", "Senaryo Sayısı"),
        ]
        for key, label in summary_items:
            card = MetricCard(title=label, value="--")
            self._summary_cards[key] = card
            self._summary_layout.addWidget(card)
        self._layout.addLayout(self._summary_layout)

        grid_title = QLabel("Verim Karşılaştırması")
        grid_title.setProperty("class", "sectionTitle")
        self._layout.addWidget(grid_title)

        grid_help = QLabel(
            "Her hücre, seçilen algoritma ve senaryo için kayıtlı son deneyin özetidir. "
            "Büyük sayı: dakikada tamamlanan görev sayısı (verim, görev/dk). "
            "Alt satır: ortalama görev süresi, kilitlenme ve çakışma sayıları. "
            "Yeşil çerçeve o senaryoda en iyi, kırmızı çerçeve en düşük verimi gösterir."
        )
        grid_help.setProperty("class", "sectionSubtitle")
        grid_help.setWordWrap(True)
        self._layout.addWidget(grid_help)

        self._grid = QGridLayout()
        self._grid.setSpacing(10)
        self._grid.setColumnStretch(0, 0)
        for j in range(len(MODES)):
            self._grid.setColumnStretch(j + 1, 1)

        # Header row
        for j, mode in enumerate(MODES):
            header = QLabel(MODE_LABELS[mode])
            header.setProperty("class", "cardTitle")
            header.setAlignment(Qt.AlignmentFlag.AlignCenter)
            header.setSizePolicy(
                QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred
            )
            self._grid.addWidget(header, 0, j + 1)

        # Header column
        for i, scenario in enumerate(SCENARIOS):
            header = QLabel(SCENARIO_LABELS[scenario])
            header.setProperty("class", "cardTitle")
            header.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self._grid.addWidget(header, i + 1, 0)

        self._grid_cards: dict[tuple[str, str], MetricCard] = {}
        for i, scenario in enumerate(SCENARIOS):
            for j, mode in enumerate(MODES):
                card = MetricCard(title="Verim (görev/dk)", value="--", label="—")
                self._grid_cards[(mode, scenario)] = card
                self._grid.addWidget(card, i + 1, j + 1)

        grid_row = QHBoxLayout()
        grid_row.addStretch(1)
        grid_container = QWidget()
        grid_container.setLayout(self._grid)
        grid_row.addWidget(grid_container)
        grid_row.addStretch(1)
        self._layout.addLayout(grid_row)
        self._layout.addStretch()

        scroll.setWidget(container)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self.refresh()

    def refresh(self):
        self._loader.reload()
        results = self._loader.all_results()

        self._summary_cards["total_runs"].set_value(str(len(results)))
        self._summary_cards["total_scenarios"].set_value("3")

        if not results:
            self._summary_cards["best_throughput"].set_value("--")
            self._summary_cards["min_deadlocks"].set_value("--")
            return

        best_tp = max((r.summary.get("throughput_per_min", 0) for r in results), default=0)
        min_dl = min((r.summary.get("deadlock_count", 999) for r in results), default=0)
        self._summary_cards["best_throughput"].set_value(f"{best_tp:.2f}", "good")
        self._summary_cards["best_throughput"].set_label("görev/dk")
        self._summary_cards["min_deadlocks"].set_value(str(min_dl), "good" if min_dl == 0 else "bad")

        # Fill 3x3 grid
        for scenario in SCENARIOS:
            throughputs = {}
            for mode in MODES:
                r = self._loader.get_latest(mode, scenario)
                card = self._grid_cards[(mode, scenario)]
                if r:
                    tp = r.summary.get("throughput_per_min", 0)
                    dl = r.summary.get("deadlock_count", 0)
                    cf = r.summary.get("conflict_count", 0)
                    avg_t = r.summary.get("avg_completion_time_sec", 0)
                    throughputs[mode] = tp

                    card.set_value(f"{tp:.2f}")
                    card.set_label(
                        f"Ort. süre: {avg_t:.1f} sn · Kilitlenme: {dl} · Çakışma: {cf}"
                    )
                else:
                    card.set_value("--")
                    card.set_label("Veri yok")
                    throughputs[mode] = None

            valid = {m: v for m, v in throughputs.items() if v is not None}
            if len(valid) >= 2:
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
                for m in valid:
                    if m not in (best_mode, worst_mode):
                        self._grid_cards[(m, scenario)].set_card_style("neutral")
