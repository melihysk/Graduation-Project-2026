"""Main window with top navigation and stacked content area."""

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QLabel, QStackedWidget, QFrame,
)
from PyQt6.QtCore import Qt

from tabs.dashboard_tab import DashboardTab
from tabs.comparison_tab import ComparisonTab
from tabs.experiment_tab import ExperimentTab
from data.results_loader import ResultsLoader


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trafik Yönetimi Paneli")
        self.setMinimumSize(520, 480)
        self.resize(900, 700)

        self._results_loader = ResultsLoader()

        central = QWidget()
        central.setObjectName("centralWidget")
        self.setCentralWidget(central)

        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        root_layout.addWidget(self._build_topbar())

        content = QWidget()
        content.setObjectName("contentArea")
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)

        self._stack = QStackedWidget()
        self._dashboard_tab = DashboardTab(self._results_loader)
        self._comparison_tab = ComparisonTab(self._results_loader)
        self._experiment_tab = ExperimentTab(self._results_loader, main_window=self)

        self._stack.addWidget(self._dashboard_tab)
        self._stack.addWidget(self._comparison_tab)
        self._stack.addWidget(self._experiment_tab)

        content_layout.addWidget(self._stack)
        root_layout.addWidget(content, 1)

        self._nav_buttons[0].setChecked(True)

    def _build_topbar(self) -> QWidget:
        topbar = QWidget()
        topbar.setObjectName("topbar")

        layout = QHBoxLayout(topbar)
        layout.setContentsMargins(20, 10, 20, 10)
        layout.setSpacing(8)

        title = QLabel("Trafik Yönetimi")
        title.setObjectName("appTitle")
        layout.addWidget(title)

        sep = QFrame()
        sep.setProperty("class", "separatorVertical")
        sep.setFrameShape(QFrame.Shape.VLine)
        layout.addWidget(sep)

        nav_items = [
            ("Özet", 0),
            ("Karşılaştırma", 1),
            ("Simülasyon", 2),
        ]

        self._nav_buttons: list[QPushButton] = []
        for label, idx in nav_items:
            btn = QPushButton(label)
            btn.setProperty("class", "navButton")
            btn.setCheckable(True)
            btn.setFixedHeight(36)
            btn.setMinimumWidth(120)
            btn.clicked.connect(lambda checked, i=idx: self._switch_tab(i))
            layout.addWidget(btn)
            self._nav_buttons.append(btn)

        layout.addStretch()

        version_label = QLabel("ROS 2 Jazzy")
        version_label.setProperty("class", "cardLabel")
        layout.addWidget(version_label)

        return topbar

    def _switch_tab(self, index: int):
        self._stack.setCurrentIndex(index)
        for i, btn in enumerate(self._nav_buttons):
            btn.setChecked(i == index)
        if index == 0:
            self._dashboard_tab.refresh()
        elif index == 1:
            self._comparison_tab.refresh()
