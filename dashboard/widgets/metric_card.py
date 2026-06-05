"""Reusable metric card widget."""

from PyQt6.QtWidgets import QFrame, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt


class MetricCard(QFrame):
    """A single metric display card with title, value, and optional label."""

    def __init__(self, title: str = "", value: str = "", label: str = "", parent=None):
        super().__init__(parent)
        self.setProperty("class", "metricCard")
        self.setMinimumSize(160, 100)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(4)

        self._title = QLabel(title)
        self._title.setProperty("class", "cardTitle")
        layout.addWidget(self._title)

        self._value = QLabel(value)
        self._value.setProperty("class", "cardValue")
        self._value.setAlignment(Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self._value)

        self._label = QLabel(label)
        self._label.setProperty("class", "cardLabel")
        layout.addWidget(self._label)

        layout.addStretch()

    def set_value(self, value: str, quality: str = "neutral"):
        self._value.setText(value)
        cls_map = {"good": "cardValueGood", "bad": "cardValueBad", "neutral": "cardValue"}
        self._value.setProperty("class", cls_map.get(quality, "cardValue"))
        self._value.style().unpolish(self._value)
        self._value.style().polish(self._value)

    def set_label(self, label: str):
        self._label.setText(label)

    def set_title(self, title: str):
        self._title.setText(title)

    def set_card_style(self, quality: str = "neutral"):
        cls_map = {"best": "metricCardBest", "worst": "metricCardWorst", "neutral": "metricCard"}
        self.setProperty("class", cls_map.get(quality, "metricCard"))
        self.style().unpolish(self)
        self.style().polish(self)
