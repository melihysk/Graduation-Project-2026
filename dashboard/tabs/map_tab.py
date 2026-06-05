"""Live map tab: nav graph + robot positions from /fleet_states."""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QGraphicsScene, QGraphicsView, QPushButton,
    QFrame,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QBrush

from data.map_parser import parse_map
from widgets.robot_graphics import WaypointItem, LaneItem, RobotItem, SCALE


class MapTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._fleet_listener = None
        self._robot_items: dict[str, RobotItem] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(12)

        header = QHBoxLayout()
        title = QLabel("Canli Harita — warehouse_starter")
        title.setProperty("class", "sectionTitle")
        header.addWidget(title)
        header.addStretch()

        self._status_lbl = QLabel("ROS2 baglantisi yok")
        self._status_lbl.setProperty("class", "statusLabel")
        header.addWidget(self._status_lbl)

        self._connect_btn = QPushButton("ROS2 Baglan")
        self._connect_btn.setProperty("class", "primaryButton")
        self._connect_btn.clicked.connect(self._toggle_ros)
        header.addWidget(self._connect_btn)

        layout.addLayout(header)

        subtitle = QLabel("Waypoint'ler: PICK (mavi), DROP (turuncu), Kavsak (kirmizi), Charger (yesil)")
        subtitle.setProperty("class", "sectionSubtitle")
        layout.addWidget(subtitle)

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(QBrush(QColor("#11111b")))

        self._view = QGraphicsView(self._scene)
        self._view.setRenderHints(
            self._view.renderHints()
            | self._view.renderHints().__class__.Antialiasing
            | self._view.renderHints().__class__.SmoothPixmapTransform
        )
        self._view.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self._view.setStyleSheet(
            "QGraphicsView { border: 1px solid #313244; border-radius: 8px; background: #11111b; }"
        )
        layout.addWidget(self._view, 1)

        self._load_map()

    def _load_map(self):
        vertices, lanes = parse_map()

        for lane in lanes:
            if lane.v1 < len(vertices) and lane.v2 < len(vertices):
                v1 = vertices[lane.v1]
                v2 = vertices[lane.v2]
                self._scene.addItem(LaneItem(v1.x, v1.y, v2.x, v2.y, lane.bidirectional))

        for v in vertices:
            radius = 8.0 if v.name else 4.0
            self._scene.addItem(WaypointItem(v.x, v.y, v.name, v.color, radius))

        self._view.fitInView(self._scene.sceneRect().adjusted(-40, -40, 40, 40),
                            Qt.AspectRatioMode.KeepAspectRatio)

    def _toggle_ros(self):
        if self._fleet_listener is not None:
            self._stop_ros()
        else:
            self._start_ros()

    def _start_ros(self):
        try:
            from ros.fleet_listener import FleetListener
            self._fleet_listener = FleetListener()
            self._fleet_listener.fleet_updated.connect(self._on_fleet_update)
            self._fleet_listener.start()
            self._status_lbl.setText("ROS2 bagli")
            self._status_lbl.setProperty("class", "statusDone")
            self._status_lbl.style().unpolish(self._status_lbl)
            self._status_lbl.style().polish(self._status_lbl)
            self._connect_btn.setText("Baglantiyi Kes")
        except ImportError:
            self._status_lbl.setText("rclpy bulunamadi")
            self._status_lbl.setProperty("class", "statusError")
            self._status_lbl.style().unpolish(self._status_lbl)
            self._status_lbl.style().polish(self._status_lbl)

    def _stop_ros(self):
        if self._fleet_listener:
            self._fleet_listener.stop()
            self._fleet_listener = None
        self._status_lbl.setText("ROS2 baglantisi kesildi")
        self._status_lbl.setProperty("class", "statusLabel")
        self._status_lbl.style().unpolish(self._status_lbl)
        self._status_lbl.style().polish(self._status_lbl)
        self._connect_btn.setText("ROS2 Baglan")

    def _on_fleet_update(self, data: dict):
        for name, info in data.items():
            x = info.get("x", 0)
            y = info.get("y", 0)

            if name not in self._robot_items:
                robot = RobotItem(name, x, y)
                self._scene.addItem(robot)
                self._robot_items[name] = robot
            else:
                self._robot_items[name].update_pos(x, y)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._scene.items():
            self._view.fitInView(self._scene.sceneRect().adjusted(-40, -40, 40, 40),
                                Qt.AspectRatioMode.KeepAspectRatio)
