"""QGraphicsItem subclasses for map visualization."""

import math
from PyQt6.QtWidgets import (
    QGraphicsEllipseItem, QGraphicsTextItem, QGraphicsLineItem,
    QGraphicsItem,
)
from PyQt6.QtCore import Qt, QPointF, QRectF
from PyQt6.QtGui import QColor, QPen, QBrush, QFont


SCALE = 2.0


class WaypointItem(QGraphicsEllipseItem):
    """A circle representing a nav graph vertex."""

    def __init__(self, x: float, y: float, name: str, color: str, radius: float = 6.0):
        r = radius
        super().__init__(-r, -r, 2 * r, 2 * r)
        self.setPos(x * SCALE, y * SCALE)
        self.setBrush(QBrush(QColor(color)))
        self.setPen(QPen(QColor(color).darker(130), 1.5))
        self.setZValue(10)

        if name:
            label = QGraphicsTextItem(name, self)
            label.setDefaultTextColor(QColor("#cdd6f4"))
            font = QFont("Ubuntu", 7)
            label.setFont(font)
            label.setPos(r + 2, -8)
            self.setToolTip(name)


class LaneItem(QGraphicsLineItem):
    """A line representing a nav graph lane."""

    def __init__(self, x1: float, y1: float, x2: float, y2: float, bidirectional: bool = True):
        super().__init__(x1 * SCALE, y1 * SCALE, x2 * SCALE, y2 * SCALE)
        color = QColor("#45475a")
        pen = QPen(color, 1.5)
        pen.setStyle(Qt.PenStyle.SolidLine if bidirectional else Qt.PenStyle.DashLine)
        self.setPen(pen)
        self.setZValue(1)


class RobotItem(QGraphicsEllipseItem):
    """A robot marker that can be moved on the map."""

    COLORS = ["#89b4fa", "#fab387", "#a6e3a1", "#cba6f7"]
    _color_idx = 0

    def __init__(self, name: str, x: float = 0, y: float = 0):
        r = 10.0
        super().__init__(-r, -r, 2 * r, 2 * r)

        color = QColor(self.COLORS[RobotItem._color_idx % len(self.COLORS)])
        RobotItem._color_idx += 1

        self.setBrush(QBrush(color))
        self.setPen(QPen(color.lighter(130), 2))
        self.setZValue(50)
        self.setPos(x * SCALE, y * SCALE)

        label = QGraphicsTextItem(name.replace("warehouseRobot", "R"), self)
        label.setDefaultTextColor(QColor("#1e1e2e"))
        font = QFont("Ubuntu", 8, QFont.Weight.Bold)
        label.setFont(font)
        br = label.boundingRect()
        label.setPos(-br.width() / 2, -br.height() / 2)

        self._name = name

    def update_pos(self, x: float, y: float):
        self.setPos(x * SCALE, y * SCALE)
