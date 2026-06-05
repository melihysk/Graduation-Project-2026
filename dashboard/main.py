"""Dashboard application entry point."""

import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Trafik Yönetimi Paneli")

    qss_path = Path(__file__).parent / "style.qss"
    if qss_path.exists():
        app.setStyleSheet(qss_path.read_text())

    font = QFont("Ubuntu", 11)
    font.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
    app.setFont(font)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
