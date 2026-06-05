"""Deney kontrolü: simülasyon başlat / durdur / kapat."""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QRadioButton, QButtonGroup, QPushButton,
    QSpinBox, QGroupBox, QProgressBar, QTextEdit,
    QScrollArea, QFrame,
)
from PyQt6.QtCore import QTimer

from data.results_loader import ResultsLoader, MODE_LABELS, SCENARIO_LABELS
from data.scenario_loader import load_all_scenarios
from utils.process_manager import LaunchProcess, RosCleanupWorker
from utils.window_manager import arrange_windows_async
from utils.gazebo_control import set_simulation_paused


LAUNCH_CONFIGS = {
    "rmf": {
        "sim": "ros2 launch rmf_demos_gz warehouse_starter.launch.xml",
        "algo": "ros2 launch metric_logger metric_logger.launch.py "
                "traffic_mode:=rmf "
                "scenario_id:={scenario} run_id:={run_id} "
                "expected_tasks:={tasks} "
                "scenario_file:={scenario_file}",
    },
    "dkr": {
        "sim": "ros2 launch rmf_demos_gz warehouse_starter_standalone.launch.xml",
        "algo": "ros2 launch dkr_controller dkr_standalone.launch.py "
                "scenario_id:={scenario} run_id:={run_id} "
                "expected_tasks:={tasks} "
                "scenario_file:={scenario_file}",
    },
    "idkr": {
        "sim": "ros2 launch rmf_demos_gz warehouse_starter_standalone.launch.xml",
        "algo": "ros2 launch idkr_controller idkr_standalone.launch.py "
                "scenario_id:={scenario} run_id:={run_id} "
                "expected_tasks:={tasks} "
                "scenario_file:={scenario_file}",
    },
}


class ExperimentTab(QWidget):
    def __init__(self, loader: ResultsLoader, main_window=None, parent=None):
        super().__init__(parent)
        self._loader = loader
        self._main_window = main_window
        self._scenarios = load_all_scenarios()
        self._sim_proc = LaunchProcess(self)
        self._algo_proc = LaunchProcess(self)
        self._experiment_active = False
        self._stop_phase = ""
        self._cleanup_worker: RosCleanupWorker | None = None
        self._pending_full_close = False
        self._gazebo_paused = False

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(16)

        title = QLabel("Simülasyon")
        title.setProperty("class", "sectionTitle")
        layout.addWidget(title)

        subtitle = QLabel(
            "Simülasyonu başlatın (Gazebo + RViz). Algoritma otomatik devreye girer. "
            "Durdur: Gazebo simülasyon saatini duraklatır; Devam Ettir ile süre devam eder. "
            "Kapat: tüm süreçleri sonlandırır."
        )
        subtitle.setProperty("class", "sectionSubtitle")
        subtitle.setWordWrap(True)
        layout.addWidget(subtitle)

        algo_group = QGroupBox("Algoritma")
        algo_layout = QHBoxLayout(algo_group)
        self._algo_btn_group = QButtonGroup(self)
        for i, (mode, label) in enumerate(MODE_LABELS.items()):
            rb = QRadioButton(label)
            rb.setProperty("mode", mode)
            if i == 0:
                rb.setChecked(True)
            self._algo_btn_group.addButton(rb, i)
            algo_layout.addWidget(rb)
        layout.addWidget(algo_group)

        sc_group = QGroupBox("Senaryo")
        sc_layout = QHBoxLayout(sc_group)
        self._sc_btn_group = QButtonGroup(self)
        for i, (sid, label) in enumerate(SCENARIO_LABELS.items()):
            n = len(self._scenarios[sid].tasks) if sid in self._scenarios else "?"
            rb = QRadioButton(f"{label} ({n} görev)")
            rb.setProperty("scenario", sid)
            if i == 0:
                rb.setChecked(True)
            self._sc_btn_group.addButton(rb, i)
            sc_layout.addWidget(rb)
        layout.addWidget(sc_group)

        run_row = QHBoxLayout()
        run_row.addWidget(QLabel("Koşu numarası:"))
        self._run_spin = QSpinBox()
        self._run_spin.setRange(1, 999)
        self._run_spin.setValue(1)
        run_row.addWidget(self._run_spin)
        run_row.addStretch()
        layout.addLayout(run_row)

        btn_row = QHBoxLayout()
        self._start_btn = QPushButton("Simülasyonu Başlat")
        self._start_btn.setProperty("class", "primaryButton")
        self._start_btn.setFixedHeight(44)
        self._start_btn.clicked.connect(self._start_simulation)
        btn_row.addWidget(self._start_btn)

        self._pause_sim_btn = QPushButton("Simülasyonu Durdur")
        self._pause_sim_btn.setProperty("class", "secondaryButton")
        self._pause_sim_btn.setFixedHeight(44)
        self._pause_sim_btn.setEnabled(False)
        self._pause_sim_btn.clicked.connect(self._toggle_gazebo_pause)
        btn_row.addWidget(self._pause_sim_btn)

        self._close_btn = QPushButton("Simülasyonu Kapat")
        self._close_btn.setProperty("class", "dangerButton")
        self._close_btn.setFixedHeight(44)
        self._close_btn.setEnabled(False)
        self._close_btn.clicked.connect(self._close_all)
        btn_row.addWidget(self._close_btn)

        layout.addLayout(btn_row)

        self._status_label = QLabel("Hazır")
        self._status_label.setProperty("class", "statusLabel")
        layout.addWidget(self._status_label)

        self._progress = QProgressBar()
        self._progress.setRange(0, 0)
        self._progress.setVisible(False)
        layout.addWidget(self._progress)

        log_label = QLabel("Çıktı günlüğü")
        log_label.setProperty("class", "cardTitle")
        layout.addWidget(log_label)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setMinimumHeight(200)
        self._log.setStyleSheet(
            "QTextEdit { background: #ffffff; color: #4c4f69; "
            "border: 1px solid #ccd0da; border-radius: 6px; "
            "font-family: 'Ubuntu Mono', 'Courier New', monospace; font-size: 12px; }"
        )
        layout.addWidget(self._log)

        layout.addStretch()
        scroll.setWidget(container)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._sim_proc.output_line.connect(self._append_log)
        self._algo_proc.output_line.connect(self._append_log)
        self._algo_proc.finished.connect(self._on_algo_finished)
        self._sim_proc.finished.connect(self._on_sim_finished)
        self._algo_proc.stop_completed.connect(self._on_algo_stop_done)
        self._sim_proc.stop_completed.connect(self._on_sim_stop_done)

        self._arrange_timer = QTimer(self)
        self._arrange_timer.setSingleShot(True)
        self._arrange_timer.timeout.connect(self._do_arrange_silent)

    def _selected_mode(self) -> str:
        btn = self._algo_btn_group.checkedButton()
        return btn.property("mode") if btn else "dkr"

    def _selected_scenario(self) -> str:
        btn = self._sc_btn_group.checkedButton()
        return btn.property("scenario") if btn else "normal"

    def _update_pause_button_label(self):
        if self._gazebo_paused:
            self._pause_sim_btn.setText("Simülasyonu Devam Ettir")
        else:
            self._pause_sim_btn.setText("Simülasyonu Durdur")

    def _refresh_buttons(self):
        sim_on = self._sim_proc.is_running()
        algo_on = self._algo_proc.is_running()
        busy = bool(self._stop_phase)

        self._start_btn.setEnabled(not sim_on and not busy)
        self._pause_sim_btn.setEnabled(sim_on and not busy)
        self._close_btn.setEnabled((sim_on or algo_on or self._experiment_active) and not busy)
        self._update_pause_button_label()

    def _start_simulation(self):
        if self._sim_proc.is_running() or self._stop_phase:
            return

        mode = self._selected_mode()
        scenario = self._selected_scenario()
        run_id = self._run_spin.value()

        sc_def = self._scenarios.get(scenario)
        task_count = len(sc_def.tasks) if sc_def else 10
        scenario_file = str(sc_def.file_path) if sc_def else ""

        config = LAUNCH_CONFIGS.get(mode, LAUNCH_CONFIGS["dkr"])

        self._log.clear()
        self._stop_phase = ""
        self._gazebo_paused = False
        self._experiment_active = True
        self._set_status("Simülasyon başlatılıyor (Gazebo + RViz)...", "running")
        self._progress.setVisible(True)
        self._refresh_buttons()

        self._sim_proc.start(config["sim"])

        algo_cmd = config["algo"].format(
            scenario=scenario,
            run_id=run_id,
            tasks=task_count,
            scenario_file=scenario_file,
        )

        QTimer.singleShot(8000, lambda: self._launch_algo(algo_cmd))
        self._arrange_timer.start(12000)
        QTimer.singleShot(16000, self._do_arrange_silent)
        QTimer.singleShot(22000, self._do_arrange_silent)

    def _launch_algo(self, cmd: str):
        if not self._experiment_active:
            return
        if self._algo_proc.is_running():
            return
        self._set_status("Algoritma ve ölçüm sistemi başlatılıyor...", "running")
        self._algo_proc.start(cmd)
        QTimer.singleShot(2000, lambda: self._set_status("Deney çalışıyor...", "running"))
        self._refresh_buttons()

    def _toggle_gazebo_pause(self):
        """Gazebo simülasyon saatini duraklat / devam ettir (Gazebo kapanmaz)."""
        if self._stop_phase or not self._sim_proc.is_running():
            return

        target_pause = not self._gazebo_paused
        ok, _world = set_simulation_paused(target_pause, log=self._append_log)
        if not ok:
            self._set_status("Simülasyon saati kontrol edilemedi", "error")
            return

        self._gazebo_paused = target_pause
        if target_pause:
            self._set_status("Simülasyon saati duraklatıldı (Gazebo açık)", "running")
        else:
            self._set_status("Simülasyon devam ediyor", "running")
        self._refresh_buttons()

    def _close_all(self):
        """Algoritma + simülasyon + kalan ROS süreçleri."""
        if self._stop_phase:
            return

        self._pending_full_close = True
        self._experiment_active = False
        self._append_log("[UI] Tam kapatma başlatılıyor...")
        self._progress.setVisible(True)
        self._arrange_timer.stop()
        self._refresh_buttons()

        if self._algo_proc.is_running():
            self._stop_phase = "algo"
            self._set_status("Algoritma durduruluyor...", "running")
            self._algo_proc.stop()
        elif self._sim_proc.is_running():
            self._begin_sim_stop_full()
        else:
            self._stop_phase = "cleanup"
            self._run_ros_cleanup()

    def _on_algo_stop_done(self):
        if self._stop_phase == "algo":
            self._append_log("[UI] Algoritma kapandı.")
            QTimer.singleShot(1200, self._begin_sim_stop_full)

    def _begin_sim_stop_full(self):
        self._stop_phase = "sim"
        if not self._sim_proc.is_running():
            self._stop_phase = "cleanup"
            self._run_ros_cleanup()
            return
        self._set_status("Gazebo ve RViz kapatılıyor...", "running")
        self._sim_proc.stop()

    def _on_sim_stop_done(self):
        if self._stop_phase == "sim":
            self._stop_phase = "cleanup"
            self._run_ros_cleanup()

    def _run_ros_cleanup(self):
        self._set_status("Kalan ROS süreçleri temizleniyor...", "running")
        if self._cleanup_worker is not None:
            self._cleanup_worker.deleteLater()
        self._cleanup_worker = RosCleanupWorker(log=self._append_log, parent=self)
        self._cleanup_worker.finished.connect(self._on_ros_cleanup_done)
        self._cleanup_worker.start()

    def _on_ros_cleanup_done(self):
        self._stop_phase = ""
        self._pending_full_close = False
        self._experiment_active = False
        self._gazebo_paused = False
        self._progress.setVisible(False)
        self._set_status("Kapatıldı", "neutral")
        self._append_log("[UI] Tüm süreçler durduruldu.")
        self._refresh_buttons()

    def _on_algo_finished(self, exit_code: int):
        if self._stop_phase in ("algo", "sim", "cleanup"):
            return
        self._progress.setVisible(False)
        if exit_code == 0:
            self._set_status("Deney tamamlandı!", "done")
            self._loader.reload()
        else:
            self._set_status(f"Algoritma sonlandı (kod: {exit_code})", "error")
        self._refresh_buttons()

    def _on_sim_finished(self, exit_code: int):
        if self._stop_phase:
            return
        if self._experiment_active:
            self._experiment_active = False
            self._gazebo_paused = False
            self._progress.setVisible(False)
            if exit_code != 0:
                self._set_status("Simülasyon beklenmedik şekilde kapandı", "error")
            else:
                self._set_status("Simülasyon kapandı", "neutral")
        self._refresh_buttons()

    def _set_status(self, text: str, style: str = "neutral"):
        self._status_label.setText(text)
        cls_map = {
            "running": "statusRunning",
            "done": "statusDone",
            "error": "statusError",
            "neutral": "statusLabel",
        }
        self._status_label.setProperty("class", cls_map.get(style, "statusLabel"))
        self._status_label.style().unpolish(self._status_label)
        self._status_label.style().polish(self._status_label)

    def _do_arrange_silent(self):
        if not self._sim_proc.is_running():
            return

        def _log(res: dict):
            dash = "OK" if res.get("dashboard") else "-"
            gz = "OK" if res.get("gazebo") else "-"
            rv = "OK" if res.get("rviz") else "-"
            self._append_log(f"[UI] Pencere düzeni: Panel={dash}, Gazebo={gz}, RViz={rv}")

        arrange_windows_async(self._main_window, on_done=_log)

    def _append_log(self, line: str):
        self._log.append(line)
        sb = self._log.verticalScrollBar()
        sb.setValue(sb.maximum())
