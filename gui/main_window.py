"""PyQt5 GUI for mission control and visualization."""

from __future__ import annotations

import threading
import time
from typing import List, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets

import config
from avoidance import AvoidanceController
from lidar_core import LidarCore
from mavlink_comm import MavlinkComm
from motion_control import MotionControl
from settings import (
    Settings,
    list_camera_devices,
    list_lidar_ports,
)
from shared_data import SharedData
from utils.camera_manager import CameraManager
from version import VERSION
from .lidar_widget import LidarWidget
from .i18n import tr, set_language, get_language


def status_dot(ok: bool, inactive_color: str = "#9e9e9e") -> str:
    color = "#4caf50" if ok else inactive_color
    return f'<span style="color:{color};font-size:16px">●</span>'


def status_text(label: str, state: str | None) -> str:
    state_normalized = (state or "not_connected").lower()
    colors = {
        "connected": "#4caf50",
        "error": "#e53935",
        "connecting": "#ffb300",
        "not_connected": "#9e9e9e",
        "disconnected": "#9e9e9e",
    }
    words = {
        "connected": tr("Connected"),
        "error": tr("Error"),
        "connecting": tr("Connecting"),
        "not_connected": tr("Not Connected"),
        "disconnected": tr("Disconnected"),
    }
    color = colors.get(state_normalized, "#9e9e9e")
    text = words.get(state_normalized, state_normalized.title())
    return f"{label}: <span style='color:{color};font-weight:bold'>{text}</span>"


def _camera_options() -> List[Tuple[str, str]]:
    """Return [(label, value)] for camera devices."""
    options: List[Tuple[str, str]] = [(tr("Raspberry Pi Camera (libcamera)"), "PI_CAMERA")]
    for dev in list_camera_devices():
        options.append((tr("USB Camera ({dev})", dev=dev), dev))
    options.append((tr("Off"), "OFF"))
    # Remove duplicates while preserving order
    dedup: List[Tuple[str, str]] = []
    seen = set()
    for label, value in options:
        if value in seen:
            continue
        seen.add(value)
        dedup.append((label, value))
    return dedup


class SettingsDialog(QtWidgets.QDialog):
    def __init__(self, settings: Settings, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.settings = settings
        self.setWindowTitle(tr("Settings"))
        form = QtWidgets.QFormLayout(self)

        # Fixed UART Pixhawk connection
        self.pixhawk_label = QtWidgets.QLabel(tr("Pixhawk Port: /dev/serial0 (UART)"))
        self.pixhawk_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.series_combo = QtWidgets.QComboBox()
        self.series_combo.addItems([f"Series {i}" for i in range(0, 6)])
        current_series = str(self.settings.get("pixhawk_series", "Series 0"))
        idx = max(0, self.series_combo.findText(current_series))
        self.series_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self.baud_spin = QtWidgets.QSpinBox()
        self.baud_spin.setRange(2400, 921600)
        self.baud_spin.setSingleStep(100)
        self.baud_spin.setValue(int(self.settings.mavlink_baud))

        self.lidar_port_combo = QtWidgets.QComboBox()
        self.lidar_port_combo.setEditable(True)
        lidar_current = self.settings.lidar_port or self.settings.get("lidar_port_hint", "")
        self.lidar_port_combo.setCurrentText(lidar_current)

        self.camera_combo = QtWidgets.QComboBox()
        self.camera_combo.setEditable(False)

        self._refresh_devices()

        form.addRow(self.pixhawk_label)
        form.addRow(tr("Series"), self.series_combo)
        form.addRow(tr("Baudrate"), self.baud_spin)
        form.addRow(tr("LiDAR Port (/dev/ttyUSB*)"), self.lidar_port_combo)
        form.addRow(tr("Camera Selection"), self.camera_combo)

        btn_box = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        btn_box.accepted.connect(self._save)
        btn_box.rejected.connect(self.reject)
        form.addRow(btn_box)

    def _save(self) -> None:
        port = self.lidar_port_combo.currentText().strip()
        self.settings.set("lidar_port", port)
        self.settings.set("lidar_port_hint", port)  # legacy key for compatibility
        camera_value = self.camera_combo.currentData() or self.camera_combo.currentText()
        camera_value = str(camera_value).strip()
        self.settings.set("camera_device", camera_value)
        self.settings.set("camera_source", camera_value)  # legacy compatibility
        self.settings.set("camera_mode", camera_value)
        self.settings.set("pixhawk_series", self.series_combo.currentText())
        self.settings.set("mavlink_baud", int(self.baud_spin.value()))
        self.accept()

    def _refresh_devices(self) -> None:
        lidar_ports = list_lidar_ports()
        camera_options = _camera_options()

        def _fill_combo(combo: QtWidgets.QComboBox, options: List[str], current: str) -> None:
            combo.blockSignals(True)
            combo.clear()
            for opt in options:
                combo.addItem(opt)
            if current and current not in options:
                combo.addItem(current)
            combo.setCurrentText(current)
            combo.blockSignals(False)

        _fill_combo(self.lidar_port_combo, lidar_ports or [self.settings.lidar_port], self.settings.lidar_port)

        self.camera_combo.blockSignals(True)
        self.camera_combo.clear()
        current_cam = self.settings.camera_device
        for label, value in camera_options:
            self.camera_combo.addItem(label, value)
        if current_cam and current_cam not in [v for _, v in camera_options]:
            self.camera_combo.addItem(f"Custom ({current_cam})", current_cam)
        index = max(0, self.camera_combo.findData(current_cam))
        self.camera_combo.setCurrentIndex(index)
        self.camera_combo.blockSignals(False)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        shared: SharedData,
        settings: Settings,
        mav: MavlinkComm,
        avoidance: AvoidanceController,
        motion: MotionControl,
        camera_mgr: CameraManager,
        lidar: LidarCore,
    ) -> None:
        super().__init__()
        self.shared = shared
        self.settings = settings
        self.mav = mav
        self.avoidance = avoidance
        self.motion = motion
        self.camera_mgr = camera_mgr
        self.lidar = lidar
        self.setWindowTitle(f"{tr('Obstacle Mission')} {VERSION}")
        self.setMinimumSize(900, 700)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setWindowFlags(
            QtCore.Qt.Window
            | QtCore.Qt.WindowMinimizeButtonHint
            | QtCore.Qt.WindowMaximizeButtonHint
            | QtCore.Qt.WindowCloseButtonHint
        )
        self._last_lidar_update = 0.0
        self._pending_status_msg: str = ""
        self._auto_start_pending: bool = False
        self._start_in_progress: bool = False
        self._start_success: bool = False
        self._viz_allowed: bool = False

        tab_widget = QtWidgets.QTabWidget()
        self.setCentralWidget(tab_widget)

        # Settings tab -------------------------------------------------------
        settings_tab = QtWidgets.QWidget()
        tab_widget.addTab(settings_tab, tr("Settings"))

        main_grid = QtWidgets.QGridLayout(settings_tab)
        main_grid.setContentsMargins(12, 12, 12, 12)
        main_grid.setHorizontalSpacing(12)
        main_grid.setVerticalSpacing(12)
        main_grid.setColumnStretch(0, 1)
        main_grid.setColumnStretch(1, 1)
        main_grid.setRowStretch(0, 0)
        main_grid.setRowStretch(1, 0)
        main_grid.setRowStretch(2, 1)

        # Status row (spans both columns)
        status_container = QtWidgets.QVBoxLayout()
        self.status_labels = {
            "mav": QtWidgets.QLabel(),
            "lidar": QtWidgets.QLabel(),
            "mission": QtWidgets.QLabel(),
            "avoidance": QtWidgets.QLabel(),
        }
        status_layout = QtWidgets.QHBoxLayout()
        for key, label in self.status_labels.items():
            status_layout.addWidget(label)
        status_layout.addStretch()
        status_container.addLayout(status_layout)
        self.message_label = QtWidgets.QLabel("")
        self.message_label.setStyleSheet("color: #9e9e9e")
        status_container.addWidget(self.message_label)
        main_grid.addLayout(status_container, 0, 0, 1, 2)

        # Top frame with two columns
        top_frame = QtWidgets.QWidget()
        top_layout = QtWidgets.QGridLayout(top_frame)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setHorizontalSpacing(12)
        top_layout.setColumnStretch(0, 1)
        top_layout.setColumnStretch(1, 1)

        # Left column frame
        left_frame = QtWidgets.QFrame()
        left_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        left_layout = QtWidgets.QVBoxLayout(left_frame)
        left_layout.setSpacing(10)

        controls = QtWidgets.QHBoxLayout()
        self.btn_auto_mission = QtWidgets.QPushButton(tr("AUTO"))
        self.btn_auto_mission.clicked.connect(self._start_mission)
        controls.addWidget(self.btn_auto_mission)

        self.auto_start_chk = QtWidgets.QCheckBox(tr("Start automatically after 5 seconds"))
        self.auto_start_chk.setChecked(self.settings.auto_start)
        self.auto_start_chk.stateChanged.connect(self._auto_start_changed)
        controls.addWidget(self.auto_start_chk)

        self.btn_settings = QtWidgets.QPushButton(tr("Settings"))
        self.btn_settings.clicked.connect(self._open_settings)
        controls.addWidget(self.btn_settings)
        controls.addStretch()
        left_layout.addLayout(controls)

        device_layout = QtWidgets.QHBoxLayout()
        self.btn_connect = QtWidgets.QPushButton(tr("START"))
        self.btn_connect.clicked.connect(self._start_sequence)
        device_layout.addWidget(self.btn_connect)
        self.btn_disconnect = QtWidgets.QPushButton(tr("Disconnect Devices"))
        self.btn_disconnect.clicked.connect(self._disconnect_devices)
        device_layout.addWidget(self.btn_disconnect)
        self.device_status_labels = {
            "lidar": QtWidgets.QLabel(status_text(tr("LiDAR"), "not_connected")),
            "mav": QtWidgets.QLabel(status_text(tr("MAVLink"), "not_connected")),
            "camera": QtWidgets.QLabel(status_text(tr("Camera"), "not_connected")),
        }
        for key in ("lidar", "mav", "camera"):
            self.device_status_labels[key].setTextFormat(QtCore.Qt.RichText)
            device_layout.addWidget(self.device_status_labels[key])
        device_layout.addStretch()
        left_layout.addLayout(device_layout)

        speed_group = QtWidgets.QGroupBox(tr("Speed & Limits"))
        speed_form = QtWidgets.QFormLayout()
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setMinimum(1000)
        self.speed_slider.setMaximum(2000)
        self.speed_slider.setValue(self.shared.base_speed_pwm)
        self.speed_slider.valueChanged.connect(self._speed_changed)
        self.speed_label = QtWidgets.QLabel(self._speed_percent_text(self.speed_slider.value()))
        speed_box = QtWidgets.QHBoxLayout()
        speed_box.addWidget(self.speed_slider)
        speed_box.addWidget(self.speed_label)
        speed_widget = QtWidgets.QWidget()
        speed_widget.setLayout(speed_box)
        speed_form.addRow(tr("Speed (PWM 1000-2000)"), speed_widget)

        self.movement_coefficient = QtWidgets.QDoubleSpinBox()
        self.movement_coefficient.setRange(0.1, 10.0)
        self.movement_coefficient.setDecimals(2)
        self.movement_coefficient.setSingleStep(0.1)
        self.movement_coefficient.setValue(float(self.settings.movement_coefficient))
        self.movement_coefficient.valueChanged.connect(self._movement_coefficient_changed)
        self.movement_coefficient.setFixedWidth(100)
        speed_form.addRow(tr("Movement Coefficient (sec per meter)"), self.movement_coefficient)

        self.limit1 = QtWidgets.QSpinBox()
        self.limit1.setRange(20, 400)
        self.limit1.setValue(self.shared.limit1_cm)
        self.limit1.valueChanged.connect(self._limits_changed)
        self.limit1.setFixedWidth(100)
        self.limit2 = QtWidgets.QSpinBox()
        self.limit2.setRange(10, 300)
        self.limit2.setValue(self.shared.limit2_cm)
        self.limit2.valueChanged.connect(self._limits_changed)
        self.limit2.setFixedWidth(100)
        speed_form.addRow(tr("Yellow Zone (cm)"), self.limit1)
        speed_form.addRow(tr("Red Zone (cm)"), self.limit2)
        speed_group.setLayout(speed_form)
        left_layout.addWidget(speed_group)

        avoidance_group = QtWidgets.QGroupBox(tr("Avoidance Settings"))
        avoidance_form = QtWidgets.QFormLayout()
        self.avoid_turn = QtWidgets.QSpinBox()
        self.avoid_turn.setRange(30, 180)
        self.avoid_turn.setValue(int(self.shared.avoidance_turn_deg))
        self.avoid_turn.valueChanged.connect(self._avoidance_settings_changed)
        self.avoid_turn.setFixedWidth(100)

        self.avoid_distance = QtWidgets.QDoubleSpinBox()
        self.avoid_distance.setRange(1.0, 3.0)
        self.avoid_distance.setDecimals(1)
        self.avoid_distance.setSingleStep(0.1)
        self.avoid_distance.setValue(float(self.shared.avoidance_distance_m))
        self.avoid_distance.valueChanged.connect(self._avoidance_settings_changed)
        self.avoid_distance.setFixedWidth(100)

        self.backoff_distance = QtWidgets.QDoubleSpinBox()
        self.backoff_distance.setRange(0.5, 3.0)
        self.backoff_distance.setDecimals(1)
        self.backoff_distance.setSingleStep(0.1)
        self.backoff_distance.setValue(float(self.shared.avoidance_backoff_m))
        self.backoff_distance.valueChanged.connect(self._avoidance_settings_changed)
        self.backoff_distance.setFixedWidth(100)

        self.reverse_duration = QtWidgets.QDoubleSpinBox()
        self.reverse_duration.setRange(0.5, 10.0)
        self.reverse_duration.setDecimals(1)
        self.reverse_duration.setSingleStep(0.5)
        self.reverse_duration.setValue(float(self.shared.reverse_duration_sec))
        self.reverse_duration.valueChanged.connect(self._timing_settings_changed)
        self.reverse_duration.setFixedWidth(100)

        self.lock_duration = QtWidgets.QDoubleSpinBox()
        self.lock_duration.setRange(0.5, 10.0)
        self.lock_duration.setDecimals(1)
        self.lock_duration.setSingleStep(0.5)
        self.lock_duration.setValue(float(self.shared.post_turn_lock_duration_sec))
        self.lock_duration.valueChanged.connect(self._timing_settings_changed)
        self.lock_duration.setFixedWidth(100)

        self.command_cooldown = QtWidgets.QDoubleSpinBox()
        self.command_cooldown.setRange(1.0, 10.0)
        self.command_cooldown.setDecimals(1)
        self.command_cooldown.setSingleStep(0.5)
        self.command_cooldown.setValue(float(self.shared.command_cooldown_sec))
        self.command_cooldown.valueChanged.connect(self._timing_settings_changed)
        self.command_cooldown.setFixedWidth(100)

        avoidance_form.addRow(tr("Turn angle (deg)"), self.avoid_turn)
        avoidance_form.addRow(tr("Travel distance (m)"), self.avoid_distance)
        avoidance_form.addRow(tr("Backoff distance (m)"), self.backoff_distance)
        avoidance_form.addRow(tr("Reverse duration (s)"), self.reverse_duration)
        avoidance_form.addRow(tr("Post-turn lock (s)"), self.lock_duration)
        avoidance_form.addRow(tr("Command/mode cooldown (s)"), self.command_cooldown)
        avoidance_group.setLayout(avoidance_form)
        left_layout.addWidget(avoidance_group)

        bounds_group = QtWidgets.QGroupBox(tr("LiDAR & Boundaries"))
        bounds_form = QtWidgets.QFormLayout()

        self.blue_boundary = QtWidgets.QSpinBox()
        self.blue_boundary.setRange(10, 400)
        self.blue_boundary.setValue(self.shared.blue_boundary_cm)
        self.blue_boundary.valueChanged.connect(self._boundaries_changed)
        self.blue_boundary.setFixedWidth(100)

        self.green_boundary = QtWidgets.QSpinBox()
        self.green_boundary.setRange(20, 500)
        self.green_boundary.setValue(self.shared.green_boundary_cm)
        self.green_boundary.valueChanged.connect(self._boundaries_changed)
        self.green_boundary.setFixedWidth(100)

        self.lidar_buffer_spin = QtWidgets.QSpinBox()
        self.lidar_buffer_spin.setRange(256, 8192)
        self.lidar_buffer_spin.setSingleStep(256)
        self.lidar_buffer_spin.setValue(self.shared.lidar_buffer_size)
        self.lidar_buffer_spin.valueChanged.connect(self._lidar_config_changed)
        self.lidar_buffer_spin.setFixedWidth(100)

        self.lidar_rate_spin = QtWidgets.QDoubleSpinBox()
        self.lidar_rate_spin.setRange(1.0, 15.0)
        self.lidar_rate_spin.setDecimals(1)
        self.lidar_rate_spin.setSingleStep(0.5)
        self.lidar_rate_spin.setValue(self.shared.lidar_read_hz)
        self.lidar_rate_spin.valueChanged.connect(self._lidar_config_changed)
        self.lidar_rate_spin.setFixedWidth(100)

        bounds_form.addRow(tr("Blue Boundary Distance (cm)"), self.blue_boundary)
        bounds_form.addRow(tr("Green Boundary Distance (cm)"), self.green_boundary)
        bounds_form.addRow(tr("LiDAR Buffer Size (bytes)"), self.lidar_buffer_spin)
        bounds_form.addRow(tr("LiDAR Read Frequency (Hz)"), self.lidar_rate_spin)
        bounds_group.setLayout(bounds_form)
        left_layout.addWidget(bounds_group)
        left_layout.addStretch()

        # Right column frame (reserved for future features)
        right_frame = QtWidgets.QFrame()
        right_frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        right_layout = QtWidgets.QVBoxLayout(right_frame)
        right_layout.setSpacing(10)
        right_layout.addStretch()

        top_layout.addWidget(left_frame, 0, 0)
        top_layout.addWidget(right_frame, 0, 1)
        main_grid.addWidget(top_frame, 1, 0, 1, 2)

        # Visualization tab -------------------------------------------------
        viz_tab = QtWidgets.QWidget()
        tab_widget.addTab(viz_tab, tr("Visualization"))
        viz_layout = QtWidgets.QVBoxLayout(viz_tab)
        viz_layout.setContentsMargins(12, 12, 12, 12)
        viz_layout.setSpacing(10)

        self.btn_refresh_lidar = QtWidgets.QPushButton(tr("Refresh LiDAR"))
        self.btn_refresh_lidar.clicked.connect(self._refresh_lidar)
        self.btn_refresh_lidar.setEnabled(False)

        self.btn_start_camera_vis = QtWidgets.QPushButton(tr("Start Camera Visualization"))
        self.btn_start_camera_vis.clicked.connect(self._start_camera_visualization)
        self.btn_stop_camera_vis = QtWidgets.QPushButton(tr("Stop Camera Visualization"))
        self.btn_stop_camera_vis.clicked.connect(self._stop_camera_visualization)
        self.btn_stop_camera_vis.setEnabled(False)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addWidget(self.btn_refresh_lidar)
        btn_row.addSpacing(12)
        btn_row.addWidget(self.btn_start_camera_vis)
        btn_row.addWidget(self.btn_stop_camera_vis)
        btn_row.addStretch()
        viz_layout.addLayout(btn_row)

        self.lidar_view = LidarWidget(self.shared)
        self.avoidance.statusChanged.connect(self.lidar_view.overlay.set_message)
        self.avoidance.statusChanged.connect(lambda msg: self.lidar_view.update())

        lidar_group = QtWidgets.QGroupBox(tr("LiDAR"))
        lidar_layout = QtWidgets.QVBoxLayout(lidar_group)
        lidar_layout.addWidget(self.lidar_view, 1)
        lidar_group.setMinimumWidth(700)
        lidar_group.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.camera_label = QtWidgets.QLabel(tr("Camera feed is off"))
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setWordWrap(True)
        self.camera_label.setFixedSize(640, 480)
        self.camera_label.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.camera_label.setStyleSheet("background-color: #0d1117; color: #9e9e9e; border: 1px solid #222;")

        camera_group = QtWidgets.QGroupBox(tr("Camera"))
        camera_layout = QtWidgets.QVBoxLayout(camera_group)
        camera_layout.setAlignment(QtCore.Qt.AlignCenter)
        camera_layout.addWidget(self.camera_label, 0, QtCore.Qt.AlignCenter)
        camera_group.setMinimumWidth(700)
        camera_group.setMaximumWidth(700)
        camera_group.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)

        views_row = QtWidgets.QHBoxLayout()
        views_row.addWidget(camera_group, 1)
        views_row.addWidget(lidar_group, 1)
        views_row.setStretch(0, 1)
        views_row.setStretch(1, 1)
        viz_layout.addLayout(views_row, 1)

        # Respect saved state on launch (visualization starts manually)
        self.shared.update_status(
            show_camera=False,
            show_lidar=False,
            camera_device=self._camera_device_from_settings(),
            visualization_running=False,
            obstacle_avoidance_on=False,
            avoidance_enabled=False,
        )

        # Timers
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self._refresh_status)
        self.timer.start()

        if self.settings.auto_start:
            QtCore.QTimer.singleShot(5000, self._auto_start_sequence)

    # Handlers ------------------------------------------------------------------
    def _speed_percent_text(self, value: int) -> str:
        pct = int((value - 1000) / 10)
        return f"{pct}%"

    def _speed_changed(self, value: int) -> None:
        self.speed_label.setText(self._speed_percent_text(value))
        self.shared.update_status(base_speed_pwm=value)
        self.settings.set("base_speed_pwm", value)
        percent = max(0.0, min(100.0, (value - 1000) / 10.0))
        self.mav.set_speed_limit_percent(percent)

    def _movement_coefficient_changed(self) -> None:
        coeff = float(self.movement_coefficient.value())
        self.settings.set("movement_coefficient", coeff)
        self.shared.update_status(movement_coefficient=coeff)

    def _limits_changed(self) -> None:
        l1 = self.limit1.value()
        l2 = self.limit2.value()
        if l2 >= l1:
            l2 = l1 - 5
            self.limit2.setValue(l2)
        self.shared.update_status(limit1_cm=l1, limit2_cm=l2)
        self.settings.set("limit1_cm", l1)
        self.settings.set("limit2_cm", l2)
        self.lidar_view.update()

    def _avoidance_settings_changed(self) -> None:
        angle = int(self.avoid_turn.value())
        distance = float(self.avoid_distance.value())
        backoff = float(self.backoff_distance.value())
        self.shared.update_status(
            avoidance_turn_deg=angle,
            avoidance_distance_m=distance,
            avoidance_backoff_m=backoff,
        )
        self.settings.set("avoidance_turn_deg", angle)
        self.settings.set("avoidance_distance_m", distance)
        self.settings.set("avoidance_backoff_m", backoff)
        self.lidar_view.update()

    def _timing_settings_changed(self) -> None:
        reverse_sec = float(self.reverse_duration.value())
        lock_sec = float(self.lock_duration.value())
        cooldown_sec = float(self.command_cooldown.value())
        self.shared.update_status(
            reverse_duration_sec=reverse_sec,
            post_turn_lock_duration_sec=lock_sec,
            command_cooldown_sec=cooldown_sec,
        )
        self.settings.set("reverse_duration_sec", reverse_sec)
        self.settings.set("post_turn_lock_duration_sec", lock_sec)
        self.settings.set("command_cooldown_sec", cooldown_sec)

    def _boundaries_changed(self) -> None:
        blue = int(self.blue_boundary.value())
        green = int(self.green_boundary.value())
        self.shared.update_status(blue_boundary_cm=blue, green_boundary_cm=green)
        self.settings.set("blue_boundary_cm", blue)
        self.settings.set("green_boundary_cm", green)
        self.lidar_view.update()

    def _lidar_config_changed(self) -> None:
        buffer_size = int(self.lidar_buffer_spin.value())
        freq = float(self.lidar_rate_spin.value())
        self.settings.set("lidar_buffer_size", buffer_size)
        self.settings.set("lidar_read_hz", freq)
        self.shared.update_status(lidar_buffer_size=buffer_size, lidar_read_hz=freq)

    def _open_settings(self) -> None:
        dlg = SettingsDialog(self.settings, self)
        if dlg.exec_() == QtWidgets.QDialog.Accepted:
            if self.shared.show_camera and self._viz_allowed:
                self._start_camera_visualization()
            if self.shared.show_lidar and self._viz_allowed:
                self._refresh_lidar()

    def _start_mission(self) -> None:
        self.mav.start_mission()
        self.shared.update_status(avoidance_enabled=True, manual_control=False, manual_override=False)

    def _auto_start_sequence(self) -> None:
        if not self.auto_start_chk.isChecked():
            return
        if getattr(self, "_start_in_progress", False):
            return
        self._start_sequence()

    def _auto_start_changed(self, state: int) -> None:
        enabled = state == QtCore.Qt.Checked
        self.settings.set("auto_start", enabled)
        self.settings.set("auto_start_after_5s", enabled)

    def _camera_device_from_settings(self) -> str:
        return str(self.settings.camera_device or "OFF")

    def _start_camera_from_settings(self) -> None:
        if not self.shared.show_camera:
            return
        device = self._camera_device_from_settings()
        if not device or device.upper() == "OFF":
            self.shared.update_status(
                camera_mode="OFF",
                camera_source="OFF",
                camera_device="OFF",
                status_message=tr("Camera disabled"),
                camera_status="not_connected",
                camera_connected=False,
            )
            self.shared.set_camera_frame(None)
            self.camera_mgr.stop(keep_flags=True)
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Camera feed is off"))
            return

        self.camera_label.setPixmap(QtGui.QPixmap())
        self.camera_label.setText(f"Opening camera ({device})...")
        self.shared.update_status(
            camera_mode=device,
            camera_source=device,
            camera_device=device,
            show_camera=True,
            camera_status="connecting",
            visualization_running=True,
        )
        self.camera_mgr.start(device, backend=self.settings.camera_backend)

    def _start_sequence(self) -> None:
        if getattr(self, "_start_in_progress", False):
            return
        self._start_in_progress = True
        self._start_success = False
        self._viz_allowed = False
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(False)
        self.shared.update_status(
            status_message=tr("Starting devices..."),
            show_lidar=False,
            show_camera=False,
            visualization_running=False,
            avoidance_enabled=False,
            obstacle_avoidance_on=False,
        )
        threading.Thread(target=self._connect_devices_worker, daemon=True).start()

    def _connect_devices_worker(self) -> None:
        lidar_ok = self.lidar.connect()
        self.mav.reconnect()

        camera_device = self._camera_device_from_settings()
        camera_ok = camera_device.upper() == "OFF" or self.camera_mgr.probe(
            camera_device, backend=self.settings.camera_backend
        )

        success = bool(lidar_ok or camera_ok or self.shared.mav_connected)
        status_msg = tr("Devices connected, initializing...") if success else tr("Device connection failed")

        self._pending_status_msg = status_msg
        self._start_success = success
        QtCore.QMetaObject.invokeMethod(self, "_on_connect_finished", QtCore.Qt.QueuedConnection)

    @QtCore.pyqtSlot()
    def _on_connect_finished(self) -> None:
        if self._pending_status_msg:
            self.shared.update_status(status_message=self._pending_status_msg)
            self._pending_status_msg = ""
        if not self._start_success:
            self._start_in_progress = False
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(True)
            return
        # Start 5s initialization window before enabling visualization
        QtCore.QTimer.singleShot(5000, self._complete_start_sequence)

    def _complete_start_sequence(self) -> None:
        if not self._start_in_progress:
            return
        self._enable_visualization()
        self._start_in_progress = False
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(True)

    def _enable_visualization(self) -> None:
        self.shared.update_status(show_lidar=True, obstacle_avoidance_on=True, avoidance_enabled=True)
        if not self.shared.lidar_connected:
            self.lidar.connect()
        started = self.lidar.start_stream()
        self.avoidance.start()
        if started:
            self.shared.update_status(lidar_status="connected", visualization_running=True)
        else:
            self.shared.update_status(show_lidar=False, visualization_running=False, status_message=tr("LiDAR stream failed"))
        self._viz_allowed = True
        self.btn_refresh_lidar.setEnabled(bool(started))
        self.lidar_view.update()
        self.shared.update_status(status_message=tr("Devices ready"))

    def _disconnect_devices(self) -> None:
        self.btn_connect.setEnabled(False)
        self.btn_disconnect.setEnabled(False)
        self.shared.update_status(status_message=tr("Disconnecting devices..."))
        threading.Thread(target=self._disconnect_devices_worker, daemon=True).start()

    def _disconnect_devices_worker(self) -> None:
        self._viz_allowed = False
        try:
            self.lidar.disconnect()
        except Exception as exc:
            self.shared.update_status(lidar_error=str(exc), lidar_status="error", status_message=str(exc))
        try:
            self.camera_mgr.disconnect()
        except Exception as exc:
            self.shared.update_status(camera_error=str(exc), camera_status="error", status_message=str(exc))
        try:
            self.mav.disconnect()
        except Exception as exc:
            self.shared.update_status(mav_error=str(exc), mav_status="error", status_message=str(exc))
        self.avoidance.stop()
        self.shared.set_lidar_points([])
        self.shared.set_validated_lidar_points([])
        self.shared.set_camera_frame(None)
        self.shared.update_status(
            lidar_connected=False,
            lidar_active=False,
            mav_connected=False,
            camera_connected=False,
            mission_active=False,
            avoidance_enabled=False,
            obstacle_avoidance_on=False,
            visualization_running=False,
            show_lidar=False,
            show_camera=False,
            lidar_status="disconnected",
            mav_status="disconnected",
            camera_status="disconnected",
            status_message=tr("Devices disconnected"),
        )
        QtCore.QMetaObject.invokeMethod(self, "_on_disconnect_finished", QtCore.Qt.QueuedConnection)

    @QtCore.pyqtSlot()
    def _on_disconnect_finished(self) -> None:
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(True)
        self.btn_refresh_lidar.setEnabled(False)
        self.btn_start_camera_vis.setEnabled(True)
        self.btn_stop_camera_vis.setEnabled(False)
        self.camera_label.setPixmap(QtGui.QPixmap())
        self.camera_label.setText(tr("Camera disconnected"))
        self.lidar_view.overlay.clear()
        self.lidar_view.update()

    def _refresh_lidar(self) -> None:
        """Clear buffers and restart LiDAR stream."""
        if not self._viz_allowed:
            self.shared.update_status(status_message=tr("Visualization locked — press START"))
            return
        if not self.shared.lidar_connected:
            connected = self.lidar.connect()
            if not connected:
                self.shared.update_status(status_message=tr("LiDAR not connected"))
                return
        try:
            self.lidar.stop_stream(keep_connection=True)
        except Exception:
            pass
        self.shared.set_lidar_points([])
        self.shared.set_validated_lidar_points([])
        self.shared.clear_lidar_ready()
        started = self.lidar.start_stream()
        if started:
            self.shared.update_status(lidar_status="connected", show_lidar=True, status_message=tr("LiDAR refreshed"))
        else:
            self.shared.update_status(status_message=tr("LiDAR refresh failed"), show_lidar=False)
        self.lidar_view.overlay.clear()
        self.lidar_view.update()

    def _start_camera_visualization(self) -> None:
        if not self._viz_allowed:
            self.shared.update_status(status_message=tr("Visualization locked — press START"))
            return
        device = self._camera_device_from_settings()
        if not device or device.upper() == "OFF":
            self.shared.update_status(
                camera_status="not_connected",
                show_camera=False,
                camera_connected=False,
                visualization_running=self.shared.show_lidar,
                status_message=tr("Select camera device in Settings"),
            )
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Camera feed is off"))
            return
        self.shared.update_status(show_camera=True)
        self.settings.set("show_camera", True)
        self._start_camera_from_settings()
        self.shared.update_status(visualization_running=True)

    def _stop_camera_visualization(self) -> None:
        self.camera_mgr.stop(keep_flags=True)
        self.shared.set_camera_frame(None)
        self.shared.update_status(
            show_camera=False,
            visualization_running=self.shared.show_lidar,
            status_message=tr("Camera visualization stopped"),
        )
        self.settings.set("show_camera", False)
        self.camera_label.setPixmap(QtGui.QPixmap())
        self.camera_label.setText(tr("Camera visualization stopped (device remains connected)"))

    def _update_camera_view(self) -> None:
        if not self._viz_allowed:
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Visualization locked — press START"))
            return
        if not self.shared.show_camera:
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Camera visualization stopped"))
            return
        device = self._camera_device_from_settings()
        if device.upper() == "OFF":
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Camera disabled in Settings"))
            return
        if self.shared.camera_status == "connecting":
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Opening camera ({device})...", device=device))
            return
        if not self.shared.camera_connected:
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Camera feed unavailable"))
            return
        frame = self.shared.get_camera_frame()
        if frame is None:
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Camera feed unavailable"))
            return
        try:
            height, width, _ = frame.shape
        except Exception:
            self.camera_label.setPixmap(QtGui.QPixmap())
            self.camera_label.setText(tr("Camera frame invalid"))
            return
        image = QtGui.QImage(frame.data, width, height, 3 * width, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(image).scaled(
            self.camera_label.width(),
            self.camera_label.height(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.camera_label.setPixmap(pixmap)
        self.camera_label.setText("")

    def _maybe_refresh_lidar_view(self) -> None:
        now = time.time()
        ready_at = self.shared.lidar_ready_at()
        if ready_at > 0 and now < ready_at:
            return
        if not self._viz_allowed:
            return
        if not self.shared.show_lidar:
            return
        if not self.lidar_view.isVisible():
            return
        _, lidar_hz = self.shared.get_lidar_config()
        interval = max(0.1, 1.0 / max(1.0, lidar_hz))
        if now - self._last_lidar_update >= interval:
            self.lidar_view.update()
            self._last_lidar_update = now

    def _refresh_status(self) -> None:
        s = self.shared
        self.status_labels["mav"].setText(f"{tr('MAVLink')} {status_dot(s.mav_connected)}")
        self.status_labels["lidar"].setText(f"{tr('LiDAR')} {status_dot(s.lidar_active)}")
        self.status_labels["mission"].setText(f"{tr('Mission')} {status_dot(s.mission_active)}")
        self.status_labels["avoidance"].setText(f"{tr('Avoidance')} {status_dot(s.avoidance_enabled)}")
        self.device_status_labels["lidar"].setText(status_text(tr("LiDAR"), s.lidar_status))
        self.device_status_labels["mav"].setText(status_text(tr("MAVLink"), s.mav_status))
        self.device_status_labels["camera"].setText(status_text(tr("Camera"), s.camera_status))
        self.device_status_labels["lidar"].setToolTip(s.lidar_error or "")
        self.device_status_labels["mav"].setToolTip(s.mav_error or "")
        self.device_status_labels["camera"].setToolTip(s.camera_error or "")
        self.btn_refresh_lidar.setEnabled(self._viz_allowed and s.lidar_connected)
        self.btn_start_camera_vis.setEnabled(self._viz_allowed and not s.show_camera)
        self.btn_stop_camera_vis.setEnabled(self._viz_allowed and s.show_camera)
        self.btn_disconnect.setEnabled(
            s.lidar_connected
            or s.mav_connected
            or s.camera_connected
            or s.lidar_active
            or s.show_camera
            or s.show_lidar
        )
        self.message_label.setText(s.status_message)
        self._maybe_refresh_lidar_view()
        self._update_camera_view()

    # Keyboard controls ---------------------------------------------------------
    def keyPressEvent(self, event: QtGui.QKeyEvent) -> None:  # noqa: N802
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QtGui.QKeyEvent) -> None:  # noqa: N802
        super().keyReleaseEvent(event)


def run_gui(
    shared: SharedData,
    settings: Settings,
    mav: MavlinkComm,
    avoidance: AvoidanceController,
    motion: MotionControl,
    camera_mgr: CameraManager,
    lidar: LidarCore,
) -> None:
    app = QtWidgets.QApplication([])
    window = MainWindow(shared, settings, mav, avoidance, motion, camera_mgr, lidar)
    screen = QtWidgets.QApplication.primaryScreen().geometry()
    window.resize(int(screen.width() * 0.90), int(screen.height() * 0.90))
    window.move(int(screen.width() * 0.05), int(screen.height() * 0.05))
    window.show()
    app.exec_()













