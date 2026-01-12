from __future__ import annotations

import cv2
from PyQt5 import QtCore, QtGui, QtWidgets

from camera_test import CameraHandler
from lidar_test import LidarHandler
from utils import device_scan
from utils.device_scan import RASPI_CAM_LABEL


class LidarViewWidget(QtWidgets.QFrame):
    """Simple 2D LiDAR scatter plot."""

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.points: list[tuple[float, float]] = []
        self.max_range_mm = 3000.0
        self.setFrameStyle(QtWidgets.QFrame.Panel | QtWidgets.QFrame.Sunken)
        self.setMinimumSize(320, 240)

    def set_points(self, points: list[tuple[float, float]]) -> None:
        self.points = points
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor(10, 10, 10))
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        center = QtCore.QPointF(self.width() / 2, self.height() / 2)
        radius = min(self.width(), self.height()) * 0.45
        scale = radius / self.max_range_mm if self.max_range_mm else 1

        painter.setPen(QtGui.QPen(QtGui.QColor("#444"), 1))
        painter.drawEllipse(center, radius, radius)
        painter.drawLine(center.x(), 0, center.x(), self.height())
        painter.drawLine(0, center.y(), self.width(), center.y())

        painter.setPen(QtGui.QPen(QtGui.QColor("lime"), 3))
        for x, y in self.points:
            px = center.x() + (x * scale)
            py = center.y() - (y * scale)
            painter.drawPoint(QtCore.QPointF(px, py))

        painter.end()


class TestMainWindow(QtWidgets.QMainWindow):
    """Standalone GUI for camera/LiDAR debugging."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Test - Camera & LiDAR Debugger")

        self.camera_handler = CameraHandler()
        self.lidar_handler = LidarHandler()

        self.lidar_status = "Not Connected"
        self.camera_status = "Not Connected"
        self.lidar_ready = False
        self.camera_ready = False

        self._build_ui()
        self._wire_signals()
        self.refresh_devices()
        self.showFullScreen()  # Auto-fullscreen for Raspberry Pi

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()

        # Connection rows
        connection_grid = QtWidgets.QGridLayout()

        self.show_lidar_cb = QtWidgets.QCheckBox("Show LIDAR")
        self.show_lidar_cb.setChecked(True)
        self.lidar_combo = QtWidgets.QComboBox()
        self.lidar_combo.setEditable(True)
        self.lidar_connect_btn = QtWidgets.QPushButton("Connect Lidar")

        self.show_camera_cb = QtWidgets.QCheckBox("Show Camera")
        self.show_camera_cb.setChecked(True)
        self.camera_combo = QtWidgets.QComboBox()
        self.camera_combo.setEditable(True)
        self.camera_connect_btn = QtWidgets.QPushButton("Connect Cam")

        connection_grid.addWidget(self.show_lidar_cb, 0, 0)
        connection_grid.addWidget(self.lidar_combo, 0, 1)
        connection_grid.addWidget(self.lidar_connect_btn, 0, 2)

        connection_grid.addWidget(self.show_camera_cb, 1, 0)
        connection_grid.addWidget(self.camera_combo, 1, 1)
        connection_grid.addWidget(self.camera_connect_btn, 1, 2)

        layout.addLayout(connection_grid)

        # Status row
        self.status_label = QtWidgets.QLabel("STATUS: LIDAR: Not Connected | CAMERA: Not Connected")
        layout.addWidget(self.status_label)

        # Start/stop
        buttons = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("START STREAM")
        self.stop_btn = QtWidgets.QPushButton("STOP STREAM")
        buttons.addWidget(self.start_btn)
        buttons.addWidget(self.stop_btn)
        layout.addLayout(buttons)

        # Views
        views = QtWidgets.QHBoxLayout()
        self.lidar_view = LidarViewWidget()
        self.lidar_view.setEnabled(False)
        self.camera_view = QtWidgets.QLabel("Camera preview")
        self.camera_view.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_view.setMinimumSize(320, 240)
        self.camera_view.setStyleSheet("background-color: #111; color: #888;")
        self.camera_view.setEnabled(False)

        views.addWidget(self.lidar_view, stretch=1)
        views.addWidget(self.camera_view, stretch=1)
        layout.addLayout(views)

        central.setLayout(layout)
        self.setCentralWidget(central)

    def _wire_signals(self) -> None:
        self.start_btn.clicked.connect(self.start_streams)
        self.stop_btn.clicked.connect(self.stop_streams)
        self.lidar_connect_btn.clicked.connect(self.connect_lidar)
        self.camera_connect_btn.clicked.connect(self.connect_camera)
        self.show_lidar_cb.toggled.connect(self._update_view_enabled)
        self.show_camera_cb.toggled.connect(self._update_view_enabled)

        self.camera_handler.frame_ready.connect(self._update_camera_frame)
        self.camera_handler.status_changed.connect(self._on_camera_status)

        self.lidar_handler.points_ready.connect(self.lidar_view.set_points)
        self.lidar_handler.status_changed.connect(self._on_lidar_status)

    def refresh_devices(self) -> None:
        """Populate dropdowns with discovered hardware."""
        self.lidar_combo.clear()
        lidar_ports = device_scan.list_lidar_ports()
        self.lidar_combo.addItems(lidar_ports or [""])

        self.camera_combo.clear()
        for cam in device_scan.list_cameras():
            self.camera_combo.addItem(cam)

    def connect_lidar(self) -> None:
        port = self.lidar_combo.currentText().strip()
        if not port:
            self._on_lidar_status(False, "No port selected")
            return

        was_connected = self.lidar_handler.connect_device(port)
        if not was_connected and self.lidar_handler.simulated:
            QtWidgets.QMessageBox.warning(
                self,
                "LiDAR",
                "LiDAR connection failed. Switching to Simulation Mode.",
            )
        self._update_view_enabled()

    def connect_camera(self) -> None:
        device = self.camera_combo.currentText().strip()
        if not device:
            self._on_camera_status(False, "No camera selected")
            return

        if not self.camera_handler.connect_device(device):
            QtWidgets.QMessageBox.warning(
                self,
                "Camera",
                f"Failed to open camera: {device}",
            )
        self._update_view_enabled()

    def start_streams(self) -> None:
        if self.show_lidar_cb.isChecked():
            if not self.lidar_handler.port:
                self.connect_lidar()
            self.lidar_handler.start_stream()
        else:
            self.lidar_handler.stop_stream()

        if self.show_camera_cb.isChecked():
            if not self.camera_handler.device_label:
                self.connect_camera()
            self.camera_handler.start_stream()
        else:
            self.camera_handler.stop_stream()

    def stop_streams(self) -> None:
        self.camera_handler.stop_stream()
        self.lidar_handler.stop_stream()
        self._update_view_enabled()

    def _on_lidar_status(self, ready: bool, message: str) -> None:
        self.lidar_status = message
        self.lidar_ready = ready
        self._update_status_bar()
        self._update_view_enabled()

    def _on_camera_status(self, ready: bool, message: str) -> None:
        self.camera_status = message
        self.camera_ready = ready
        self._update_status_bar()
        self._update_view_enabled()

    def _update_status_bar(self) -> None:
        self.status_label.setText(
            f"STATUS: LIDAR: {self.lidar_status} | CAMERA: {self.camera_status}"
        )

    def _update_view_enabled(self) -> None:
        lidar_active = self.lidar_handler.is_streaming() and self.lidar_ready
        camera_active = self.camera_handler.is_streaming() and self.camera_ready
        self.lidar_view.setEnabled(self.show_lidar_cb.isChecked() and lidar_active)
        self.camera_view.setEnabled(self.show_camera_cb.isChecked() and camera_active)
        if not self.camera_view.isEnabled():
            self.camera_view.setPixmap(QtGui.QPixmap())

    def _update_camera_frame(self, frame) -> None:
        if not self.show_camera_cb.isChecked():
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        image = QtGui.QImage(
            rgb.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888
        )
        pixmap = QtGui.QPixmap.fromImage(image).scaled(
            self.camera_view.width(),
            self.camera_view.height(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.camera_view.setPixmap(pixmap)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802
        self.stop_streams()
        event.accept()

