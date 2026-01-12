from __future__ import annotations

import cv2
from PyQt5 import QtCore

from utils.device_scan import RASPI_CAM_LABEL


class CameraWorker(QtCore.QObject):
    """Background capture worker that emits frames via a Qt signal."""

    frame_ready = QtCore.pyqtSignal(object)
    status = QtCore.pyqtSignal(bool, str)
    finished = QtCore.pyqtSignal()

    def __init__(self, device_label: str) -> None:
        super().__init__()
        self.device_label = device_label
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def _capture_source(self):
        if self.device_label == RASPI_CAM_LABEL:
            return 0
        return self.device_label

    def run(self) -> None:
        cap = cv2.VideoCapture(self._capture_source())
        if not cap.isOpened():
            self.status.emit(False, f"Failed to open camera: {self.device_label}")
            self.finished.emit()
            return

        self.status.emit(True, f"Streaming {self.device_label}")
        while not self._stop:
            ok, frame = cap.read()
            if not ok:
                self.status.emit(False, "Camera frame read failed")
                QtCore.QThread.msleep(100)
                continue
            self.frame_ready.emit(frame)
            QtCore.QThread.msleep(15)

        cap.release()
        self.status.emit(True, "Camera stream stopped")
        self.finished.emit()


class CameraHandler(QtCore.QObject):
    """Thin wrapper around OpenCV capture + Qt threads."""

    frame_ready = QtCore.pyqtSignal(object)
    status_changed = QtCore.pyqtSignal(bool, str)

    def __init__(self) -> None:
        super().__init__()
        self.device_label: str | None = None
        self.connected = False
        self._streaming = False
        self._thread: QtCore.QThread | None = None
        self._worker: CameraWorker | None = None

    def _capture_source(self):
        if self.device_label == RASPI_CAM_LABEL:
            return 0
        return self.device_label

    def connect_device(self, device_label: str) -> bool:
        """Validate that the selected device can be opened."""
        self.device_label = device_label
        cap = cv2.VideoCapture(self._capture_source())
        self.connected = cap.isOpened()
        cap.release()

        if self.connected:
            self.status_changed.emit(True, f"Connected to {device_label}")
        else:
            self.status_changed.emit(False, f"Failed to open {device_label}")
        return self.connected

    def start_stream(self) -> None:
        if not self.device_label:
            self.status_changed.emit(False, "No camera selected")
            return
        if self._streaming:
            return
        if not self.connected and not self.connect_device(self.device_label):
            return

        self._thread = QtCore.QThread()
        self._worker = CameraWorker(self.device_label)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.frame_ready.connect(self.frame_ready)
        self._worker.status.connect(self.status_changed)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._on_thread_finished)

        self._streaming = True
        self._thread.start()

    def _on_thread_finished(self) -> None:
        self._thread = None
        self._worker = None
        self._streaming = False

    def stop_stream(self) -> None:
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit()
            self._thread.wait()
        if self.connected and self.device_label:
            self.status_changed.emit(True, f"Connected to {self.device_label} (stopped)")

    def disconnect(self) -> None:
        self.stop_stream()
        self.connected = False
        self.device_label = None
        self.status_changed.emit(False, "Camera disconnected")

    def is_streaming(self) -> bool:
        return self._streaming

