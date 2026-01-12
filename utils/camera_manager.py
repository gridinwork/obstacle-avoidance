"""Camera capture for Pi/USB cameras using libcamera or V4L2."""

from __future__ import annotations

import threading
import time

from shared_data import SharedData


class CameraManager:
    def __init__(self, shared: SharedData, log_fn=print) -> None:
        self.shared = shared
        self.log = log_fn
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._capture = None
        self._picam = None
        self._device: str = "OFF"
        self._backend: str = "auto"
        self._connected: bool = False

    # Connection helpers -----------------------------------------------------
    def probe(self, device: str, backend: str = "auto") -> bool:
        """Lightweight connection test without starting the streaming thread."""
        device = (device or "OFF").strip()
        self._device = device
        self._backend = backend or "auto"
        if device.upper() == "OFF":
            self._connected = False
            self.shared.update_status(
                camera_connected=False,
                camera_status="not_connected",
                camera_device="OFF",
                camera_mode="OFF",
                camera_source="OFF",
            )
            return False

        ok = False
        if device.upper() == "PI_CAMERA":
            ok = self._probe_picamera2()
        else:
            ok = self._probe_opencv(device, backend=self._backend)

        status = "connected" if ok else "error"
        self._connected = ok
        self.shared.update_status(
            camera_connected=ok,
            camera_status=status,
            camera_device=device,
            camera_mode=device,
            camera_source=device,
            camera_error="" if ok else self.shared.camera_error or "Camera open failed",
        )
        return ok

    def start(self, device: str, backend: str = "auto") -> bool:
        """Start camera capture on a background thread."""
        self.stop(keep_flags=True)
        self.shared.set_camera_frame(None)
        device = (device or "OFF").strip()
        self._device = device
        self._backend = backend or "auto"
        if device.upper() == "OFF":
            self._connected = False
            self.shared.update_status(
                camera_mode="OFF",
                camera_source="OFF",
                camera_device="OFF",
                camera_connected=False,
                camera_status="not_connected",
            )
            return True
        self._connected = False
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, args=(device,), daemon=True)
        self._thread.start()
        self.shared.update_status(
            camera_mode=device,
            camera_source=device,
            camera_device=device,
            camera_connected=False,
            camera_status="connecting",
            show_camera=True,
            camera_error="",
        )
        self.log(f"Camera starting: {device} (backend={self._backend})")
        return True

    def stop(self, keep_flags: bool = False) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
        self._capture = None
        if self._picam is not None:
            try:
                self._picam.stop()
            except Exception:
                pass
            self._picam = None
        self.shared.set_camera_frame(None)
        if keep_flags:
            self.shared.update_status(
                camera_connected=self._connected,
                camera_status="connected" if self._connected else "not_connected",
            )
        else:
            self._connected = False
            self.shared.update_status(
                camera_mode="OFF",
                camera_source="OFF",
                camera_device="OFF",
                show_camera=False,
                camera_connected=False,
                camera_status="not_connected",
            )
        self.log("Camera stopped")

    def disconnect(self) -> None:
        """Stop and mark camera as fully disconnected."""
        self.stop(keep_flags=False)
        self.shared.update_status(
            camera_connected=False,
            camera_status="disconnected",
            camera_error="",
            status_message="Camera disconnected",
        )

    def _loop(self, device: str) -> None:
        try:
            import cv2
        except Exception:
            self.log("OpenCV not installed; camera disabled")
            self._connected = False
            self.shared.update_status(
                camera_mode="OFF",
                camera_source="OFF",
                camera_device="OFF",
                status_message="OpenCV missing; camera disabled",
                camera_connected=False,
                camera_status="error",
                camera_error="OpenCV missing",
            )
            return

        backend = (self._backend or "auto").lower()
        dev_upper = device.upper()

        # Prefer libcamera on Pi if requested
        if dev_upper == "PI_CAMERA":
            if self._try_picamera2():
                return
            # Fallback to V4L2 index 0
            device_to_open: int | str = 0
        else:
            device_to_open = int(device) if str(device).isdigit() else device

        self._run_opencv_capture(cv2, device_to_open, backend)

    def _run_opencv_capture(self, cv2, device: int | str, backend: str) -> None:  # type: ignore[no-untyped-def]
        cap_backend = cv2.CAP_V4L2 if backend in {"auto", "v4l2"} and hasattr(cv2, "CAP_V4L2") else 0
        try:
            self._capture = cv2.VideoCapture(device, cap_backend) if cap_backend else cv2.VideoCapture(device)
            if not self._capture.isOpened():
                msg = f"Camera {device} could not open"
                self.log(msg)
                self._connected = False
                self.shared.update_status(
                    camera_mode="OFF",
                    camera_source="OFF",
                    camera_device="OFF",
                    status_message=msg,
                    camera_connected=False,
                    camera_status="error",
                    camera_error=msg,
                )
                return
            self._connected = True
            self.shared.update_status(camera_connected=True, camera_status="connected", camera_error="")
            # Reduce CPU load on Pi by limiting resolution/frame rate.
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self._capture.set(cv2.CAP_PROP_FPS, 24)
            self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)

            while not self._stop_event.is_set() and self.shared.running:
                ret, frame = self._capture.read()
                if not ret:
                    time.sleep(0.1)
                    continue
                try:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    pass
                self.shared.set_camera_frame(frame)
                self.shared.update_status(camera_connected=True, camera_status="connected")
                time.sleep(0.01)
        finally:
            if self._capture is not None:
                try:
                    self._capture.release()
                except Exception:
                    pass
                self._capture = None

    def _probe_opencv(self, device: str, backend: str = "auto") -> bool:
        """Attempt to open a single frame to validate device availability."""
        try:
            import cv2
        except Exception as exc:
            self.log(f"OpenCV not available for probe: {exc}")
            self.shared.update_status(camera_error=str(exc), camera_status="error")
            return False

        cap_backend = cv2.CAP_V4L2 if backend.lower() in {"auto", "v4l2"} and hasattr(cv2, "CAP_V4L2") else 0
        device_to_open = int(device) if str(device).isdigit() else device
        cap = cv2.VideoCapture(device_to_open, cap_backend) if cap_backend else cv2.VideoCapture(device_to_open)
        try:
            if not cap.isOpened():
                return False
            ret, _ = cap.read()
            return bool(ret)
        finally:
            try:
                cap.release()
            except Exception:
                pass

    def _probe_picamera2(self) -> bool:
        """Check availability of Raspberry Pi CSI camera without long-running stream."""
        try:
            from picamera2 import Picamera2  # type: ignore
        except Exception as exc:
            self.log(f"Picamera2 not available for probe: {exc}")
            self.shared.update_status(camera_error=str(exc), camera_status="error")
            return False

        cam = None
        try:
            cam = Picamera2()
            config = cam.create_preview_configuration(main={"format": "RGB888", "size": (320, 240)})
            cam.configure(config)
            cam.start()
            _ = cam.capture_array("main")
            cam.stop()
            return True
        except Exception as exc:
            self.log(f"Pi Camera probe failed: {exc}")
            self.shared.update_status(camera_error=str(exc), camera_status="error")
            return False
        finally:
            try:
                if cam:
                    cam.stop()
            except Exception:
                pass

    def _try_picamera2(self) -> bool:
        """Attempt to stream from Raspberry Pi CSI camera via Picamera2."""
        try:
            from picamera2 import Picamera2  # type: ignore
        except Exception as exc:
            self.log(f"Picamera2 not available: {exc}")
            return False

        try:
            self._picam = Picamera2()
            config = self._picam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
            self._picam.configure(config)
            self._picam.start()
            self._connected = True
            self.shared.update_status(camera_connected=True, camera_status="connected", camera_error="")
            while not self._stop_event.is_set() and self.shared.running:
                frame = self._picam.capture_array("main")
                self.shared.set_camera_frame(frame.copy())
                time.sleep(0.01)
            return True
        except Exception as exc:
            self.log(f"Pi Camera capture failed: {exc}")
            self._connected = False
            self.shared.update_status(
                status_message=f"Pi Camera failed: {exc}",
                camera_device="OFF",
                camera_mode="OFF",
                camera_connected=False,
                camera_status="error",
                camera_error=str(exc),
            )
            return False
        finally:
            try:
                if self._picam:
                    self._picam.stop()
            except Exception:
                pass
            self._picam = None

