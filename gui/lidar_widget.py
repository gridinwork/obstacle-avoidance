from __future__ import annotations

import math
import time
from typing import List

from PyQt5 import QtCore, QtGui, QtWidgets

from lidar_core import MAX_DIST_CM
from shared_data import Point, SharedData

from .lidar_status_overlay import LidarStatusOverlay
from .i18n import tr


class LidarWidget(QtWidgets.QWidget):
    """LiDAR visualization widget with optional text overlay."""

    MARKER_YELLOW_CM = 200  # 2.0 m visual marker
    MARKER_RED_CM = 100     # 1.0 m visual marker

    def __init__(self, shared: SharedData, show_overlay: bool = True) -> None:
        super().__init__()
        self.shared = shared
        self.show_overlay = show_overlay
        self.overlay = LidarStatusOverlay()
        self.setMinimumHeight(320)
        self.setFocusPolicy(QtCore.Qt.ClickFocus)
        self._last_draw_ts = 0.0

        # Cached paint context (updated every paintEvent)
        self._center = QtCore.QPointF()
        self._scale = 1.0
        self._boundaries: tuple[int, int, int] = self.shared.get_boundaries()
        self._points: List[Point] = []
        self._lidar_ready = False

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:  # noqa: N802
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        self._update_context()

        self.draw_background(painter)
        self.draw_pointcloud(painter)
        self.draw_boundaries(painter)
        self.draw_status_message(painter)

    # Paint helpers -------------------------------------------------------------
    def draw_background(self, painter: QtGui.QPainter) -> None:
        painter.fillRect(self.rect(), QtGui.QColor("#0d1117"))

    def draw_pointcloud(self, painter: QtGui.QPainter) -> None:
        painter.save()

        if not self.shared.show_lidar:
            painter.setPen(QtGui.QPen(QtGui.QColor("#9e9e9e")))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, tr("LiDAR visualization is off"))
            painter.restore()
            return

        if not self._lidar_ready:
            painter.setPen(QtGui.QPen(QtGui.QColor("#9e9e9e")))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, tr("LiDAR warming up..."))
            painter.restore()
            return

        if not self._points:
            painter.setPen(QtGui.QPen(QtGui.QColor("#9e9e9e")))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, tr("LiDAR data unavailable"))
            painter.restore()
            return

        painter.setPen(QtGui.QPen(QtGui.QColor("#64b5f6"), 3))
        groups = self._group_points(self._points)
        for group in groups:
            if len(group) > 2:
                path = QtGui.QPainterPath()
                first = self._to_point(group[0])
                path.moveTo(first)
                for p in group[1:]:
                    path.lineTo(self._to_point(p))
                painter.drawPath(path)

        painter.setPen(QtGui.QPen(QtGui.QColor("#90caf9"), 4))
        for p in self._points:
            painter.drawPoint(self._to_point(p))

        painter.restore()

    def draw_boundaries(self, painter: QtGui.QPainter) -> None:
        painter.save()
        blue_cm, green_cm, green_limit_cm = self._boundaries
        center = self._center
        scale = self._scale
        ground_y = center.y()
        top_y = max(10.0, center.y() - green_limit_cm * scale)

        # Blue (inner) boundaries
        painter.setPen(QtGui.QPen(QtGui.QColor("#64b5f6"), 2, QtCore.Qt.SolidLine))
        for offset_cm in (-blue_cm, blue_cm):
            x = center.x() + offset_cm * scale
            painter.drawLine(QtCore.QPointF(x, ground_y), QtCore.QPointF(x, top_y))

        # Green (outer) boundaries
        painter.setPen(QtGui.QPen(QtGui.QColor("#66bb6a"), 2, QtCore.Qt.DashLine))
        for offset_cm in (-green_cm, green_cm):
            x = center.x() + offset_cm * scale
            painter.drawLine(QtCore.QPointF(x, ground_y), QtCore.QPointF(x, top_y))
        painter.drawLine(
            QtCore.QPointF(center.x() - green_cm * scale, top_y),
            QtCore.QPointF(center.x() + green_cm * scale, top_y),
        )

        # Distance markers (horizontal ticks) for limit thresholds.
        tick_len = max(4.0, 8.0 * scale)
        for dist_cm, color in [
            (min(self.MARKER_YELLOW_CM, green_limit_cm), QtGui.QColor("#ffb300")),
            (min(self.MARKER_RED_CM, green_limit_cm), QtGui.QColor("#ff5252")),
        ]:
            y = center.y() - dist_cm * scale
            painter.setPen(QtGui.QPen(color, 1, QtCore.Qt.DotLine))
            painter.drawLine(
                QtCore.QPointF(center.x() - green_cm * scale, y),
                QtCore.QPointF(center.x() + green_cm * scale, y),
            )
            painter.drawLine(
                QtCore.QPointF(center.x(), y),
                QtCore.QPointF(center.x(), y + tick_len),
            )

        painter.restore()

    def draw_status_message(self, painter: QtGui.QPainter) -> None:
        """Render the latest avoidance status at the top-center of the widget."""
        if not self.show_overlay:
            return
        self.overlay.clear_if_expired()
        lines = []
        overlay_text = (self.overlay.message or "").strip()
        if overlay_text:
            lines.append(overlay_text)
        lines.extend(self._debug_lines())
        if not lines:
            return
        msg = "\n".join(lines)
        painter.save()
        rect = self.rect().adjusted(8, 6, -8, -6)
        align = QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter

        font = QtGui.QFont("Arial", 16, QtGui.QFont.DemiBold)
        painter.setFont(font)

        # Shadow for readability
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 160)))
        painter.drawText(rect.translated(1, 1), align, msg)

        # Foreground text (slightly translucent)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 255)))
        painter.drawText(rect, align, msg)
        painter.restore()

    # Context helpers -----------------------------------------------------------
    def _update_context(self) -> None:
        self._last_draw_ts = time.time()
        width = float(self.width())
        height = float(self.height())
        self._center = QtCore.QPointF(width / 2, height * 0.95)
        self._boundaries = self.shared.get_boundaries()
        self._lidar_ready = self.shared.lidar_ready()

        pts_raw = self.shared.get_lidar_points() if (self.shared.show_lidar and self._lidar_ready) else []
        self._points = self._filter_for_display(pts_raw, self._boundaries)
        max_range_cm = self._max_display_range(self._points, self._boundaries[2])
        self._scale = self._scale_factor(width, height, max_range_cm, self._boundaries)

    def _group_points(self, pts: List[Point]) -> List[List[Point]]:
        pts_sorted = sorted(pts, key=lambda p: p[0])
        groups: List[List[Point]] = []
        group: List[Point] = []
        for ang, dist in pts_sorted:
            if not group:
                group.append((ang, dist))
                continue
            prev_ang, prev_dist = group[-1]
            if abs(ang - prev_ang) < 6 and abs(dist - prev_dist) < 12:
                group.append((ang, dist))
            else:
                groups.append(group)
                group = [(ang, dist)]
        if group:
            groups.append(group)
        return groups

    def _to_point(self, point: Point) -> QtCore.QPointF:
        ang, dist = point
        x = self._center.x() + dist * self._scale * math.sin(math.radians(ang))
        y = self._center.y() - dist * self._scale * math.cos(math.radians(ang))
        return QtCore.QPointF(x, y)

    def _scale_factor(
        self, width: float, height: float, max_range_cm: float, boundaries: tuple[int, int, int]
    ) -> float:
        blue_cm, green_cm, green_limit_cm = boundaries
        usable_height = max(50.0, height * 0.9)
        usable_width = max(50.0, width * 0.9)
        vertical_scale = usable_height / max(1.0, max_range_cm)
        horizontal_scale = usable_width / max(1.0, 2 * max(blue_cm, green_cm, 1))
        return min(vertical_scale, horizontal_scale)

    def _filter_for_display(self, pts: List[Point], boundaries: tuple[int, int, int]) -> List[Point]:
        """Points are clipped in lidar_core; keep a lightweight copy for painting."""
        return list(pts)

    def _max_display_range(self, pts: List[Point], green_limit_cm: float) -> float:
        """Choose a stable max radius so guide lines match the current scale."""
        distances = [min(dist, MAX_DIST_CM, green_limit_cm) for _, dist in pts]
        distances.extend(
            [
                min(self.MARKER_YELLOW_CM, MAX_DIST_CM, green_limit_cm),
                min(self.MARKER_RED_CM, MAX_DIST_CM, green_limit_cm),
                green_limit_cm,
            ]
        )
        return max(d for d in distances if d > 0)

    def _debug_lines(self) -> List[str]:
        if not self.shared.show_lidar or not self._lidar_ready:
            return []
        size = int(getattr(self.shared, "lidar_cluster_size", 0) or 0)
        valid = bool(getattr(self.shared, "lidar_cluster_valid", False))
        frames_stable = int(getattr(self.shared, "lidar_frames_stable", 0) or 0)
        frames_required = int(getattr(self.shared, "lidar_frames_required", 0) or 0)
        if frames_required <= 0:
            frames_required = max(1, int(self.shared.lidar_read_hz or 1))
        reason = str(getattr(self.shared, "lidar_noise_reason", "") or "")

        lines = [tr("LiDAR cluster size: {size}", size=size)]
        if valid:
            lines.append(tr("Obstacle cluster validated"))
        elif reason:
            lines.append(reason)
        else:
            lines.append(tr("Noise filtered — insufficient points"))
        lines.append(tr("Frames stable: {stable} / {required} (>=1 sec)", stable=frames_stable, required=max(1, frames_required)))
        return lines
