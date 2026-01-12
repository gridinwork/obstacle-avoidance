from __future__ import annotations

import time


class LidarStatusOverlay:
    def __init__(self) -> None:
        self.message: str = ""
        self.timestamp: float = 0.0

    def set_message(self, msg: str) -> None:
        """Store a status message with a timestamp (no auto-expire)."""
        self.message = msg or ""
        self.timestamp = time.time()

    def clear_if_expired(self) -> None:
        """Messages persist until explicitly cleared or replaced."""
        return

    def clear(self) -> None:
        """Immediately hide the current message."""
        self.message = ""
        self.timestamp = 0.0

