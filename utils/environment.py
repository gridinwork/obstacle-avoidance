import platform


def is_windows() -> bool:
    return platform.system().lower().startswith("win")


def is_raspberry_pi() -> bool:
    """Best-effort detection for Raspberry Pi environment."""
    try:
        with open("/proc/cpuinfo", "r", encoding="utf-8") as f:
            return "Raspberry Pi" in f.read()
    except Exception:
        return False

