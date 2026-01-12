"""Automatic dependency installer for Raspberry Pi OS 2025 (Bookworm, Python 3.11+)."""

from __future__ import annotations

import ast
import importlib
import os
import platform
import shutil
import subprocess
import sys
import threading
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Set

PROJECT_ROOT = Path(__file__).resolve().parent
LOG_FILE = PROJECT_ROOT / "install_log.txt"


class Color:
    RESET = "\033[0m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BLUE = "\033[94m"
    BOLD = "\033[1m"


def log_line(msg: str, level: str = "info", color: str | None = None) -> None:
    """Print colored output and append to the log file."""
    prefix = level.upper()
    text = f"[{prefix}] {msg}"
    if color is None:
        color = {
            "ok": Color.GREEN,
            "info": Color.BLUE,
            "warn": Color.YELLOW,
            "error": Color.RED,
            "stream": Color.BLUE,
        }.get(level, Color.BLUE)
    print(f"{color}{text}{Color.RESET}")
    LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    with LOG_FILE.open("a", encoding="utf-8") as fh:
        fh.write(text + "\n")


def detect_environment() -> Dict[str, object]:
    os_release: Dict[str, str] = {}
    try:
        os_release = platform.freedesktop_os_release()  # type: ignore[attr-defined]
    except Exception:
        try:
            with open("/etc/os-release", "r", encoding="utf-8") as f:
                for line in f:
                    if "=" in line:
                        k, v = line.strip().split("=", 1)
                        os_release[k] = v.strip().strip('"')
        except Exception:
            os_release = {}
    arch_raw = platform.machine().lower()
    arch = "ARM64 (aarch64)" if "aarch64" in arch_raw or "arm64" in arch_raw else (
        "ARMv7/armhf" if "armv7" in arch_raw or "armv6" in arch_raw else arch_raw
    )
    pretty = os_release.get("PRETTY_NAME", os_release.get("NAME", "Unknown OS"))
    codename = os_release.get("VERSION_CODENAME", "")
    is_pi = False
    try:
        model_path = Path("/proc/device-tree/model")
        if model_path.exists():
            is_pi = "raspberry pi" in model_path.read_text(encoding="utf-8").lower()
    except Exception:
        is_pi = False
    return {
        "pretty": pretty,
        "codename": codename,
        "arch": arch,
        "python": platform.python_version(),
        "is_pi": is_pi,
    }


def print_environment(env: Dict[str, object]) -> None:
    log_line(f"OS: {env.get('pretty')} (codename: {env.get('codename') or 'unknown'})", "info", Color.GREEN)
    log_line(f"Python: {env.get('python')}", "info", Color.GREEN)
    log_line(f"Architecture: {env.get('arch')}", "info", Color.GREEN)
    log_line(
        f"Raspberry Pi detected: {env.get('is_pi')}",
        "info",
        Color.GREEN if env.get("is_pi") else Color.YELLOW,
    )


def discover_external_imports(root: Path) -> Set[str]:
    modules: Set[str] = set()
    local_modules: Set[str] = {p.stem for p in root.rglob("*.py")}
    local_modules.update({p.parent.name for p in root.rglob("__init__.py")})
    stdlib: Set[str] = set(getattr(sys, "stdlib_module_names", set()))
    for path in root.rglob("*.py"):
        if path.name == Path(__file__).name:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except Exception as exc:
            log_line(f"Skipping {path.name}: {exc}", "warn", Color.YELLOW)
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for name in node.names:
                    modules.add(name.name.split(".")[0])
            elif isinstance(node, ast.ImportFrom) and node.module:
                modules.add(node.module.split(".")[0])
    external = {
        m
        for m in modules
        if m not in stdlib and m not in local_modules and m not in {"__future__", ""}
    }
    return external


PACKAGE_RULES: Dict[str, Dict[str, object]] = {
    "PyQt5": {
        "installer": "apt",
        "apt": ["python3-pyqt5"],
        "check": "PyQt5",
        "note": "PyQt5 GUI - install via apt on Raspberry Pi OS.",
    },
    "cv2": {
        "installer": "apt",
        "apt": ["python3-opencv", "libatlas3-base"],
        "check": "cv2",
        "note": "OpenCV should come from apt (opencv-python via pip is blocked).",
    },
    "pymavlink": {
        "installer": "pip",
        "pip": "pymavlink==2.4.39",
        "check": "pymavlink",
        "note": "MAVLink bindings pinned for Python 3.11.",
    },
    "serial": {
        "installer": "pip",
        "pip": "pyserial==3.5",
        "check": "serial",
        "note": "Serial port utilities.",
    },
    "rplidar": {
        "installer": "pip",
        "pip": "rplidar-roboticia==0.9.2",
        "check": "rplidar",
        "note": "RPLidar driver, pure Python wheel works on ARM.",
    },
    "numpy": {
        "installer": "pip",
        "pip": "numpy==1.26.4",
        "check": "numpy",
        "pip_args": ["--only-binary=:all:"],
        "note": "Use ARM wheels only; avoid source builds on Pi.",
    },
    "scipy": {
        "installer": "apt",
        "apt": ["python3-scipy"],
        "check": "scipy",
        "note": "Install via apt to skip long builds on ARM.",
    },
    "matplotlib": {
        "installer": "apt",
        "apt": ["python3-matplotlib"],
        "check": "matplotlib",
        "note": "Prefer apt on Bookworm; fall back to pip if apt missing.",
        "fallback_pip": "matplotlib==3.8.2",
    },
    "picamera2": {
        "installer": "apt",
        "apt": ["python3-picamera2", "libcamera-dev"],
        "check": "picamera2",
        "note": "Camera stack shipped via apt on Raspberry Pi OS.",
    },
}


def resolve_requirements(modules: Iterable[str]) -> List[Dict[str, object]]:
    requirements: List[Dict[str, object]] = []
    for mod in sorted(modules):
        rule = PACKAGE_RULES.get(mod)
        if not rule:
            log_line(f"No install rule for detected module '{mod}'. Skipping.", "warn", Color.YELLOW)
            continue
        requirements.append({"module": mod, **rule})
    return requirements


def already_installed(module_name: str) -> bool:
    try:
        importlib.import_module(module_name)
        return True
    except Exception:
        return False


def needs_sudo() -> bool:
    if os.name == "nt":
        return False
    if hasattr(os, "geteuid"):
        try:
            return os.geteuid() != 0  # type: ignore[attr-defined]
        except Exception:
            return True
    return True


def stream_pipe(pipe, color: str) -> None:
    for line in iter(pipe.readline, ""):
        clean = line.rstrip()
        if clean:
            log_line(clean, "stream", color)
    pipe.close()


def run_command(cmd: List[str], env: Dict[str, str] | None = None) -> int:
    log_line("Running: " + " ".join(cmd), "info", Color.BLUE)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    threads = [
        threading.Thread(target=stream_pipe, args=(proc.stdout, Color.GREEN), daemon=True),
        threading.Thread(target=stream_pipe, args=(proc.stderr, Color.RED), daemon=True),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    proc.wait()
    return proc.returncode


def apt_update_if_needed(use_sudo: bool, apt_env: Dict[str, str]) -> None:
    cmd = (["sudo"] if use_sudo else []) + ["apt-get", "update"]
    run_command(cmd, env=apt_env)


def install_requirement(req: Dict[str, object], use_sudo: bool, pip_env: Dict[str, str], apt_env: Dict[str, str]) -> None:
    check_name = str(req.get("check", req["module"]))
    if already_installed(check_name):
        log_line(f"{check_name} already installed; skipping.", "warn", Color.YELLOW)
        return
    installer = req.get("installer")
    if installer == "apt":
        apt_pkgs = req.get("apt", [])
        if not apt_pkgs:
            log_line(f"No apt packages listed for {req['module']}.", "warn", Color.YELLOW)
            return
        cmd = (["sudo"] if use_sudo else []) + ["apt-get", "install", "-y"]
        cmd.extend([str(p) for p in apt_pkgs])
        attempts = 2
        for attempt in range(1, attempts + 1):
            code = run_command(cmd, env=apt_env)
            if code == 0:
                log_line(f"{req['module']} installed via apt.", "ok", Color.GREEN)
                break
            if attempt < attempts:
                log_line(f"{req['module']} install failed (attempt {attempt}); retrying.", "warn", Color.YELLOW)
            else:
                log_line(f"{req['module']} install failed after retry.", "error", Color.RED)
    elif installer == "pip":
        package_spec = str(req.get("pip", req["module"]))
        pip_args = req.get("pip_args", [])
        cmd = [sys.executable, "-m", "pip", "install", "--upgrade"]
        if pip_args:
            cmd.extend([str(a) for a in pip_args])
        cmd.append(package_spec)
        attempts = 2
        code = -1
        for attempt in range(1, attempts + 1):
            code = run_command(cmd, env=pip_env)
            if code == 0:
                log_line(f"{req['module']} installed via pip.", "ok", Color.GREEN)
                break
            if attempt < attempts:
                log_line(f"{req['module']} install failed (attempt {attempt}); retrying.", "warn", Color.YELLOW)
            else:
                log_line(f"{req['module']} install failed after retry.", "error", Color.RED)
        fallback = req.get("fallback_pip")
        if code != 0 and fallback:
            log_line(f"Attempting fallback pip package for {req['module']}: {fallback}", "warn", Color.YELLOW)
            cmd = [sys.executable, "-m", "pip", "install", "--upgrade", str(fallback)]
            run_command(cmd, env=pip_env)
    else:
        log_line(f"Unknown installer for {req['module']}", "warn", Color.YELLOW)


def main() -> None:
    LOG_FILE.write_text(f"Install run started {datetime.now().isoformat()}\n", encoding="utf-8")
    env = detect_environment()
    print_environment(env)
    if not env.get("is_pi"):
        log_line(
            "This script targets Raspberry Pi OS Bookworm; running on a different platform may fail.",
            "warn",
            Color.YELLOW,
        )

    modules = discover_external_imports(PROJECT_ROOT)
    if modules:
        log_line("External modules detected: " + ", ".join(sorted(modules)), "info", Color.BLUE)
    else:
        log_line("No external modules detected.", "info", Color.GREEN)
    requirements = resolve_requirements(modules)
    if not requirements:
        log_line("Nothing to install.", "ok", Color.GREEN)
        return

    apt_required = any(r.get("installer") == "apt" for r in requirements)
    apt_available = shutil.which("apt-get") is not None
    if apt_required and not apt_available:
        log_line("apt-get not found. Apt packages cannot be installed.", "error", Color.RED)
        requirements = [r for r in requirements if r.get("installer") != "apt"]

    pip_env = os.environ.copy()
    pip_env.setdefault("PIP_DISABLE_PIP_VERSION_CHECK", "1")
    if str(env.get("arch", "")).lower().startswith("arm") or env.get("is_pi"):
        pip_env.setdefault("PIP_EXTRA_INDEX_URL", "https://www.piwheels.org/simple")
        pip_env.setdefault("PIP_PREFER_BINARY", "1")
    apt_env = os.environ.copy()
    apt_env.setdefault("DEBIAN_FRONTEND", "noninteractive")

    use_sudo = needs_sudo()
    if apt_required and apt_available:
        apt_update_if_needed(use_sudo, apt_env)

    for req in requirements:
        install_requirement(req, use_sudo, pip_env, apt_env)

    log_line("Installation routine completed.", "ok", Color.GREEN)


if __name__ == "__main__":
    main()
