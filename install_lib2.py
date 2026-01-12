#!/usr/bin/env python3
import subprocess
import sys
import os
import time
import platform

PRINT_OK = lambda msg: print(f"\033[92m[SUCCESS]\033[0m {msg}")
PRINT_WARN = lambda msg: print(f"\033[93m[WARNING]\033[0m {msg}")
PRINT_ERR = lambda msg: print(f"\033[91m[ERROR]\033[0m {msg}")
PRINT_INFO = lambda msg: print(f"\033[96m[INFO]\033[0m {msg}")

REQUIRED_PIP_LIBS = [
    ("pyserial", "3.5"),
    ("pymavlink", "2.4.41"),
    ("numpy", "1.26.4"),
    ("PyQt5", "5.15.10"),
    ("pyqtgraph", "0.13.3"),
    ("rplidar", "0.9.2"),
]

APT_PACKAGES = [
    "python3-pyqt5",
    "python3-opencv",
    "python3-serial",
    "python3-smbus",
    "v4l-utils",
    "libatlas-base-dev",
]


def run(cmd: str):
    """Run shell command and show readable status."""
    PRINT_INFO(f"Running: {cmd}")
    result = subprocess.run(cmd, shell=True)
    if result.returncode == 0:
        PRINT_OK(f"Completed: {cmd}")
    else:
        PRINT_ERR(f"Failed: {cmd}")
    return result.returncode == 0


def install_apt_packages():
    PRINT_INFO("Updating system…")
    run("sudo apt update")

    PRINT_INFO("Installing system packages…")
    for pkg in APT_PACKAGES:
        PRINT_INFO(f"Trying to install: {pkg}")
        if not run(f"sudo apt install -y {pkg}"):
            PRINT_WARN(f"Failed to install {pkg} — continuing…")


def install_pip_packages():
    PRINT_INFO("\nInstalling Python pip libraries…\n")

    # ensure pip exists
    run("sudo apt install -y python3-pip")

    for pkg, ver in REQUIRED_PIP_LIBS:
        PRINT_INFO(f"Installing {pkg}=={ver}...")
        if not run(f"sudo pip3 install {pkg}=={ver}"):
            PRINT_WARN(f"Could NOT install {pkg} {ver}. Will retry without version...")
            run(f"sudo pip3 install {pkg}")


def check_system():
    PRINT_INFO("Checking OS and Python version...")
    PRINT_INFO(f"OS: {platform.platform()}")
    PRINT_INFO(f"Python: {sys.version}")


def main():
    PRINT_INFO("=== AUTO INSTALLER FOR OBSTACLE MISSION PROJECT ===")
    time.sleep(1)

    check_system()

    PRINT_INFO("\n=== STEP 1: APT packages ===")
    install_apt_packages()

    PRINT_INFO("\n=== STEP 2: PIP libraries ===")
    install_pip_packages()

    PRINT_OK("\nInstallation completed.")
    PRINT_INFO("You can now run the obstacle mission software.\n")


if __name__ == "__main__":
    main()
