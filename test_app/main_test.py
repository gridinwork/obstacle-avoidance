import sys

from PyQt5 import QtWidgets

from gui_test import TestMainWindow


def main() -> None:
    """Entry point for the standalone camera/LiDAR debug tool."""
    app = QtWidgets.QApplication(sys.argv)
    window = TestMainWindow()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

