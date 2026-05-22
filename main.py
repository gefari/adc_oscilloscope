import sys
from PySide6 import QtWidgets
from Oscilloscope import Oscilloscope


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("ADS1263 Oscilloscope")
    win = Oscilloscope()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
