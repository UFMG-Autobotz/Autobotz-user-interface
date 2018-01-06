#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore
import sys

from MainWindow import MainWindow

# --------------------- #

def main():
    app = QtGui.QApplication(sys.argv)

    # quit if a config file is not sent
    if len(sys.argv) <= 1:
        print "Error: Invalid number of arguments!"
        quit()

    w = MainWindow(sys.argv[1])
    w.show()

    sys.exit(app.exec_())

# --------------------- #

if __name__ == '__main__':
    main()
