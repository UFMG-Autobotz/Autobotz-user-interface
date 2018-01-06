from PyQt4 import QtGui
from PyQt4 import QtCore

import yaml

from SubWindow import SubWindow
from SubWindowGamepad import SubWindowGamepad

class MainWindow(QtGui.QWidget):
    def __init__(self, config):
        super(MainWindow, self).__init__()
        self.setWindowTitle('Keyboard Control')

        data = self.loadConfig(config)
        self.subWindows = [];
        for robot in data['Robot']:
            if (robot['KeyConfig'] == 'gamepad' or robot['KeyConfig'] == 'gamepad2'):
                print("Hellow")
                self.subWindows.append(SubWindowGamepad(self, robot))
            else:
                self.subWindows.append(SubWindow(self, robot))

        for idx, subwindow in enumerate(self.subWindows):

            subwindow.move(5, 5 + (subwindow.size().height()+25)*idx)

    def loadConfig(self, config):
        try:
            with open(config, 'r') as f:
                return yaml.load(f)
        except:
            print "Error: Invalid configuration file!"
            quit()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return

        for subwindow in self.subWindows:
            subwindow.keyPressEvent(event)

        event.accept()

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return

        for subwindow in self.subWindows:
            subwindow.keyReleaseEvent(event)
        event.accept()
