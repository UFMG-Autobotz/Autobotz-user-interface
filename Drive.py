#!/usr/bin/python
# -*- coding: utf-8 -*-

from PyQt4 import Qt
from PyQt4 import QtGui
from PyQt4 import QtCore
import sys

import rospy

import yaml
import pygame

from lib.keyboard import Keyboard
from lib.gamepad import Gamepad
from lib.general_utils import get_yaml_dict

# --------------------- #

class calcVelocityGamepad(QtCore.QThread):
    def __init__(self,subwindow):
        QtCore.QThread.__init__(self)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(41) #Assim, a velocidade é calculada ~24x por segundo (mesmo fps de filmes em cinemas)
        self.subwindow = subwindow
    def run(self):
        self.subwindow.calcVelocity()

# --------------------- #

class Drive_Window(QtGui.QWidget):
    def __init__(self, config_file, parent = None):
        super(Drive_Window, self).__init__()
        self.calcVelocityGamepadThreads = []
        self.initUI(get_yaml_dict(config_file))

    def initUI(self, data):
        self.subWindows = []
        gamepadID = 0
        for robot in data['Robot']:
            if (robot['KeyConfig'] == 'gamepad'):
                gamepad_subwindow = Gamepad(self, robot, gamepadID)
                self.subWindows.append(gamepad_subwindow)
                self.calcVelocityGamepadThreads.append(calcVelocityGamepad(gamepad_subwindow))
                gamepadID += 1
            else:
                self.subWindows.append(Keyboard(self, robot))

        self.layout=QtGui.QVBoxLayout(self)
        for subwindow in self.subWindows:
            self.layout.addWidget(subwindow)

        self.resize(300, len(data['Robot'])*130)

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

    def mousePressEvent(self, event):
        self.setFocus();

# --------------------- #

if __name__ == '__main__':
    rospy.init_node('Drive', anonymous=True)
    app = QtGui.QApplication(sys.argv)

    # use defaut config if not sent
    if len(sys.argv) <= 1:
        config_file = './config/Drive/VSS_1on1_keyboard.yaml'
    elif len(sys.argv) <= 2:
        config_file = './config/Drive/' + sys.argv[1]
    else:
        config_file = sys.argv[2] + sys.argv[1]

    w = Drive_Window(config_file)
    w.setWindowTitle('Drive')
    w.show()

    print "Drive module from Autobotz User Interface running."

    sys.exit(app.exec_())
