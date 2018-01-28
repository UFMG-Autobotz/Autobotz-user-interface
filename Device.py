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

class Device_Window(QtGui.QWidget):
    def __init__(self, config_file, parent = None):
        super(Device_Window, self).__init__()
        self.calcVelocityGamepadThreads = []
        self.initUI(self.loadConfig(config_file))

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

    def loadConfig(self, config_file):
        try:
            with open(config_file, 'r') as f:
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

    def mousePressEvent(self, event):
        self.setFocus();

# --------------------- #

if __name__ == '__main__':
    rospy.init_node('Device', anonymous=True)
    app = QtGui.QApplication(sys.argv)

    # use defaut config if not sent
    if len(sys.argv) <= 1:
        config_file = './config/Device/VSS_1on1_keyboard.yaml'
    else:
        config_file = sys.argv[1]

    w = Device_Window(config_file)
    w.setWindowTitle('Device')
    w.show()

    print "Device module from Autobotz User Interface running."

    sys.exit(app.exec_())
