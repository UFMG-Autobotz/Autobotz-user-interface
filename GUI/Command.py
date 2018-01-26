#!/usr/bin/python
# -*- coding: utf-8 -*-

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

class Command_Window(QtGui.QWidget):
    def __init__(self, config, parent = None):
        super(Command_Window, self).__init__()
        data = self.loadConfig(config)
        self.subWindows = []
        i = gamepadID = 0
        self.calcVelocityGamepadThreads = []
        for robot in data['Robot']:
            if (robot['KeyConfig'] == 'gamepad'):
                self.subWindows.append(Gamepad(self, robot, gamepadID))
                self.calcVelocityGamepadThreads.append(calcVelocityGamepad(self.subWindows[i]))
                gamepadID += 1
            else:
                self.subWindows.append(Keyboard(self, robot))
            i += 1
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

# --------------------- #

if __name__ == '__main__':
    rospy.init_node('Command', anonymous=True)
    app = QtGui.QApplication(sys.argv)

    # use defaut config if not sent
    if len(sys.argv) <= 1:
        config = 'configs/VSS_1on1_fr.yaml'
    else:
        config = sys.argv[1]

    w = Command_Window(config)
    w.setWindowTitle('Command')
    w.show()

    sys.exit(app.exec_())
