# -*- coding: utf-8 -*-
from PyQt4 import QtGui
from PyQt4 import QtCore

import yaml
import pygame

from SubWindow import SubWindow
from SubWindowGamepad import SubWindowGamepad

class calcVelocityGamepad(QtCore.QThread):
    def __init__(self,subwindow):
        QtCore.QThread.__init__(self)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(41) #Assim, a velocidade é calculada ~24x por segundo (mesmo fps de filmes em cinemas)
        self.subwindow = subwindow
    def run(self):
        self.subwindow.calcVelocity()

class MainWindow(QtGui.QWidget):
    def __init__(self, config):
        super(MainWindow, self).__init__()
        self.setWindowTitle('Keyboard Control')
        data = self.loadConfig(config)
        self.subWindows = [];
        for robot in data['Robot']:
            if (robot['KeyConfig'] == 'gamepad'):
                self.subWindows.append(SubWindowGamepad(self, robot, 0))
            elif (robot['KeyConfig'] == 'gamepad2'):
                self.subWindows.append(SubWindowGamepad(self, robot, 1))
            else:
                self.subWindows.append(SubWindow(self, robot))
        for idx, subwindow in enumerate(self.subWindows):
            subwindow.move(5, 5 + (subwindow.size().height()+25)*idx)
        self.calcVelocityGamepadThreads = []
        for subwindow in self.subWindows:
            if (type(subwindow) is SubWindowGamepad):
                self.calcVelocityGamepadThreads.append(calcVelocityGamepad(subwindow))

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
