# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore

import numpy as np

import rospy
from std_msgs.msg import Float32

from lib.command_subwindow import Command_Subwindow

class Keyboard(Command_Subwindow):
    def __init__(self, parent, data):
        super(Keyboard, self).__init__(parent, data)
        self.keys = np.array([0, 0, 0, 0]) # [up, down, left, rigth], 1 when pressed
        self.dispKeys = data['KeyConfig']
        self.initUI()

    # load .yaml file and set configuration data (called from constructor)
    def config(self, data):
        self.keyNames = self.keyConfig(data['KeyConfig'])
        super(Keyboard, self).config(data)

    def UIinfo(self):
        displayDeviceName = QtGui.QLabel('Device: keyboard (' + self.dispKeys + ')')
        self.displayVelL = QtGui.QLabel('Left wheel speed: ' + str(self.velL) + ' rads/s')
        self.displayVelR = QtGui.QLabel('Right wheel speed: ' + str(self.velR) + ' rads/s')
        # self.layout = QtGui.QVBoxLayout(self)

        self.layout.addWidget(displayDeviceName)
        self.layout.addWidget(self.displayVelL)
        self.layout.addWidget(self.displayVelR)

    def keyConfig(self, config):
        if (config == 'wasd'):
            return np.array([QtCore.Qt.Key_W, QtCore.Qt.Key_S, QtCore.Qt.Key_A, QtCore.Qt.Key_D])
        if (config == 'zqsd'):
            return np.array([QtCore.Qt.Key_Z, QtCore.Qt.Key_S, QtCore.Qt.Key_Q, QtCore.Qt.Key_D])
        if (config == 'ijkl'):
            return np.array([QtCore.Qt.Key_I, QtCore.Qt.Key_K, QtCore.Qt.Key_J, QtCore.Qt.Key_L])
        if (config == 'oklm'):
            return np.array([QtCore.Qt.Key_O, QtCore.Qt.Key_L, QtCore.Qt.Key_K, QtCore.Qt.Key_M])
        return np.array([QtCore.Qt.Key_Up, QtCore.Qt.Key_Down, QtCore.Qt.Key_Left, QtCore.Qt.Key_Right])

    # called each time a key is pressed
    def keyPressEvent(self, event):
        self.keyMap(event, 1)
        event.accept()

    # called each time a key is released
    def keyReleaseEvent(self, event):
        self.keyMap(event, 0)
        event.accept()

    # save each arrow keys are pressed (called from keyPressEvent and keyReleaseEvent)
    def keyMap(self, event, status):
        if event.key() == self.keyNames[0]:
            self.keys[0] = status
        elif event.key() == self.keyNames[1]:
            self.keys[1] = status
        elif event.key() == self.keyNames[2]:
            self.keys[3] = status
        elif event.key() == self.keyNames[3]:
            self.keys[2] = status
        self.calcVelocity()

    # determine velocity of left an right wheels according to the keys being pressed (called form keyMap)
    def calcVelocity(self):
        direction = -2*self.keys[1] + 1 # used to deal with backwards motion, -1 when down is pressed 1 otherwise
        self.velL = np.dot(np.array([self.velS, -self.velS, self.velC*direction, -self.velC*direction]), self.keys)
        self.velR = np.dot(np.array([self.velS, -self.velS, -self.velC*direction, self.velC*direction]), self.keys)
        self.displayVelL.setText('Left wheel speed: ' + str('{:>.1f}'.format(self.velL)) + ' rads/s')
        self.displayVelR.setText('Right wheel speed: ' + str('{:>.1f}'.format(self.velR)) + ' rads/s')
        for topic in self.pubL:
            topic.publish(self.velL)
        for topic in self.pubR:
            topic.publish(self.velR)
