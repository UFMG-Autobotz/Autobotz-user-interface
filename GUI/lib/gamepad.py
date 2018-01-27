# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore

import pygame

import numpy as np

import rospy
from std_msgs.msg import Float32

from lib.command_subwindow import Command_Subwindow

class Gamepad(Command_Subwindow):
    def __init__(self, parent, data, joystickNumber):
        super(Gamepad, self).__init__(parent, data)
        self.keys = np.array([0, 0])

        pygame.joystick.init()
        pygame.display.init() # I don't have idea why it's necessary, but it is

        self.joystick_count = pygame.joystick.get_count()
        self.notEnoughJoysticks = False
        if (self.joystick_count < (joystickNumber+1)): # checks if there is enough joysticks
            self.notEnoughJoysticks = True
        else:
            self.joystick = pygame.joystick.Joystick(joystickNumber)
            self.joystickName = self.joystick.get_name()
            self.joystick.init()

        self.initUI()

    def UIinfo(self):
        if (self.notEnoughJoysticks):
            self.errorMessage = QtGui.QLabel('JOYSTICK NOT FOUND')
            self.layout.addWidget(self.errorMessage)
        else:
            self.displayDeviceName = QtGui.QLabel('Device: ' + self.joystick.get_name())
            self.displayVelL = QtGui.QLabel('Left wheel speed: ' + str(self.velL) + ' rads/s')
            self.displayVelR = QtGui.QLabel('Right wheel speed: ' + str(self.velR) + ' rads/s')

            self.layout.addWidget(self.displayDeviceName)
            self.layout.addWidget(self.displayVelL)
            self.layout.addWidget(self.displayVelR)

        # self.layout = QtGui.QVBoxLayout(self)

    # determine velocity of left an right wheels according to the keys being pressed (called form keyMap)
    def calcVelocity(self):
        if (not self.notEnoughJoysticks):
            pygame.event.pump()
            Yaxis = self.joystick.get_axis(1)
            if (Yaxis>0): # used to deal with backwards motion
                Xaxis = -self.joystick.get_axis(0)
            else:
                Xaxis = self.joystick.get_axis(0)
            self.keys = np.array([-Yaxis, Xaxis])
            self.velL = np.dot(np.array([self.velS, self.velC]), self.keys)
            self.velR = np.dot(np.array([self.velS, -self.velC]), self.keys)
            self.displayVelL.setText('Left wheel speed: ' + str('{:>.1f}'.format(self.velL)) + ' rads/s')
            self.displayVelR.setText('Right wheel speed: ' + str('{:>.1f}'.format(self.velR)) + ' rads/s')
            for topic in self.pubL:
                topic.publish(self.velL)
            for topic in self.pubR:
                topic.publish(self.velR)
