from PyQt4 import QtGui
from PyQt4 import QtCore

import pygame

import numpy as np

import rospy
from std_msgs.msg import Float32

class SubWindowGamepad(QtGui.QWidget):
    def __init__(self, parent, data, joystickNumber):
        super(SubWindowGamepad, self).__init__(parent)
        self.config(data)
        self.velR = self.velL = 0.0
        self.keys = np.array([0, 0]) # [up, down, left, rigth], 1 when pressed
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
        self.initUI(self.notEnoughJoysticks)

    # load .yaml file and set configuration data (called from constructor)
    def config(self, data):
        self.name = data['Name']
        self.velS = data['VelStraight']
        self.velC = data['VelCurve']
        self.initROS(data)

    # start initialize ros and create publisher for left and right sides (called from loadConfig)
    def initROS(self, data):
        rospy.init_node('keyboardControl', anonymous=True)
        self.pubL = []
        for topic in data['Left']:
            self.pubL.append(rospy.Publisher(topic, Float32, queue_size = 100))
        self.pubR = []
        for topic in data['Right']:
            self.pubR.append(rospy.Publisher(topic, Float32, queue_size = 100))

    # initialie interface elements (called from constructor)
    def initUI(self, errorFlag):
        self.resize(300, 100)
        self.frame = QtGui.QGroupBox(self)
        self.frame.setTitle(self.name)
        self.layout = QtGui.QVBoxLayout(self)
        self.layout.addWidget(self.frame)
        if (errorFlag):
            self.errorMessage = QtGui.QLabel('JOYSTICK NOT FOUND')
            self.layout.addWidget(self.errorMessage)
        else:
            self.displayDeviceName = QtGui.QLabel('Device: ' + self.joystick.get_name(), parent=self.frame)
            self.displayVelL = QtGui.QLabel('Left wheel speed: ' + str(self.velL) + ' rads/s', parent=self.frame)
            self.displayVelR = QtGui.QLabel('Right wheel speed: ' + str(self.velR) + ' rads/s', parent=self.frame)
            self.layout.addWidget(self.displayDeviceName)
            self.layout.addWidget(self.displayVelL)
            self.layout.addWidget(self.displayVelR)

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
