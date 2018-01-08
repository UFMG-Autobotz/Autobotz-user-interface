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

        self.velR = self.velL = 0
        self.keys = np.array([0, 0, 0, 0]) # [up, down, left, rigth], 1 when pressed
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if (joystick_count < (joystickNumber+1)): # checks if there is enough joysticks
            self.initUI(True) # enable error message "Joystick not found"
        else:
            self.initUI(False)


    # load .yaml file and set configuration data (called from constructor)
    def config(self, data):
        self.name = data['Name']
        self.velS = data['VelStraight']
        self.velC = data['VelCurve']
        self.keyNames = self.keyConfig(data['KeyConfig'])

        self.initROS(data)

    def keyConfig(self, config):
        if (config == 'gamepad'):
            return np.array([])

        return np.array([QtCore.Qt.Key_Up, QtCore.Qt.Key_Down, QtCore.Qt.Key_Left, QtCore.Qt.Key_Right])


    # start initialize ros and create publisher for left and right sides (called from loadConfig)
    def initROS(self, data):
        rospy.init_node('keyboardControl', anonymous=True)

        self.pubL = []
        for topic in data['Left']:
            self.pubL.append(rospy.Publisher(topic, Float32, queue_size = 100));

        self.pubR = []
        for topic in data['Right']:
            self.pubR.append(rospy.Publisher(topic, Float32, queue_size = 100));

    # initialie interface elements (called from constructor)
    def initUI(self, errorFlag):
        self.resize(300, 100)

        self.frame = QtGui.QGroupBox(self)
        self.frame.setTitle(self.name)
        self.layout = QtGui.QVBoxLayout(self)
        self.layout.addWidget(self.frame);

        if (errorFlag):
            self.errorMessage = QtGui.QLabel('JOYSTICK NOT FOUND')
            self.layout.addWidget(self.errorMessage)
        else:
            self.displayVelL = QtGui.QLabel('Left wheel speed: ' + str(self.velL) + ' rads/s', parent=self.frame)
            self.displayVelR = QtGui.QLabel('Right wheel speed: ' + str(self.velR) + ' rads/s', parent=self.frame)
            self.layout.addWidget(self.displayVelL);
            self.layout.addWidget(self.displayVelR);

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

        self.velL = np.dot(np.array([self.velS, -self.velS, self.velC*direction, -self.velC*direction]), self.keys);
        self.velR = np.dot(np.array([self.velS, -self.velS, -self.velC*direction, self.velC*direction]), self.keys);

        self.displayVelL.setText('Left wheel speed: ' + str(self.velL) + ' rads/s');
        self.displayVelR.setText('Right wheel speed: ' + str(self.velR) + ' rads/s');

        for topic in self.pubL:
            topic.publish(self.velL)

        for topic in self.pubR:
            topic.publish(self.velR)
