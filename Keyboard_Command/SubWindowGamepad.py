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
        self.keys = np.array([0, 0]) # [up, down, left, rigth], 1 when pressed
        pygame.joystick.init()
        pygame.display.init()
        joystick_count = pygame.joystick.get_count()
        if (joystick_count < (joystickNumber+1)): # checks if there is enough joysticks
            self.initUI(True) # enable error message "Joystick not found"
        else:
            self.initUI(False)
            self.joystick = pygame.joystick.Joystick(joystickNumber)
            self.joystick.init()


    # load .yaml file and set configuration data (called from constructor)
    def config(self, data):
        self.name = data['Name']
        self.velS = data['VelStraight']
        self.velC = data['VelCurve']
        # self.keyNames = self.keyConfig(data['KeyConfig'])
        self.initROS(data)

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

    def keyPressEvent(self, event):
        self.calcVelocity()

    # called each time a key is released
    def keyReleaseEvent(self, event):
        self.calcVelocity()

    # determine velocity of left an right wheels according to the keys being pressed (called form keyMap)
    def calcVelocity(self):
        pygame.event.pump()
        Xaxis = self.joystick.get_axis(0)
        Yaxis = self.joystick.get_axis(1)
        print('{:>6.3f}'.format(Xaxis))
        self.keys = np.array([Xaxis, Yaxis])

        direction = -2*self.keys[1] + 1 # used to deal with backwards motion, -1 when down is pressed 1 otherwise

        self.velL = np.dot(np.array([self.velS, -self.velC*direction]), self.keys);
        self.velR = np.dot(np.array([self.velS, self.velC*direction]), self.keys);

        self.displayVelL.setText('Left wheel speed: ' + str(self.velL) + ' rads/s');
        self.displayVelR.setText('Right wheel speed: ' + str(self.velR) + ' rads/s');

        for topic in self.pubL:
            topic.publish(self.velL)

        for topic in self.pubR:
            topic.publish(self.velR)
