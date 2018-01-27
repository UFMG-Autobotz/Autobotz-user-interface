# -*- coding: utf-8 -*-

from PyQt4 import QtGui
from PyQt4 import QtCore

import rospy
from std_msgs.msg import Float32

class Command_Subwindow(QtGui.QWidget):
    def __init__(self, parent, data):
        super(Command_Subwindow, self).__init__(parent)
        self.config(data)
        self.velR = self.velL = 0.0

    # load .yaml file and set configuration data (called from constructor)
    def config(self, data):
        self.name = data['Name']
        self.velS = data['VelStraight']
        self.velC = data['VelCurve']
        self.initROS(data)

    # start initialize ros and create publisher for left and right sides (called from loadConfig)
    def initROS(self, data):
        self.pubL = []
        for topic in data['Left']:
            self.pubL.append(rospy.Publisher(topic, Float32, queue_size = 100))
        self.pubR = []
        for topic in data['Right']:
            self.pubR.append(rospy.Publisher(topic, Float32, queue_size = 100))

    # initialie interface elements (called from constructor)
    def initUI(self):
        self.frame = QtGui.QGroupBox(self)
        self.frame.setTitle(self.name)

        self.layout=QtGui.QVBoxLayout(self)
        self.frame.resize(300, 125)

        self.frame.setLayout(self.layout)

        self.UIinfo()

        # self.layout.addWidget(self.frame)

    def UIinfo(self):
        text = QtGui.QLabel('UIinfo here')
        self.layout.addWidget(text)
