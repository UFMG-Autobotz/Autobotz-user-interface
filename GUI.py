#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4 import QtCore
from PyQt4 import QtGui
import sys

import rospy
# from std_msgs.msg import Float32

from Slider import Slider_Window
from Graph import Graph_Window
from Image import Image_Window
from Device import Device_Window

from lib.DockTitleBar import DockTitleBar

# --------------------- #

class Main_Window(QtGui.QMainWindow):
	def __init__(self, parent = None):
		super(Main_Window, self).__init__(parent)
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

		self.setTabPosition( QtCore.Qt.TopDockWidgetArea , QtGui.QTabWidget.North )
		self.setDockOptions( QtGui.QMainWindow.AllowNestedDocks )
		self.setDockOptions( QtGui.QMainWindow.AllowTabbedDocks )
		# self.setDockOptions( QtGui.QMainWindow.ForceTabbedDocks )

		self.slider_config_file = './config/Slider/VT_default.yaml'
		self.device_config_file = './config/Device/VSS_1on1_keyboard.yaml'
		self.graph_config_file = './config/Graph/GENERIC_default.yaml'
		self.image_config_file = './config/Image/GENERIC_sample.yaml'

		self.my_menu = self.menuBar()

		self.teleoperation = self.my_menu.addMenu('Teleoperation')
		self.teleoperation.addAction('Slider control...')
		self.teleoperation.addAction('Device input...')
		self.teleoperation.triggered[QtGui.QAction].connect(self.teleoperation_action)

		self.telemetry = self.my_menu.addMenu('Telemetry')
		self.telemetry.addAction('Graph plotter...')
		self.telemetry.addAction('Image viewer...')
		self.telemetry.triggered[QtGui.QAction].connect(self.telemetry_action)

		self.dockList = []
		self.dockTitleList = []

		w = 900; h = 600
		self.resize(w, h)

		self.setWindowTitle("Main Window")

	def new_Dock(self, name):
		dock = QtGui.QDockWidget(name, self)
		dock.setAllowedAreas(QtCore.Qt.TopDockWidgetArea)
		dock.setAttribute(QtCore.Qt.WA_DeleteOnClose)
		dock_title = DockTitleBar(dock)
		self.addDockWidget(QtCore.Qt.TopDockWidgetArea, dock)
		self.dockList.append( dock )

		# if len(self.dockList) > 1:
		# 	self.tabifyDockWidget(self.dockList[-2],self.dockList[-1])

	def keyPressEvent(self, event):
		if event.isAutoRepeat():
			return
		for tab in self.dockList:
			try: # try except used to not raise error when looking for alread closed widget
				# check it is the active tab
				#(keyboard device can act umpredictably if it's allowed to be used when unfocused)
				if tab.widget().hasFocus():
					tab.widget().keyPressEvent(event)
					event.accept()
			except:
				pass

	def keyReleaseEvent(self, event):
		if event.isAutoRepeat():
			return
		for tab in self.dockList:
			try: # try except used to not raise error when looking for alread closed widget
				tab.widget().keyReleaseEvent(event)
				event.accept()
			except:
				pass

	def teleoperation_action(self,q):
		if q.text() == 'Slider control...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.slider_config_file)
			if config_file:
				self.slider_config_file = config_file
				self.new_Dock('Slider control')
				self.dockList[-1].setWidget(Slider_Window(self.slider_config_file, self.dockList[-1]))
		elif q.text() == 'Device input...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.device_config_file)
			if config_file:
				self.device_config_file = config_file
				self.new_Dock('Device input')
				self.dockList[-1].setWidget(Device_Window(self.device_config_file, self.dockList[-1]))

	def telemetry_action(self,q):
		if q.text() == 'Graph plotter...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.graph_config_file)
			if config_file:
				self.graph_config_file = config_file
				self.new_Dock('Graph plotter')
				self.dockList[-1].setWidget(Graph_Window(self.graph_config_file, self.dockList[-1]))
		elif q.text() == 'Image viewer...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.image_config_file)
			if config_file:
				self.image_config_file = config_file
				self.new_Dock('Image viewer')
				self.dockList[-1].setWidget(Image_Window(self.image_config_file, self.dockList[-1]))

# --------------------- #

if __name__ == '__main__':
	rospy.init_node("Autobotz_GUI", anonymous=True)
	app = QtGui.QApplication(sys.argv)
	app.setStyle("Plastique")

	w = Main_Window()
	w.setWindowTitle('Autobotz User Interface')
	w.show()

	print "Autobotz User Interface running."

	sys.exit(app.exec_())
