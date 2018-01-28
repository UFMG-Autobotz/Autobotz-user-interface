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

		# self.file_menu = self.my_menu.addMenu("File")
		# self.exit_action = QtGui.QAction('Exit', self)
		# self.exit_action.triggered.connect(exit)
		# self.file_menu.addAction(exit_action)

		self.tele_op = self.my_menu.addMenu("Teleop")

		self.slider_window = self.tele_op.addMenu("Slider control")
		self.slider_window.addAction("New tab")
		self.slider_window.addAction("Set config...")
		self.slider_window.triggered[QtGui.QAction].connect(self.slider_action)

		self.device_window = self.tele_op.addMenu("Device input")
		self.device_window.addAction("New tab")
		self.device_window.addAction("Set config...")
		self.device_window.triggered[QtGui.QAction].connect(self.device_action)

		self.telemetry = self.my_menu.addMenu("Telemetry")

		self.graph_window = self.telemetry.addMenu("Graph plotter")
		self.graph_window.addAction("New tab")
		self.graph_window.addAction("Set config...")
		self.graph_window.triggered[QtGui.QAction].connect(self.graph_Plotter_action)

		self.image_window = self.telemetry.addMenu("Image viewer")
		self.image_window.addAction("New tab")
		self.image_window.addAction("Set config...")
		self.image_window.triggered[QtGui.QAction].connect(self.image_Viewer_action)

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

	def slider_action(self,q):
		if q.text() == 'Set config...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.slider_config_file)
			if config_file:
				self.slider_config_file = config_file
		elif q.text() == 'New tab' and self.slider_config_file:
			self.new_Dock('Slider control')
			self.dockList[-1].setWidget(Slider_Window(self.slider_config_file, self.dockList[-1]))

	def device_action(self,q):
		if q.text() == 'Set config...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.device_config_file)
			if config_file:
				self.device_config_file = config_file
		elif q.text() == 'New tab' and self.device_config_file:
			self.new_Dock('Device input')
			self.dockList[-1].setWidget(Device_Window(self.device_config_file, self.dockList[-1]))

	def graph_Plotter_action(self,q):
		if q.text() == 'Set config...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.graph_config_file)
			if config_file:
				self.graph_config_file = config_file
		elif q.text() == 'New tab' and self.graph_config_file:
			self.new_Dock('Graph plotter')
			self.dockList[-1].setWidget(Graph_Window(self.graph_config_file, self.dockList[-1]))

	def image_Viewer_action(self,q):
		if q.text() == 'Set config...':
			config_file = QtGui.QFileDialog.getOpenFileName(w, 'Choose config file', self.image_config_file)
			if config_file:
				self.image_config_file = config_file
		elif q.text() == 'New tab' and self.image_config_file:
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
