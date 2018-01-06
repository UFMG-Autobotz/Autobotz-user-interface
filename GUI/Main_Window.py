#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4 import QtCore
from PyQt4 import QtGui
from Slider import Sliders_Window
from Graphs import Graphs_Window
from Image import Image_Window
from DockTitleBar import DockTitleBar

class Main_Window(QtGui.QMainWindow):
	def __init__(self, parent = None):
		super(Main_Window, self).__init__(parent)
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

		self.setTabPosition( QtCore.Qt.TopDockWidgetArea , QtGui.QTabWidget.North )
		self.setDockOptions( QtGui.QMainWindow.AllowNestedDocks )
		self.setDockOptions( QtGui.QMainWindow.AllowTabbedDocks )
		# self.setDockOptions( QtGui.QMainWindow.ForceTabbedDocks )

		self.sliders_config_file = 'configs/slider_config_teste.yaml'
		self.graphs_config_file = 'configs/graph_config_teste.yaml'
		self.image_config_file = 'configs/image_config_teste.yaml'

		self.my_menu = self.menuBar()

		# self.file_menu = self.my_menu.addMenu("File")
		# self.exit_action = QtGui.QAction('Exit', self)
		# self.exit_action.triggered.connect(exit)
		# self.file_menu.addAction(exit_action)

		self.tele_op = self.my_menu.addMenu("Teleop")

		self.sliders_window = self.tele_op.addMenu("Sliders")
		self.sliders_window.addAction("Nova aba")
		self.sliders_window.triggered[QtGui.QAction].connect(self.sliders_action)
		self.sliders_window.addAction("Set config")

		self.telemetry = self.my_menu.addMenu("Telemetry")

		self.graphs_window = self.telemetry.addMenu("Graph Plotter")
		self.graphs_window.addAction("Nova aba")
		self.graphs_window.triggered[QtGui.QAction].connect(self.graph_Plotter_action)
		self.graphs_window.addAction("Set config")

		self.image_window = self.telemetry.addMenu("Image Viewer")
		self.image_window.addAction("Nova aba")
		self.image_window.triggered[QtGui.QAction].connect(self.image_Viewer_action)
		self.image_window.addAction("Set config")

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

	def sliders_action(self,q):
		if q.text() == 'Nova aba':
			self.new_Dock('Slider')
			self.dockList[-1].setWidget(Sliders_Window(self.sliders_config_file, self.dockList[-1]))

	def graph_Plotter_action(self,q):
		if q.text() == 'Nova aba':
			self.new_Dock('Graph')
			self.dockList[-1].setWidget(Graphs_Window(self.graphs_config_file, self.dockList[-1]))

	def image_Viewer_action(self,q):
		if q.text() == 'Nova aba':
			self.new_Dock('Image')
			self.dockList[-1].setWidget(Image_Window(self.image_config_file, self.dockList[-1]))

if __name__ == '__main__':
	import sys
	app = QtGui.QApplication(sys.argv)
	w = Main_Window()
	w.setWindowTitle('PyQT Main Window')
	w.show()
	app.exec_()
