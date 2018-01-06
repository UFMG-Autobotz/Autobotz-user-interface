#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

from PyQt4 import QtCore
from PyQt4 import QtGui

from general_utils import get_yaml_dict
from RTPlotter import RTPlotter

class Graphs_Window(QtGui.QWidget):
	def __init__(self, configs_file, parent = None):
		super(Graphs_Window, self).__init__(parent)
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
		self.configs_file = configs_file
		self.subs = [[]]
		self.update_topics()
		self.create_window()

	def update_topics(self):
		self.active_topics = sorted(rospy.get_published_topics(), key=lambda x: x[0])

	def highlighted_update(self,i):
		old_topics = self.active_topics
		self.update_topics()
		if old_topics != self.active_topics:
			for i, cb_group in enumerate(self.cb):
				for j, cb in enumerate(cb_group):
					self.cb[i][j].clear()
					for topic in self.active_topics:
						self.cb[i][j].addItem( topic[0] )

	def selectionchange(self,i=None):
		self.stacked_windows.setCurrentIndex(i)

	def new_screen(self,n_cb):
		self.cb.append([])
		new_widget = QtGui.QWidget()
		layout = QtGui.QFormLayout()
		label_choice = [['Y'], ['X', 'Y'], ['X', 'Y', 'Z']][n_cb-1]
		for i in xrange(n_cb):
			new_cb = QtGui.QComboBox()
			for topic in self.active_topics:
				new_cb.addItem( topic[0] )
			new_cb.highlighted.connect(self.highlighted_update)
			self.cb[-1].append(new_cb)
			layout.addRow(label_choice[i],self.cb[-1][-1])
		new_widget.setLayout(layout)
		return new_widget

	def new_graph(self):
		if self.graph_window is not None:
			self.graph_window.hide()
			self.graph_window.close()
		if len(self.subs) > 0:
			for i in range(len(self.subs)):
				if len(self.subs[i]) > 0:
					for j in range(len(self.subs[i])):
						self.subs[i][j].unregister()
		self.subs = [[]]
		self.graph_specs = get_yaml_dict(self.configs_file)
		self.graph_specs =  {k.lower(): v for k, v in self.graph_specs.items() if v is not None}
		self.graph_specs['dims'] = self.dims
		self.graph_specs['parent'] = self
		self.graph_window = RTPlotter(**self.graph_specs)
		self.splitter.addWidget(self.graph_window)
		self.splitter.setStretchFactor(1,1)

	def new_curve(self):
		dims = self.graph_type_choice.currentIndex() + 1
		if dims != self.dims or self.graph_window is None:
			self.dims = dims
			self.new_graph()
		self.graph_window.add_curve()

		@self.graph_window.data_wrapper
		def callback_data(data, dim, n_curve):
			return (data, dim, n_curve)

		for i in range(self.dims):
			sub_name = str(self.cb[self.dims-1][i].currentText())
			n_curve = self.graph_window.n_curves -1
			self.subs[-1].append(rospy.Subscriber(sub_name, Float32, lambda data, dim=i, n_curve=n_curve: callback_data(data.data, dim, n_curve)))
		self.subs.append([])

	def create_window(self):
		layout = QtGui.QHBoxLayout()

		self.graph_window = None
		self.dims = 1
		self.n_curves = 0

		self.selection_widget = QtGui.QWidget(self)
		choice_layout = QtGui.QVBoxLayout()

		self.cb = []

		self.graph_type_choice = QtGui.QComboBox(self)
		self.graph_type_choice.addItem('Y vs t')
		self.graph_type_choice.addItem('X vs Y')
		# self.graph_type_choice.addItem('X vs Y vs Z')

		self.graph_type_choice.currentIndexChanged.connect(self.selectionchange)

		self.stacked_windows = QtGui.QStackedWidget(self)
		self.options = []
		for i in range(2):
			self.options.append(self.new_screen(i+1))
			self.stacked_windows.addWidget(self.options[-1])

		self.graph_type_choice.setCurrentIndex(0)
		self.stacked_windows.setCurrentIndex(0)

		self.plot_button = QtGui.QPushButton('Plot')
		self.plot_button.clicked.connect( self.new_curve )

		self.clear_button = QtGui.QPushButton('Clear')
		self.clear_button.clicked.connect( self.new_graph )

		choice_layout.addWidget(self.graph_type_choice)
		choice_layout.addWidget(self.stacked_windows)
		choice_layout.addStretch(1)
		choice_layout.addWidget(self.plot_button)
		choice_layout.addStretch(1)
		choice_layout.addWidget(self.clear_button)
		choice_layout.addStretch(10)

		self.selection_widget.setLayout(choice_layout)

		# label = QtGui.QLabel(u"Lugar onde serão exibidos gráficos plotados a partir de tópicos do ROS")
		# label.setAlignment(QtCore.Qt.AlignCenter)

		self.splitter = QtGui.QSplitter(QtCore.Qt.Horizontal)
		self.splitter.addWidget(self.selection_widget)
		layout.addWidget(self.splitter)
		self.setLayout(layout)
		self.new_graph()
		# self.splitter.addWidget(self.graph_window)
		# self.splitter.addWidget(label)

if __name__ == '__main__':
	import sys
	app = QtGui.QApplication(sys.argv)
	w = Graphs_Window('configs/graph_config_teste.yaml')
	w.setWindowTitle('PyQT Slider')
	w.show()
	app.exec_()