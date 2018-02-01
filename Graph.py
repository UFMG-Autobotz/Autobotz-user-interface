#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4 import QtCore
from PyQt4 import QtGui
import sys

import rospy
from importlib import import_module

# from std_msgs.msg import Float64
# from geometry_msgs.msg import Vector3, Pose

from lib.general_utils import get_yaml_dict, check_file, check_arguments
from lib.RTPlotter import RTPlotter

# --------------------- #

class Graph_Window(QtGui.QWidget):
	def __init__(self, config_file, parent = None):
		super(Graph_Window, self).__init__(parent)
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
		self.config_file = check_file(config_file)
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
			new_cb.setEditable(True)
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
		self.graph_specs = get_yaml_dict(self.config_file)
		self.graph_specs =  {k.lower(): v for k, v in self.graph_specs.items() if v is not None}
		self.graph_specs['dims'] = self.dims
		self.graph_specs['parent'] = self
		self.graph_window = RTPlotter(**self.graph_specs)
		self.splitter.addWidget(self.graph_window)
		self.splitter.setStretchFactor(1,1)

	def new_curve(self):
		dims = self.graph_type_choice.currentIndex() + 1 # number of dimensions 1D, 2D or 3D

		# if number of dimensions changed of there is no graph_window yet, create new graph
		if dims != self.dims or self.graph_window is None:
			self.dims = dims
			self.new_graph()

		self.graph_window.add_curve()

		def binary_callback_data(data, d):
			connection_header =  data._connection_header['type'].split('/')

			ros_pkg = connection_header[0] + '.msg'
			msg_type = connection_header[1]
			print msg_type
			msg_class = getattr(import_module(ros_pkg), msg_type)

			self.subs[d["x"]][d["y"]].unregister()
			self.subs[d["x"]].append(rospy.Subscriber(d["topic"], msg_class,
				lambda data, dim=d["dim"], n_curve=d["n_curve"],
				attributes=d["attributes"]:callback_data(data, dim, n_curve, attributes)))

		@self.graph_window.data_wrapper
		def callback_data(data, dim, n_curve, attributes):
			# access attributes recursively
			for i in attributes:
				try:
					data = getattr(data, i)
				except AttributeError:
					raise

			# if no attribute was given, try to read it as Float64 (attribute is data)
			# if len(attributes) == 0:
			# 	try:
			# 		data = getattr(data, "data")
			# 	except AttributeError:
			# 		raise

			return (data, dim, n_curve)

		for i in range(self.dims):
			input_list = str(self.cb[self.dims-1][i].currentText()).split('.')

			sub_dict = {}
			sub_dict["dim"] = i
			sub_dict["n_curve"] = self.graph_window.n_curves -1
			sub_dict["topic"] = input_list[0]
			sub_dict["attributes"] = input_list[1:]
			sub_dict["x"] = len(self.subs)-1
			sub_dict["y"] = len(self.subs[-1])

			self.subs[-1].append(rospy.Subscriber(sub_dict["topic"], rospy.AnyMsg,
				lambda data, sub_dict=sub_dict: binary_callback_data(data, sub_dict)))
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

	def mousePressEvent(self, event):
		self.setFocus();

# --------------------- #

if __name__ == '__main__':
	check_arguments(sys.argv, 'Graph') # check for special arguments:

	rospy.init_node('Graph', anonymous=True)
	app = QtGui.QApplication(sys.argv)

	# use defaut config if not sent
	if len(sys.argv) <= 1:
		config_file = './config/Graph/GENERIC_default.yaml'
	elif len(sys.argv) <= 2:
		config_file = './config/Graph/' + sys.argv[1]
	else:
		config_file = sys.argv[2] + sys.argv[1]

	w = Graph_Window(config_file)
	w.setWindowTitle('Graph')
	w.show()

	print "Graph module from Autobotz User Interface running."

	sys.exit(app.exec_())
