#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

import numpy as np

from PyQt4 import QtCore
from PyQt4 import QtGui

from general_utils import get_yaml_dict
from qt_utils import qt_Line

class Sliders_Window(QtGui.QWidget):
	def __init__(self, configs_file, parent = None):
		super(Sliders_Window, self).__init__(parent)
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
		specs = get_yaml_dict(configs_file)
		self.init_groups(specs)
		self.create_window()

	def init_groups(self, specs):
		self.groups = []
		self.pubs = []
		self.subs = []

		for k, key in enumerate( sorted( specs.keys() ) ):
			self.groups.append(specs[key])
			self.pubs.append([])
			self.subs.append([])

			for pub_group in self.groups[-1]['pubs']:
				self.pubs[-1].append([])
				for pub_name in pub_group:
					self.pubs[-1][-1].append(rospy.Publisher(pub_name, Float32, queue_size=1))

			if 'subs' in self.groups[-1]:
				for i, sub_group in enumerate(self.groups[-1]['subs']):
					self.subs[-1].append([])
					for j, sub_name in enumerate(sub_group):
						rospy.Subscriber(sub_name[1], Float32, lambda data, topic=k, slider=i, name=j: self.sub_callback(data.data, topic, slider, name))
						self.subs[-1][-1].append(sub_name[0] + ' = ')

	def change_value(self, topic, slider, tipo, info):
		_min, _max, ticks = info
		slider_value = float(self.sliders[ topic ][ slider ].value())

		if tipo == 'int':
			if ticks > _max-_min:
				value = int(_min+slider_value)
			else:
				value = int(_min + (_max-_min)*slider_value/ticks)
		else:
			value = round(_min + (_max-_min)*slider_value/ticks, 3)

		self.current_set_values[ topic, slider ] = value
		self.display_values[ topic ][ slider ].setText(str(value))

		if self.radio_buttons[topic][slider][1].isChecked() and self.onchange_publish[topic][slider] == True:
			self.publish(topic, slider)

	def enterPress(self, topic, slider, tipo, info):
		text_value = self.display_values[ topic ][ slider ].text()
		_min, _max, ticks = info

		try:
			value = round(float(text_value), 3)
			value = min(_max, value)
			value = max(_min, value)

			if tipo == 'int':
				value = int(value)

			self.sliders[ topic ][ slider ].setValue( int((value-_min)*ticks/(_max-_min)) )
			self.current_set_values[ topic, slider ] = value
			self.display_values[ topic ][ slider ].setText(str(value))

		except:
			pass

	def sub_callback(self, value, topic, slider, ID):
		self.current_get_values[topic][slider][ID] = value
		self.sub_labels[topic][slider][ID].setText(self.subs[topic][slider][ID]+str(value))

	def publish(self, topic, slider):
		if self.radio_buttons[topic][slider][1].isChecked():
			self.onchange_publish[topic][slider] = True
		elif self.radio_buttons[topic][slider][2].isChecked():
			self.stream_publish[topic][slider] = True
			self.start_timer(topic, slider)
		for pub in self.pubs[ topic ][ slider ]:
			pub.publish( self.current_set_values[ topic, slider ] )

	def start_timer(self, topic, slider):
		freq = self.groups[topic]['stream_freq']
		self.stream_timer[topic][slider].start(1000.0 / freq)

	def stop_timer(self, topic, slider):       
		self.stream_timer[topic][slider].stop()

	def radio_response(self, topic, slider, button):
		if button == 0 or button == 1:
			self.stream_publish[topic][slider] = False
			self.stop_timer(topic, slider)
		if button == 0 or button == 2:
			self.onchange_publish[topic][slider] = False

	def create_window(self):
		layout = QtGui.QHBoxLayout()
		self.current_set_values = np.zeros((len(self.groups), 1))
		self.current_get_values = np.zeros((len(self.groups), 1, 1))
		self.display_values = []
		self.sliders = []
		self.pub_buttons = []
		self.sub_labels = []
		self.radio_buttons = []
		self.radio_buttons_groups = []
		self.onchange_publish = []
		self.stream_publish = []
		self.stream_timer = []

		for k, group in enumerate(self.groups):
			group_vbox = QtGui.QVBoxLayout()

			label = QtGui.QLabel(group['nome'])
			label.setAlignment(QtCore.Qt.AlignCenter)

			n_sliders = len(group['sliders'])
			diff = n_sliders - self.current_set_values.shape[1]
			if diff > 0:
				self.current_set_values = np.column_stack( ( self.current_set_values, np.zeros( ( len(self.groups), diff ) ) ) )
				self.current_get_values = np.column_stack( ( self.current_get_values, np.zeros( ( len(self.groups), diff , self.current_get_values.shape[2] ) ) ) )

			self.display_values.append([])
			self.sliders.append([])
			self.pub_buttons.append([])
			self.sub_labels.append([])
			self.radio_buttons.append([])
			self.radio_buttons_groups.append([])
			self.onchange_publish.append([])
			self.stream_publish.append([])
			self.stream_timer.append([])

			sliders_hbox = QtGui.QHBoxLayout()
			for i in xrange(n_sliders):

				value_vbox = QtGui.QVBoxLayout()

				slider_label = QtGui.QLabel(group['sliders'][i])
				slider_label.setAlignment(QtCore.Qt.AlignCenter)

				if 'subs' in group:
					n_subs = len(group['subs'][i])
					diff2 = n_subs - self.current_get_values.shape[2]
					if diff2 > 0:
						self.current_get_values = np.dstack( ( self.current_get_values, np.zeros( (len(self.groups), self.current_get_values.shape[1], diff2 ) ) ) )

					subs_hbox = QtGui.QHBoxLayout()
					self.sub_labels[-1].append([])
					for sub in self.subs[k][i]:
						sub_label = QtGui.QLabel(sub + '?')
						sub_label.setAlignment(QtCore.Qt.AlignCenter)
						self.sub_labels[-1][-1].append(sub_label)
						subs_hbox.addWidget(self.sub_labels[-1][-1][-1])

				self.sliders[-1].append(QtGui.QSlider(QtCore.Qt.Vertical))
				_min = float(group['range'][i][0])
				_max = float(group['range'][i][1])
				ticks = float(group['ticks'])
				slider_info = [_min, _max, ticks]

				if group['tipo'][i] == 'int' and ticks > _max-_min:
					self.sliders[-1][-1].setMaximum(_max-_min)
				else:
					self.sliders[-1][-1].setMaximum(ticks)

				self.sliders[-1][-1].setMinimum(0)
				self.sliders[-1][-1].setValue(0)
				self.sliders[-1][-1].setTickPosition(QtGui.QSlider.TicksBelow)
				self.sliders[-1][-1].setTickInterval(1)
				self.sliders[-1][-1].valueChanged.connect( lambda _, topic=k, slider=i, tipo=group['tipo'][i], info=slider_info: self.change_value(topic, slider, tipo, info) )

				self.display_values[-1].append(QtGui.QLineEdit())
				self.display_values[-1][-1].setText(str(0))
				# self.display_values[-1][-1].setReadOnly(True)
				self.display_values[-1][-1].editingFinished.connect(lambda topic=k, slider=i, tipo=group['tipo'][i], info=slider_info: self.enterPress(topic, slider, tipo, info) )

				self.pub_buttons[-1].append(QtGui.QPushButton('Publish'))
				self.pub_buttons[-1][-1].clicked.connect( lambda _, topic=k, slider=i: self.publish(topic, slider) )

				self.onchange_publish[-1].append(False)
				self.stream_publish[-1].append(False)
				self.stream_timer[-1].append(QtCore.QTimer(self))
				self.stream_timer[-1][-1].timeout.connect(lambda topic=k, slider=i: self.publish(topic, slider))
				self.stream_timer[-1][-1].setSingleShot(False)
				# self.stop_timer(k,i)

				radio_hbox = QtGui.QHBoxLayout()
				self.radio_buttons_groups[-1].append(QtGui.QButtonGroup())

				self.radio_buttons[-1].append([QtGui.QRadioButton("Once"), QtGui.QRadioButton("OnChange"), QtGui.QRadioButton("Stream")])
				self.radio_buttons[-1][-1][0].setChecked(True)
				self.radio_buttons[-1][-1][0].clicked.connect( lambda _, topic=k, slider=i, button_ID=0: self.radio_response(topic, slider, button_ID) )
				self.radio_buttons[-1][-1][1].clicked.connect( lambda _, topic=k, slider=i, button_ID=1: self.radio_response(topic, slider, button_ID) )
				self.radio_buttons[-1][-1][2].clicked.connect( lambda _, topic=k, slider=i, button_ID=2: self.radio_response(topic, slider, button_ID) )
				
				self.radio_buttons_groups[-1][-1].addButton(self.radio_buttons[-1][-1][0])
				self.radio_buttons_groups[-1][-1].addButton(self.radio_buttons[-1][-1][1])
				self.radio_buttons_groups[-1][-1].addButton(self.radio_buttons[-1][-1][2])
				radio_hbox.addWidget(self.radio_buttons[-1][-1][0])
				radio_hbox.addStretch(1)
				radio_hbox.addWidget(self.radio_buttons[-1][-1][1])
				radio_hbox.addStretch(1)
				radio_hbox.addWidget(self.radio_buttons[-1][-1][2])

				value_vbox.addWidget(slider_label)
				if 'subs' in group:
					value_vbox.addLayout(subs_hbox)
				value_vbox.addWidget(self.display_values[-1][-1])
				value_vbox.addWidget(self.pub_buttons[-1][-1])
				value_vbox.addLayout(radio_hbox)
				value_vbox.addWidget(self.sliders[-1][-1])
				sliders_hbox.addLayout(value_vbox)

			group_vbox.addWidget(label)
			group_vbox.addLayout(sliders_hbox)
			layout.addLayout(group_vbox)
			if k+1 < len(self.groups):
				layout.addWidget( qt_Line(1,2) )

		self.setLayout(layout)

if __name__ == '__main__':
	rospy.init_node('teste_slider', anonymous=True)
	import sys
	app = QtGui.QApplication(sys.argv)
	w = Sliders_Window('configs/slider_config_teste.yaml')
	w.setWindowTitle('PyQT Slider')
	w.show()
	app.exec_()