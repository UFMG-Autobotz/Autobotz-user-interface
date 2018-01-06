#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, importlib
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from PyQt4 import QtCore
from PyQt4 import QtGui

from general_utils import get_yaml_dict


class Image_Window(QtGui.QWidget):

	img_ready = QtCore.pyqtSignal()

	def __init__(self, configs_file, parent = None):
		super(Image_Window, self).__init__(parent)
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose)
		self.configs_file = configs_file
		self.im_specs = get_yaml_dict(self.configs_file)
		Vision_class = getattr(importlib.import_module(self.im_specs['vision_file']), self.im_specs['vision_class'])
		self.vision_obj = Vision_class()
		self.func_avaiable_list = self.vision_obj.get_functions_nicks()
		self.func_applied_list = []
		self.img_orig = None
		self.img_current = None
		self.subs = [[]]
		self.bridge = CvBridge()
		self.image_sub = None
		self.update_topics()
		self.create_window()

	def update_topics(self):
		self.active_topics = sorted(rospy.get_published_topics(), key=lambda x: x[0])

	def highlighted_update(self,i):
		old_topics = self.active_topics
		self.update_topics()
		if old_topics != self.active_topics:
			self.img_source.clear()
			for topic in self.active_topics:
				self.img_source.addItem( topic[0] )

	def set_img_source(self):
		def callback_img(data):
			try:
				self.img_orig = self.bridge.imgmsg_to_cv2(data, "bgr8")
				# self.set_view()
				self.img_ready.emit()
			except CvBridgeError as e:
				print(e)

		if self.image_sub is not None:
			self.image_sub.unregister()
		self.image_sub = rospy.Subscriber(str(self.img_source.currentText()), Image, callback_img)

	def add_function(self):
		name = str(self.func_avaiable.currentText())
		self.func_applied.insertItem(len(self.func_applied_list), name)
		self.func_applied_list.append(name)

	def remove_function(self):
		n_items = self.func_applied.count()
		rangedList =range(n_items)
		rangedList.reverse()
		for i in rangedList:
			if self.func_applied.isItemSelected(self.func_applied.item(i))==True:
				if i == 0:
					break
				self.func_applied.takeItem(i)
				del self.func_applied_list[i]
				if self.last_ID >= i:
					self.last_ID -= 1
				break

	def set_view(self, ID=None):

		if self.img_orig is None:
			return

		if ID is None:
			ID = self.last_ID
		else:
			self.last_ID = ID

		self.img_current = self.img_orig.copy()
		for i in range(ID):
			self.img_current = self.vision_obj.run(self.img_current, self.func_applied_list[i+1])
			if len(self.img_current.shape) == 2:
				self.img_current = cv2.cvtColor(self.img_current, cv2.COLOR_GRAY2RGB)

		self.imgShow = cv2.cvtColor(self.img_current, cv2.COLOR_BGR2RGB)
		height, width = self.img_current.shape[:2]

		# https://srinikom.github.io/pyside-docs/PySide/QtGui/QImage.html
		# QImage.Format_Invalid
		# QImage.Format_Mono
		# QImage.Format_MonoLSB
		# QImage.Format_Indexed8
		# QImage.Format_RGB32
		# QImage.Format_ARGB32
		# QImage.Format_ARGB32_Premultiplied
		# QImage.Format_RGB16
		# QImage.Format_ARGB8565_Premultiplied
		# QImage.Format_RGB666
		# QImage.Format_ARGB6666_Premultiplied
		# QImage.Format_RGB555
		# QImage.Format_ARGB8555_Premultiplied
		# QImage.Format_RGB888
		# QImage.Format_RGB444
		# QImage.Format_ARGB4444_Premultiplied
		self.mQImage = QtGui.QImage(self.imgShow, width, height, QtGui.QImage.Format_RGB888)
		self.pixmap = QtGui.QPixmap(self.mQImage)
		self.img_label.setPixmap(self.pixmap)

	def create_window(self):
		layout = QtGui.QHBoxLayout()

		self.selection_widget = QtGui.QWidget(self)
		choice_layout = QtGui.QVBoxLayout()

		self.img_source_label = QtGui.QLabel(u"Origem da imagem:")
		self.img_source_label.setWordWrap(True)

		self.img_source = QtGui.QComboBox(self)
		for topic in self.active_topics:
			self.img_source.addItem( topic[0] )
		self.img_source.highlighted.connect(self.highlighted_update)

		self.avaiable_label = QtGui.QLabel(u"Funções disponíveis:")
		self.avaiable_label.setWordWrap(True)

		self.func_avaiable = QtGui.QComboBox(self)
		for name in self.func_avaiable_list:
			self.func_avaiable.addItem(name)
		self.func_avaiable.setCurrentIndex(0)

		self.applied_label = QtGui.QLabel(u"Funções aplicadas:")
		self.applied_label.setWordWrap(True)

		self.func_applied = QtGui.QListWidget ()
		self.func_applied.currentRowChanged.connect(self.set_view)

		self.show_button = QtGui.QPushButton('Show')
		self.show_button.clicked.connect( self.set_img_source )

		self.add_button = QtGui.QPushButton('Add to list')
		self.add_button.clicked.connect( self.add_function )

		self.remove_button = QtGui.QPushButton('Remove from list')
		self.remove_button.clicked.connect( self.remove_function )


		choice_layout.addWidget(self.img_source_label)
		choice_layout.addWidget(self.img_source)
		choice_layout.addWidget(self.show_button)
		choice_layout.addWidget(self.avaiable_label)
		choice_layout.addWidget(self.func_avaiable)
		choice_layout.addWidget(self.applied_label)
		choice_layout.addWidget(self.func_applied)
		# choice_layout.addStretch(1)
		choice_layout.addWidget(self.add_button)
		# choice_layout.addStretch(1)
		choice_layout.addWidget(self.remove_button)
		# choice_layout.addStretch(10)

		self.selection_widget.setLayout(choice_layout)

		self.img_label = QtGui.QLabel(u"Lugar onde serão exibidas imagens vindas da câmera ou Visão")
		self.img_label.setWordWrap(True)
		self.img_label.setSizePolicy( QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored )

		self.pixmap = None

		self.img_label.installEventFilter(self)
		self.img_label.setAlignment(QtCore.Qt.AlignCenter)



		# self.img_label.setScaledContents(True)
		# http://doc.qt.io/qt-4.8/qsizepolicy.html
		# Fixed
		# Minimum
		# Maximum
		# Preferred
		# Expanding
		# MinimumExpanding
		# Ignored

		self.img_orig = None
		self.img_ready.connect(self.set_view)

		self.last_ID = 0
		self.func_applied.insertItem(0, '(Original)')
		self.func_applied_list.append('(Original)')

		self.splitter = QtGui.QSplitter(QtCore.Qt.Horizontal)
		self.splitter.addWidget(self.selection_widget)
		self.splitter.addWidget(self.img_label)
		self.splitter.setStretchFactor(1,1)
		layout.addWidget(self.splitter)

		self.setLayout(layout)

	def eventFilter(self, widget, event):
		if (event.type() == QtCore.QEvent.Resize and widget is self.img_label):
			if self.pixmap is not None:
				self.img_label.setPixmap(self.pixmap.scaled( self.img_label.width(), self.img_label.height(), QtCore.Qt.KeepAspectRatio))
				return True
		return QtGui.QMainWindow.eventFilter(self, widget, event)

	# def closeEvent(self, event):
	# 	if self.image_sub is not None:
	# 		self.image_sub.unregister()
	# 	event.accept()
if __name__ == '__main__':
	import sys
	rospy.init_node('image_show', anonymous=True)
	app = QtGui.QApplication(sys.argv)
	w = Image_Window('configs/image_config_teste.yaml')
	w.setWindowTitle('PyQT OpenCV')
	w.show()
	app.exec_()
