#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt4 import QtCore
import pyqtgraph as pg
import numpy as np

class RTPlotter(pg.PlotWidget):
	def __init__(self, sampleinterval=0.01, timewindow=10., colors = None, title=None, dims=1, labels=None, limits=None, parent = None):
		super(RTPlotter, self).__init__(parent)
		self.setAttribute(QtCore.Qt.WA_DeleteOnClose)

		# Data stuff
		self._interval = int(sampleinterval*1000)
		self._bufsize = int(timewindow/sampleinterval)
		self.timewindow = timewindow
		self.dims = dims
		self.n_curves = 0
		self.givedata_buffer = None
		self.plotData_x = None
		self.plotData_y = None

		# PyQtGraph stuff
		self.setWindowTitle('Real Time Plotting with PyQtGraph' if title is None else title)
		self.showGrid(x=True, y=True)

		if labels is None:
			self.setLabel('left', 'amplitude', 'Y')
			self.setLabel('bottom', 'time', 'X')
		else:
			for key in labels:
				self.setLabel(key, labels[key][0], labels[key][1])

		if self.dims == 1:
			self.t = np.linspace(-self.timewindow, 0.0, self._bufsize)

		# QTimer
		self.timer = QtCore.QTimer()
		self.timer.timeout.connect(self.updateplot)
		self.timer.start(self._interval)

		# Plotting buffers
		self.curve_colors = [(255, 255, 255), (0, 255, 0), (0, 0, 255), (255, 255, 255), (0, 255, 0), (0, 0, 255), (0, 255, 255), (0, 255, 255), (255, 0, 155), (100, 255, 0), (100, 100, 100), (0, 255, 255), (255, 0, 155), (100, 255, 0), (100, 100, 100)]  if colors is None else colors

		self.plot_lines = []
		self.ranges = np.zeros((self.dims, 2))
		if limits is not None:
			if len(limits) == 1:
				self.ranges[1,0] = limits[0][0]
				self.ranges[1,1] = limits[0][1]
				self.setYRange(self.ranges[1, 0], self.ranges[1, 1], padding=0.05)
			elif len(limits) == 2:
				self.ranges[0,0] = limits[0][0]
				self.ranges[0,1] = limits[0][1]
				self.ranges[1,0] = limits[1][0]
				self.ranges[1,1] = limits[1][1]
				self.setXRange(self.ranges[0, 0], self.ranges[0, 1], padding=0.05)
				self.setYRange(self.ranges[1, 0], self.ranges[1, 1], padding=0.05)

	def add_curve(self):
		# CustomDashLine
		# DashDotDotLine
		# DashDotLine
		# DashLine
		# DotLine
		# SolidLine
		if self.n_curves == 0:
			self.givedata_buffer = np.zeros((1,self.dims), dtype=np.float)
			self.plotData_y = np.zeros((1, self._bufsize), dtype=np.float)
			if self.dims != 1:
				self.plotData_x = np.zeros((1, self._bufsize), dtype=np.float)
		else:
			self.givedata_buffer = np.vstack( (self.givedata_buffer, np.zeros((1,self.dims), dtype=np.float)) )
			self.plotData_y = np.vstack( (self.plotData_y, np.zeros((1, self._bufsize), dtype=np.float)) )
			if self.dims != 1:
				self.plotData_x = np.vstack( (self.plotData_x, np.zeros((1, self._bufsize), dtype=np.float)) )

		if self.dims == 1:
			self.plot_lines.append( self.plot(x=self.t, y=self.plotData_y[-1], pen=pg.mkPen(color=self.curve_colors[self.n_curves % len(self.curve_colors)], width=3, style=QtCore.Qt.DotLine) ) )
		else:
			self.plot_lines.append( self.plot(x=self.plotData_x[-1], y=self.plotData_y[-1], pen=pg.mkPen(color=self.curve_colors[self.n_curves % len(self.curve_colors)], width=3, style=QtCore.Qt.DotLine) ) )

		self.n_curves+=1

	# def set_dims(self, dims):
	# 	self.dims = dims

	def givedata(self, value, dim, n_curve):
		# Plot settings
		if value*1.1 < self.ranges[dim, 0]:
			self.ranges[dim, 0] = value*1.1
			if dim == 0 and self.dims == 2:
				self.setXRange(self.ranges[dim, 0], self.ranges[dim, 1], padding=0.05)
			else:
				self.setYRange(self.ranges[dim, 0], self.ranges[dim, 1], padding=0.05)

		elif value*1.1 > self.ranges[dim, 1]:
			self.ranges[dim, 1] = value*1.1
			if dim == 0 and self.dims == 2:
				self.setXRange(self.ranges[dim, 0], self.ranges[dim, 1], padding=0.05)
			else:
				self.setYRange(self.ranges[dim, 0], self.ranges[dim, 1], padding=0.05)

		self.givedata_buffer[n_curve, dim] = value

	def updateplot(self):
		if self.givedata_buffer is not None:
			# Get the data from buffer variable
			if self.dims == 1:
				new_y  = self.givedata_buffer[:,0]
			else:
				new_x  = self.givedata_buffer[:,0]
				new_y  = self.givedata_buffer[:,1]
				self.plotData_x = np.hstack( (self.plotData_x, new_x.reshape((-1, 1)) ) )
				self.plotData_x = self.plotData_x[:, -self._bufsize:]
			self.plotData_y = np.hstack( (self.plotData_y, new_y.reshape((-1, 1)) ) )
			self.plotData_y = self.plotData_y[:, -self._bufsize:]

			for i in range(self.n_curves):
				# If non-keyword arguments are used, they will be interpreted as setData(y) for a single argument and setData(x, y) for two arguments.
				if self.dims == 1:
					self.plot_lines[i].setData(self.t, self.plotData_y[i])
				else:
					self.plot_lines[i].setData(self.plotData_x[i], self.plotData_y[i])
		else:
			pass

	def data_wrapper(self, func, *args, **kwargs):
		def wrapped(*args, **kwargs):
			value, dim, n_curve = func(*args, **kwargs)
			self.givedata(value, dim, n_curve)
		return wrapped