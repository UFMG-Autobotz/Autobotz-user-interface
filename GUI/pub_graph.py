import numpy as np
import math, os, random, sys, threading, time, rospy
from std_msgs.msg import Float32

def plot_XY(pub_x_name, pub_y_name, pub_z_name):

	rospy.init_node('graph_pub', anonymous=True)

	def data_gen(dim):
		noise = random.normalvariate(0., .2)
		if dim == 1:
			data_x = math.cos(time.time() * 2 *math.pi)
			return data_x
		if dim == 2:
			data_y = math.sin(time.time() * 2 *math.pi)**3
			return data_y


	pub_x = rospy.Publisher(pub_x_name, Float32, queue_size=1)
	pub_y = rospy.Publisher(pub_y_name, Float32, queue_size=1)
	# pub_z = rospy.Publisher(pub_z_name, Float32, queue_size=1)
	rate = rospy.Rate(200) # 10hz
	while not rospy.is_shutdown():
		pub_x.publish( data_gen(1) )
		pub_y.publish( data_gen(2) )
		# pub_z.publish( data_gen(3) )
		rate.sleep()

if __name__ == '__main__':
	plot_XY('/graph_x', '/graph_y', '/graph_z')