#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def callback(data, pub):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	pub.publish( data**2 )

def listener():
	rospy.init_node('listener', anonymous=True)
	pub = rospy.Publisher('/my_velodyne/vel_feedback', Float32, queue_size=1)
	rospy.Subscriber("/my_velodyne/vel_cmd", Float32, lambda data: callback(data.data, pub))
	rospy.spin()

if __name__ == '__main__':
	listener()