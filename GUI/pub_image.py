import sys, rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main(args):
	rospy.init_node('image_converter', anonymous=True)

	image_pub = rospy.Publisher("image_topic",Image, queue_size=1)
	bridge = CvBridge()

	cap = cv2.VideoCapture(0)
	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		ret, frame = cap.read()
		image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
		rate.sleep()
	cap.release()

if __name__ == '__main__':
	main(sys.argv)