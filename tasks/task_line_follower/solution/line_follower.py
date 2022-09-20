#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class Follower:
	def __init__(self):
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/follower/camera1/image_raw', Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('/follower_diff_drive_controller/cmd_vel', Twist, queue_size=1)
		self.twist = Twist()
	def image_callback(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_yellow = numpy.array([10, 10, 10])
		upper_yellow = numpy.array([255, 255, 250])
		mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
		h, w, d = image.shape
		search_top = int(3*h/4)
		search_bot = int(3*h/4 + 20)
		mask[0:search_top, 0:w] = 0
		mask[search_bot:h, 0:w] = 0
		M = cv2.moments(mask)
		if M['m00'] > 0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			# CONTROL starts
			err = cx - w/2
			self.twist.linear.x = 0.4
			self.twist.angular.z = -float(err)*2.2 / 1000
			self.cmd_vel_pub.publish(self.twist)
			# CONTROL ends
		else:
			self.twist.linear.x = 0.0
			self.twist.angular.z = 0.5
			self.cmd_vel_pub.publish(self.twist)
		cv2.imshow("mask",mask)
		cv2.imshow("output", image)
		cv2.waitKey(3)

if __name__ == '__main__':
	rospy.init_node('follower')
	follower = Follower()
	rospy.spin()