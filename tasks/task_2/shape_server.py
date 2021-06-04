#! /usr/bin/env python3

import rospy
import actionlib
import pkg_task2.msg
import math
import angles
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class ShapeAction:
	# create messages that are used to publish feedback/result
	_feedback = pkg_task2.msg.DrawShapeFeedback()
	_result = pkg_task2.msg.DrawShapeResult()

	def __init__(self, name):
		self._action_name = name
		rospy.Subscriber('/turtle1/pose', Pose, self.turtle_pose_cb)
		self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.vel_msg = Twist()
		self._as = actionlib.SimpleActionServer(self._action_name, pkg_task2.msg.DrawShapeAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()
		rospy.loginfo('Server Started.')

	def execute_cb(self, goal):
		success = True
		if goal.shape == 'square':
			# helper variables
			r = rospy.Rate(1)
			edge_len = math.sqrt(2)*goal.radius
			self._result.side_length = edge_len
			self._result.interior_angle = math.pi/2
			l_scale = 6.0
			a_scale = 6.0
			min_error = 0.00001
			
			# starting execution
			rospy.loginfo('Executing, drawing a square with edge length %f', edge_len)
			for edge in range(4): # drawing individual edges
				# check that preempt has not been requested by the client
				if self._as.is_preempt_requested():
					rospy.loginfo('%s: Preempted' % self._action_name)
					self._as.set_preempted()
					success = False
					break

				start_x = self.turtle_x
				start_y = self.turtle_y
				start_theta = self.turtle_theta

				rospy.loginfo('Rotating...')
				theta_error = angles.normalize_angle_positive(math.pi - self._result.interior_angle - (self.turtle_theta - start_theta))
				while (theta_error > min_error):
					self.vel_msg.linear.x = 0
					self.vel_msg.angular.z = theta_error*a_scale
					self.vel_pub.publish(self.vel_msg)
					theta_error = angles.normalize_angle_positive(math.pi - self._result.interior_angle - (self.turtle_theta - start_theta))

				rospy.loginfo('Forward march!')
				dis_error = edge_len - abs(math.sqrt((start_x - self.turtle_x)*(start_x - self.turtle_x) + (start_y - self.turtle_y)*(start_y - self.turtle_y)))
				while (dis_error > min_error):
					self.vel_msg.linear.x = dis_error*l_scale
					self.vel_msg.angular.z = 0
					self.vel_pub.publish(self.vel_msg)
					dis_error = edge_len - abs(math.sqrt((start_x - self.turtle_x)*(start_x - self.turtle_x) + (start_y - self.turtle_y)*(start_y - self.turtle_y)))
				
				self._feedback.percent = 25.0+25.0*edge
				self._as.publish_feedback(self._feedback)
		elif goal.shape == 'circle':
			circumference = 2*math.pi*goal.radius
			rospy.loginfo('Executing, drawing a circle with circumference %f', circumference)
			self.vel_msg.linear.x = circumference
			self.vel_msg.angular.z = 2*math.pi
			self.vel_pub.publish(self.vel_msg)
			self._result.side_length = circumference
			self._result.interior_angle = 0

		if success:
			rospy.loginfo('%s: Succeeded' % self._action_name)
			self._as.set_succeeded(self._result)

	def turtle_pose_cb(self, pose):
		self.turtle_x = pose.x
		self.turtle_y = pose.y
		self.turtle_theta = pose.theta
		self.l_vel = pose.linear_velocity
		self.a_vel = pose.angular_velocity

if __name__ == '__main__':
	rospy.init_node('DrawShape')
	server = ShapeAction(rospy.get_name())
	rospy.spin()