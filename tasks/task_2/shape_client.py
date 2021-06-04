#! /usr/bin/env python3

import rospy
import actionlib
import pkg_task2.msg

def shape_client():
	client = actionlib.SimpleActionClient('DrawShape', pkg_task2.msg.DrawShapeAction)
	rospy.loginfo('client ready.... waiting for server')
	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	# Creates a goal to send to the server
	rospy.loginfo('Sending gaol1...')
	goal = pkg_task2.msg.DrawShapeGoal()
	goal.shape = 'square'
	goal.radius = 1
	# Sends goal to the action server
	client.send_goal(goal)


	r = rospy.Rate(0.25)
	r.sleep()
	rospy.loginfo('Sending gaol2...')
	goal.shape = 'circle'
	goal.radius = 2
	client.send_goal(goal)

	# wait for ther server to finish performing the action
	client.wait_for_result()

	# return result
	return client.get_result()

if __name__ == '__main__':
	try:
		# Initializes a rospy node so that the SimpleActionClient can
		rospy.init_node('shape_client_py')
		result = shape_client()
		rospy.loginfo("Result: side length = %f, interior angle = %f" % (result.side_length, result.interior_angle))
	except:
		rospy.loginfo('program interrupted before completion')