#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun pkg_task2 autoeval.py


def main():
	bag_files = []
	is_passed = True
	# Change this path to where all bag files are located.
	path = "/home/ubuntu/workspace/src/pkg_task2/bagfiles"
	for file in glob.glob(path + "/*.bag"):
		bag_files.append(file)

	# iterate through each file
	for file in bag_files:
		bag = rosbag.Bag(file)
		# look throuhg topics
		topics = bag.get_type_and_topic_info()[1].keys()
		if '/turtle1/pose' and '/turtle1/cmd_vel' and '/DrawShape/status' and '/DrawShape/goal' and '/DrawShape/feedback' and '/DrawShape/result' in topics:
			is_topic_present = True
		else:
			is_topic_present = False
			print('topic not present')
			is_passed = False

		count = [0, 0, 0, 0]
		last_val = 0
		for topic, msg, t in bag.read_messages(topics=['/DrawShape/status']):
			if not msg.status_list:
				continue
			if msg.status_list[0].status < last_val:
				print('last_val')
				is_passed = False
				break
			last_val = msg.status_list[0].status
			count[last_val] = count[last_val]+1

		if count[1] == 0 or count[2] == 0 or count[3] == 0:
			print('count')
			is_passed = False

		circle_mov = 0
		square_mov = 0
		for topic, msg, t in bag.read_messages(topics=['/turtle1/cmd_vel']):
			if msg.linear.x != 0 and msg.angular.z != 0:
				circle_mov = circle_mov+1
			elif msg.linear.x != 0 and msg.angular.z == 0:
				square_mov = square_mov+1

		if square_mov >= 1 and circle_mov >= 1:
			is_passed = True
		else:
			print(square_mov, circle_mov, 'sc')
			is_passed = False

		if is_topic_present == True and is_passed == True:
			print(file + ' Result: passed!')
		else:
			print(file + ' Result: failed!')

if __name__ == '__main__':
	main()