#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun pkg_task2 autoeval.py
# This file will print results on the terminal window

def lat_to_x(input_latitude):
    return 110692.0702932625 * (input_latitude - 19)

def long_to_y(input_longitude):
    return -105292.0089353767 * (input_longitude - 72)

def x_to_lat(input_x):
    return 19 + (input_x)/110692.0702932625

def y_to_lon(input_y):
    return 72 - (input_y)/105292.0089353767

def main():
	bag_files = []
	is_passed = False
	# Change this path to where all bag files are located.
	path = "/home/ubuntu/workspace/src/eysip/vitarana_drone/bagfiles"
	for file in glob.glob(path + "/*.bag"):
		bag_files.append(file)

	# iterate through each file
	for file in bag_files:
		bag = rosbag.Bag(file)
		# look throuhg topics
		topics = bag.get_type_and_topic_info()[1].keys()
		if '/edrone/gps' in topics:
			is_topic_present = True
		else:
			is_topic_present = False
			print('topic not present')
			is_passed = False

		for topic, msg, t in bag.read_messages(topics=['/edrone/gps']):
			if abs(msg.altitude-32.15999670352447) <= 0.2 and abs(lat_to_x(msg.latitude)-lat_to_x(18.999981931836018)) <= 0.2 and abs(long_to_y(msg.longitude)-long_to_y(71.99983191056211)) <= 0.2:
				is_passed = True

		if is_topic_present == True and is_passed == True:
			print(file + ' Result: passed!')
		else:
			print(file + ' Result: failed!')

if __name__ == '__main__':
	main()