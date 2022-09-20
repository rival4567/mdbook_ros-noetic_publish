#!/usr/bin/env python3
import glob
import rosbag
import math
import csv
import tf
from operator import countOf, itemgetter

# USAGE: rosrun micromouse autoeval.py
# ROS Topics required: /gazebo/link_states


def main():
    bag_files = []
    # Change this path to where all bag files are located.
    path = "/home/ubuntu/workspace/src/pkg_gazebo/bag_files"
    for file in glob.glob(path + "/*.bag"):
        bag_files.append(file)
    with open(path + '/' + 'results.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',',
                                 quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow(['Filename', 'Does bag files has rostopic?', 'Total Pins knocked',
                              'Strike', 'Remark'])

        # iterate through each file
        for file in bag_files:
            bag = rosbag.Bag(file)
            strike = False
            is_topic_present = False
            pins_knocked = 0
            # look throuhg topics
            topics = bag.get_type_and_topic_info()[1].keys()
            if '/gazebo/link_states' in topics:
                is_topic_present = True
            else:
                is_topic_present = False
                print('topic not present')
                strike = False

            for topic, msg, t in bag.read_messages(topics=['/gazebo/link_states']):
                # check the orientation of links
                rpy_list = []
                index = [i for i in range(2, 12)]
                bowling_pins_links = itemgetter(*index)(msg.pose)
                for pose in bowling_pins_links:
                    quaternion = (pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w)
                    rpy_list.append(tf.transformations.euler_from_quaternion(
                        quaternion))

            # check if all pins are knocked down
            strike = all(round(rpy[0], 2) != 1.57 for rpy in rpy_list)

            for rpy in rpy_list:
                if round(rpy[0], 2) != 1.57:
                    pins_knocked += 1

            if is_topic_present and strike:
                remark = "Strike! All pins knocked down."
                print(file + ' Result: passed!')
            elif is_topic_present and pins_knocked > 0:
                remark = str(pins_knocked) + " pins knocked down!"
            else:
                remark = "Failed!"
                print(file + ' Result: failed!')

            data_writer.writerow(
                [file.split('/')[-1].split('.')[-2], is_topic_present, pins_knocked, strike, remark])

        bag.close()
    data_file.close()


if __name__ == '__main__':
    main()
