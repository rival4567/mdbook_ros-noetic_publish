#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun pkg_simbha_bot autoeval.py
# This file will print results on the terminal window


def calc_simulation_time(pos_time_list):
    '''Take space-time list of simbhabot as argument and returns simulation
    time between the time micromouse starts moving till it reaches the centre
    of maze.'''
    start_pos = (pos_time_list[0][0], pos_time_list[0][1])
    if abs(start_pos[0]) < 0.1 and abs(start_pos[1]) < 0.1:
        is_spawned_at_correct_pos = True
    else:
        is_spawned_at_correct_pos = False
    is_start_time_found = False
    for space_time in pos_time_list:
        # Going over the space-time list of micrmouse
        if (abs(start_pos[0]-space_time[0]) > 0.1 or abs(start_pos[1]-space_time[1]) > 0.1) and not is_start_time_found:
            # If the micromouse starts moving
            start_time = space_time[2].to_sec()
            is_start_time_found = True
        if 6.9 < space_time[0] < 7.1 and -0.1 < space_time[1] < 0.1:
            end_time = space_time[2].to_sec()
            break
    return is_spawned_at_correct_pos, end_time - start_time


def main():
    bag_files = []
    # Change this path to where all bag files are located.
    path = "/home/ubuntu/workspace/src/pkg_simbha_bot/bag_files"
    for file in glob.glob(path + "/*.bag"):
        bag_files.append(file)
    with open(path + '/' + 'results.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',',
                                 quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow(['Filename', 'Does bag files has rostopics?',
                              'Is spawned at correct position?', 'Collision', 'Remark', 'Simulation Time'])

        # iterate through each file
        for file in bag_files:
            bag = rosbag.Bag(file)
            is_passed = False
            is_topic_present = False
            dest_reached = False
            collision = False
            is_spawned_correctly = False
            pos_time_list = []
            time_taken = None
            # look throuhg topics
            topics = bag.get_type_and_topic_info()[1].keys()
            if '/odom' in topics and '/scan' in topics:
                is_topic_present = True
            else:
                is_topic_present = False
                print('topic not present')
                is_passed = False

            for topic, msg, t in bag.read_messages(topics=['/odom']):
                # check if destination reached
                x_bot = msg.pose.pose.position.x
                y_bot = msg.pose.pose.position.y

                pos_time_list.append((x_bot, y_bot, t))

            for topic, msg, t in bag.read_messages(topics=['/scan']):
                # check collision
                laser_ranges = msg.ranges
                no_of_points = len(laser_ranges)
                for i in range(no_of_points):
                    # assuming distance less than 0.07 will result in collision
                    # This value won't even be detected if the min_range parameter of laser is too high in urdf
                    if laser_ranges[i] < 0.07:
                        collision = True

            if 6.9 < x_bot < 7.1 and -0.1 < y_bot < 0.1:
                dest_reached = True
                is_spawned_correctly, time_taken = calc_simulation_time(
                    pos_time_list)

            if dest_reached and not collision and is_spawned_correctly:
                is_passed = True

            if is_topic_present and is_passed:
                remark = "Passed!"
                print(file + ' Result: passed!')
            elif not dest_reached:
                remark = "Failed!, goal not reached."
                print(file + ' Result: failed!')
            elif collision:
                remark = "Failed!, collision detected."
            else:
                remark = "Failed!"
                print(file + ' Result: failed!')

            data_writer.writerow(
                [file.split('/')[-1].split('.')[-2], is_topic_present, is_spawned_correctly, collision, remark, time_taken])

        bag.close()
    data_file.close()


if __name__ == '__main__':
    main()
