#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun pkg_task0 autoeval.py
# It will create a results.csv to the location where all bag_files is located.


def evaluate_score(bag):
    '''This method evaulates scores by looking into bag files. It then
    returns strings showing outcome.'''

    topics = bag.get_type_and_topic_info()[1].keys()
    if '/turtle1/pose' and '/turtle2/pose' and '/turtle1/cmd_vel' and '/turtle2/cmd_vel' in topics:
        is_topic_present = True
    else:
        is_topic_present = False

    t1_pos_msg_list = []
    t2_pos_msg_list = []

    for topic, msg, t in bag.read_messages(topics=['/turtle1/pose']):
        t1_pos_msg_list.append((t, msg))
    for topic, msg, t in bag.read_messages(topics=['/turtle2/pose']):
        t2_pos_msg_list.append((t, msg))

    dist_list = merge_list_with_calculated_distance(
        t1_pos_msg_list, t2_pos_msg_list)

    t1_cmd_vel_list = []
    t2_cmd_vel_list = []
    for topic, msg, t in bag.read_messages(topics=['/turtle1/cmd_vel']):
        t1_cmd_vel_list.append((t, msg))
    for topic, msg, t in bag.read_messages(topics=['/turtle2/cmd_vel']):
        t2_cmd_vel_list.append((t, msg))

    is_turtle2_running = is_turtle2_moving_when_turtle1_moves(
        t1_cmd_vel_list, t2_pos_msg_list)

    # Get the first publish timestamp of /turtle1/cmd_vel
    first_pub = t1_cmd_vel_list[0][0]

    # Find the closest value in the dist_list timestamp
    before_mov_t1 = min(dist_list, key=lambda list: abs(list[0] - first_pub))
    idx = dist_list.index(before_mov_t1)

    i = 1   # To ignore first element starting from 1 instead of 0
    count = 0   # Count the number of time distance is increasing.
    while i < idx:
        if dist_list[i][1] > dist_list[i-1][1]:
            count += 1
        i += 1
    percent_of_distance_increasing = (count * 100) / idx

    if dist_list[idx - 1][1] < 0.05:
        is_reach_turtle1_spawn = True
    else:
        is_reach_turtle1_spawn = False

    if dist_list[-1][1] < 0.05 and percent_of_distance_increasing < 5 and is_turtle2_running:
        is_turtle2_following = True
    else:
        is_turtle2_following = False

    bag.close()
    return is_topic_present, is_reach_turtle1_spawn, is_turtle2_following


def calculate_distance(x0, y0, x1, y1):
    '''Calculate distance between two points in 2D. Input points (x0, y0) and
    (x1, y1).'''
    return math.sqrt((x0 - x1)**2 + (y0 - y1)**2)


def is_turtle2_moving_when_turtle1_moves(list1, list2):
    '''Check if turtle2 is running when turtle1 is moving. Return True
    if successful.'''
    closest_value = []
    # Create a list with timestamps similar to turtle1 moving out of turtle2
    # timestamps.
    for t, msg in list1:
        closest_value.append(min(
            list2, key=lambda list: abs(list[0] - t)))

    count = 0
    for t, msg in closest_value:
        if msg.linear_velocity == 0 and msg.angular_velocity == 0:
            count += 1
    # Currently it is set at 5 percent.
    if (count * 100) / len(closest_value) < 5:
        return True
    else:
        return False


def merge_list_with_calculated_distance(list1, list2):
    '''Takes turtle1 and turtl2 position list with timestamps as arguments and
    return a list of distance between turtle1 and turtle2 with timestamps.'''
    l1 = [(t, msg, 1) for (t, msg) in list1]
    l2 = [(t, msg, 2) for (t, msg) in list2]
    new_list = l1 + l2
    # Sort list based on timestamps and index provided for t1 and t2 as 1, 2
    sorted_list = sorted(
        new_list, key=lambda element: (element[0], element[2]))
    c1 = list1[0][1]  # first value of msg
    c2 = list2[0][1]
    last_time = -1
    result = []
    for t, msg, index in sorted_list:
        if index == 1:
            c1 = msg
        elif index == 2:
            c2 = msg
        elif t == last_time:
            result.pop()
        result.append((t, calculate_distance(c1.x, c1.y, c2.x, c2.y)))
    return result


def get_remark(score):
    '''Returns the result based on success rate.'''
    if not score[0]:
        return "Task Failed! Bag file doesn't have all topics.", False
    else:
        if not score[1]:
            return "Task Failed! turtle2 Couldn't reach turtle1 at it's spawn position.", True
        else:
            if not score[2]:
                return "Task Failed! Probably the bag file isn't complete.", True
            else:
                return "Passed.", False


def main():
    bag_files = []
    # Change this path to where all bag files are located.
    path = "/home/ubuntu/workspace/src/pkg_task0/bag_files"
    for file in glob.glob(path + "/*.bag"):
        bag_files.append(file)

    print(bag_files)
    with open(path + '/' + 'results.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',',
                                 quotechar='"', quoting=csv.QUOTE_MINIMAL)

        data_writer.writerow(['filename', 'Is turtle2 spawned?',
                             'Does turtle2 reach turtle1 spawn position?', 'Is turtle2 following turtle1?', 'Remark', 'Is Manual Evaluation required if a query is raised?'])
        for file in bag_files:
            bag = rosbag.Bag(file)
            score = evaluate_score(bag)
            remark = get_remark(score)
            data_writer.writerow(
                [file.split('/')[-1].split('.')[-2], score[0], score[1], score[2], remark[0], remark[1]])

    print("Finished creating csv file!")


if __name__ == '__main__':
    main()
