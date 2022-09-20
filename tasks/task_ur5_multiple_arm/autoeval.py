#!/usr/bin/env python3
import glob
import rosbag
import math
import csv

# USAGE: rosrun pkg_vb_solution autoeval.py
# ROS Topics required: /eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1
# /eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2


def check_for_successful_drop(package):
    # Check for successful drop from the last location of package
    # UR5_1 drop location
    if 0.5 < package[1] < 1.0 and 0.2 < package[2] < 0.7:
        return True
    # UR5_2 drop location
    if -2.5 < package[1] < -2.0 and -0.5 < package[2] < 0.0:
        return True
    else:
        return False


def main():
    bag_files = []
    # Change this path to where all bag files are located.
    path = "/home/ubuntu/workspace/src/pkg_vb_solution/bag_files"
    for file in glob.glob(path + "/*.bag"):
        bag_files.append(file)
    with open(path + '/' + 'results.csv', mode='w') as data_file:
        data_writer = csv.writer(data_file, delimiter=',',
                                 quotechar='"', quoting=csv.QUOTE_MINIMAL)
        data_writer.writerow(['Filename', 'Does bag files has rostopic?', 'Package detected by UR5_1',
                              'Package detected by UR5_2', 'Total Package detected', 'Successfully dropped by UR5_1',
                              'Successfully dropped by UR5_2', 'Total Dropped', 'Simulation Time', 'Is Passed', 'Remark'])

        # iterate through each file
        for file in bag_files:
            bag = rosbag.Bag(file)
            is_topic_present = False
            is_passed = False
            ur5_1_package_detected = []
            ur5_2_package_detected = []
            package_detected = 0
            ur5_1_successful_drop = []
            ur5_2_successful_drop = []
            package_dropped = 0
            time_taken = None
            previous_model = None
            ur5_1_model_list = []
            ur5_2_model_list = []
            remark = ''

            # look throuhg topics
            topics = bag.get_type_and_topic_info()[1].keys()
            if '/eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1' in topics and \
                    '/eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2' in topics:
                is_topic_present = True
            else:
                is_topic_present = False
                print('topic not present')
                is_passed = False

            for topic, msg, t in bag.read_messages(
                    topics=['/eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1']):
                for model in msg.models:
                    ur5_1_model_list.append(
                        (model.type, msg.pose.position.x, msg.pose.position.y, t))
                    if model.type != previous_model:
                        ur5_1_package_detected.append(model.type)
                        package_detected += 1
                    previous_model = model.type

            for topic, msg, t in bag.read_messages(
                    topics=['/eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2']):
                for model in msg.models:
                    ur5_2_model_list.append(
                        (model.type, msg.pose.position.x, msg.pose.position.y, t))
                    if model.type != previous_model:
                        ur5_2_package_detected.append(model.type)
                        package_detected += 1
                    previous_model = model.type

            for package in ur5_1_package_detected:
                package_info = [(model_type, x, y, t) for (
                    model_type, x, y, t) in ur5_1_model_list if model_type == package]
                # Check for successful drop from the last location of package
                success = check_for_successful_drop(
                    package_info[-1])
                if success:
                    package_dropped += 1
                    ur5_1_successful_drop.append(package_info[0][0])

            for package in ur5_2_package_detected:
                package_info = [(model_type, x, y, t) for (
                    model_type, x, y, t) in ur5_2_model_list if model_type == package]
                # Check for successful drop from the last location of package
                success = check_for_successful_drop(
                    package_info[-1])
                if success:
                    package_dropped += 1
                ur5_2_successful_drop.append(package_info[0][0])

            if is_topic_present:
                if ur5_1_model_list[0][3] < ur5_2_model_list[0][3]:
                    start_time = ur5_1_model_list[0][3].to_sec()
                elif ur5_1_model_list[0][3] > ur5_2_model_list[0][3]:
                    start_time = ur5_2_model_list[0][3].to_sec()

                if ur5_1_model_list[-1][3] > ur5_2_model_list[-1][3]:
                    end_time = ur5_1_model_list[-1][3].to_sec()
                elif ur5_1_model_list[-1][3] < ur5_2_model_list[-1][3]:
                    end_time = ur5_1_model_list[-1][3].to_sec()

                time_taken = end_time - start_time

            if package_dropped == 6:
                is_passed = True

            if is_topic_present and is_passed:
                remark = "Passed! All packages are sorted."
                print(file + ' Result: passed!')
            elif is_topic_present and package_dropped < 6:
                remark = "Only " + str(package_dropped) + \
                    " packages are sorted correctly."
                print(file + ' Result: Incomplete.')
            else:
                remark = "Failed!"
                print(file + ' Result: failed!')

            data_writer.writerow(
                [file.split('/')[-1].split('.')[-2], is_topic_present, ur5_1_package_detected, ur5_2_package_detected,
                 package_detected, ur5_1_successful_drop, ur5_2_successful_drop, package_dropped, time_taken, is_passed, remark])

        bag.close()
    data_file.close()


if __name__ == '__main__':
    main()
