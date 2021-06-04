#!/usr/bin/env python3

import rospy

def main():

    # 1. Make the script a ROS Node.
    rospy.init_node('node_hello_ros', anonymous=True)

    # 2. Print Hello World!
    rospy.loginfo("Hello World!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass