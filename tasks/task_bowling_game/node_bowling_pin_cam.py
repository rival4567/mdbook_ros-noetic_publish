#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import math
import cv2


class DepthCamera(object):
    def __init__(self):
        rospy.init_node('node_live_cam', anonymous=True)

        self.bridge = CvBridge()

        self.sub_cam = rospy.Subscriber(
            "/depth_camera/color/image_raw", Image, self.clbk_cam)

        self.image = None
        self.rate = rospy.Rate(20)  # Rate in Hz

    def clbk_cam(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as err:
            rospy.logerr(err)

        self.image = cv_image
        cv2.imshow("bowling pin cam", self.image)
        cv2.waitKey(1)

    def work(self):
        # Infinite Loop
        while not rospy.is_shutdown():
            self.rate.sleep()

    def __del__(self):
        self.sub_cam.unregister()
        rospy.loginfo("Object of class BowlingBall deleted.")


def main():
    dc = DepthCamera()
    dc.work()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
