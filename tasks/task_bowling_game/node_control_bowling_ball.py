#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import math


class BowlingBall:
    def __init__(self):
        rospy.init_node('control_bowling_ball', anonymous=True)

        self.sub_ = rospy.Subscriber(
            '/bowling_game/joint_states', JointState, self.clbk_pos)

        self.pub_x = rospy.Publisher(
            '/bowling_game/x_effort_joint_controller/command', Float64, queue_size=1)

        self.pub_y = rospy.Publisher(
            '/bowling_game/y_effort_joint_controller/command', Float64, queue_size=1)

        self.pub_z = rospy.Publisher(
            '/bowling_game/z_effort_joint_controller/command', Float64, queue_size=1)

        self.ball_state = JointState()
        self.pos_y = 0
        self.rate = rospy.Rate(20)  # Rate in Hz

    def clbk_pos(self, msg):
        self.ball_state.position = msg.position
        self.pos_y = msg.position[1]

    def knock_pins(self):
        # Infinite Loop
        i = 0
        while not rospy.is_shutdown():
            if i < 5:
                self.pub_x.publish(3.5)
            print(self.pos_y)
            if self.pos_y < -0.1:
                self.pub_y.publish(-0.5)
            elif self.pos_y > -0.1:
                self.pub_y.publish(0.5)
            self.rate.sleep()

    def __del__(self):
        self.sub_.unregister()
        rospy.loginfo("Object of class BowlingBall deleted.")


def main():
    b = BowlingBall()
    b.knock_pins()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
