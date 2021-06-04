#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SpawnResponse
from turtlesim.srv import SetPen
import math


class turtle:
    '''This class is used to spawn another turtle on turtlesim env and make it follow original
    spawned turtle.'''

    # Constructor
    def __init__(self):

        # Initialize node
        rospy.init_node('node_catch_me_turtle', anonymous=True)

        # Subscribe to pose of turtles
        self.t1_pose_subscriber = rospy.Subscriber(
            '/turtle1/pose', Pose, self.t1_pose_callback)
        self.t2_pose_subscriber = rospy.Subscriber(
            '/turtle2/pose', Pose, self.t2_pose_callback)

        # Spawn second turtle on turtlesim env
        self.spawn_turtle_srv = rospy.ServiceProxy('/spawn', Spawn)
        self.spawn_turtle_srv.wait_for_service()
        rospy.loginfo(self.spawn_turtle_srv.call(
            1.0, 1.0, 0.0, 'turtle2').name)

        # To publish on cmd_vel of turtle2
        self.velocity_publisher = rospy.Publisher(
            '/turtle2/cmd_vel', Twist, queue_size=10)

        # Set pen of both turtles
        self.set_t1_pen_srv = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.set_t2_pen_srv = rospy.ServiceProxy('/turtle2/set_pen', SetPen)
        self.set_t1_pen_srv.wait_for_service()
        self.set_t2_pen_srv.wait_for_service()
        self.set_t1_pen_srv.call(255, 255, 0, 5, 0)  # yellow
        self.set_t2_pen_srv.call(255, 0, 0, 4, 0)   # red

        self.t1_pose = Pose()
        self.t2_pose = Pose()
        self.twist = Twist()
        self.rate = rospy.Rate(10)  # Rate in Hz

    def t1_pose_callback(self, pos_msg):
        self.t1_pose = pos_msg
        self.t1_pose.x = pos_msg.x
        self.t1_pose.y = pos_msg.y

    def t2_pose_callback(self, pos_msg):
        self.t2_pose = pos_msg
        self.t2_pose.x = pos_msg.x
        self.t2_pose.y = pos_msg.y
        self.t2_pose.theta = pos_msg.theta

    def turtle_follow_turtle(self):
        '''This method makes turtle2 follow turtle1.'''

        # Constants to move turtle. Change these to speed up or speed down motion.
        k_linear = 0.5
        k_angular = 4.0

        # Infinite loop
        while not rospy.is_shutdown():

            # Calculate distance between two turtles
            distance = math.sqrt(
                (self.t2_pose.x - self.t1_pose.x)**2 + (self.t2_pose.y - self.t1_pose.y)**2)
            linear_speed = k_linear * distance

            # Angle between two turtles in radians
            angle = math.atan2(self.t1_pose.y - self.t2_pose.y,
                               self.t1_pose.x - self.t2_pose.x)
            # Need to subtract current angle of turtle2 to move relatively
            angular_speed = k_angular * (angle - self.t2_pose.theta)

            # resolution of 0.01
            if distance > 0.01:
                self.twist.linear.x = linear_speed
                self.twist.angular.z = angular_speed
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0

            # Publishing values on /turtle2/cmd_vel
            self.velocity_publisher.publish(self.twist)
            self.rate.sleep()

    # Destructor
    def __del__(self):
        # Unregister from rostopics
        self.t1_pose_subscriber.unregister()
        self.t2_pose_subscriber.unregister()
        rospy.loginfo(
            '\033[94m' + "Object of class controlSimEnv Deleted." + '\033[0m')


def main():
    t = turtle()
    t.turtle_follow_turtle()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
