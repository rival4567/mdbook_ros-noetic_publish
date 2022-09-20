#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf import transformations
import random_spawn

import math

MX_RANGE = 10

FINAL_TARGET = Point()
FINAL_TARGET.x = 7.0
FINAL_TARGET.y = 0.0

YAW_PRECISION = 5 * (math.pi / 180)  # 5 degrees
DIST_PRECISION = 0.10


class ObstacleAvoider(object):

    def __init__(self):

        self.regions = {
            'right': 0.0,
            'fright': 0.0,
            'front': 0.0,
            'fleft': 0.0,
            'left': 0.0
        }

        self.velocity_msg = Twist()
        self.move_state = 0
        self.obstacle_avoidance_state = 0
        self.robot_state = 0
        self.reached_destination = False

        self.pose = [0.0, 0.0, 0.0]
        self.curr_position = Point()

        self.publisher_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.subscription = rospy.Subscriber(
            '/odom', Odometry, self.odom_callback)

        self.subscription = rospy.Subscriber(
            '/scan', LaserScan, self.laserscan_callback)

        self.rate = rospy.Rate(20)

    # angle difference or yaw between any point and the bot.
    def error_angle(self, point):
        rx = self.pose[0]
        ry = self.pose[1]
        theta = self.pose[2]*(math.pi/180)
        theta_goal = math.atan2(point.y - ry, point.x - rx)
        error = self.normalize_angle(theta_goal - theta)
        return error

    # function that gives linear distance between any point and the bot.
    def calc_dist_points(self, point1, point2):
        dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
        return dist

    # function to limit the bot rotation between -pi to pi
    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def fix_yaw(self, des_pos):
        err_yaw = self.error_angle(des_pos)

        if math.fabs(err_yaw) > YAW_PRECISION:
            if err_yaw > 0:
                self.velocity_msg.linear.x = 0.0
                self.velocity_msg.angular.z = 0.5
            else:
                self.velocity_msg.linear.x = 0.0
                self.velocity_msg.angular.z = -0.5

        if math.fabs(err_yaw) < YAW_PRECISION:
            self.move_state = 1

        self.publisher_.publish(self.velocity_msg)

    def go_straight_ahead(self, des_pos):
        err_yaw = self.error_angle(des_pos)
        curr_pos = Point()
        curr_pos.x = self.pose[0]
        curr_pos.y = self.pose[1]
        err_pos = self.calc_dist_points(des_pos, curr_pos)

        if err_pos > DIST_PRECISION:

            self.velocity_msg.linear.x = 0.55
            self.velocity_msg.angular.z = 0.0 if err_yaw > YAW_PRECISION else -0.0
            self.publisher_.publish(self.velocity_msg)
        else:
            self.done()

        # state change conditions
        if math.fabs(err_yaw) > YAW_PRECISION:
            self.move_state = 0

    def done(self):
        self.velocity_msg.linear.x = 0.0
        self.velocity_msg.angular.z = 0.0
        self.publisher_.publish(self.velocity_msg)
        self.reached_destination = True

    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return [X, Y, Z]

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
            math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
            math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def odom_callback(self, data):
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.curr_position = data.pose.pose.position
        self.pose = [data.pose.pose.position.x,
                     data.pose.pose.position.y,
                     self.quaternion_to_euler(x, y, z, w)[2]]

    def reach_goal(self):

        if self.regions['front'] > 0.50 and self.regions['front'] < 1:
            self.robot_state = 1

        if self.robot_state == 0:
            if self.move_state == 0:
                self.fix_yaw(FINAL_TARGET)
                rospy.loginfo('Fixing Heading to reach Target')
            elif self.move_state == 1:
                self.go_straight_ahead(FINAL_TARGET)
                rospy.loginfo('Moving towards the Target')

        if self.robot_state == 1:
            err_yaw = self.error_angle(FINAL_TARGET)
            if math.fabs(err_yaw) < (math.pi / 6) and \
                    self.regions['front'] > 1.5 and self.regions['fright'] > 1 and self.regions['fleft'] > 1:
                # print ('less than 30')
                self.robot_state = 0

            # between 30 and 90
            if err_yaw > 0 and \
                    math.fabs(err_yaw) > (math.pi / 6) and \
                    math.fabs(err_yaw) < (math.pi / 2) and \
                    self.regions['left'] > 1.5 and self.regions['fleft'] > 1:
                # print ('between 30 and 90 - to the left')
                self.robot_state = 0

            if err_yaw < 0 and \
                    math.fabs(err_yaw) > (math.pi / 6) and \
                    math.fabs(err_yaw) < (math.pi / 2) and \
                    self.regions['right'] > 1.5 and self.regions['fright'] > 1:
                # print ('between 30 and 90 - to the right')
                self.robot_state = 0

            self.follow_wall()

    def laserscan_callback(self, msg):
        self.regions = {
            'right': min(min(msg.ranges[0:127]), MX_RANGE),
            'fright': min(min(msg.ranges[128:255]), MX_RANGE),
            'front': min(min(msg.ranges[256:383]), MX_RANGE),
            'fleft': min(min(msg.ranges[384:511]), MX_RANGE),
            'left': min(min(msg.ranges[512:640]), MX_RANGE),
        }
        self.take_action(self.regions)

    def find_wall(self):

        self.velocity_msg.linear.x = 0.55
        self.velocity_msg.angular.z = -0.25
        self.publisher_.publish(self.velocity_msg)

    def turn_left(self):
        self.velocity_msg.linear.x = 0.0
        self.velocity_msg.angular.z = 0.55
        self.publisher_.publish(self.velocity_msg)

    def follow_the_wall(self):

        self.velocity_msg.linear.x = 0.35
        self.velocity_msg.angular.z = 0.0
        self.publisher_.publish(self.velocity_msg)
        # rospy.loginfo('%s'%str(self.velocity_msg))

    def check_sensors_for_nan(self, regions):
        for key in regions.keys():
            if regions[key] != regions[key]:
                regions[key] = math.inf
        return regions

    def take_action(self, regions):

        if self.robot_state == 1:
            state_description = ''
            d = 1.5
            regions = self.check_sensors_for_nan(regions)
            if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
                state_description = 'case 1 - nothing'
                self.obstacle_avoidance_state = 0

            elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
                state_description = 'case 2 - front'
                self.obstacle_avoidance_state = 1

            elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
                state_description = 'case 3 - fright'
                self.obstacle_avoidance_state = 2

            elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
                state_description = 'case 4 - fleft'
                self.obstacle_avoidance_state = 0

            elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
                state_description = 'case 5 - front and fright'
                self.obstacle_avoidance_state = 1

            elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
                state_description = 'case 6 - front and fleft'
                self.obstacle_avoidance_state = 1

            elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
                state_description = 'case 7 - front and fleft and fright'
                self.obstacle_avoidance_state = 1

            elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
                state_description = 'case 8 - fleft and fright'
                self.obstacle_avoidance_state = 0

            else:
                state_description = 'unknown case'
                rospy.loginfo('Laser_scan_data: "%s"' % str(self.regions))

    def follow_wall(self):
        if self.obstacle_avoidance_state == 0:
            rospy.loginfo('Finding nearby wall')
            self.find_wall()
        elif self.obstacle_avoidance_state == 1:
            rospy.loginfo('Turning Left')
            self.turn_left()
        elif self.obstacle_avoidance_state == 2:
            rospy.loginfo('Following the wall')
            self.follow_the_wall()


def main():
    rospy.init_node('node_bug_algorithm')

    simbha_mover = ObstacleAvoider()
    while not rospy.is_shutdown():
        simbha_mover.reach_goal()
        simbha_mover.rate.sleep()


if __name__ == '__main__':
    main()
