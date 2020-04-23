#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospkg


# # function to run turtlebot3
# def runner(order):
#
#     # declaring object for class Twist
#     msg = Twist()
#     print(msg)
#
#     for i in range(len(order)):
#         rate = rospy.Rate(2)
#
#         msg.linear.x = float(order[i][0])
#         msg.angular.z = float(order[i][1])
#
#         rospy.loginfo(msg)
#         pub.publish(msg)
#
#         rate.sleep()


if __name__ == '__main__':

    # initialize node
    rospy.init_node('controller', anonymous=True)

    # publisher object publishing to command velocity
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # initialize subscriber node to get robot's odometry
    rospy.Subscriber('odom', Odometry, queue_size=10)

    # get odom data
    odom = Odometry()

    # frequency rate
    # rate = rospy.Rate(1)

    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y

    print('x: ', x, 'y: ', y)

    # rate.sleep()
