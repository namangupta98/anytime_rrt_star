#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospkg
import sys

global param, err

# PID parameters
param = [1, 0, 0]

# PID Errors
err = [0, 0]


# function to get Cross-Track Error
def getCTE(current_pose):

    # global current_pose, cte_
    # current_pose = current_pose_

    # coordinates of start point
    start_pose = (-4, -4)

    # coordinates of goal point
    goal_pose = (-4, -3)

    # Kp error
    cte = (((current_pose[1] - start_pose[1]) * (goal_pose[0] - start_pose[0])) -
           ((current_pose[0] - start_pose[0]) * (goal_pose[1] - start_pose[0]))) / \
          (((goal_pose[0] - start_pose[0]) ** 2) + ((goal_pose[1] - start_pose[1]) ** 2))

    # cte_ = cte

    return -cte


# function to get angular velocity
def control(error):

    global err, param

    # angular velocity
    omega = param[0] * error + param[1] * (error - err[0]) + param[2] * err[1]

    # for Kd and Ki error
    err[1] += error
    err[0] = error

    return omega


# function to move turtlebot3
# def runner(parameters, errors):
#
#     # declaring object for class Twist
#     # msg = Twist()
#
#
#
#     # loop rate
#     # rate = rospy.Rate(1)
#
#
#
#     # publish angular velocity
#     rospy.loginfo(msg)
#
#     # rate.sleep()

# function to update waypoints
def update():
    if current_pose == waypoints[way_n]:
        way_n += 1


# function to callback subscriber node
def callback_odom(odom):
    # global current_pose_
    msg = Twist()

    # current position stored
    current_pose = (odom.pose.pose.position.x, odom.pose.pose.position.y)

    # get error
    error = getCTE(current_pose)
    print('error', error)

    # getting angular velocity
    angular_velocity = control(error)

    # assign angular velocity
    msg.angular.z = angular_velocity
    msg.linear.x = 0.05
    print(msg)

    # publishing
    pub.publish(msg)


if __name__ == '__main__':

    waypoints = [(-4, -4), (-4, -3), (-4, -2), (-3, -2)]

    way_n = 0

    # initialize node
    rospy.init_node('controller', anonymous=True)

    # publisher object publishing to command velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # initialize subscriber node to get robot's odometry
    sub = rospy.Subscriber('/odom', Odometry, callback_odom)

    rospy.spin()

    # call function runner
    # while not rospy.is_shutdown():
    #     runner(param, err)
