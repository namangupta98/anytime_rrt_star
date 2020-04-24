#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospkg


# function to get Cross-Track Error
def getCTE():

    global current_pose_, cte_
    current_pose = current_pose_

    # coordinates of start point
    start_pose = (-4, -4)

    # coordinates of goal point
    goal_pose = (-3, -3)

    # Kp error
    cte = (((current_pose[1] - start_pose[1]) * (goal_pose[0] - start_pose[0])) -
           ((current_pose[0] - start_pose[0]) * (goal_pose[1] - start_pose[0]))) / \
          (((goal_pose[0] - start_pose[0]) ** 2) + ((goal_pose[1] - start_pose[1]) ** 2))

    cte_ = cte


# function to get angular velocity
def control(error, paramet, errors):
    # angular velocity
    omega = paramet[0] * error + paramet[1] * (error - errors[0]) + paramet[2] * errors[1]

    # for Kd and Ki error
    errors[1] += error
    errors[0] = error

    return omega, errors


# function to move turtlebot3
def runner(parameters, errors):

    # declaring object for class Twist
    msg = Twist()

    # getting angular velocity
    angular_velocity, errors = control(error, parameters, errors)

    # loop rate
    rate = rospy.Rate(1)

    # assign angular velocity
    msg.angular.z = angular_velocity
    msg.linear.x = 0.2

    # publish angular velocity
    rospy.loginfo(msg)
    pub.publish(msg)

    rate.sleep()


# function to callback subscriber node
def callback_odom(odom):
    global current_pose_

    # current position stored
    current_pose_ = (odom.pose.pose.position.x, odom.pose.pose.position.y)

    # get error
    getCTE()


if __name__ == '__main__':

    global pub

    # initialize node
    rospy.init_node('controller', anonymous=True)

    # publisher object publishing to command velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # initialize subscriber node to get robot's odometry
    sub = rospy.Subscriber('/odom', Odometry, callback_odom)

    # PID parameters
    param = [1, 0, 0]

    # PID Errors
    err = [0, 0]

    # call function runner
    while not rospy.is_shutdown():
        runner(param, err)
