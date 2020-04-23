#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospkg


# function to get Cross-Track Error
def getCTE(current_pose):

    # coordinates of start point
    start_pose = (-4, -4)

    # coordinates of goal point
    goal_pose = (-3, -3)

    # Kp error
    cte = (((current_pose[1] - start_pose[1])*(goal_pose[0] - start_pose[0])) - \
        ((current_pose[0] - start_pose[0])*(goal_pose[1] - start_pose[0])))/\
        (((goal_pose[0] - start_pose[0])**2) + ((goal_pose[1] - start_pose[1])**2))

    return cte


# function to get angular velocity
def control(error, paramet, errors):

    # angular velocity
    omega = paramet[0]*error + paramet[1]*(error - errors[0]) + paramet[2]*errors[1]

    # for Kd and Ki error
    errors[1] += error
    errors[0] = error

    return omega, errors


# function to move turtlebot3
def runner(parameters, errors):

    # declaring object for class Twist
    msg = Twist()
    odom = Odometry()

    # loop rate
    rate = rospy.Rate(1)

    # get current pose
    current_position = (odom.pose.pose.position.x, odom.pose.pose.position.y)
    print('current_position', current_position)

    # getting error
    error = getCTE(current_position)
    print('error', error)

    # getting angular velocity
    angular_velocity, errors = control(error, parameters, errors)

    # assign angular velocity
    msg.angular.z = angular_velocity
    msg.linear.x = 0.2

    # publish angular velocity
    rospy.loginfo(msg)
    pub.publish(msg)

    rate.sleep()


if __name__ == '__main__':

    # initialize node
    rospy.init_node('controller', anonymous=True)

    # publisher object publishing to command velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # initialize subscriber node to get robot's odometry
    rospy.Subscriber('/odom', Odometry, queue_size=10)

    # PID parameters
    param = [1, 0, 0]

    # PID Errors
    err = [0, 0]

    # call function runner
    runner(param, err)

    rospy.spin()