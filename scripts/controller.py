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
def control(error, prev_error=0, summ=0):
	
	# PID parameters
	Kp, Kd, Ki = 1, 0, 0

	# angular velocity
	omega = Kp*error + Kd*(error - prev_error) + Ki*summ

	# for Kd and Ki error
	summ += error
	prev_error = error

	return omega, summ, prev_error


# function to move turtlebot3
def runner():

    # declaring object for class Twist
    msg = Twist()
    odom = Odometry()

    # get current pose
    current_position = (odom.pose.pose.position.x, odom.pose.pose.position.y)

    # getting error
    error = getCTE(current_position)

    # getting angular velocity
    angular_velocity, Ki_error, Kd_error =  control(error, Ki_error, Kd_error)

    # assign angular velocity
    msg.angular.z = angular_velocity
    msg.linear.x = 0.2

    # publish angular velocity
    rospy.loginfo(msg)
    pub.publish(msg)


if __name__ == '__main__':

    # initialize node
    rospy.init_node('controller', anonymous=True)

    # publisher object publishing to command velocity
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # initialize subscriber node to get robot's odometry
    rospy.Subscriber('odom', Odometry, queue_size=10)

    # frequency rate
    rate = rospy.Rate(1)

    # call function runner
    runner()

    rate.sleep()
