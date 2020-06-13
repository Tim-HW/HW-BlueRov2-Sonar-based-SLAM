#!/usr/bin/env python


import sys
import rospy
import numpy as np
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


odom_gt       = Odometry()
odom_robot    = Odometry()

def callback_gt(odom):

    global odom_gt

    roll = 0
    pitch = 0
    theta = 0

    odom_gt[0] = odom.pose.pose.position.x
    odom_gt[1] = odom.pose.pose.position.y


    rot_q                = odom.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_gt[2] = theta

def callback_odom(odom):

    global odom_robot

    roll = 0
    pitch = 0
    theta = 0

    odom_robot[0] = odom.pose.pose.position.x
    odom_robot[1] = odom.pose.pose.position.y


    rot_q                = odom.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_robot[2] = theta



def write_CSVfile(x,y,x_gt,y_gt):

	my_path = os.path.abspath(os.path.dirname(__file__))
	path = os.path.join(my_path, "reccord.csv")
	with open(path, mode="w") as csv_file:
		fieldname=[x','y','x_gt','y_gt']
		csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
		csv_writer.writeheader()
		for i in range(0,len(x)):
			csv_writer.writerow({'x':x[i],'y':y[i],'x_gt':x_gt[i],'y_gt':y_gt[i]})





if __name__ == '__main__':


    x    = []
    y    = []
    x_gt = []
    y_gt = []

    rospy.init_node('Reccord', anonymous=True) 	# initiate the node
    sub_gt       = rospy.Subscriber('/desistek_saga/pose_gt', Odometry, callback_gt)                   # Subscribes to the Ground Truth pose
    sub_odom     = rospy.Subscriber('/odom', Odometry, callback_odom)

    while not rospy.is_shutdown():
