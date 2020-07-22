#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
import csv
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt


odom_gt       = np.zeros((3,1))
odom_robot    = np.zeros((3,1))

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



def write_CSVfile(time,x,y,yaw,x_gt,y_gt,yaw_gt,error_x,error_y,error_yaw,RMS):

	my_path = os.path.abspath(os.path.dirname(__file__))
	path = os.path.join(my_path, "reccord_localization.csv")
	with open(path, mode="w") as csv_file:
		fieldname=['time','x','y','yaw','x_gt','y_gt','yaw_gt','error_x','error_y','error_yaw','RMS']
		csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
		csv_writer.writeheader()
		for i in range(len(x)):
			csv_writer.writerow({'time':time[i],'x':x[i],'y':y[i],'yaw':yaw[i],'x_gt':x_gt[i],'y_gt':y_gt[i],'yaw_gt':yaw_gt[i],'error_x':error_x[i],'error_y':error_y[i],'error_yaw':error_yaw[i],'RMS':RMS[i]})




if __name__ == '__main__':


    x     = []
    y     = []
    yaw   = []

    x_gt   = []
    y_gt   = []
    yaw_gt = []

    RMS       = []
    error_x   = []
    error_y   = []
    error_yaw = []

    time      = []


    rospy.init_node('Reccord', anonymous=True) 	# initiate the node
    rospy.Subscriber('/desistek_saga/pose_gt', Odometry, callback_gt)                   # Subscribes to the Ground Truth pose
    rospy.Subscriber('/odom', Odometry, callback_odom)

    rate = rospy.Rate(2) # 2Hz
    i = 0

    while not rospy.is_shutdown():

        try:

            x.append(odom_robot[0,0])
            y.append(odom_robot[1,0])
            yaw.append(odom_robot[2,0])
            x_gt.append(odom_gt[0,0])
            y_gt.append(odom_gt[1,0])
            yaw_gt.append(odom_gt[2,0])
            error_x.append(np.abs(odom_robot[0,0] - odom_gt[0,0]))
            error_y.append(np.abs(odom_robot[1,0] - odom_gt[1,0]))
            error_yaw.append(np.abs(odom_robot[2,0] - odom_gt[2,0]))
            RMS.append(math.sqrt(np.abs(odom_robot[0,0] - odom_gt[0,0])**2 + np.abs(odom_robot[0,0] - odom_gt[0,0])**2 + np.abs(odom_robot[2,0] - odom_gt[2,0])**2))
            time.append(i*0.5)
            i += 1
            rate.sleep()

        except rospy.ROSInterruptException:

            write_CSVfile(time,x,y,yaw,x_gt,y_gt,yaw_gt,error_x,error_y,error_yaw,RMS)

            pass
