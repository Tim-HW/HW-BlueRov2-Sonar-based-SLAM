#!/usr/bin/env python


import sys
import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from sonar_mapping.msg import my_msg
import matplotlib.pyplot as plt
import roslaunch

from class_icp import Align2D
from class_KF import KF
from class_retrive_data import retrive_data

odom    = np.zeros((3,1))
odom_gt = np.zeros((3,1))

def callback_odom(msg):

    global odom

    roll = 0
    pitch = 0
    theta = 0

    odom[0,0] = msg.pose.pose.position.x
    odom[1,0] = msg.pose.pose.position.y


    rot_q                = msg.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom[2,0] = theta





def callback_gt(msg):

    global odom_gt

    roll = 0
    pitch = 0
    theta = 0

    odom_gt[0,0] = msg.pose.pose.position.x
    odom_gt[1,0] = msg.pose.pose.position.y


    rot_q                = msg.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_gt[2,0] = theta














if __name__ == '__main__':

    rospy.init_node('Static_SLAM', anonymous=True) 	# initiate the node

    rospy.Subscriber('/desistek_saga/pose_gt'        , Odometry  , callback_gt) # Subscribes to the Ground Truth pose
    rospy.Subscriber('/SLAM/buffer/odom_source'        , Odometry  , callback_odom) # Subscribes to the Ground Truth pose

    while not rospy.is_shutdown():
        """
        T = np.eye(3)

        th = odom[2,0]

        odom_gt[2,0] = 1

        T[0,0] =  np.cos(th)
        T[1,0] = -np.sin(th)
        T[0,1] =  np.sin(th)
        T[1,1] =  np.cos(th)

        T[0,2] = -odom[0,0]
        T[1,2] = -odom[1,0]






        new_pose = np.dot(T,odom_gt)
        """
        print "\n",odom_gt,"\n"
