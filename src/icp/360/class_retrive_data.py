#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np



class retrive_data():

    def __init__(self):

        self.source_PC   = []
        self.target_PC   = []
        self.T_source    = np.zeros((3,1))
        self.T_target    = np.zeros((3,1))


        self.sub_pc_source   = rospy.Subscriber('/SLAM/buffer/pointcloud_source', PointCloud, self.callback_source)
        self.sub_pc_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_traget', PointCloud, self.callback_target)

        self.sub_odom_source = rospy.Subscriber('/SLAM/buffer/odom_source', Odometry, self.callback_odom_source)
        self.sub_odom_source = rospy.Subscriber('/SLAM/buffer/odom_traget', Odometry, self.callback_odom_target)



    def callback_target(self,var):

        x       = []
        y       = []
        ones    = []

        for i in range(len(var.points)):

            if var.points[i].x == 0 and var.points[i].y == 0:

                pass

            else:

                x.append(var.points[i].x)
                y.append(var.points[i].y)
                ones.append(1)

        self.target_PC = np.vstack((x,y,ones)).T




    def callback_source(self,var):

        x       = []
        y       = []
        ones    = []

        for i in range(len(var.points)):

            if var.points[i].x == 0 and var.points[i].y == 0:

                pass

            else:

                x.append(var.points[i].x)
                y.append(var.points[i].y)
                ones.append(1)

        self.source_PC = np.vstack((x,y,ones)).T





    def callback_odom_source(self,odom):



        roll = 0
        pitch = 0
        theta = 0

        self.T_source[0] = odom.pose.pose.position.x
        self.T_source[1] = odom.pose.pose.position.y


        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.T_source[2] = theta






    def callback_odom_target(self,odom):



        roll = 0
        pitch = 0
        theta = 0



        self.T_target[0] = odom.pose.pose.position.x
        self.T_target[1] = odom.pose.pose.position.y

        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.T_target[2] = theta









    def initial_guess(self):


    	T = np.zeros((3,3))

    	T[0,2] = self.T_target[0] - self.T_source[0] # x axis
    	T[1,2] = self.T_target[1] - self.T_source[1] # y axis

    	T[0,0] = np.cos(self.T_target[2] - self.T_source[2])   # cos(a)
    	T[1,1] = np.cos(self.T_target[2] - self.T_source[2])   #	      cos(a)
    	T[1,0] = -np.sin(self.T_target[2] - self.T_source[2])	 # sin(a)
    	T[0,1] = np.sin(self.T_target[2] - self.T_source[2])   #		  -sin(a)


    	T[2,2] = 1

    	return T


    def return_source(self):
        return self.source_PC, self.T_source

    def return_target(self):
        return self.target_PC, self.T_target
