#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np



class retrive_data():

    def __init__(self):

        self.source_PC   = np.ones((396,3))
        self.target_PC   = np.ones((396,3))
        self.T_source    = np.zeros((3,3))
        self.T_target    = np.zeros((3,3))


        self.sub_pc_source   = rospy.Subscriber('/SLAM/buffer/pointcloud_source', PointCloud, self.callback_source)
        self.sub_pc_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_traget', PointCloud, self.callback_target)

        self.sub_odom_source = rospy.Subscriber('/SLAM/buffer/odom_source', Odometry, self.callback_odom_source)
        self.sub_odom_source = rospy.Subscriber('/SLAM/buffer/odom_traget', Odometry, self.callback_odom_target)



    def callback_target(self,var):

        for i in range(len(var.points)):

            self.target_PC[i,0] = var.points[i].x
            self.target_PC[i,1] = var.points[i].y





    def callback_source(self,var):

        for i in range(len(var.points)):

            self.source_PC[i,0] = var.points[i].x
            self.source_PC[i,1] = var.points[i].y





    def callback_odom_source(self,odom):



        roll = 0
        pitch = 0
        theta = 0


        self.T_source[1,2] = odom.pose.pose.position.y - 300

        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.T_source[0,0] = theta
        self.T_source[0,2] = odom.pose.pose.position.x + 250





    def callback_odom_target(self,odom):



        roll = 0
        pitch = 0
        theta = 0



        self.T_target[0,2] = odom.pose.pose.position.x + 250

        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.T_target[0,0] = theta
        self.T_target[1,2] = odom.pose.pose.position.y - 300






    def initial_guess(self):



    	T = np.zeros((3,3))

    	T[0,2] = self.target_PC[0,2] - self.source_PC[0,2] # x axis
    	T[1,2] = self.target_PC[1,2] - self.source_PC[1,2] # y axis

    	T[0,0] = np.cos(self.target_PC[0,0] - self.source_PC[0,0])   # cos(a)
    	T[1,1] = np.cos(self.target_PC[0,0] - self.source_PC[0,0])   #	      cos(a)
    	T[1,0] = -np.sin(self.target_PC[0,0] - self.source_PC[0,0])	  # sin(a)
    	T[0,1] = np.sin(self.target_PC[0,0] - self.source_PC[0,0])  #		  -sin(a)


    	T[2,2] = 1

    	return T


    def return_source_pc(self):
        return self.source_PC

    def return_target_pc(self):
        return self.target_PC
