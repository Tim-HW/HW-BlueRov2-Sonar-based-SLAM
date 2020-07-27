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

        self.sub_pc_source   = rospy.Subscriber('/SLAM/map'                     , PointCloud, self.callback_source)
        self.sub_pc_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, self.callback_target)

        self.sub_odom_source = rospy.Subscriber('/SLAM/buffer/odom_source'      , Odometry  , self.callback_odom_source)
        self.sub_odom_target = rospy.Subscriber('/SLAM/buffer/odom_target'      , Odometry  , self.callback_odom_target)



    def callback_source(self,var):


        if len(self.source_PC) != len(var.points):
            x       = []
            y       = []
            ones    = []


            for i in range(len(var.points)):

                x.append(var.points[i].x)
                y.append(var.points[i].y)
                ones.append(1)

            self.source_PC = np.vstack((x,y,ones)).T




    def callback_target(self,var):


        if len(self.target_PC) != len(var.points):
            x       = []
            y       = []
            ones    = []


            for i in range(len(var.points)):

                x.append(var.points[i].x)
                y.append(var.points[i].y)
                ones.append(1)

            self.target_PC = np.vstack((x,y,ones)).T










    def callback_odom_source(self,odom):

        roll  = 0
        pitch = 0
        theta = 0

        self.T_source[0,0] = odom.pose.pose.position.x
        self.T_source[1,0] = odom.pose.pose.position.y


        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.T_source[2,0] = theta


    def callback_odom_target(self,odom):

        roll = 0
        pitch = 0
        theta = 0

        self.T_target[0,0] = odom.pose.pose.position.x
        self.T_target[1,0] = odom.pose.pose.position.y

        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.T_target[2,0] = theta






    def initial_guess(self):


        #print "t",self.T_target[2,0]
        #print "s",self.T_source[2,0]

        T = np.eye(3)

        delta = np.zeros((3,1))



    	delta_x   = self.T_source[0,0] - self.T_target[0,0]
    	delta_y   = self.T_source[1,0] - self.T_target[1,0]




        """
        T[0,0] =  np.cos(delta[2,0])
        T[1,0] = -np.sin(delta[2,0])
        T[0,1] =  np.sin(delta[2,0])
        T[1,1] =  np.cos(delta[2,0])
        """
        T[0,2] = delta_x
        T[1,2] = delta_y


    	return T





    def return_source(self):

        """
        T = np.eye(3)

        T[0,0] =  np.cos(-self.T_source[2,0])
        T[1,1] =  np.cos(-self.T_source[2,0])
        T[1,0] = -np.sin(-self.T_source[2,0])
        T[0,1] =  np.sin(-self.T_source[2,0])

        self.source_PC = np.dot(self.source_PC,T.T)
        """

        return self.source_PC, self.T_source






    def return_target(self):


        """
        T = np.eye(3)


        T[0,2] = -5
        T[1,2] = -5

        theta = 0
        theta = theta*np.pi/180

        T[0,0] =  np.cos(theta)
        T[1,1] =  np.cos(theta)
        T[1,0] = -np.sin(theta)
        T[0,1] =  np.sin(theta)

        self.target_PC = np.dot(self.target_PC,T.T)
        """

        T = np.eye(3)

        delta = np.zeros((3,1))

    	delta[2,0]   = self.T_target[2,0] - self.T_source[2,0]




        T[0,0] =  np.cos(delta[2,0])
        T[1,0] =  np.sin(delta[2,0])
        T[0,1] =  -np.sin(delta[2,0])
        T[1,1] =  np.cos(delta[2,0])


        self.target_PC = np.dot(self.target_PC,T.T)

        return self.target_PC, self.T_target
