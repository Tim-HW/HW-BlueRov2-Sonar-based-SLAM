#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np




def from_world2icp(odom,T):

    ##################################################################################
    #   Function to change coordinate frame from world -> map
    ##################################################################################

    point = np.zeros((3,1)) # initialize the variable

    point[0,0] = T[0,0]     # create the tmp vector
    point[1,0] = T[1,0]
    point[2,0] =   1

    # matrix transform
    tmp = np.array([[ np.cos(-odom[2,0]) , np.sin(-odom[2,0]) , -odom[0,0] ],
                    [-np.sin(-odom[2,0]) , np.cos(-odom[2,0]) , -odom[0,0] ],
                    [        0           ,          0         ,      1     ]])

    # transform the point
    point = np.dot(tmp,point)


    return point







class retrive_data():


    def __init__(self):

        self.source_PC   = []                   # buffer: the homogeneous coordinates of the point from the map : [x,y,1]
        self.target_PC   = []                   # buffer: the homogeneous coordinates of the point from the buffer : [x,y,1]
        self.T_source    = np.zeros((3,1))      # buffer: origin of the map
        self.T_target    = np.zeros((3,1))      # buffer: odometry of the buffer

        self.sub_pc_source   = rospy.Subscriber('/SLAM/map'                     , PointCloud, self.callback_source) # Subscribes to the map
        self.sub_pc_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, self.callback_target) # Subscribes to the buffer

        self.sub_odom_source = rospy.Subscriber('/SLAM/buffer/odom_source'      , Odometry  , self.callback_odom_source) # Subscribes to the origin of the map
        self.sub_odom_target = rospy.Subscriber('/SLAM/buffer/odom_target'      , Odometry  , self.callback_odom_target) # Subscribes to the odometry of the buffer



    def callback_source(self,var):

        ##################################################################################
        #   Function transform the PointCloud of the map into array format
        ##################################################################################


        if len(self.source_PC) != len(var.points): # while the value of the  pointcloud and the array are not equal

            x       = []
            y       = []
            ones    = []


            for i in range(len(var.points)): # for every points in the PointCloud

                x.append(var.points[i].x)    # a new x coordinate is added to the x list
                y.append(var.points[i].y)    # a new y coordinate is added to the y list
                ones.append(1)               # a new 1 is added to the list

            self.source_PC = np.vstack((x,y,ones)).T # fuse all the lists into one matrix




    def callback_target(self,var):

        ##################################################################################
        #   Function transform the PointCloud of the map into array format
        ##################################################################################


        if len(self.target_PC) != len(var.points): # while the value of the  pointcloud and the array are not equal

            x       = []
            y       = []
            ones    = []


            for i in range(len(var.points)):

                x.append(var.points[i].x) # a new x coordinate is added to the x list
                y.append(var.points[i].y) # a new y coordinate is added to the y list
                ones.append(1)            # a new 1 is added to the list

            self.target_PC = np.vstack((x,y,ones)).T  # fuse all the lists into one matrix










    def callback_odom_source(self,odom):

        ##################################################################################
        #   Function transform odometry message in a matrix [x,y,theta]
        ##################################################################################

        roll  = 0
        pitch = 0
        theta = 0

        self.T_source[0,0] = odom.pose.pose.position.x
        self.T_source[1,0] = odom.pose.pose.position.y


        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.T_source[2,0] = theta


    def callback_odom_target(self,odom):

        ##################################################################################
        #   Function transform odometry message in a matrix [x,y,theta]
        ##################################################################################


        roll = 0
        pitch = 0
        theta = 0

        self.T_target[0,0] = odom.pose.pose.position.x
        self.T_target[1,0] = odom.pose.pose.position.y

        rot_q  = odom.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.T_target[2,0] = theta






    def initial_guess(self):

        ##################################################################################
        #   Function to compute the approximative transformation between the scan and the map
        ##################################################################################


        T = np.eye(3)

        delta = np.ones((3,1))



    	delta[0,0] = self.T_source[0,0] - self.T_target[0,0] # delta x
    	delta[1,0] = self.T_source[1,0] - self.T_target[1,0] # delta y


        from_world2icp(self.T_source,delta) # transform into map frame reference

        T[0,2] = delta[0,0]
        T[1,2] = delta[1,0]


    	return T # returns the transformation





    def return_source(self):


        ##################################################################################
        #   Function to return the the map and its origin
        ##################################################################################


        return self.source_PC, self.T_source






    def return_target(self):


        ##################################################################################
        #   Function to return the the map and its origin
        ##################################################################################


        T = np.eye(3)

        delta = np.zeros((3,1))

    	delta[2,0]   = self.T_target[2,0] - self.T_source[2,0]




        T[0,0] =  np.cos(delta[2,0])
        T[1,0] =  np.sin(delta[2,0])
        T[0,1] =  -np.sin(delta[2,0])
        T[1,1] =  np.cos(delta[2,0])


        self.target_PC = np.dot(self.target_PC,T.T)

        return self.target_PC, self.T_target
