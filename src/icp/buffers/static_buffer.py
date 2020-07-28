#!/usr/bin/env python

import os
import sys
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, String,Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

state = False


class Buffer():

    def __init__(self):

        self.pointcloud_buffer                      = PointCloud()                                          # buffer object
        self.pointcloud_buffer.header.stamp.secs    = int(rospy.get_time())                                 # time seconds
        self.pointcloud_buffer.header.stamp.nsecs   = 1000000000*(rospy.get_time()-int(rospy.get_time()))   # time in nano seconds
        self.pointcloud_buffer.header.frame_id      = "map"                                                 # map as reference

        self.Timer                    = rospy.get_time()     # initialize timerer
        self.current_odom             = Odometry()           # temporary buffer for odom
        self.final_odom               = Odometry()           # final buffer for odom
        self.sampled                  = False                # variable to treat data once gathered one time only
        self.x                        = []                   # buffer of x position
        self.y                        = []                   # buffer of y position
        self.theta                    = []                   # buffer of theta/yaw position

        self.sub_PC                   = rospy.Subscriber("/sonar/PC", PointCloud, self.callback)                                # Subscribe to the dynamic sonar
        self.pub_PC                   = rospy.Publisher("/SLAM/buffer/pointcloud_target"  , PointCloud, queue_size = 1)         # Publish the final buffer
        self.sub_odom                 = rospy.Subscriber("/odom"                          , Odometry  , self.callback_odom)     # Subscribe to Odometry
        self.pub_odom                 = rospy.Publisher("/SLAM/buffer/odom_target"        , Odometry  , queue_size = 1)         # publish the mean value of the Odometry during the process


    def clear(self):

        ##################################################################################
        #   Function to clear the cache from previous values
        ##################################################################################

        self.pointcloud_buffer                      = PointCloud()
        self.pointcloud_buffer.header.stamp.secs    = int(rospy.get_time())
        self.pointcloud_buffer.header.stamp.nsecs   = 1000000000*(rospy.get_time()-int(rospy.get_time()))
        self.pointcloud_buffer.header.frame_id      = "map"

        self.Timer                    = rospy.get_time()
        self.current_odom             = Odometry()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = []
        self.y                        = []
        self.theta                    = []

    def callback_odom(self,var):

        ##################################################################################
        #   Function retrive the current Odometry
        ##################################################################################

        self.current_odom = var


    def callback(self,arg):

        ##################################################################################
        #   Function retrive and add the sonar points in the buffer
        ##################################################################################

        if len(arg.points) != 0: # if the sonar topics is not empty


            if rospy.get_time() < self.Timer + 12 :   # during 15 second

                self.pointcloud_buffer.points.append(arg.points[0])           # the buffer will gather new points from the sonar

                rot_q  = self.current_odom.pose.pose.orientation              # transform quaternion to euler angle

                orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
                (roll, pitch, theta) = euler_from_quaternion(orientation_list)

                self.x.append(self.current_odom.pose.pose.position.x)             # add the current x position in a list
                self.y.append(self.current_odom.pose.pose.position.y)             # add the current y position in a list
                self.theta.append(theta)                                          # add the current theta position in a list

                self.final_odom = self.current_odom                               # current Odometry = final_odom

                self.pub_PC.publish(self.pointcloud_buffer)                   # publish the current state of the buffer (useful for debugging)
                self.pub_odom.publish(self.final_odom)                        # publish the current state of the Odometry (useful for debugging)


            else:   # if timer is over then



                if self.sampled == False:   # if the sampling has not been called


                    try:
                        """
                        self.x       = np.sum(self.x)       /   (len(self.x))        # compute the mean value of x
                        self.y       = np.sum(self.y)       /   (len(self.y))        # compute the mean value of y
                        self.theta   = np.sum(self.theta)   /   (len(self.theta))    # compute the mean value of theta




                        self.final_odom = self.current_odom                  # the final odom takes the header of the current odom

                        self.final_odom.pose.pose.position.x = self.x        # the x position will be the mean value of the x position during the process
                        self.final_odom.pose.pose.position.y = self.y        # the y position will be the mean value of the y position during the process

                                                                             # same for theta but we need first to transform it into quaternion

                        quat = quaternion_from_euler(0, 0, self.theta)       # transform the euler angle into quaternion

                        self.final_odom.pose.pose.orientation.z = quat[2]
                        self.final_odom.pose.pose.orientation.w = quat[3]
                        """
                        self.remove_duplicates(self.pointcloud_buffer)     # removes duplcated points
                        self.sampling(self.pointcloud_buffer)              # sample the number of points

                        """
                        pt = Point32()
                        pt.x = 1
                        pt.y = 0

                        self.pointcloud_buffer.points.append(pt)

                        pt.x = 0
                        pt.y = 1

                        self.pointcloud_buffer.points.append(pt)
                        """
                        self.pub_PC.publish(self.pointcloud_buffer)          # Publishes the pointcloud_buffer
                        self.pub_odom.publish(self.final_odom)               # Publishes the final Odometry


                        self.sampled = True

                    except ZeroDivisionError :
                        pass
                self.pub_PC.publish(self.pointcloud_buffer)          # Publishes the pointcloud_buffer
                self.pub_odom.publish(self.final_odom)               # Publishes the final Odometry




    def remove_duplicates(self,PointCloud):

        ##################################################################################
        #
        #   Function the remove remove_duplicates from a PointCloud
        #
        ##################################################################################

        threshold = 1.0         # this values means that the threshold value is at 1m
        list = []

        for i in range(0,int(len(PointCloud.points)/2)): # for half of the values
            for k in range(int(len(PointCloud.points)/2),len(PointCloud.points)): # for the other half of the other values

                if np.abs(PointCloud.points[i].x - PointCloud.points[k].x) < threshold and np.abs(PointCloud.points[i].y - PointCloud.points[k].y) < threshold: # if the point i is at least than 1m from the point k

                    list.append(k) # add in a list the value of k

        list = set(list) # organise the list (increasing values) and remove duplicates

        for i in range(0,len(list)):    # for every values in the list

            del PointCloud.points[i]    # removes the point






    def sampling(self,PointCloud):

        i = 0
        list = []

        if len(PointCloud.points) % 2 == 1:

            while i < (len(PointCloud.points)-1):

                PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x)/2
                PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y)/2

                list.append(i+1)

                i = i + 2

        else:

            while i < (len(PointCloud.points)):
                PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x)/2
                PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y)/2

                list.append(i+1)

                i = i + 2

        list = set(list)
        #print list
        for i in range(0,len(list)):

            del PointCloud.points[i]




def callback(msg):
    global state
    state = msg.data





if __name__ == '__main__':



    rospy.init_node('Static_Buffer', anonymous=True) # create the node

    initialization = True                            # initialization

    while not rospy.is_shutdown():


        if state == False: # if the main didn't call the node
            sub = rospy.Subscriber('/SLAM/buffer_2', Bool, callback) # refresh the old value

            if initialization == False: # if the initialization has been done
                buffer.clear()  # clear the cache
                rospy.sleep(1)  # wait 1s



        elif state == True: # if the main called the node to fire

            if initialization == True: # it's the first time

                buffer = Buffer()   # create the buffer
                print "\n   ############################## Buffer  Created ####################################\n"
                initialization = False  # initialization done
