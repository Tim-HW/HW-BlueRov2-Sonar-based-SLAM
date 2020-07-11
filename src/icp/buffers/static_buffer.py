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

        self.pointcloud_buffer                      = PointCloud()
        self.pointcloud_buffer.header.stamp.secs    = int(rospy.get_time())
        self.pointcloud_buffer.header.stamp.nsecs   = 1000000000*(rospy.get_time()-int(rospy.get_time()))
        self.pointcloud_buffer.header.frame_id      = "desistek_saga/base_link"

        self.Timer                    = rospy.get_time()    # initialize timerer
        self.current_odom             = Odometry()          # temporary buffer for odom
        self.final_odom               = Odometry()          # final buffer for odom
        self.sampled                  = False               # variable to treat data once gathered one time only
        self.x                        = 0                   # buffer of x position
        self.y                        = 0                   # buffer of y position
        self.theta                    = 0                   # buffer of theta/yaw position

        self.sub_PC                   = rospy.Subscriber("/own/simulated/dynamic/sonar_PC", PointCloud, self.callback)          # Subscribe to the dynamic sonar
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
        self.pointcloud_buffer.header.frame_id      = "desistek_saga/base_link"

        self.Timer                    = rospy.get_time()
        self.current_odom             = Odometry()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = 0
        self.y                        = 0
        self.theta                    = 0

    def callback_odom(self,var):

        ##################################################################################
        #   Function retrive the current Odometry
        ##################################################################################

        self.current_odom = var


    def callback(self,arg):

        ##################################################################################
        #   Function retrive the current Odometry
        ##################################################################################

        if len(arg.points) != 0: # if the sonar topics is not empty


            if rospy.get_time() < self.Timer + 12 :   # during 15 second

                self.pointcloud_buffer.points.append(arg.points[0])           # the buffer will gather new points

                rot_q  = self.current_odom.pose.pose.orientation              # transform quaternion to euler angle

                orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
                (roll, pitch, theta) = euler_from_quaternion(orientation_list)


                self.x      += self.current_odom.pose.pose.position.x         # add the current x position to the mean variable of x
                self.y      += self.current_odom.pose.pose.position.y         # add the current y position to the mean variable of y
                self.theta  += theta                                          # add the current theta position to the mean variable of theta

                self.pub_PC.publish(self.pointcloud_buffer)                   # publish the current state of the buffer (useful for debugging)





            else:   # if timer is over then



                if self.sampled == False:

                    #self.remove_duplicates(self.pointcloud_buffer)     # removes duplcated points
                    #self.sampling(self.pointcloud_buffer)              # sample the number of points
                    try:

                        self.x       = self.x/len(self.pointcloud_buffer.points)        # compute the mean value of x
                        self.y       = self.y/len(self.pointcloud_buffer.points)        # compute the mean value of y
                        self.theta   = self.theta/len(self.pointcloud_buffer.points)    # compute the mean value of theta

                        self.final_odom = self.current_odom                  # the final odom takes the header of the current odom

                        self.final_odom.pose.pose.position.x = self.x        # the x position will be the mean value of the x position during the process
                        self.final_odom.pose.pose.position.y = self.y        # the y position will be the mean value of the y position during the process

                                                                             # same for theta but we need first to transform it into quaternion

                        quat = quaternion_from_euler(0, 0, self.theta)       # transform the euler angle into quaternion

                        self.final_odom.pose.pose.orientation.z = quat[2]
                        self.final_odom.pose.pose.orientation.w = quat[3]

                        self.pub_PC.publish(self.pointcloud_buffer)          # Publishes the pointcloud_buffer
                        self.pub_odom.publish(self.final_odom)               # Publishes the final Odometry


                        self.sampled = True

                    except ZeroDivisionError :
                        pass


                             # variable to make sur that this process happen only once




    def remove_duplicates(self,PointCloud):

        ##################################################################################
        #   Function the remove remove_duplicates from a PointCloud
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

            while i < (len(PointCloud.points-3)):

                PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x + PointCloud.points[i + 2].x )/3
                PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y + PointCloud.points[i + 2].y )/3

                list.append(i+1)
                list.append(i+2)

                i = i + 3

        else:

            while i < (len(PointCloud.points)-2):
                PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x)/2
                PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y)/2

                list.append(i+1)

                i = i + 2

        list = set(list)

        for i in range(0,len(list)):

            del PointCloud.points[i]




def callback(msg):
    global state
    state = msg.data





if __name__ == '__main__':



    rospy.init_node('Static_Buffer', anonymous=True)

    initialization = True

    while not rospy.is_shutdown():


        if state == False:
            sub = rospy.Subscriber('/SLAM/buffer_2', Bool, callback)
            if initialization == False:
                buffer.clear()
                rospy.sleep(1)



        elif state == True:
            if initialization == True:

                buffer = Buffer()
                print "\n   ############################## Buffer  Created ####################################\n"
                initialization = False
