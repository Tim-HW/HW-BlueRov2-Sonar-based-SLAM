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

        point = Point32()
        """
        This callback is made to retrive data from the PointCloud
        """


        if len(arg.points) != 0: # if the sonar topics gives values


            if rospy.get_time() < self.timer + 15: # while the length of the buffer is below 99 values


                rot_q  = self.current_odom.pose.pose.orientation

                orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


                point_array = np.array([arg.points[0].x, arg.points[0].y, 1])

                T[0,2] = self.current_odom.pose.pose.position.x
                T[1,2] = self.current_odom.pose.pose.position.y

                T[0,0] =  np.cos(-yaw)
                T[1,1] =  np.cos(-yaw)
                T[1,0] = -np.sin(-yaw)
                T[0,1] =  np.sin(-yaw)

                point_array = np.dot(T,point_array.T)

                point.x = point_array[0]
                point.y = point_array[1]
                point.z = -5


                self.pointcloud_buffer.points.append(point)    # add the new point
                self.pub_PC.publish(self.pointcloud_buffer)
                self.pub_odom.publish(self.final_odom)


            else:   # if the buffer is full



                if self.sampled == False:

                    self.remove_duplicates(self.pointcloud_buffer)
                    #self.sampling(self.pointcloud_buffer)

                    self.pub_PC.publish(self.pointcloud_buffer)
                    self.pub_odom.publish(self.final_odom)

                    self.sampled = True

        try:
            self.pub_PC.publish(self.pointcloud_buffer)
            self.pub_odom.publish(self.final_odom)
        except AttributeError:
            pass



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



    rospy.init_node('Dynamic_Buffer', anonymous=True)
    initialization = True
    printing       = False
    while not rospy.is_shutdown():


        if state == False:
            sub = rospy.Subscriber('/SLAM/buffer_2', Bool, callback)
            if initialization == False:
                buffer.clear()
                print "\n   ############################## Buffer  Cleared ####################################\n"


        elif state == True:
            if initialization == True:

                buffer = Buffer()
                print "\n   ############################## Buffer  Created ####################################\n"
                initialization = False
