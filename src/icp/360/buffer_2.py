#!/usr/bin/env python

import os
import sys
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, String,Bool
from nav_msgs.msg import  Odometry


state = False


class Buffer_2():

    def __init__(self):

        self.max_value                = 396
        self.pointcloud_buffer        = PointCloud()
        self.current_odom             = Odometry()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = 0
        self.y                        = 0
        self.orientation_x            = 0
        self.orientation_y            = 0
        self.orientation_z            = 0
        self.orientation_w            = 0

        self.sub_PC                   = rospy.Subscriber("/own/simulated/dynamic/sonar_PC", PointCloud, self.callback)
        self.pub_PC                   = rospy.Publisher("/SLAM/buffer/pointcloud_target",PointCloud,queue_size = 1)
        self.sub_odom                 = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.pub_odom                 = rospy.Publisher("/SLAM/buffer/odom_target", Odometry, queue_size = 1)


    def clear(self):

        self.max_value                = 396
        self.pointcloud_buffer        = PointCloud()
        self.current_odom             = Odometry()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = 0
        self.y                        = 0
        self.orientation_x            = 0
        self.orientation_y            = 0
        self.orientation_z            = 0
        self.orientation_w            = 0

    def callback_odom(self,var):

        #print(len(self.pointcloud_buffer1.points))
        self.current_odom = var

    def callback(self,arg):





        #self.voidbuffer = PointCloud() # reset the buffer void

        """
        This callback is made to retrive data from the PointCloud
        """
        self.pointcloud_buffer.header = arg.header # give the same time stamp


        if len(arg.points) != 0: # if the sonar topics gives values


            if len(self.pointcloud_buffer.points) < self.max_value: # while the length of the buffer is below 99 values

                #print(len(self.pointcloud_buffer1.points))

                self.pointcloud_buffer.points.append(arg.points[0])    # add the new point

                self.x              += self.current_odom.pose.pose.position.x
                self.y              += self.current_odom.pose.pose.position.y

                rot_q  = self.current_odom.pose.pose.orientation

                self.orientation_x   += rot_q.x
                self.orientation_y   += rot_q.y
                self.orientation_z   += rot_q.z
                self.orientation_w   += rot_q.w

                self.pub_PC.publish(self.pointcloud_buffer)





            else:   # if the buffer is full



                if self.sampled == False:

                    self.remove_duplicates(self.pointcloud_buffer)
                    #self.sampling(self.pointcloud_buffer)
                    self.x  = self.x/396
                    self.y  = self.y/396
                    self.orientation_x   = self.orientation_x/396
                    self.orientation_y   = self.orientation_y/396
                    self.orientation_z   = self.orientation_z/396
                    self.orientation_w   = self.orientation_w/396


                    self.final_odom = self.current_odom

                    self.final_odom.pose.pose.position.x = self.x
                    self.final_odom.pose.pose.position.y = self.y
                    self.final_odom.pose.pose.orientation.x = self.orientation_x
                    self.final_odom.pose.pose.orientation.y = self.orientation_y
                    self.final_odom.pose.pose.orientation.z = self.orientation_z
                    self.final_odom.pose.pose.orientation.w = self.orientation_w


                    self.sampled = True


                self.pub_PC.publish(self.pointcloud_buffer)          # The pointcloud buffer 2 is published
                self.pub_odom.publish(self.final_odom)



    def remove_duplicates(self,PointCloud):

        threshold = 1.0

        for i in range(0,int(len(PointCloud.points)/2)):
            for k in range(int(len(PointCloud.points)/2),len(PointCloud.points)):

                if np.abs(PointCloud.points[i].x - PointCloud.points[k].x) < threshold and np.abs(PointCloud.points[i].y - PointCloud.points[k].y) < threshold:
                    PointCloud.points[k].x = 0
                    PointCloud.points[k].y = 0



    def sampling(self,PointCloud):

        i = 0

        while i < (len(PointCloud.points)):
            PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x + PointCloud.points[i + 2].x )/3
            PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y + PointCloud.points[i + 2].y )/3

            PointCloud.points[i+1].x = PointCloud.points[i+2].x = PointCloud.points[i+1].y = PointCloud.points[i+2].y = 0
            i = i + 3






def callback(msg):
    global state
    state = msg.data


if __name__ == '__main__':



    rospy.init_node('Buffer_2', anonymous=True)
    counter = 0

    while not rospy.is_shutdown():


        if state == False:
            sub = rospy.Subscriber('/SLAM/buffer_2', Bool, callback)
            if counter != 0:
                buffer.clear()
                 #print "buffer 2 cleared"


        elif state == True:
            if counter == 0:
                buffer = Buffer_2()
                print "buffer 2 created"
                counter = 1
