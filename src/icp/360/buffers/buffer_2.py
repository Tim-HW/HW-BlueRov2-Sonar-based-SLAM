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


class Buffer_2():

    def __init__(self):

        self.Timer                    = rospy.get_time()
        self.max_value                = 396
        self.pointcloud_buffer        = PointCloud()
        self.current_odom             = Odometry()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = 0
        self.y                        = 0
        self.theta                    = 0

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
        self.theta                    = 0

    def callback_odom(self,var):

        #print(len(self.pointcloud_buffer1.points))
        self.current_odom = var

    def callback(self,arg):

        """
        This callback is made to retrive data from the PointCloud
        """
        self.pointcloud_buffer.header = arg.header # give the same time stamp


        if len(arg.points) != 0: # if the sonar topics gives values


            if rospy.get_time() < self.Timer + 15 : # while the length of the buffer is below 99 values

                self.pointcloud_buffer.points.append(arg.points[0])    # add the new point

                rot_q  = self.current_odom.pose.pose.orientation

                orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

                self.x      += self.current_odom.pose.pose.position.x
                self.y      += self.current_odom.pose.pose.position.y
                self.theta  += yaw

                self.pub_PC.publish(self.pointcloud_buffer)





            else:   # if the buffer is full



                if self.sampled == False:

                    #self.remove_duplicates(self.pointcloud_buffer)
                    #self.sampling(self.pointcloud_buffer)
                    self.x       = self.x/395
                    self.y       = self.y/395
                    self.theta   = self.theta


                    self.final_odom = self.current_odom

                    self.final_odom.pose.pose.position.x = self.x
                    self.final_odom.pose.pose.position.y = self.y

                    quat = quaternion_from_euler(0, 0, self.theta)

                    self.final_odom.pose.pose.orientation.x = quat[0]
                    self.final_odom.pose.pose.orientation.y = quat[1]
                    self.final_odom.pose.pose.orientation.z = quat[2]
                    self.final_odom.pose.pose.orientation.w = quat[3]

                    self.pub_PC.publish(self.pointcloud_buffer)          # The pointcloud buffer 2 is published
                    self.pub_odom.publish(self.final_odom)


                    self.sampled = True




    def remove_duplicates(self,PointCloud):

        threshold = 1.0

        for i in range(0,int(len(PointCloud.points)/2)):
            for k in range(int(len(PointCloud.points)/2),len(PointCloud.points)):

                if np.abs(PointCloud.points[i].x - PointCloud.points[k].x) < threshold and np.abs(PointCloud.points[i].y - PointCloud.points[k].y) < threshold:
                    PointCloud.points[k].x = 0
                    PointCloud.points[k].y = 0


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



    rospy.init_node('Buffer_2', anonymous=True)
    initialization = True

    while not rospy.is_shutdown():


        if state == False:
            sub = rospy.Subscriber('/SLAM/buffer_2', Bool, callback)
            if initialization == False:
                buffer.clear()
                 #print "buffer 2 cleared"


        elif state == True:
            if initialization == True:
                buffer = Buffer_2()
                print "buffer 2 created"
                initialization = False
