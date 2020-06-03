#!/usr/bin/env python

import sys
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, String,Bool
from nav_msgs.msg import  Odometry


state = False


class Buffer_1():

    def __init__(self):

        self.max_value               = 396
        self.pointcloud_buffer       = PointCloud()
        self.current_odom            = Odometry()
        self.final_odom              = Odometry()
        self.sampled                 = False

        self.sub_PC                  = rospy.Subscriber("/own/simulated/dynamic/sonar_PC", PointCloud, self.callback)
        self.pub_PC                  = rospy.Publisher("/SLAM/buffer/pointcloud_source",PointCloud,queue_size = 1)

        self.sub_odom                = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.pub_odom                = rospy.Publisher("/SLAM/buffer/odom_source", Odometry, queue_size = 1)

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
                self.final_odom = self.current_odom
                self.pub_PC.publish(self.pointcloud_buffer)





            else:   # if the buffer is full

                #print(self.pointcloud_buffer)
                #print(len(self.pointcloud_buffer2.points))
                #print(self.pointcloud_buffer.points.x)
                if self.sampled == False:

                    self.remove_duplicates(self.pointcloud_buffer)
                    #self.sampling(self.pointcloud_buffer)

                    self.sampled = True
                    print("ok")

                self.pub_PC.publish(self.pointcloud_buffer)          # The pointcloud buffer 2 is published
                self.pub_odom.publish(self.final_odom)
                rospy.sleep(10)


    def remove_duplicates(self,PointCloud):

        threshold = 1.0

        for i in range(0,int(len(PointCloud.points)/2)):
            for k in range(int(len(PointCloud.points)/2),len(PointCloud.points)):

                if np.abs(PointCloud.points[i].x - PointCloud.points[k].x) < threshold and np.abs(PointCloud.points[i].y - PointCloud.points[k].y) < threshold:
                    PointCloud.points[k].x = 0
                    PointCloud.points[k].y = 0

    def sampling(self,PointCloud):

        i = 0

        while i < (len(PointCloud.points)) and PointCloud.points[i].x != 0 and PointCloud.points[i].y != 0 :
            PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x + PointCloud.points[i + 2].x )/3
            PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y + PointCloud.points[i + 2].y )/3

            PointCloud.points[i+1].x = PointCloud.points[i+2].x = PointCloud.points[i+1].y = PointCloud.points[i+2].y = 0
            i = i + 3







def callback(msg):
    global state
    state = msg.data


if __name__ == '__main__':



    rospy.init_node('Buffer_1', anonymous=True)

    while state == False:
        sub = rospy.Subscriber('/SLAM/buffer_1', Bool, callback)
    #print(state)
    buffer = Buffer_1()

    while(1):
        pass
