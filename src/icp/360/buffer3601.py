#!/usr/bin/env python

import sys
import rospy
import struct
import numpy as np


from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, String,Bool
from nav_msgs.msg import  Odometry





class buffer():

    def __init__(self):

        self.pointcloud_buffer       = PointCloud()
        self.current_odom            = Odometry()
        self.final_odom              = Odometry()

        self.sub_PC                  = rospy.Subscriber("/own/simulated/dynamic/sonar_PC", PointCloud, self.callback)
        self.pub_PC                  = rospy.Publisher("/SLAM/buffer/pointcloud_traget",PointCloud,queue_size = 1)
        self.sub_odom                = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.pub_odom                = rospy.Publisher("/SLAM/buffer/odom_traget", Odometry, queue_size = 1)

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


            if len(self.pointcloud_buffer.points) < 396: # while the length of the buffer is below 99 values

                print(len(self.pointcloud_buffer.points))

                self.pointcloud_buffer.points.append(arg.points[0])    # add the new point
                self.final_odom = self.current_odom

                #print(len(self.pointcloud_buffer.points))



            else:   # if the buffer is full

                #print(self.pointcloud_buffer)
                #print(len(self.pointcloud_buffer2.points))

                self.pub_PC.publish(self.pointcloud_buffer)          # The pointcloud buffer 2 is published
                self.pub_odom.publish(self.final_odom)          # The pointcloud buffer 2 is published







if __name__ == '__main__':



    rospy.init_node('Buffer_2', anonymous=True)

    while not rospy.is_shutdown():

        buffer = buffer()
        print("buffer",buffer)
        rospy.spin()
