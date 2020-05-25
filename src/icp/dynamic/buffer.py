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

        self.pointcloud_buffer1      = PointCloud()
        self.pointcloud_buffer2      = PointCloud()
        self.voidbuffer              = PointCloud()
        self.current_odom            = Odometry()
        self.odom_buffer_1           = np.zeros((99,2))
        self.odom_buffer_2           = np.zeros((99,2))



        self.sub_odom                = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.sub_PC                  = rospy.Subscriber("/own/simulated/dynamic/sonar_PC", PointCloud, self.callback)
        self.pub_PC                  = rospy.Publisher("/SLAM/buffer/pointcloud",PointCloud,queue_size = 1)
        self.pub_T                   = rospy.Publisher("/SLAM/buffer/initial_T", String, queue_size = 1)
        self.pub_state               = rospy.Publisher("/SLAM/buffer/state", Bool, queue_size = 1)




    def callback_odom(self,var):
        #print(len(self.pointcloud_buffer1.points))
        self.current_odom = var

    def callback(self,arg):



        self.voidbuffer = PointCloud() # reset the buffer void

        """
        This callback is made to retrive data from the PointCloud
        """
        self.pointcloud_buffer1.header = arg.header # give the same time stamp
        self.pointcloud_buffer2.header = arg.header # give the same time stamp

        if len(arg.points) != 0: # if the sonar topics gives values

            #print(" ")
            #print("buffer 1:",len(self.pointcloud_buffer1.points))
            #print("buffer 2:",len(self.pointcloud_buffer2.points))
            #print(" ")
            #print(self.odom_buffer_1)




            if len(self.pointcloud_buffer1.points) < 99: # while the length of the buffer is below 99 values

                #print(len(self.pointcloud_buffer1.points))

                self.pointcloud_buffer1.points.append(arg.points[0])    # add the new point

                #self.odom_buffer_1[len(self.pointcloud_buffer1.points)-1][0] = self.current_odom.pose.pose.position.x # store the x position
                #self.odom_buffer_1[len(self.pointcloud_buffer1.points)-1][1] = self.current_odom.pose.pose.position.y # store the y position



            else:   # if the buffer is full

                self.pointcloud_buffer2 = self.pointcloud_buffer1  # Points are tranfered from the first buffer to the second
                #print(self.pointcloud_buffer2)
                self.pointcloud_buffer1 = self.voidbuffer          # The first buffer is cleared
                #print(self.odom_buffer_1)
                #self.odom_buffer_2 = self.odom_buffer_1            # Odometry related is tranfered from the frist buffer to the second
                #self.odom_buffer_1 = np.zeros((99,2))              # Then the odometry buffer is cleared
                #print(self.odom_buffer_1)
                #self.pub_T.publish(str(self.odom_buffer_2[0,0]) + " " + str(self.odom_buffer_2[0,1]))

                self.pub_PC.publish(self.pointcloud_buffer2)          # The pointcloud buffer 2 is published



                #print(self.state)


                # to make the pointcloud matches between one buffer to another we need to fill the new buffer with the latest value of the second buffer
                for i in range(79,99):

                    self.pointcloud_buffer1.points.append(self.pointcloud_buffer2.points[i])

                    #self.odom_buffer_1[i-79][0] = self.odom_buffer_2[i][0]
                    #self.odom_buffer_1[i-79][1] = self.odom_buffer_2[i][1]






    def transform(self,odom):

        T = String()

        T = str(odom[12,0])
        return T






if __name__ == '__main__':



    rospy.init_node('Buffer', anonymous=True)

    while not rospy.is_shutdown():

        icp = buffer()
        rospy.spin()
