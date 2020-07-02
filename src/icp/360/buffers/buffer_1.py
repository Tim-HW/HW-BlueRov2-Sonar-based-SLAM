#!/usr/bin/env python

import sys
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, String,Bool
from nav_msgs.msg import  Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from sonar_mapping.msg import my_msg

state       = False
target_PC   = PointCloud()
odom_target = np.zeros((3,1))

T = np.zeros((3,3))


class Buffer_1():

    def __init__(self):

        self.max_value                = 396
        self.pointcloud_buffer        = PointCloud()
        self.pointcloud_buffer.header.frame_id = "odom"

        self.current_odom             = Odometry()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = 0
        self.y                        = 0
        self.theta                    = 0


        self.sub_PC                   = rospy.Subscriber("/own/simulated/dynamic/sonar_PC", PointCloud, self.callback)
        self.pub_PC                   = rospy.Publisher("/SLAM/buffer/pointcloud_source",PointCloud,queue_size = 1)
        self.sub_odom                 = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.pub_odom                 = rospy.Publisher("/SLAM/buffer/odom_source", Odometry, queue_size = 1)

    def clear(self):

        self.max_value                = 396
        self.pointcloud_buffer        = PointCloud()
        self.current_odom             = Odometry()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = 0
        self.y                        = 0
        self.theta                    = 0





    def update(self,pointcloud,T):



        point = np.zeros((3,1))


        for i in range(len(pointcloud.points)):

            if pointcloud.points[i].x != 0 and pointcloud.points[i].y != 0:

                point[0,0] = pointcloud.points[i].x
                point[1,0] = pointcloud.points[i].y
                point[2,0] = 1

                point = np.dot(T,point)


                pointcloud.points[i].x = point[0,0]
                pointcloud.points[i].y = point[1,0]
                pointcloud.points[i].z = 0

                #self.pointcloud_buffer.points.append(pointcloud.points[i])




    def change_origin(self,origin):

        new_origin      = np.eye(3)
        print"source :",self.final_odom.pose.pose.position.x
        print"origin :",origin[0,0]
        new_origin[0,2] = self.final_odom.pose.pose.position.x - origin[0,0]
        new_origin[1,2] = self.final_odom.pose.pose.position.y - origin[1,0]
        print(new_origin)
        point = np.zeros((3,1))

        for i in range(len(self.pointcloud_buffer.points)):

            point[0,0] = self.pointcloud_buffer.points[i].x
            point[1,0] = self.pointcloud_buffer.points[i].y
            point[2,0] = 1

            point = np.dot(new_origin,point)


            self.pointcloud_buffer.points[i].x = point[0,0]
            self.pointcloud_buffer.points[i].y = point[1,0]
            self.pointcloud_buffer.points[i].z = 0


        self.pub_PC.publish(self.pointcloud_buffer)          # The pointcloud buffer 2 is published





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

                orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                self.theta += yaw

                self.pub_PC.publish(self.pointcloud_buffer)






            else:   # if the buffer is full



                if self.sampled == False:

                    self.remove_duplicates(self.pointcloud_buffer)
                    #self.sampling(self.pointcloud_buffer)
                    """
                    self.x  = self.x/396
                    self.y  = self.y/396
                    """

                    self.final_odom = self.current_odom

                    self.final_odom.pose.pose.position.x = -250
                    self.final_odom.pose.pose.position.y = 300

                    self.pub_PC.publish(self.pointcloud_buffer)          # The pointcloud buffer 2 is published
                    self.pub_odom.publish(self.final_odom)

                    self.sampled = True

            self.pub_PC.publish(self.pointcloud_buffer)          # The pointcloud buffer 2 is published
            self.pub_odom.publish(self.final_odom)

    def update_odom(self,odom):

        self.final_odom.Header = self.current_odom.Header

        self.final_odom.pose.pose.position.x += self.current_odom.pose.pose.position.x
        self.final_odom.pose.pose.position.y += self.current_odom.pose.pose.position.y
        self.final_odom.pose.pose.position.z += self.current_odom.pose.pose.position.z

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



def callback_T(msg):



    T[0,0] = np.cos(msg.theta)
    T[1,0] = -np.sin(msg.theta)
    T[0,1] = np.sin(msg.theta)
    T[1,1] = np.cos(msg.theta)

    T[0,2] = msg.x
    T[1,2] = msg.y

    T[2,2] = 1




def callback_pc(msg):
    global target_PC

    target_PC = msg


def callback(msg):

    global state
    state = msg.data



def callback_odom_target(msg):

    global target_odom

    odom_target[0,0] = msg.pose.pose.position.x
    odom_target[1,0] = msg.pose.pose.position.y
    odom_target[2,0] = 1




if __name__ == '__main__':

    rospy.init_node('Buffer_1', anonymous=True)

    sub1 = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_pc)
    sub2 = rospy.Subscriber('/SLAM/T', my_msg, callback_T)
    sub3 = rospy.Subscriber('/SLAM/buffer/odom_target', Odometry, callback_odom_target)


    initialization = True
    update = False



    while not rospy.is_shutdown():





        if state == False:

            sub4 = rospy.Subscriber('/SLAM/buffer_1', Bool, callback)

            if initialization == False:

                if update == False:

                    buffer.update(target_PC,T)
                    buffer.change_origin(odom_target)
                    update = True

                else:
                    pass




        elif state == True:

            update = False

            if initialization == True:
                buffer = Buffer_1()
                print "buffer 1 created"
                initialization = False
