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
from geometry_msgs.msg import Quaternion, Point32
from sonar_mapping.msg import my_msg




state       = False
target_PC   = PointCloud()
odom_target = np.zeros((3,1))

T = np.zeros((3,3))









def from_odom2world(point,odom):


    T = np.eye(3)


    point_array = np.array([point.x, point.y, 1])

    T[0,2] = odom[0,0]
    T[1,2] = odom[1,0]

    T[0,0] =  np.cos(-odom[2,0])
    T[1,1] =  np.cos(-odom[2,0])
    T[1,0] = -np.sin(-odom[2,0])
    T[0,1] =  np.sin(-odom[2,0])



    point_array = np.dot(T,point_array.T)

    point.x = point_array[0]
    point.y = point_array[1]
    point.z = -5

    return point








class Mapping():


    def __init__(self):


        self.map                      = PointCloud()
        self.map.header.stamp.secs    = int(rospy.get_time())
        self.map.header.stamp.nsecs   = 1000000000*(rospy.get_time()-int(rospy.get_time()))
        self.map.header.frame_id      = "world"

        self.timer                    = rospy.get_time()
        self.final_odom               = Odometry()
        self.sampled                  = False
        self.x                        = 0
        self.y                        = 0
        self.theta                    = 0


        self.sub_sonar                = rospy.Subscriber("/own/simulated/dynamic/sonar_PC"  , PointCloud, self.callback)
        self.sub_odom                 = rospy.Subscriber("/odom"                            , Odometry  , self.callback_odom)

        self.pub_odom                 = rospy.Publisher("/SLAM/buffer/odom_source"          , Odometry  , queue_size = 1)
        self.pub_map                  = rospy.Publisher("/SLAM/map"                         , PointCloud, queue_size = 1)






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

                self.map.points.append(pointcloud.points[i])






    def change_origin(self,pointcloud,T):


        tmp = Point32()

        rot_q  = self.final_odom.pose.pose.orientation

        orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x = self.final_odom.pose.pose.position.x
        y = self.final_odom.pose.pose.position.y

        odom = np.array([[x + T[0,2]],
                         [y + T[1,2]],
                            [yaw]])



        for i in range(len(pointcloud.points)):

            #print(len(self.pointcloud_buffer1.points))
            tmp_point = pointcloud.points[i]


            tmp_point = from_odom2world(tmp_point,odom)
            #print(tmp_point)
            #print " "

            self.map.points.append(tmp_point)






    def callback_odom(self,var):


        self.final_odom = var
        self.final_odom.pose.pose.position.x = -250
        self.final_odom.pose.pose.position.y = 300





    def callback(self,arg):

        point = Point32()
        """
        This callback is made to retrive data from the PointCloud
        """


        if len(arg.points) != 0: # if the sonar topics gives values


            if rospy.get_time() < self.timer + 15: # while the length of the buffer is below 99 values


                rot_q  = self.final_odom.pose.pose.orientation

                orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


                point_array = np.array([arg.points[0].x, arg.points[0].y, 1])

                T[0,2] = self.final_odom.pose.pose.position.x
                T[1,2] = self.final_odom.pose.pose.position.y

                T[0,0] =  np.cos(-yaw)
                T[1,1] =  np.cos(-yaw)
                T[1,0] = -np.sin(-yaw)
                T[0,1] =  np.sin(-yaw)

                point_array = np.dot(T,point_array.T)

                point.x = point_array[0]
                point.y = point_array[1]
                point.z = -5


                self.map.points.append(point)    # add the new point
                self.pub_odom.publish(self.final_odom)


            else:   # if the buffer is full



                if self.sampled == False:

                    self.remove_duplicates(self.map)
                    #self.sampling(self.pointcloud_buffer)

                    self.pub_map.publish(self.map)
                    self.pub_odom.publish(self.final_odom)

                    self.sampled = True

        try:
            self.pub_map.publish(self.map)
            self.pub_odom.publish(self.final_odom)
        except AttributeError:
            pass







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

            while i < (len(PointCloud.points)):

                PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x + PointCloud.points[i + 2].x )/3
                PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y + PointCloud.points[i + 2].y )/3

                list.append(i+1)
                list.append(i+2)

                i = i + 3

        else:

            while i < (len(PointCloud.points)):
                PointCloud.points[i].x = (PointCloud.points[i].x + PointCloud.points[i + 1].x)/2
                PointCloud.points[i].y = (PointCloud.points[i].y + PointCloud.points[i + 1].y)/2

                list.append(i+1)

                i = i + 2

        list = set(list)

        for i in range(0,len(list)):

            del PointCloud.points[i]

















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





if __name__ == '__main__':

    rospy.init_node('Mapping', anonymous=True)

    sub1 = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_pc)
    sub2 = rospy.Subscriber('/SLAM/T'                       , my_msg    , callback_T)



    initialization = True
    update = False



    while not rospy.is_shutdown():



        if state == False:

            sub3 = rospy.Subscriber('/SLAM/buffer_1', Bool, callback)

            if initialization == False:

                if update == False:

                    map.change_origin(target_PC,T)
                    print "\n   ############################### Map Updated #######################################\n"
                    update = True


        elif state == True:

            update = False

            if initialization == True:
                map = Mapping()
                print "\n   ############################### Map Created #######################################\n"
                initialization = False
