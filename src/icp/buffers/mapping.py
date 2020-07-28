#!/usr/bin/env python

import tf
import sys
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt
import roslaunch

from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header, String,Bool
from nav_msgs.msg import  Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion, Point32
from sonar_mapping.msg import my_msg



state       = False
target_PC   = PointCloud()
odom_target = np.zeros((3,1))

T = np.zeros((3,1))





def from_odom2world(point,odom):


    T = np.eye(3)


    point_array = np.array([point.x, point.y, 1])

    T[0,2] = odom[0,0]
    T[1,2] = odom[1,0]

    T[0,0] =  np.cos(odom[2,0])
    T[1,1] =  np.cos(odom[2,0])
    T[1,0] = -np.sin(odom[2,0])
    T[0,1] =  np.sin(odom[2,0])



    point_array = np.dot(point_array,T.T)

    point.x = point_array[0]
    point.y = point_array[1]
    point.z = 0

    return point








class Mapping():


    def __init__(self):


        self.map                      = PointCloud()                                          # buffer object
        self.map.header.stamp.secs    = int(rospy.get_time())                                 # time seconds
        self.map.header.stamp.nsecs   = 1000000000*(rospy.get_time()-int(rospy.get_time()))   # time in nano seconds
        self.map.header.frame_id      = "map"                                                 # map as reference

        self.timer                    = rospy.get_time()     # initialize timerer
        self.final_odom               = Odometry()           # final buffer for odom
        self.sampled                  = False                # variable to treat data once gathered one time only


        self.sub_sonar                = rospy.Subscriber("/sonar/PC"              , PointCloud , self.callback)         # Subscribe to the dynamic sonar
        self.sub_odom                 = rospy.Subscriber("/odom"                  , Odometry   , self.callback_odom)    # Subscribe to Odometry

        self.pub_odom                 = rospy.Publisher("/SLAM/buffer/odom_source", Odometry   , queue_size = 1)        # publish the origin of the map
        self.pub_map                  = rospy.Publisher("/SLAM/map"               , PointCloud , queue_size = 1)        # Publish the final map








    def change_origin(self,pointcloud,T):

        ##################################################################################
        #   Function add more points to the original map
        ##################################################################################
        # poincloud : the scan
        # T         : the transform matrix
        ##################################################################################

        tmp = Point32()

        rotation = np.array([[  0   ],      # create the rotation matrix
                             [  0   ],
                             [T[2,0]]])

        translation = np.array([[ T[0,0] ], # create the translation matrix
                                [ T[1,0] ],
                                [   0    ]])

        for i in range(len(pointcloud.points)): # for every point in the scan


            tmp_point = pointcloud.points[i]    # tmp variable

            tmp_point = from_odom2world(tmp_point, rotation)        # rotate the point in the map coordinate
            tmp_point = from_odom2world(tmp_point, translation)     # translate the point in the map coordinate


            self.map.points.append(tmp_point)  # it to the map pointcloud


        self.remove_duplicates(self.map)       # once every point added : remove the duplicates





    def callback_odom(self,var):

        ##################################################################################
        #   Function set the origin of the map
        ##################################################################################


        self.final_odom = var
        self.final_odom.pose.pose.position.x = -250
        self.final_odom.pose.pose.position.y =  300
        self.final_odom.pose.pose.position.z =  -5

        quat = quaternion_from_euler(0, 0, -1.0)       # transform the euler angle into quaternion

        self.final_odom.pose.pose.orientation.z = quat[2]
        self.final_odom.pose.pose.orientation.w = quat[3]



    def callback(self,arg):

        ##################################################################################
        #   Function retrive and add the sonar points in the map
        ##################################################################################

        point = Point32()
        """
        This callback is made to retrive data from the PointCloud
        """


        if len(arg.points) != 0: # if the sonar topics gives values


            if rospy.get_time() < self.timer + 15:  # during 15 second


                self.map.points.append(arg.points[0])    # add the new point
                #self.pub_map.publish(self.map)
                self.pub_odom.publish(self.final_odom)   # publishes the origin of the map


            else:    # if timer is over then


                if self.sampled == False:


                    self.remove_duplicates(self.map) # removes duplcated points
                    #self.sampling(self.map)         # sample the number of points

                    self.pub_map.publish(self.map)          # Publishes the pointcloud mzp
                    self.pub_odom.publish(self.final_odom)  # Publishes the final origin of the map

                    self.sampled = True

        self.pub_map.publish(self.map)
        self.pub_odom.publish(self.final_odom)







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

















def callback_T(msg):

    global T

    T[0,0] = msg.x
    T[1,0] = msg.y
    T[2,0] = msg.theta




def callback_pc(msg):

    global target_PC

    target_PC = msg


def callback(msg):

    global state
    state = msg.data





if __name__ == '__main__':

    rospy.init_node('Mapping', anonymous=True)

    sub1 = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_pc)  # Subscribes to the scan buffer
    sub2 = rospy.Subscriber('/SLAM/T'                       , my_msg    , callback_T)   # Subscribes to the transform computed by the ICP



    initialization = True
    update = False



    while not rospy.is_shutdown():


        if state == False: # if the main didn't call the node

            sub3 = rospy.Subscriber('/SLAM/buffer_1', Bool, callback)  # refresh the old value

            if initialization == False: # if the initialization has been done

                if update == False: # if the map has not been updated before

                    map.change_origin(target_PC,T) # the new scan to the existing map with the transform
                    print "\n   ############################### Map Updated #######################################\n"
                    update = True


        elif state == True:  # if the main called the node to fire

            update = False   # clear the update value

            if initialization == True:  # it's the first time
                map = Mapping() # create the mapping object
                print "\n   ############################### Map Created #######################################\n"
                initialization = False
