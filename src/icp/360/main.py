#!/usr/bin/env python


import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from class_icp import Align2D
from EKF import EKF
from class_retrive_data import retrive_data





pc_source = PointCloud()
pc_target = PointCloud()
odom_gt   = np.zeros((3,1))


def create_buffer_2():
    pub2.publish(True)

def delete_buffer_2():
    pub2.publish(False)



def create_buffer_1():
    pub1.publish(True)

def delete_buffer_1():
    pub1.publish(False)





def callback_source(var):
    global pc_source

    pc_source = var


def callback_target(var):
    global pc_target

    pc_target = var

def callback(odom):
    global odom_gt

    roll = 0
    pitch = 0
    theta = 0

    odom_gt[0] = odom.pose.pose.position.x
    odom_gt[1] = odom.pose.pose.position.y


    rot_q  = odom.pose.pose.orientation
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_gt[2] = theta









if __name__ == '__main__':




    rospy.init_node('Static_SLAM', anonymous=True) 	# initiate the node

    sub_source   = rospy.Subscriber('/SLAM/buffer/pointcloud_source', PointCloud, callback_source)
    sub_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_target)
    sub_source   = rospy.Subscriber('/desistek_saga/pose_gt', Odometry, callback)


    pub1 = rospy.Publisher('/SLAM/buffer_1', Bool, queue_size=1)
    pub2 = rospy.Publisher('/SLAM/buffer_2', Bool, queue_size=1)

    data = retrive_data() # create the class to retrive the data from the scans



    while not rospy.is_shutdown():




        raw_input("would you like to process scan 1 ? [ENTER]") # ask you the permition to execute the first scan

        while(len(pc_source.points) != 396):    # wait for the scan to be completed
            create_buffer_1()
            print"scan 1: ", 100*len(pc_source.points)/396, "%"
            rospy.sleep(1)






        raw_input("would you like to process scan 2 ? [ENTER]") # ask you the permition to execute the second scan

        while(len(pc_target.points) != 396):    # wait for the scan to be completed
            create_buffer_2()
            print"scan 2: ", 100*len(pc_target.points)/396, "%"
            rospy.sleep(1)




        raw_input("would you like to retrive the data ? [ENTER]") # ask you the permition to retrive the data

        T       = data.initial_guess()      # initial guess of the transform
        source,odom_source  = data.return_source()   # PointCloud of the source scan
        target,odom_target  = data.return_target()   # PointCloud of the target scan

        print "\n Odometry -1 :\n",odom_source
        print "\n Odometry :\n",odom_target



        #raw_input("would you like to process ICP ? [ENTER]") # ask you the permition to process the ICP


        ICP = Align2D(source,target,T)      # create object for ICP algo


        #raw_input("would you like to process re-Localization ? [ENTER]") # ask you the permition to process the ICP
        odometry = odom_target

        observation = ICP.transform
        observation = np.array([[observation[0,2]],[observation[1,2]],[np.arccos(observation[0,0])]])
        observation = odom_source + observation

        print "\n Sensor :\n",observation

        ekf = EKF(odom_source,odometry)
        ekf.prediction()
        new_pose = ekf.correction(observation)
        print"\n new position :\n", new_pose

        print"\n GROUND TRUTH position :\n", odom_gt
