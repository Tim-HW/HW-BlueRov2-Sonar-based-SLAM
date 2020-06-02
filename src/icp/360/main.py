#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.spatial import KDTree
import random
import matplotlib.pyplot as plt

from class_icp import Align2D
from class_retrive_data import retrive_data
from buffer_1 import Buffer_1
from buffer_2 import Buffer_2





pc_source = PointCloud()
pc_target = PointCloud()





def callback_source(var):
    global pc_source

    pc_source = var


def callback_target(var):
    global pc_target

    pc_target = var







if __name__ == '__main__':




    rospy.init_node('Static_SLAM', anonymous=True) 	# initiate the node

    sub_source   = rospy.Subscriber('/SLAM/buffer/pointcloud_source', PointCloud, callback_source)
    sub_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_target)

    data = retrive_data() # create the class to retrive the data from the scans






    raw_input("would you like to process scan 1 ?") # ask you the permition to execute the first scan

    buff1 = Buffer_1() # create the scan

    while(len(pc_source.points) != 396):    # wait for the scan to be completed
        pass




    raw_input("would you like to process scan 2 ?") # ask you the permition to execute the second scan

    buff2 = Buffer_2() # create the scan

    while(len(pc_target.points) != 396):    # wait for the scan to be completed
        pass




    raw_input("would you like to retrive the data ?") # ask you the permition to retrive the data






    T       = data.initial_guess()      # initial guess of the transform
    source  = data.return_source_pc()   # PointCloud of the source scan
    target  = data.return_target_pc()   # PointCloud of the target scan



    raw_input("would you like to process ICP ?") # ask you the permition to process the ICP


    ICP = Align2D(source,target,T)      # create object for ICP algo
