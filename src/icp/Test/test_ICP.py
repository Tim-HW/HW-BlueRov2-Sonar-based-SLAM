#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
import csv



from EKF import EKF






if __name__ == '__main__':


    rospy.init_node('ICP_test', anonymous=True) 	# initiate the node

    sub_source   = rospy.Subscriber('/SLAM/buffer/pointcloud_source', PointCloud, callback_source)  # Subscribes to the buffer 1
    sub_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_target)  # Subscribes to the buffer 2
    data = retrive_data(0) # create the class to retrive the data from the scans



    rospy.sleep(5)

    #T                   = data.initial_guess()   # initial guess of the transform
    source,odom_source  = data.return_source()   # PointCloud of the source scan
    target,odom_target  = data.return_target()   # PointCloud of the target scan


    test_th(source,target)
