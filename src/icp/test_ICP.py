#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
import csv


from class_icp import Align2D
from class_KF import KF
from class_retrive_data import retrive_data






if __name__ == '__main__':


    rospy.init_node('ICP_test', anonymous=True) 	# initiate the node


    data = retrive_data() # create the class to retrive the data from the scans



    rospy.sleep(5)




    T                   = data.initial_guess()   # initial guess of the transform
    source,odom_source  = data.return_source()   # PointCloud of the source scan
    target,odom_target  = data.return_target()   # PointCloud of the target scan

    print len(source)
    print len(target)


    ICP = Align2D(source,target,T)               # create an ICP object
    new_pose,error = ICP.transform
    print new_pose
