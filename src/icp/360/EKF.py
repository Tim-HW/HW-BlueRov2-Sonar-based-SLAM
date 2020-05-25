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



if __name__ == '__main__':



    rospy.init_node('EKF', anonymous=True) 	# initiate the node
    data = retrive_data()


    rospy.sleep(5)
    while not rospy.is_shutdown():

        T       = data.initial_guess()
        source  = data.return_source_pc()
        target  = data.return_target_pc()

        ICP = Align2D(source,target,T)
        print(ICP.transform)

        #print(ICP.transform())
