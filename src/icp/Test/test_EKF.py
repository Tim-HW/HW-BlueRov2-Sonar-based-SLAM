#!/usr/bin/env python


import sys
import rospy
import numpy as np

from class_KF import KF


if __name__ == '__main__':




    rospy.init_node('Static_SLAM', anonymous=True) 	# initiate the node


    odom_source = np.array([[0.0],
                            [0.0],
                            [0.0]])


    odometry = np.array([[10],
                         [10],
                         [1.5]])                        # the odometry becomes the last scan done


    observation = np.array([[20],
                            [20],
                            [1.5]])                        # the odometry becomes the last scan done


    print "\n Odometry :\n",odometry

    print "\n Sensor :\n",observation

                                                     # print the pose of the observation

    kf = KF()                 # initiate EKF
    kf.prediction(odometry)                                # prediction step
    new_pose = kf.correction(observation)          # correction step

    print"\n new position :\n", new_pose            # print the new pose
