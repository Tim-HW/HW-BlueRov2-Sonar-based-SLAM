#!/usr/bin/env python


import sys
import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from sonar_mapping.msg import my_msg
import matplotlib.pyplot as plt
import roslaunch

from class_icp import Align2D
from class_KF import KF
from class_retrive_data import retrive_data

odom    = np.zeros((3,1))
odom_gt = np.zeros((3,1))

def callback_odom(msg):

    global odom

    roll = 0
    pitch = 0
    theta = 0

    odom[0,0] = msg.pose.pose.position.x
    odom[1,0] = msg.pose.pose.position.y


    rot_q                = msg.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom[2,0] = theta





def callback_gt(msg):

    global odom_gt

    roll = 0
    pitch = 0
    theta = 0

    odom_gt[0,0] = msg.pose.pose.position.x
    odom_gt[1,0] = msg.pose.pose.position.y


    rot_q                = msg.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_gt[2,0] = theta






def from_icp2world(odom,point):


    fact = 1

    tmp = np.array([[np.cos((fact*np.pi)-odom[2,0])  , np.sin((fact*np.pi)-odom[2,0]) ,     -odom[0,0]     ],
                    [-np.sin((fact*np.pi)-odom[2,0]) , np.cos((fact*np.pi)-odom[2,0]) ,     -odom[1,0]     ],
                    [        0          ,             0       ,     1     ],])

    det_theta = point[2,0]

    point[2,0] = 1



    point = np.dot(tmp,point)

    point[2,0] = det_theta

    point = -1 * point

    return point




def callback_gt(odom):

    global odom_gt

    roll = 0
    pitch = 0
    theta = 0

    odom_gt[0,0] = odom.pose.pose.position.x
    odom_gt[1,0] = odom.pose.pose.position.y


    rot_q                = odom.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_gt[2,0] = theta



if __name__ == '__main__':

    rospy.init_node('Static_SLAM', anonymous=True) 	# initiate the node

    rospy.Subscriber('/desistek_saga/pose_gt'        , Odometry  , callback_gt) # Subscribes to the Ground Truth pose

    data = retrive_data()

    rospy.sleep(3)

    while not rospy.is_shutdown():

        raw_input("\n           scan ?          \n")

        source,odom_source = data.return_source()

        target,odom_target = data.return_target()

        T = data.initial_guess()             # initial guess of the transform

        ICP = Align2D(target,source,T)       # create an ICP object

        output_ICP,error = ICP.transform    # get the position according to the ICP



        print "\nOutput ICP :\n", output_ICP

        # transform the 3x3 matrix into a 3x1 matrix




        observation = np.array([[output_ICP[0,2]],
                                [output_ICP[1,2]],
                                [np.arccos(output_ICP[0,0])]])     # theta


        print "\nobservation before frame changed :\n",observation,"\n"


        observation = from_icp2world(odom_source,observation)

        print "\nobservation after frame changed :\n",observation,"\n"

        print "\nGT :\n",odom_gt,"\n"
