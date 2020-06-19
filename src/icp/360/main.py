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

from class_icp import Align2D
from class_KF import KF
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





def callback_gt(odom):

    global odom_gt

    roll = 0
    pitch = 0
    theta = 0

    odom_gt[0] = odom.pose.pose.position.x
    odom_gt[1] = odom.pose.pose.position.y


    rot_q                = odom.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_gt[2] = theta









def call_buffer_1():


    #Function to erased the previous scan of th buffer 1 and collect a new one



    while len(pc_source.points) > 2: # while the buffer is not empty

        delete_buffer_1()            # ask for the buffer to empty it current data
        rospy.sleep(1)               # wait fir 1 second

    while(len(pc_source.points) != 396):                         # once empty, wait for the buffer to be filled
        create_buffer_1()                                        # allow the buffer to collect data
        print"scan 1: ", 100*len(pc_source.points)/396, "%"      # Print the current situation of the buffer
        rospy.sleep(1)                                           # wait for 1 second




def call_buffer_2():


    #Function to erased the previous scan of th buffer 2 and collect a new one


    raw_input("would you like to process second scan ? [ENTER]") # ask you the permition to execute the second scan

    while len(pc_target.points) > 2: # while the buffer is not empty

        delete_buffer_2()            # ask for the buffer to empty it current data
        rospy.sleep(1)               # wait fir 1 second

    while(len(pc_target.points) != 396):                    # once empty, wait for the buffer to be filled
        create_buffer_2()                                   # allow the buffer to collect data
        print"scan 2: ", 100*len(pc_target.points)/396, "%" # Print the current situation of the buffer
        rospy.sleep(1)                                      # wait fir 1 second




if __name__ == '__main__':







    rospy.init_node('Static_SLAM', anonymous=True) 	# initiate the node

    sub_source   = rospy.Subscriber('/SLAM/buffer/pointcloud_source', PointCloud, callback_source)  # Subscribes to the buffer 1
    sub_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_target)  # Subscribes to the buffer 2
    sub_gt       = rospy.Subscriber('/desistek_saga/pose_gt', Odometry, callback_gt)                   # Subscribes to the Ground Truth pose

    pub1         = rospy.Publisher('/SLAM/buffer_1', Bool, queue_size=1)                                    # Create publisher to enable or disable the buffer 1 [True = enable / False = disable]
    pub2         = rospy.Publisher('/SLAM/buffer_2', Bool, queue_size=1)                                    # Create publisher to enable or disable the buffer 2 [True = enable / False = disable]
    pub_odom     = rospy.Publisher('/SLAM/offset',my_msg, queue_size=1)

    data = retrive_data() # create the class to retrive the data from the scans
    counter = 0                                                                                    # initiate the counter
    rospy.sleep(5)
    kf = KF()

    while not rospy.is_shutdown():








        if counter == 0:

            call_buffer_1()



            call_buffer_2()

        if counter == 1:

            call_buffer_2()

        #raw_input("would you like to retrive the data ? [ENTER]") # ask you the permition to retrive the data


        #in the second case the source and the target PointCloud and odometry will be inverted
        #because we already have the scan of the buffer 2, we want the scan of the buffer 1. Consequently
        #the source becomes the new target and the target becomes the new source


        T = data.initial_guess()   # initial guess of the transform
        source,odom_source = data.return_source()
        target,odom_target = data.return_target()





        ICP = Align2D(source,target,T)               # create an ICP object

        observation = ICP.transform                  # get the position according to the ICP

        print "before frame change: \n",observation

        observation = np.array([[observation[0,2]],
                                [observation[1,2]],
                                [np.arccos(observation[0,0])]])

        tmp = np.array([[np.cos(odom_source[2,0])  , np.sin(odom_source[2,0]) , 0],
                        [-np.sin(odom_source[2,0]) , np.cos(odom_source[2,0]) , 0],
                        [        0                 ,             0            , 1],])

        det_theta = observation[2,0]

        observation[2,0] = 1



        observation = np.dot(tmp,observation)

        observation[2,0] = det_theta

        observation[0,0] = -1*observation[0,0]

        print "\n output ICP: \n", observation



        observation = odom_source + observation                                       # observation becomes the last corrected odometry + the new observation
        #print "\n ICP + last odom: \n", observation

        odometry = odom_target                       # the odometry becomes the last scan done



        print "\n Odometry :\n",odometry          # print the odometru of the buffer_2
        print "\n Sensor   :\n",observation                                                               # print the pose of the observation

        kf.prediction(odometry)                                 # prediction step
        new_pose = kf.correction(observation)          # correction step

        print"\n new position :\n", new_pose            # print the new pose
        print"\n GROUND TRUTH position :\n", odom_gt    # print the Ground Truth pose


        new_pose = new_pose - odometry

        print"\n offset :\n", new_pose            # print the new pose

        msg = my_msg()
        msg.x     = new_pose[0,0]
        msg.y     = new_pose[1,0]
        msg.theta = new_pose[2,0]
        pub_odom.publish(msg)

        counter = 1                                    # change the case
