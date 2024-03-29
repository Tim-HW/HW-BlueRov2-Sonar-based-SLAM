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

    odom_gt[0,0] = odom.pose.pose.position.x
    odom_gt[1,0] = odom.pose.pose.position.y


    rot_q                = odom.pose.pose.orientation
    roll, pitch, theta   = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    odom_gt[2,0] = theta













def call_buffer_1():


    #Function to erased the previous scan of th buffer 1 and collect a new one
    Timer = rospy.get_time()
    while rospy.get_time() < Timer + 15 :                         # once empty, wait for the buffer to be filled
                                    # wait for 1 second
        create_buffer_1()                                        # allow the buffer to collect data
        rospy.sleep(1)
        print"   # map initialization: ", int(100*(rospy.get_time() - Timer)/15), "%"      # Print the current situation of the buffer


    return data.return_source()






def call_buffer_2():


    #Function to erased the previous scan of th buffer 2 and collect a new one


    raw_input("\n      would you like to process the scan ? [ENTER]\n") # ask you the permition to execute the second scan

    print "\n   ############################## Buffer  Cleared ####################################\n"


    delete_buffer_2()            # ask for the buffer to empty it current data
    rospy.sleep(1)

    #Function to erased the previous scan of th buffer 1 and collect a new one
    Timer = rospy.get_time()
    while rospy.get_time() < Timer + 15:                         # once empty, wait for the buffer to be filled

        create_buffer_2()                                        # allow the buffer to collect data
        rospy.sleep(1)
        print"   # scan buffer: ", int(100*(rospy.get_time() - Timer)/15), "%"      # Print the current situation of the buffer
                                 # wait for 1 second


    return data.return_target()








def from_icp2world(odom,point):


    fact = 0

    det_theta = point[2,0] + odom[2,0]

    point[2,0] = 1





    tmp = np.array([[np.cos((fact*np.pi)+odom[2,0])  ,  np.sin((fact+np.pi)+odom[2,0])  ,        0      ],
                    [-np.sin((fact*np.pi)+odom[2,0]) ,  np.cos((fact+np.pi)+odom[2,0])   ,        0      ],
                    [              0                 ,                   0               ,        1      ]])

    point = np.dot(tmp,point)

    point[1,0] = -1*point[1,0]

    point[0,0] += odom[0,0]
    point[1,0] += odom[1,0]


    point[2,0] = det_theta

    return point










def from_world2icp(odom,T):

    point = np.zeros((3,1))

    point[0,0] = T[0,0]-odom[0,0]
    point[1,0] = T[1,0]-odom[1,0]
    point[2,0] =   1



    tmp = np.array([[ np.cos(odom[2,0]) , np.sin(odom[2,0]) ,  0 ],
                    [-np.sin(odom[2,0]) , np.cos(odom[2,0]) ,  0 ],
                    [        0           ,          0         ,  1 ],])

    point = np.dot(tmp,point)


    return point







def callback_target(var):

    global pc_target

    pc_target = var













if __name__ == '__main__':


    rospy.init_node('Static_SLAM', anonymous=True) 	# initiate the node

    sub_gt       = rospy.Subscriber('/desistek_saga/pose_gt'        , Odometry  , callback_gt) # Subscribes to the Ground Truth pose
    sub_target   = rospy.Subscriber('/SLAM/buffer/pointcloud_target', PointCloud, callback_target) # Subscribes to the Ground Truth pose

    pub1         = rospy.Publisher('/SLAM/buffer_1', Bool   , queue_size=1) # Create publisher to enable or disable the buffer 1 [True = enable / False = disable]
    pub2         = rospy.Publisher('/SLAM/buffer_2', Bool   , queue_size=1) # Create publisher to enable or disable the buffer 2 [True = enable / False = disable]
    pub_odom     = rospy.Publisher('/SLAM/offset'  , my_msg , queue_size=1)
    pub_T        = rospy.Publisher('/SLAM/T'       , my_msg , queue_size=1)

    data = retrive_data() # create the class to retrive the data from the scans
    kf = KF()             # create the Kalman Filter
    initialization = True # initialization variable
    error = 0.0001        # initiate the counter

    rospy.sleep(4)


    print " "
    print "   ###################################################################################"
    print "   #                             BLUEROV2 - ICP-SLAM                                 #"
    print "   ###################################################################################"
    print "   # Author : Timothee Freville                                                      #"
    print "   # Supervisor : Yvan Petillot                                                      #"
    print "   # Departement : Engineering & Physical Departement                                #"
    print "   # University : Heriot Watt - Edinburgh                                            #"
    print "   # Github : https://github.com/Tim-HW/Tim-HW-BlueRov2_Sonar_based_SLAM-            #"
    print "   ###################################################################################"
    print "   # The project was created for a Msc Robotics final thesis.                        #"
    print "   # The idea was to implemente a robust sonar-base SLAM method for underwater ROV.  #"
    print "   # To achived this we used the an ICP method coupled with a Kalman Filter for the  #"
    print "   # Localization and octomap server for the Mapping method.                         #"
    print "   ###################################################################################"
    print " "


    print " "
    print "   ###################################################################################"
    print "   #                           Mapping initialization                                #"
    print "   ###################################################################################"
    print " "

    #raw_input("\n            Would you like to initialize the map here ? [ENTER]")

    source,odom_source = call_buffer_1() # ask the buffer 1 to scan








    while not rospy.is_shutdown():








        print " "
        print "   ###################################################################################"
        print "   #                                   SCAN                                          #"
        print "   ###################################################################################"


        source,odom_source = data.return_source()

        target,odom_target = call_buffer_2() # empty the scan 2 and re-ask for scan


        print " "
        print "   ###################################################################################"
        print "   #                                   ICP                                           #"
        print "   ###################################################################################"

        print "\n   # Number of points in the map    :",len(source)
        print "\n   # Number of points in the buffer :",len(target)
        print " "


        T = data.initial_guess()   # initial guess of the transform

        ICP = Align2D(target,source,T)               # create an ICP object




        output_ICP,error = ICP.transform                  # get the position according to the ICP



        while error > 0.5:  # if the error is too high

            print " "
            print "   ###################################################################################"
            print "   #                                   ICP                                           #"
            print "   ###################################################################################"

            print "\n   # The ICP error is :", error, "\n"
                                                                                         # display the error
            answer = raw_input("\n           would you like to continue or re-scan the environment ([Y] : continue / [N] : re-scan)\n")    # asking you what to do

            if answer == "Y":       # if you want to continue the error becomes 0 and you escape the loop

                error = 0

            else:                   # otherwise relaunch the scans


                target,odom_target = call_buffer_2() # empty the scan 2 and re-ask for scan

                T = data.initial_guess()             # initial guess of the transform

                ICP = Align2D(target,source,T)               # create an ICP object


                output_ICP,error = ICP.transform    # get the position according to the ICP




        print "\nOutput ICP :\n", output_ICP,"\n"

        # transform the 3x3 matrix into a 3x1 matrix




        observation = np.array([[output_ICP[0,2]],
                                [output_ICP[1,2]],
                                [(odom_target[2,0] - odom_source[2,0]) + np.arctan(output_ICP[1,0]/output_ICP[0,0]) ]])     # theta





        print "\nobservation before frame changed :\n",observation,"\n"


        offset_update = observation             # initial_guess + ICP output

        #print "offset map", offset_update

        msg = my_msg()              # create msg type
        msg.x     = offset_update[0,0]   # offset in x
        msg.y     = offset_update[1,0]   # offset in y
        msg.theta = -offset_update[2,0]   # offset in theta
        pub_T.publish(msg)



        observation = from_icp2world(odom_source,observation)


        #print "\nobservation after frame changed :\n",observation,"\n"



        odometry    = odom_target # the odometry becomes the last scan done






        print "\n Odometry :\n",odometry          # print the odometry
        print "\n Sensor   :\n",observation       # print the scan-matching solution                                                       # print the pose of the observation

        kf.prediction(odometry)                         # prediction step
        new_pose = kf.correction(observation)           # correction step

        print"\n Kalman Filter :\n", new_pose            # print the new pose
        print"\n GROUND TRUTH position :\n", odom_gt    # print the Ground Truth pose

        #print"\n RMS before SLAM : ",RMS_pre_KF,"\n"
        #print"\n RMS after SLAM  : ",RMS_post_KF,"\n"

        new_pose = new_pose - odometry # we compute the offset between the new pose and the odometry

        print"\n offset :\n", new_pose  # print the offset

        msg = my_msg()              # create msg type
        msg.x     = new_pose[0,0]   # offset in x
        msg.y     = new_pose[1,0]   # offset in y
        msg.theta = new_pose[2,0]   # offset in theta
        pub_odom.publish(msg)       # publish the offset

        delete_buffer_1()

        del ICP

        rospy.sleep(1)

        create_buffer_1()
