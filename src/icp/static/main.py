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
    Timer = rospy.get_time()
    while rospy.get_time() < Timer + 15 :                         # once empty, wait for the buffer to be filled
                                    # wait for 1 second
        create_buffer_1()                                        # allow the buffer to collect data
        rospy.sleep(1)
        print"   # map initialization: ", int(100*(rospy.get_time() - Timer)/15), "%"      # Print the current situation of the buffer


    return data.return_source()


def call_buffer_2():


    #Function to erased the previous scan of th buffer 2 and collect a new one


    raw_input("\n           would you like to process second scan ? [ENTER]\n") # ask you the permition to execute the second scan
    """
    while len(pc_target.points) > 2: # while the buffer is not empty

        delete_buffer_2()            # ask for the buffer to empty it current data
        rospy.sleep(1)               # wait fir 1 second
    """
    #Function to erased the previous scan of th buffer 1 and collect a new one
    Timer = rospy.get_time()
    while rospy.get_time() < Timer + 15:                         # once empty, wait for the buffer to be filled

        create_buffer_2()                                        # allow the buffer to collect data
        rospy.sleep(1)
        print"   # scan buffer: ", int(100*(rospy.get_time() - Timer)/15), "%"      # Print the current situation of the buffer
                                 # wait for 1 second


    return data.return_target()



def from_icp2world(odom,point):




    tmp = np.array([[np.cos(odom[2,0])  , np.sin(odom[2,0]) , 0],
                    [-np.sin(odom[2,0]) , np.cos(odom[2,0]) , 0],
                    [        0          ,             0     , 1],])

    det_theta = point[2,0]

    point[2,0] = 1



    point = np.dot(tmp,point)

    point[2,0] = det_theta

    point[0,0] = -1*point[0,0]

    #point = odom + point

    return point



def from_world2icp(odom,T):

    point = np.zeros((3,1))

    point[0,0] = T[0,2]
    point[1,0] = T[1,2]
    point[2,0] =   1


    tmp = np.array([[ np.cos(-odom[2,0]) , np.sin(-odom[2,0]) , 0],
                    [-np.sin(-odom[2,0]) , np.cos(-odom[2,0]) , 0],
                    [        0           ,          0         , 1],])

    point = np.dot(tmp,point)

    point[2,0] = np.arccos(T[0,0])

    point[0,0] = -1*point[0,0]

    T[0,2] = point[0,0]
    T[1,2] = point[1,0]

    return T




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

    while not rospy.is_shutdown():


        if initialization == True:

            print " "
            print "   ###################################################################################"
            print "   #                      BLUEROV2 - ICP-Static_SLAM                                 #"
            print "   ###################################################################################"
            print "   # Author : Timothee Freville                                                      #"
            print "   # Supervisor : Yvan Petillot                                                      #"
            print "   # Departement : Engineering & Physical Departement                                #"
            print "   # University : Heriot Watt - Edinburgh                                            #"
            print "   ###################################################################################"
            print " "
            print "   ###################################################################################"
            print "   #                           Mapping initialization                                #"
            print "   ###################################################################################"
            print " "

            raw_input("\n            Would you like to initialize the map here ? [ENTER]")

            source,odom_source = call_buffer_1() # ask the buffer 1 to scan

            print " "
            print "   ###################################################################################"
            print "   #                                   SCAN                                          #"
            print "   ###################################################################################"

            target,odom_target = call_buffer_2() # ask the buffer 2 to scan

        if initialization == False:

            print " "
            print "   ###################################################################################"
            print "   #                                   SCAN                                          #"
            print "   ###################################################################################"

            target,odom_target = call_buffer_2() # empty the scan 2 and re-ask for scan

        print "   # Number of points in the map    :",len(source)
        print "   # Number of points in the buffer :",len(target)

        T = data.initial_guess()   # initial guess of the transform
        T = np.eye(3)
        #T = from_world2icp(odom_source,T)

        #print(T)

        print " "
        print "   ###################################################################################"
        print "   #                                   ICP                                           #"
        print "   ###################################################################################"



        ICP = Align2D(source,target,T)               # create an ICP object

        observation,error = ICP.transform                  # get the position according to the ICP



        while error > 0.1:  # if the error is too high

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

                ICP = Align2D(source,target,T)       # create an ICP object

                observation,error = ICP.transform    # get the position according to the ICP




        print "\nOutput ICP :\n",observation

        # transform the 3x3 matrix into a 3x1 matrix


        observation = np.array([[observation[0,2]],                 # x
                                [observation[1,2]],                 # y
                                [np.arccos(observation[0,0])]])     # theta

        offset_update = from_icp2world(odom_source,observation)

        msg = my_msg()              # create msg type
        msg.x     = offset_update[0,0]   # offset in x
        msg.y     = offset_update[1,0]   # offset in y
        msg.theta = offset_update[2,0]   # offset in theta
        pub_T.publish(msg)       # publish the offset


        observation = from_icp2world(odom_source, observation)  # transform the position from a ICP frame reference to World reference

        odometry    = odom_target # the odometry becomes the last scan done

        observation = odom_source + observation


        print "\n Odometry :\n",odometry          # print the odometry
        print "\n Sensor   :\n",observation       # print the scan-matching solution                                                       # print the pose of the observation

        kf.prediction(odometry)                         # prediction step
        new_pose = kf.correction(observation)           # correction step

        print"\n new position :\n", new_pose            # print the new pose
        print"\n GROUND TRUTH position :\n", odom_gt    # print the Ground Truth pose


        new_pose = new_pose - odometry # we compute the offset between the new pose and the odometry

        print"\n offset :\n", new_pose  # print the offset

        msg = my_msg()              # create msg type
        msg.x     = new_pose[0,0]   # offset in x
        msg.y     = new_pose[1,0]   # offset in y
        msg.theta = new_pose[2,0]   # offset in theta
        pub_odom.publish(msg)       # publish the offset

        delete_buffer_1()

        initialization = False      # change the case
