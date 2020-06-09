#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
import csv
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib as plt

from class_icp import Align2D
from EKF import EKF
from class_retrive_data import retrive_data





pc_source = PointCloud()
pc_target = PointCloud()

def callback_source(var):
    global pc_source

    pc_source = var


def callback_target(var):
    global pc_target

    pc_target = var






# write in the CSV file the corrected position
def write_CSVfile_yaw(yaw,error):

	my_path = os.path.abspath(os.path.dirname(__file__))
	path = os.path.join(my_path, "yaw_test.csv")
	with open(path, mode="w") as csv_file:
		fieldname=['yaw','error']
		csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
		csv_writer.writeheader()
		for i in range(0,len(yaw)):
			csv_writer.writerow({'yaw':yaw[i],'error':error[i]})

def write_CSVfile_x(x,error):

	my_path = os.path.abspath(os.path.dirname(__file__))
	path = os.path.join(my_path, "x_test.csv")
	with open(path, mode="w") as csv_file:
		fieldname=['x','error']
		csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
		csv_writer.writeheader()
		for i in range(0,len(x)):
			csv_writer.writerow({'x':x[i],'error':error[i]})

def write_CSVfile_y(y,error):

	my_path = os.path.abspath(os.path.dirname(__file__))
	path = os.path.join(my_path, "y_test.csv")
	with open(path, mode="w") as csv_file:
		fieldname=['y','error']
		csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
		csv_writer.writeheader()
		for i in range(0,len(y)):
			csv_writer.writerow({'y':y[i],'error':error[i]})

def write_CSVfile_xy(x,y,error):

	my_path = os.path.abspath(os.path.dirname(__file__))
	path = os.path.join(my_path, "xy_test.csv")
	with open(path, mode="w") as csv_file:
		fieldname=['x','y','error']
		csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
		csv_writer.writeheader()
		for i in range(0,len(x)):
			csv_writer.writerow({'x':x[i],'y':y[i],'error':error[i]})







def test_th(source,target):

    x  = 0
    y  = 0

    error_list = []
    theta_list = []

    for i in range(314):

        print i*100/314,"%"

        th = 0.02*i

        T = np.array([[ np.cos(th) , -np.sin(th) , x ],
                      [ np.sin(th) , np.cos(th)  , y ],
                      [     0      ,      0      , 1 ]])

        ICP = Align2D(source,target,T)               # create an ICP object

        error = ICP.transform

        if error > 5:
            error = 5

        error_list.append(error)
        theta_list.append(th)


    write_CSVfile_yaw(theta_list,error_list)





def test_y(source,target):

    error_list  = []
    y_list      = []
    th = 0.00
    x  = 0

    for i in range(300):

        print i*100/300,"%"
        y  = - 30 + 0.2*i


        T = np.array([[ np.cos(th) , -np.sin(th) , x ],
                      [ np.sin(th) , np.cos(th)  , y ],
                      [     0      ,      0      , 1 ]])



        ICP = Align2D(source,target,T)               # create an ICP object
        error = ICP.transform

        if error > 5:
            error = 5
        error_list.append(error)
        y_list.append(y)

    write_CSVfile_y(y_list,error_list)






def test_x(source,target):

    error_list  = []
    x_list      = []
    y  = 0
    th = 0.00

    for i in range(300):

        print i*100/300,"%"
        x  = - 30 + 0.2*i


        T = np.array([[ np.cos(th) , -np.sin(th) , x ],
                      [ np.sin(th) , np.cos(th)  , y ],
                      [     0      ,      0      , 1 ]])


        ICP = Align2D(source,target,T)               # create an ICP object

        error = ICP.transform

        if error > 5:
            error = 5

        error_list.append(error)
        x_list.append(x)

    write_CSVfile_x(x_list,error_list)






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
