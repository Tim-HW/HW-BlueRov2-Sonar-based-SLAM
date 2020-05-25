#!/usr/bin/env python


import rospy
import roslib
import sys
import matplotlib.pyplot as plt
import numpy as np
from sensor_msgs.msg import PointCloud
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D



class points():

    def __init__(self):

        self.pub = rospy.Publisher("/PointCloud",PointCloud,queue_size=1)               #Create a topic with the new PointCloud
        self.sub = rospy.Subscriber("/tritech_micron/scan", PointCloud, self.get_data)  # Subscribs to the topcis to get the data

    def get_data(self,var):


        PC = PointCloud()               # create the new object PointCloud to be published later
        PC.header = var.header          # copy the content of topic to another PointCloud in order to manipulate it
        PC.channels = var.channels
        PC.points = var.points

        for i in range((len(PC.points))):

            PC.points[i].z = PC.channels[0].values[i]/100   # write in the z axis of the points the values of the "intensity" divided by 100 to make it readable on rviz

        self.pub.publish(PC)  # publish it into the new topic



if __name__ == "__main__":
    # craeting the node
    rospy.init_node("get_data")
    classe = points()
    rospy.spin()
