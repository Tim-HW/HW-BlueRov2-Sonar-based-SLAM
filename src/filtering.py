#!/usr/bin/env python


import rospy
import roslib
import sys
import numpy as np
from sensor_msgs.msg import PointCloud, PointCloud2



class points():

    def __init__(self):
        self.pub1 = rospy.Publisher("/tritech_micron/filttered_scan",PointCloud,queue_size=1)               #Create a topic with the new PointCloud
        self.pub2 = rospy.Publisher("/tritech_micron/scan_3D",PointCloud,queue_size=1)               #Create a topic with the new PointCloud
        self.sub = rospy.Subscriber("/tritech_micron/scan", PointCloud, self.get_data)  # Subscribs to the topcis to get the data

    def get_data(self,var):


        PC = PointCloud()               # create the new object PointCloud to be published later

        PC.header = var.header          # copy the content of topic to another PointCloud in order to manipulate it
        PC.channels = var.channels
        PC.points = var.points


        """
        Function to get the "3D" map
        """

        for i in range((len(PC.points))):

            PC.points[i].z = PC.channels[0].values[i]/100   # write in the z axis of the points the values of the "intensity" divided by 100 to make it readable on rviz

        self.pub2.publish(PC)  # publish it into the new topic

        """
        Function to filtter the map and get only one point by degree
        """

        index_max = PC.channels[0].values.index(max(PC.channels[0].values))

        for i in range(len(PC.points)): #396

            if i != index_max:
                PC.points[i].x = PC.points[i].y = PC.points[i].z = 0
            else:
                PC.points[i].z = 0

        self.pub1.publish(PC)  # publish it into the new topic


######################################################################################





if __name__ == "__main__":
    # craeting the node
    rospy.init_node("get_data")
    classe = points()
    rospy.spin()
