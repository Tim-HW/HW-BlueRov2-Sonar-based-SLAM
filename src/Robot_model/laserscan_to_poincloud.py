#!/usr/bin/env python


import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import math

class Laser2PC():

    def __init__(self):
        self.laserProj = LaserProjection()

        #self.pub = rospy.Publisher("/own/true/sonar_PC2",PointCloud2, queue_size=1)
        #self.pub1 = rospy.Publisher("/own/simulated/sonar_PC2",PointCloud2, queue_size=1)
        self.pub2 = rospy.Publisher("/own/simulated/dynamic/sonar_PC2",PointCloud2, queue_size=1)

        #self.sub = rospy.Subscriber("/desistek_saga/sonar",LaserScan, self.callback)
        #self.sub1 = rospy.Subscriber("/own/simulated/sonar_LS",LaserScan, self.callback1)
        self.sub2 = rospy.Subscriber("/own/simulated/dynamic/sonar_LS",LaserScan, self.callback2)

    """

    def callback(self,data):

        laser = LaserScan()

        laser.header = data.header
        laser.angle_min = data.angle_min
        laser.angle_max = data.angle_max
        laser.angle_increment = data.angle_increment
        laser.time_increment = data.time_increment
        laser.scan_time = data.scan_time
        laser.range_min = data.range_min
        laser.range_max = data.range_max
        laser.intensities = data.intensities
        laser.ranges = data.ranges

        cloud = self.laserProj.projectLaser(laser)

        self.pub.publish(cloud)


    def callback1(self,arg):

        laser = LaserScan()

        laser.header = arg.header
        laser.angle_min = arg.angle_min
        laser.angle_max = arg.angle_max
        laser.angle_increment = arg.angle_increment
        laser.time_increment = arg.time_increment
        laser.scan_time = arg.scan_time
        laser.range_min = arg.range_min
        laser.range_max = arg.range_max
        laser.intensities = arg.intensities
        laser.ranges = arg.ranges

        cloud = self.laserProj.projectLaser(laser)

        self.pub1.publish(cloud)
    """

    def callback2(self,arg):

        laser = LaserScan()

        laser.header = arg.header
        laser.angle_min = arg.angle_min
        laser.angle_max = arg.angle_max
        laser.angle_increment = arg.angle_increment
        laser.time_increment = arg.time_increment
        laser.scan_time = arg.scan_time
        laser.range_min = arg.range_min
        laser.range_max = arg.range_max
        laser.intensities = arg.intensities
        laser.ranges = arg.ranges

        cloud = self.laserProj.projectLaser(laser)

        self.pub2.publish(cloud)



if __name__ == "__main__":

    rospy.init_node("laser_to_cloud")
    l2px = Laser2PC()
    rospy.spin()
