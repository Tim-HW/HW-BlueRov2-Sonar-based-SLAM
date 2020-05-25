#!/usr/bin/env python

import rospy
import roslib
import sys
import math
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan
import tf

class ConvertPC2LS():
	def __init__(self):
		self.sub = rospy.Subscriber("/tritech_micron/filtered_scan", PointCloud, self.convert)
		self.publs = rospy.Publisher("/tritech_micron/sonar_laserscan", LaserScan, queue_size=1)

		self.i = 0
		self.scan = [0] * 200	# number of values from sonar

		self.seq = 0
		self.secs = 0
		self.nsecs = 0

	def convert(self,msg):
		self.seq = msg.header.seq
		self.secs = msg.header.stamp.secs
		self.nsecs = msg.header.stamp.nsecs

		w = 0
		while w < 392:
			if msg.points[w].x != 0 or msg.points[w].y != 0:
				self.scan[self.i] = math.sqrt(msg.points[w].x**2+msg.points[w].y**2)
				w = 1000
			else:
				w+=1
		if w < 1000:
			self.scan[self.i] = 0
		self.i +=1
		if self.i >= 200:
			self.i = 0
		self.publish()

	def publish(self):
		ls = LaserScan()

		ls.header.seq = self.seq
		ls.header.stamp.secs = self.secs
		ls.header.stamp.nsecs = self.nsecs
		ls.header.frame_id = "sonar"

		ls.angle_min = -math.pi
		ls.angle_max = math.pi
		ls.angle_increment = 1.8*math.pi/180
		ls.scan_time =  9.4
		ls.range_min:0.0
		ls.range_max = 70.0
		ls.ranges = self.scan
		self.publs.publish(ls)

if __name__ == "__main__":
    # creating the node
    rospy.init_node("convert_data")
    classe = ConvertPC2LS()
    rospy.spin()
