#!/usr/bin/env python

"""
This program converts datas from DVL and IMU into the position of the robot.
Input:	/desisek_saga/dvl
		/desistek_saga/imu
Output:	/odom
In __init__, set OFFSET_X, OFFSET_Y and OFFSET_Z equal to the distance in xyz between the DVL and the inertial center of the robot.
	   , set STARTING_X, Y, Z equal to the xyz starting position of the robot, and STARTING_radianX, Y, Z equal to its orientation in radians
"""

import rospy
from sonar_mapping.msg import my_msg
from uuv_sensor_ros_plugins_msgs.msg import DVL
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import math
import sys

class dvl:




	def __init__(self):

		sub_dvl = rospy.Subscriber('/desistek_saga/dvl', DVL, self.dvl_sub)
		sub_imu = rospy.Subscriber('/desistek_saga/imu', Imu, self.imu_sub)
		sub_correction = rospy.Subscriber('/SLAM/offset', my_msg, self.odom_sub)

		self.pubOdom = rospy.Publisher("/odom", Odometry, queue_size=1)

		self.offset = my_msg()

		# value to correct the odomtry with the SLAM

		self.timeDVL = rospy.get_time()
		self.previous_time = 0
		self.dvlseq = 0
		self.dvlsecs = 0
		self.dvlnsecs = 0
		self.dvlX = 0
		self.dvlY = 0
		self.dvlZ = 0

		####################### To be se correctly depending on the robot configuration ###################
		self.OFFSET_X = 0
		self.OFFSET_Y = 0
		self.OFFSET_Z = 0

		self.STARTING_X = -250
		self.STARTING_Y =  300
		self.STARTING_Z = -5
		self.STARTING_radianX = 0
		self.STARTING_radianY = 0
		self.STARTING_radianZ = -1.0
		###################################################################################################

		self.timeIMU = rospy.get_time()
		self.quaternionX = 0
		self.quaternionY = 0
		self.quaternionZ = 0
		self.quaternionW = 0
		self.imuX = 0
		self.imuY = 0
		self.imuZ = 0
		self.angvelX = 0
		self.angvelY = 0
		self.angvelZ = 0
		self.lastImuX = 0
		self.lastImuY = 0
		self.lastImuZ = 0

		self.dvlReceived = False

		self.estimated_traj_x = self.STARTING_X
		self.estimated_traj_y = self.STARTING_Y
		self.estimated_traj_z = self.STARTING_Z

	def odom_sub(self,msg):

		if msg.x == self.offset.x and msg.y == self.offset.y and msg.theta == self.offset.theta:
			pass
		else:
			self.offset.x += msg.x
			self.offset.y += msg.y
			self.offset.theta += msg.theta


	# The frequency of the DVL is lower than the IMU
	def dvl_sub(self,msg):
		self.timeDVL = rospy.get_time()

		self.dvlseq = msg.header.seq
		self.dvlsecs = msg.header.stamp.secs
		self.dvlnsecs = msg.header.stamp.nsecs
		self.dvlX = msg.velocity.z 		############ /!\ ###########
		self.dvlY = msg.velocity.y
		self.dvlZ = msg.velocity.x      ############ /!\ ###########

		self.dvlReceived = True

	# The frequency of the IMU is bigger than the DVL's
	def imu_sub(self,msg):
		if self.dvlReceived == True:
			self.dvlReceived = False
			self.timeIMU = rospy.get_time()
			self.quaternionX = msg.orientation.x
			self.quaternionY = msg.orientation.y
			self.quaternionZ = msg.orientation.z
			self.quaternionW = msg.orientation.w
			X,Y,Z = self.quaternion_to_euler(self.quaternionX,self.quaternionY,self.quaternionZ,self.quaternionW)
			self.imuX = X
			self.imuY = Y
			self.imuZ = Z

			self.angvelX = msg.angular_velocity.x
			self.angvelY = msg.angular_velocity.y
			self.angvelZ = msg.angular_velocity.z

			self.estimateTraj()


	def estimateTraj(self):
		dt = float(self.timeDVL - self.previous_time)
		X = self.dvlX - self.OFFSET_X*(self.imuZ-self.lastImuZ)/dt
		Y = self.dvlY - self.OFFSET_Y*(self.imuZ-self.lastImuZ)/dt

		self.estimated_traj_x = self.estimated_traj_x + (X * dt * math.cos(self.imuZ) - Y * dt * math.sin(self.imuZ))
		self.estimated_traj_y = self.estimated_traj_y + (X * dt * math.sin(self.imuZ) + Y * dt * math.cos(self.imuZ))
		self.estimated_traj_z = self.estimated_traj_z - self.dvlZ * dt
		self.previous_time = self.timeDVL
		self.lastImuX = self.imuX + self.STARTING_radianX
		self.lastImuY = self.imuY + self.STARTING_radianY
		self.lastImuZ = self.imuZ + self.STARTING_radianZ

		self.convert_to_odom()

	def convert_to_odom(self):
		odm = Odometry()
		odm.header.seq = self.dvlseq
		rostime = rospy.get_time()
		odm.header.stamp.secs = int(rostime)
		odm.header.stamp.nsecs = 1000000000*(rostime-int(rostime))

		odm.header.frame_id = "world"
		#odm.child_frame_id = "desistek_saga/base_link"

		odm.pose.pose.position.x = self.offset.x + self.estimated_traj_x
		odm.pose.pose.position.y = self.offset.y + self.estimated_traj_y


		odm.pose.pose.position.z = self.estimated_traj_z

		odm.pose.pose.orientation.x = self.quaternionX
		odm.pose.pose.orientation.y = self.quaternionY
		odm.pose.pose.orientation.z = self.quaternionZ
		odm.pose.pose.orientation.w = self.quaternionW

		self.pubOdom.publish(odm)

	def quaternion_to_euler(self,x,y,z,w):
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		X = math.degrees(math.atan2(t0, t1))

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		Y = math.degrees(math.asin(t2))

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		Z = math.atan2(t3, t4)

		return X, Y, Z

def main(args):

	rospy.init_node('read_dvl', anonymous=True)

	a = dvl()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
