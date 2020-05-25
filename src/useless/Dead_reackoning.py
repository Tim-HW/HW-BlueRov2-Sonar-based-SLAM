#! /usr/bin/env python

import sys
import roslib
import rospy
import matplotlib.pyplot as plt
import csv
import os
import math
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

final_x = []
final_y = []
final_z = []
final_yaw = []

odom_x = []
odom_y = []
odom_z = []
odom_yaw = []

tmp_odom_x = []
tmp_odom_y = []
tmp_odom_z = []
tmp_odom_yaw = []

tmp_final_x = []
tmp_final_y = []
tmp_final_z = []
tmp_final_yaw = []


error_x = []
error_y = []
error_z = []
error_yaw = []






# write in the CSV file the corrected position

def dvl_callback(data):



	global tmp_final_x
	global tmp_final_y
	global tmp_final_z
	global tmp_final_yaw


	global final_x
	global final_y
	global final_z
	global final_yaw


	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z

	rot_q = data.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

	tmp_final_x.append(x)
	tmp_final_y.append(y)
	tmp_final_z.append(z)
	tmp_final_yaw.append(yaw)



def perfect_callback(data):

	global tmp_odom_x
	global tmp_odom_y
	global tmp_odom_z
	global tmp_odom_yaw

	global odom_x
	global odom_y
	global odom_z
	global odom_yaw

	global error_x
	global error_y
	global error_z
	global error_yaw


	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	z = data.pose.pose.position.z

	rot_q = data.pose.pose.orientation
	(roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

	tmp_odom_x.append(x)
	tmp_odom_y.append(y)
	tmp_odom_z.append(z)
	tmp_odom_yaw.append(yaw)








rospy.init_node("plot_DVL")



while rospy.get_time() < 50:


	tmpx = 0
	tmpy = 0
	tmpz = 0
	tmpyaw = 0


	#print""                                          # initialization of the node
	#print('time: ',rospy.get_time())
	sub = rospy.Subscriber("/odom", Odometry, dvl_callback)       # initialization of the final publisher
	sub2 = rospy.Subscriber("/desistek_saga/pose_gt", Odometry, perfect_callback)       # initialization of the final publisher
	if (len(tmp_odom_x) > 27):
		for i in range(len(tmp_odom_x)-10,len(tmp_odom_x)):
			tmpx += tmp_odom_x[i]
			tmpy += tmp_odom_y[i]
			tmpz += tmp_odom_z[i]
			tmpyaw += tmp_odom_yaw[i]

		odom_x.append(tmpx/len(tmp_odom_x))
		odom_z.append(tmpy/len(tmp_odom_z))
		odom_y.append(tmpz/len(tmp_odom_y))
		odom_yaw.append(tmpyaw/len(tmp_odom_yaw))

	tmpx = 0
	tmpy = 0
	tmpz = 0
	tmpyaw = 0


	if (len(tmp_final_x) > 10):

		for i in range(len(tmp_final_x)-10,len(tmp_final_x)):
			tmpx += tmp_final_x[i]
			tmpy += tmp_final_y[i]
			tmpz += tmp_final_z[i]
			tmpyaw += tmp_final_yaw[i]

		final_x.append(tmpx/len(tmp_final_x))
		final_z.append(tmpy/len(tmp_final_z))
		final_y.append(tmpz/len(tmp_final_y))
		final_yaw.append(tmpyaw/len(tmp_final_yaw))
	tmpx = 0
	tmpy = 0
	tmpz = 0
	tmpyaw = 0




	tmp_odom_x = []
	tmp_odom_y = []
	tmp_odom_z = []
	tmp_odom_yaw = []

	tmp_final_x = []
	tmp_final_y = []
	tmp_final_z = []
	tmp_final_yaw = []
	rospy.sleep(0.5)



print("odom_x:", len(odom_x))
print("odom_y:", len(odom_y))
print("odom_z:", len(odom_z))
print("odom_yaw:", len(odom_yaw))
print " "

print("final_x:", len(final_x))
print("final_y:", len(final_y))
print("final_z:", len(final_z))
print("final_yaw:", len(final_yaw))
#print"min :",min(len(odom_x),len(odom_y),len(odom_z),len(odom_yaw))

for i in range(min(len(odom_x),len(final_x))):
	error_x.append(np.abs(odom_x[i] - final_x[i]))
	error_y.append(np.abs(odom_y[i] - final_y[i]))
	error_z.append(np.abs(odom_z[i] - final_z[i]))
	error_yaw.append(np.abs(odom_yaw[i] - final_yaw[i]))

print("error_x:", len(error_x))
print("error_y:", len(error_y))
print("error_z:", len(error_z))
print("error_yaw:",len(error_yaw))

t = np.linspace(0.0, 200.0, num=min(len(odom_x),len(final_x)), endpoint=True)

print("t",len(t))


print('valeur 1 :',error_x[0])
print('valeur derniere :',error_x[len(error_x)-10])

plt.figure()
plt.subplot(121)
plt.plot(odom_x,odom_y, 'r-')	 #print the GPS
plt.plot(final_x,final_y, 'g-')  #print the correction
plt.title('Odometry and Ground Truth')


plt.figure()
plt.subplot(221)
plt.plot(t,error_x, 'r-')				   #print the GPS
plt.title('x error[m]')
plt.subplot(222)
plt.plot(t,error_y, 'r-')				   #print the GPS
plt.title('y error[m]')
plt.subplot(223)
plt.plot(t,error_z, 'r-')				   #print the GPS
plt.title('z error[m]')
plt.subplot(224)
plt.plot(t,error_yaw, 'r-')				   #print the GPS
plt.title('yaw error[rad]')
plt.show() #show

"""
my_path = os.path.abspath(os.path.dirname(__file__))
path = os.path.join(my_path, "dead_reck.csv")
with open(path, mode="w") as csv_file:
	fieldname=['x','y','z','yaw']
	csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
	csv_writer.writeheader()
	for i in range(min(len(final_x),len(final_y),len(final_z),len(final_yaw))):
		csv_writer.writerow({'x':final_x[i],'y':final_y[i],'z':final_z[i],'yaw':final_yaw[i]})

path = os.path.join(my_path, "odom.csv")
with open(path, mode="w") as csv_file:
	fieldname=['x','y','z','yaw']
	csv_writer = csv.DictWriter(csv_file, fieldnames=fieldname)
	csv_writer.writeheader()
	for i in range(min(len(odom_x),len(odom_y),len(odom_z),len(odom_yaw))):
		csv_writer.writerow({'x':odom_x[i],'y':odom_y[i],'z':odom_z[i],'yaw':odom_yaw[i]})
"""
