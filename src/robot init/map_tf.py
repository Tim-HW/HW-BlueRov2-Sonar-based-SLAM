#!/usr/bin/env python

import roslib
roslib.load_manifest('learning_tf')
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header, String,Bool

state = False


x = 0
y = 0
z = 0
yaw = 0

def callback_state(msg):

	global state

	if state == False:

		state = msg.data

def handle_map_pose(var):

	global x,y,z,yaw

	if state == False:

		rot_q  = var.pose.pose.orientation

		x = var.pose.pose.position.x
		y = var.pose.pose.position.y
		z = var.pose.pose.position.z
		orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

		br = tf.TransformBroadcaster()
		br.sendTransform((x, y, z),
	                        tf.transformations.quaternion_from_euler(0, 0, yaw),
	                        rospy.Time.now(),
	                        'map',
	                        "world")


	if state == True:


		br = tf.TransformBroadcaster()
		br.sendTransform((x, y, z),
	                        tf.transformations.quaternion_from_euler(0, 0, yaw),
	                        rospy.Time.now(),
	                        'map',
	                        "world")







if __name__ == '__main__':

	rospy.init_node('tf_broadcaster_map')

	rospy.Subscriber('/odom', Odometry, handle_map_pose)

	rospy.Subscriber('/SLAM/buffer_1', Bool, callback_state)

	rospy.spin()
