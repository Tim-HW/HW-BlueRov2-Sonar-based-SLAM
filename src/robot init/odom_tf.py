#!/usr/bin/env python

import roslib
roslib.load_manifest('learning_tf')
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler




def handle_odom_pose(msg):

	rot_q  = msg.pose.pose.orientation

	orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
	(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	z = msg.pose.pose.position.z

	br = tf.TransformBroadcaster()
	br.sendTransform((x, y, -5), tf.transformations.quaternion_from_euler(0, 0, yaw), rospy.Time.now(),'odom',"world")




if __name__ == '__main__':

	rospy.init_node('tf_broadcaster')

	rospy.Subscriber('/odom', Odometry, handle_odom_pose)

	rospy.spin()
