#!/usr/bin/env python

import roslib
import rospy
import tf

if __name__ == '__main__':
	rospy.init_node("sonar_fixed_frame")
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		br.sendTransform((0.0,0.0,0.0),
			(0.0,0.0,0.0,1.0),
			rospy.Time.now(),
			"fakescan",
			"sonar")
		rate.sleep()