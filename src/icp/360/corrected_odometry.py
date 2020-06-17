#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from sonar_mapping.msg import my_msg

odom     = Odometry()
offset   = my_msg()
new_odom = Odometry()

def callback_odom(msg):
    global odom

def callback_offset(msg):
    global offset



if __name__ == '__main__':

    rospy.init_node('odomtry_fusion', anonymous=True) 	# initiate the node

    sub_offset   = rospy.Subscriber('/SLAM/odom_offset', Odometry, callback_offset)                   # Subscribes to the Ground Truth pose
    sub_odom     = rospy.Subscriber('/odom', Odometry, callback_odom)
    pub          = rospy.Publisher('/SLAM/corrected_odom', Odometry, queue_size = 1)

    while not rospy.is_shutdown():

        x = odom.pose.pose.position.x + offset.x
        y = odom.pose.pose.position.y + offset.y

        new_odom = odom

        new_odom.pose.pose.position.x = x
        new_odom.pose.pose.position.y = y

        pub.publish(new_odom)
