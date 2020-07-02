#!/usr/bin/env python


import sys
import rospy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion,Point32
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from sonar_mapping.msg import my_msg


def from_odom2world(point,odom):


    T = np.eye(3)


    point_array = np.array([point.x, point.y, 1])

    T[0,2] = odom[0,0]
    T[1,2] = odom[1,0]

    T[0,0] =  np.cos(-odom[2,0])
    T[1,1] =  np.cos(-odom[2,0])
    T[1,0] = -np.sin(-odom[2,0])
    T[0,1] =  np.sin(-odom[2,0])



    point_array = np.dot(T,point_array.T)

    point.x = point_array[0]
    point.y = point_array[1]
    point.z = -5

    return point



class mapping():

    def __init__(self):

        self.odom_source      = np.zeros((3,1))
        self.PC_source        = PointCloud()
        self.PC2_source       = PointCloud2()
        self.PC2_target       = PointCloud2()

        self.map              = PointCloud()
        self.map.header.stamp.secs = int(rospy.get_time())
        self.map.header.stamp.nsecs = 1000000000*(rospy.get_time()-int(rospy.get_time()))
        self.map.header.frame_id = "world"


        self.sub_odom        = rospy.Subscriber("/SLAM/buffer/odom_source", Odometry , self.callback_odom)
        self.sub_PC_source   = rospy.Subscriber("/SLAM/buffer/pointcloud_source", PointCloud , self.callback_source)
        self.sub_PC_source   = rospy.Subscriber("/SLAM/buffer/pointcloud2_source", PointCloud2, self.callback_source2)
        self.sub_PC_target   = rospy.Subscriber("/SLAM/buffer/pointcloud2_target", PointCloud2, self.callback_target)

        self.pub_map         = rospy.Publisher("/SLAM/map", PointCloud , queue_size = 1)


    def callback_odom(self,msg):

        self.odom_source[0,0] = msg.pose.pose.position.x
        self.odom_source[1,0] = msg.pose.pose.position.y

        rot_q  = msg.pose.pose.orientation

        orientation_list = [rot_q.x, rot_q.y, rot_q.z, rot_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.odom_source[2,0] = yaw


    def callback_source2(self,msg):

        self.PC2_source = msg

    def callback_source(self,msg):

        self.PC_source = msg


    def callback_target(self,msg):

        self.PC_target = msg

    def mapping(self):

        rostime = rospy.get_time()
        self.map.header.stamp.secs = int(rospy.get_time())
        self.map.header.stamp.nsecs = 1000000000*(rospy.get_time()-int(rospy.get_time()))
        self.map.header.frame_id = "world"

        point = Point32()
        point.x = 0
        point.y = 0
        point.z = 0

        self.map.points.append(point)



        print "\n", self.map,"\n"

        self.pub_map.publish(self.map)


    def update(self):

        while len(self.map.points) < 396:
            tmp_point = Point32()

            for i in range(len(self.PC_source.points)):


                tmp_point = self.PC_source.points[i]

                #print(tmp_point)
                tmp_point = from_odom2world(tmp_point,self.odom_source)
                #print(tmp_point)
                #print " "

                self.map.points.append(tmp_point)

        self.pub_map.publish(self.map)












if __name__ == '__main__':

    rospy.init_node('mapping', anonymous=True) 	# initiate the node
    map = mapping()
    init = True

    while not rospy.is_shutdown():

        if len(map.PC_source.points) < 396:
            pass
        else:
            print "publishing"
            map.update()
