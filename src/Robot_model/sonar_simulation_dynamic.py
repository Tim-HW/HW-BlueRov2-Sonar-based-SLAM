#!/usr/bin/env python


import rospy
import struct
import numpy as np


from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


# initialization of the variables
laser = LaserScan()
final_laser = LaserScan()
laser_list = []



def callback(arg):
    """
    This callback is made to retrive data from the laserscan and make it global
    """
    global laser
    laser = arg  #laser is the current state of the laserscan



if __name__ == "__main__":

    rospy.init_node("sonar_dynamic")                                                        # initialization of the node
    pub = rospy.Publisher("/own/simulated/dynamic/sonar_LS", LaserScan, queue_size=1)       # initialization of the final publisher

    while not rospy.is_shutdown():

        for i in range(1,396): #for every value in the Laserscan

            sub = rospy.Subscriber("/desistek_saga/sonar", LaserScan, callback) # update the value of the laserscan

            #if the laser is empty does nothing
            if len(laser.ranges) == 0:
                pass

            else:

                #otherwise if lenght of the value is less than the max lenght of the laserscan
                if len(laser_list) < 396:

                    laser_list.append(laser.ranges[i])# add another element to a list

                    if len(laser_list) > 0 :

                        for a in range(len(laser_list)-1):

                            laser_list[a] = 0

                else:

                    laser_list[i] = laser.ranges[i] # if the list is already filled just change de value


                    for a in range(i-1):

                        laser_list[a] = 0





                laser_tuple = tuple(laser_list)# change list into a tuple

                final_laser.header              = laser.header
                final_laser.angle_min           = laser.angle_min
                final_laser.angle_max           = laser.angle_max
                final_laser.angle_increment     = laser.angle_increment
                final_laser.time_increment      = laser.time_increment
                final_laser.scan_time           = laser.scan_time
                final_laser.range_min           = laser.range_min
                final_laser.range_max           = laser.range_max

                final_laser.ranges              = laser_tuple
                # make the final_laser with the same time as the real conversion
                # but the ranges will be incremented one by one such as a sonar
                rospy.sleep(0.02020202)
                """
                print (" ")
                print(laser_list)
                print(" ")
                rospy.sleep(0.25)
                print(len(laser_list))
                #laser_list = []
                """
                pub.publish(final_laser) # publish the final laser
