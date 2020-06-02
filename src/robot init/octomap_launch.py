#!/usr/bin/env python


import roslaunch
import rospy


rospy.sleep(20)
rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/tim/uuv_ws/src/sonar_mapping/launch/octomap.launch"])
launch.start()
rospy.loginfo("started")

while not rospy.is_shutdown():
    pass
