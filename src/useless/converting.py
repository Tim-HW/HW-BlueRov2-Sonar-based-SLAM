#!/usr/bin/env python
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud ,PointCloud2, PointField
from std_msgs.msg import Header


def callback(arg):

    points = []
    for k in range(len(arg.points)):

        x = float(arg.points[k].x)
        y = float(arg.points[k].y)
        z = float(0)

        r = int(255.0)
        g = int(255.0)
        b = int(255.0)
        a = 255

        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [x, y, z, rgb]
        points.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 0, PointField.FLOAT32, 1),
              PointField('z', 0, PointField.FLOAT32, 1),
              # PointField('rgb', 12, PointField.UINT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]


    header = Header()
    header.frame_id = "sonar"
    pc2 = point_cloud2.create_cloud(header, fields, points)
    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node("create_cloud_xyzrgb")
        pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=1)
        sub = rospy.Subscriber("/tritech_micron/filttered_scan", PointCloud, callback)
