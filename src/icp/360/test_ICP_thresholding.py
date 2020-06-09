#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np
import csv
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib as plt



if __name__ == '__main__':
my_path = os.path.abspath(os.path.dirname(__file__))
path = os.path.join(my_path, "xy_test.csv")
with open(path, mode="r") as csv_file:
  data = csv.reader(f)
  for row in data:
        print(row)
