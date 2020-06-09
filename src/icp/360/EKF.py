#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.spatial import KDTree
import random
import matplotlib.pyplot as plt

from class_icp import Align2D
from class_retrive_data import retrive_data


class EKF(object):

    def __init__(self, u_t,u_t_1):




        self.u_t_1  = u_t_1

        self.u_t    = u_t

        self.sig        = 50*np.eye(3)

        self.mu         = np.array([0.,0.,0.])

        self.mu_bar     = []
        self.sig_bar    = np.array([[0,0,0]])

        self.C          = np.eye(3)     # obersavtion model
        self.R          = 50*np.eye(3)     # noise of the motion
        self.Q          = 50*np.eye(3)     # noise of the observation







    def prediction(self):



        A = np.eye(3)

        B = np.array([[-np.cos(self.u_t[2][0]), np.sin(self.u_t[2][0]),0],
                      [np.sin(self.u_t[2][0]) , np.cos(self.u_t[2][0]),0],
                      [0,0,1]])

        self.mu_bar     = self.u_t_1
        #self.mu_bar     = np.dot(A,self.u_t_1) + np.dot(B,self.u_t)
        self.sig_bar    = np.dot(np.dot(A,self.sig),A.T) + self.R
        #print(self.sig_bar)








    def correction(self,observation):

        z  = observation


        K          = np.dot(self.sig_bar,np.dot(self.C.T,np.linalg.inv(np.dot(np.dot(self.C,self.sig_bar),self.C.T) + self.Q)))
        #print(self.mu)

        self.mu    = self.mu_bar + np.dot(K,(z - np.dot(self.C,self.mu_bar)))

        #self.sig   = (np.eye(3) - K * self.C)* self.sig_bar
        #print(self.mu_bar)
        return self.mu
