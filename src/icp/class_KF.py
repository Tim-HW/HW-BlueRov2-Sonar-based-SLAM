#!/usr/bin/env python

import rospy
import numpy as np



class KF(object):


    ############################################################
    #             Kalman Filter Object
    ############################################################

    def __init__(self):

        ############################################################
        #             Kalman Filter initialization
        ############################################################


        self.sig        = 50*np.eye(3)              # initial covariance
        self.sig_bar    = np.array([[0,0,0]])       # initialization of the predicted covariance

        self.mu         = np.array([0.,0.,0.])      # initialization of robot the position
        self.mu_bar     = []                        # initialization of the predicted robot position


        self.R          = 100*np.eye(3)     # noise of the motion
        self.Q          = 10*np.eye(3)      # noise of the observation







    def prediction(self,u_t):

        ############################################################
        #             Kalman Filter : prediction method
        ############################################################
        # Input : predicted motion form the Odometry
        ############################################################


        self.u_t    = u_t   # the pose is directly equal to the current pose of the robot

        A = np.eye(3)       # motion mode equal to a diagonal matrix


        self.mu_bar     = self.u_t                                  # predicted pose = last pose

        self.sig_bar    = np.dot(np.dot(A,self.sig),A.T) + self.R   # compute the covariance associate









    def correction(self,observation):


        ############################################################
        #             Kalman Filter : prediction method
        ############################################################
        # Input : Observation model
        # Output: New pose
        ############################################################
        self.C          = np.eye(3)     # obersavtion model

        z  = observation


        K = np.dot(self.sig_bar,np.dot(self.C.T,np.linalg.inv(np.dot(np.dot(self.C,self.sig_bar),self.C.T) + self.Q))) # compute the Kalman Gain

        self.mu = self.mu_bar + np.dot(K,(z - np.dot(self.C,self.mu_bar))) # compute the new pose using the innovation
        self.sig   = (np.eye(3) - K * self.C)* self.sig_bar                # compute the new covariance of the new pose

        return self.mu  # return the new pose
