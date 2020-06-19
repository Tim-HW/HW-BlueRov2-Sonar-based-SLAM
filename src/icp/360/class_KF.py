#!/usr/bin/env python


import rospy
import numpy as np



class KF(object):

    def __init__(self):




        self.sig        = 50*np.eye(3)
        self.sig_bar    = np.array([[0,0,0]])

        self.mu         = np.array([0.,0.,0.])
        self.mu_bar     = []



        self.R          = 100*np.eye(3)     # noise of the motion
        self.Q          = 10*np.eye(3)     # noise of the observation







    def prediction(self,u_t):

        self.u_t    = u_t

        A = np.eye(3)


        self.mu_bar     = self.u_t
        #self.mu_bar     = np.dot(A,self.u_t_1) + np.dot(B,self.u_t)
        self.sig_bar    = np.dot(np.dot(A,self.sig),A.T) + self.R
        #print(self.sig_bar)








    def correction(self,observation):

        self.C          = np.eye(3)     # obersavtion model

        z  = observation


        K = np.dot(self.sig_bar,np.dot(self.C.T,np.linalg.inv(np.dot(np.dot(self.C,self.sig_bar),self.C.T) + self.Q)))
        #print(self.mu)

        self.mu = self.mu_bar + np.dot(K,(z - np.dot(self.C,self.mu_bar)))
        self.sig   = (np.eye(3) - K * self.C)* self.sig_bar
        #print(self.mu_bar)
        return self.mu
