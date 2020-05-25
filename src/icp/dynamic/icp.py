#!/usr/bin/env python

from __future__ import print_function
import roslib
roslib.load_manifest('sonar_mapping') # my package
import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, PointCloud
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.spatial import KDTree
import random
import matplotlib as mpl
import matplotlib.pyplot as plt

count = 0
source_PC = np.zeros((99,2))
target_PC = np.zeros((99,2))
T = 1

subscriber_state = False


class ICP:



    def __init__(self,source_points,target_points,initial_T):

		self.source = source_points
		self.target = target_points
		self.init_T = initial_T
		self.target_tree = KDTree(target_points[:,:2])
		self.transform = self.ASD(50, 0.1)










    def ASD(self,max_iter,min_delta_err):

        new_err = 10
        mean_sq_error = 1.0e6       # initialize error as large number
        delta_err = 0.1             # change in error (used in stopping condition)
        T = self.init_T
        num_iter = 0                # number of iterations
        tf_source = self.source

        while new_err > min_delta_err and num_iter < max_iter:
            # find correspondences via nearest-neighbor search
            matched_trg_pts,matched_src_pts,indices = self.FindCorrespondences(tf_source)
            # find alingment between source and corresponding target points via SVD
            # note: svd step doesn't use homogeneous points
            new_T = self.AlignSVD(matched_src_pts, matched_trg_pts)
            # update transformation between point sets
            T = np.dot(T,new_T)
            #print("T:\n", T)
            Rotation = T[:2,:2]
            #print("\n Rotation : \n",Rotation)
            Translation = T[:2,2]
            #print("\nTranslation: \n",Translation)
            # apply transformation to the source points
            #print("\nBefore roation \n",tf_source)
            tf_source = np.dot(Rotation,tf_source.T)
            tf_source = tf_source.T
            #tf_source = np.dot(self.source,T.T)
            #print("\nAfter roation \n",tf_source)
            tf_source = tf_source + Translation
            #print("\nAfter Translation \n",tf_source)
            # find mean squared error between transformed source points and target points
            new_err = 0
            for i in range(len(indices)):
                if indices[i] != -1:
                    print(tf_source[i,:2])
                    diff = tf_source[i,:2] - self.target[indices[i],:2]
                    new_err += np.dot(diff,diff.T)

            new_err /= float(len(matched_trg_pts))

            # update error and calculate delta error
            delta_err = abs(mean_sq_error - new_err)
            mean_sq_error = new_err
            print("error: ",mean_sq_error)

            num_iter += 1


        """
        plt.figure()
        plt.subplot(221)
        plt.plot(self.source[0,:],self.source[1,:], 'or')
        plt.plot(self.target[0,:],self.target[1,:], 'ob')
        plt.title('After ICP')
        plt.subplot(222)
        plt.plot(tf_source[0,:],tf_source[1,:], 'og')
        plt.title('Before ICP')
        plt.show() #show
        """

        plt.figure()
        plt.plot(self.target[:,0],self.target[:,1], 'or')
        plt.plot(tf_source[:,0],tf_source[:,1], 'ob')
        plt.title("after ICP")
        plt.show()
        #print("\nmatrix \n:",T)
        print("\niterations : ",num_iter)

        return T



    def AlignSVD(self,source,target):

        # first find the centroids of both point clouds
        #print(source)
        #print(" ")
        #print(target)
        src_centroid = np.sum(source,axis=0)/len(source)
        trg_centroid = np.sum(target,axis=0)/len(target)
        #print("centroids source:",src_centroid)
        #print("centroids target:",trg_centroid)

        # get the point clouds in reference to their centroids
        source_centered = source - src_centroid
        target_centered = target - trg_centroid



        # get cross covariance matrix M
        M = np.dot(target_centered.T,source_centered)

        # get singular value decomposition of the cross covariance matrix
        U,W,V_t = np.linalg.svd(M)

        # get rotation between the two point clouds
        R = np.dot(U,V_t)
        #print("\n Rotation matrix :\n",R)
        # get the translation (simply the difference between the point clound centroids)
        t = np.expand_dims(trg_centroid,0).T - np.dot(R,np.expand_dims(src_centroid,0).T)
        #print("\n Translation matrix:\n",t)

        # assemble translation and rotation into a transformation matrix
        T = np.identity(3)
        #print(" ")
        #print(T)
        T[:2,2] = np.squeeze(t)
        #print(" ")
        #print(T)

        T[:2,:2] = R
        #print(" ")
        #print(T)

        return T






    def FindCorrespondences(self,src_pts):

        """
        finds nearest neighbors in the target point for all points
        in the set of source points

        params:
        src_pts: array of source points for which we will find neighbors
        points are assumed to be homogeneous

        returns:
        array of nearest target points to the source points (not homogeneous)
        """

        # get distances to nearest neighbors and indices of nearest neighbors
        matched_src_pts = src_pts[:,:2]
        #print(matched_src_pts)
        dist,indices = self.target_tree.query(matched_src_pts)

        # remove multiple associatons from index list
        # only retain closest associations
        unique = False
        while not unique:
          unique = True
          for i in range(len(indices)):
        	  if indices[i] == -1:
        		  continue
        	  for j in range(i+1,len(indices)):
        		  if indices[i] == indices[j]:
        			  if dist[i] < dist[j]:
        			  	  indices[j] = -1
        			  else:
        				  indices[i] = -1
        				  break
        # build array of nearest neighbor target points
        # and remove unmatched source points
        point_list = []
        src_idx = 0
        for idx in indices:
          if idx != -1:
        	  point_list.append(self.target[idx,:])
        	  src_idx += 1
          else:
        	  matched_src_pts = np.delete(matched_src_pts,src_idx,axis=0)

        matched_pts = np.array(point_list)
        """
        print(matched_pts)
        print(" ")
        print(matched_src_pts)
        print(" ")
        print(indices)
        print(" ")
        print(self.source)
        print(" ")
        print(self.target)
        """
        return matched_pts[:,:2],matched_src_pts,indices





















def random_points(num):
    """
    Function to create random_points to test the code
    """


    ref = np.ones((num,2))
    obs = np.ones((num,2))

    for i in range(num):
        ref[i,0] = i
        ref[i,1] = i
        obs[i,0] = i
        obs[i,1] = i + 5

    #print(ref)
    #print(obs)
    plt.figure()
    plt.subplot(221)
    plt.plot(ref[:,0],ref[:,1], 'or')
    plt.plot(obs[:,0],obs[:,1], 'ob')
    plt.show()
    return ref,obs




##############################################################################################


def callback_PC(var):

    global count



    if count == 0 and source_PC[50,0] != var.points[50].x :

        for i in range(len(var.points)):

            source_PC[i,0] = var.points[i].x
            source_PC[i,1] = var.points[i].y
        count += 1

    elif count == 1 and source_PC[50,0] != var.points[50].x :

        for i in range(len(var.points)):

            target_PC[i,0] = var.points[i].x
            target_PC[i,1] = var.points[i].y
        count += 1



        #print("target filled")

##############################################################################################


def callback_state(var):

    subscriber_state = var.data



##############################################################################################





if __name__ == '__main__':



    rospy.init_node('ICP', anonymous=True)

    while not rospy.is_shutdown():


        rospy.Subscriber('/SLAM/buffer/pointcloud', PointCloud, callback_PC)




        if count == 2:

            #print(source_PC)
            #print(" ")
            #print(target_PC)

            plt.figure()
            plt.plot(target_PC[:,0],target_PC[:,1], 'or')
            plt.plot(source_PC[:,0],source_PC[:,1], 'ob')
            plt.title("before ICP")
            plt.show()

            icp = ICP(source_PC,target_PC,T)
