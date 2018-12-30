#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math
# import rospy #TODO for use on robot

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        # self.last_time = rospy.get_time() # TODO for on bot Used to keep track of time between measurements 

        # control covarience (1,1) is variance of v from commanded: deviation of 0.03 m/s
        # (2,2) is varience of w from gyro measurement: deviation of 0.1 rad/s
        # self.Q_t = np.array([[0.0009,0],[0,0.01]]) # original guesses
        self.Q_t = np.array([[1000000,0],[0,1000000]]) # increase so AprilTag measurements are highly trusted
													
        # covarience about the AprilTag measurements backpropegated location
        # 2cm for x,y -> 0.0004    
        # 2*pi/20 (0.3) rad for phi -> 0.09
        self.R_t = np.array([[0.0004,0,0],[0,0.0004,0],[0,0,0.09]]) 

        self.x_t = np.array([[0.5],[0.5],[1.5]]) # initilize state to the origin
        self.P_t = np.array([[1,0,0],[0,1,0],[0,0,9]]) # initialize all variences very high
        
    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        
        w = imu_meas[3][0] # get the rotational velocity from the gyro measurement
        # w = -1*w # TODO for robot only
        #print(w)
        #print(v)

        # update the estimated state
        # TODO for robot only        
        #cur_time = rospy.get_time()
        #dt = cur_time - self.last_time

        # TODO dt on robot simulator
        dt = 0.05;

        dx = dt*np.array([[v*math.cos(self.x_t[2])],
            [v*math.sin(self.x_t[2])],
            [w]])

        self.x_t = np.add(self.x_t,dx)

        # update the covarience
        df_dx = np.array([[1,0,dt*-v*math.sin(self.x_t[2])],
            [0,1,v*math.cos(self.x_t[2])],[0,0,1]])

        df_dn = dt*np.array([[math.cos(self.x_t[2]),0],
            [math.sin(self.x_t[2]),0],[0,1]])

        front = np.matmul(np.matmul(df_dx,self.P_t),np.transpose(df_dx))
        back = np.matmul(np.matmul(df_dn,self.Q_t),np.transpose(df_dn))
        self.P_t = np.add(front,back) 

        # store time for next prediction TODO for on bot
        # self.last_time = cur_time


    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """

        # for each AprilTag seen, perform an update step
        z_shape = len(z_t)
        marker_shape = self.markers.shape
        num_markers = marker_shape[0]
        for tag_idx in range(0, z_shape):
            
            # extract the predicted rover position from the AprilTag
            # get the current tags position in the robot frame
            cur_tag_meas = z_t[tag_idx]
            '''
            tag_x_r = cur_tag_meas[0][0]
            tag_y_r = cur_tag_meas[0][1]
            tag_phi_r = cur_tag_meas[0][2]
            tag_id = cur_tag_meas[0][3]
            '''
            tag_x_r = cur_tag_meas[0]
            tag_y_r = cur_tag_meas[1]
            tag_phi_r = cur_tag_meas[2]
            tag_id = cur_tag_meas[3]

            #print('tag data (x,y,phi,id):')
            #print(tag_x_r)
            #print(tag_y_r)
            #print(tag_phi_r) 
            #print(tag_id)

            H_tag_r = np.array([[math.cos(tag_phi_r),-1*math.sin(tag_phi_r),tag_x_r],
                [math.sin(tag_phi_r),math.cos(tag_phi_r),tag_y_r],
                [0,0,1]])

            #print('tag measurement')
            #print(cur_tag_meas)

            # get the current tags logged position in the world frame
            for search_idx in range(0, num_markers):
                if self.markers[search_idx][3] == tag_id:
                    tag_x_w = self.markers[search_idx][0]
                    tag_y_w = self.markers[search_idx][1]
                    tag_phi_w = self.markers[search_idx][2]
                    #print('tag from map')
                    #print(self.markers[search_idx])
                    break                    

            H_tag_w = np.array([[math.cos(tag_phi_w),-1*math.sin(tag_phi_w),tag_x_w],
                [math.sin(tag_phi_w),math.cos(tag_phi_w),tag_y_w],
                [0,0,1]])

            # generate the predicted rover position (z_cur)
            H_bot_w = np.matmul(H_tag_w,np.linalg.inv(H_tag_r))

            z_cur = np.array([[H_bot_w[0][2]],
                [H_bot_w[1][2]],
                [math.atan2(H_bot_w[1][0],H_bot_w[0][0])]])

            #print('predicted pose:')
            #print(z_cur)

            # generate the Kalman gain for this measurement
            K = np.matmul(self.P_t,np.linalg.inv(np.add(self.P_t,self.R_t)))
    
            # update the estimated state        
            self.x_t = np.add(self.x_t,np.matmul(K,(np.subtract(z_cur,self.x_t))))

            # update the covariance
            self.P_t = np.subtract(self.P_t,np.matmul(K,self.P_t))

        
    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """

        #print(imu_meas)
        #print(z_t)

        # if there is v and an imu_meas, run the prediction step
        if v is not None and imu_meas is not None:
            self.prediction(v, imu_meas)

        # if there is a z_t ie AprilTags in view, run the update step
        if z_t is not None:
            self.update(z_t)


