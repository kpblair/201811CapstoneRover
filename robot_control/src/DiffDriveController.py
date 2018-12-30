#!/usr/bin/python

import numpy as np
import math as math

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        self.kp=0.8 #m/s / m must be > 0 old 0.5
        self.ka=2 #rad/s / rad must be > kp
        self.kb=0
        #self.MAX_SPEED = max_speed
        self.MAX_SPEED = 0.15 # slower is more contrallable
        self.MAX_OMEGA = max_omega
        self.done_distance = 0.17 #old 0.15
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """

        """
        Unicycle model control law:
        [v;w] = [kp 0 0; 0 ka kb]*[p;a;b]
        v = commanded linear velocity of robot
        w = commanded rotational velcoity of robot
        kp = gain parameter where kp > 0
        ka = gain parameter where ka - kp > 0
        kb = gain parameter where kb < 0
        p = distance from robot to goal
        a = angle between current robot heading and heading to goal
        b = error between current heading to goal and target end heading
        """
        
        #print('state,goal,v,w')
        #print(state)
        #print(goal)

        xr = state[0][0] # m in world frame
        yr = state[1][0] # m in world frame
        thetar = state[2][0] #rads

        xg = goal[0] # m in world frame
        yg = goal[1] # m in world frame

        dy = yg - yr
        dx = xg - xr
        
        # Calculate a
        a = -1*thetar + math.atan2(dy,dx)

        while a > 2*math.pi:
            a = a - math.pi

        while a < -2*math.pi:
            a = a + math.pi

        # Set omega according to control law
        omega = self.ka*a
        if math.fabs(omega) > self.MAX_OMEGA:
            if omega > 0:
                omega = self.MAX_OMEGA
            else:
                omega = -1*self.MAX_OMEGA

        # Calculate P
        p = math.sqrt(dy*dy + dx*dx)

        # Set v 
        v = self.kp*p
        if v > self.MAX_SPEED:
            v = self.MAX_SPEED

        # set the done value
        done = (p <= self.done_distance)

        #print(v)
        #print(omega)

        out_tuple = (v, omega, done)
       
        return out_tuple
