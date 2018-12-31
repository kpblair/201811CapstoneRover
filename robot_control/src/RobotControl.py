#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""
# TODO for robot control only
import rospy

import yaml
import numpy as np

import sys

# TODO for student: Comment this section when running on the robot 
#from RobotSim import RobotSim
#import matplotlib.pyplot as plt

# TODO for student: uncomment when changing to the robot
from RosInterface import ROSInterface

# TODO for student: User files, uncomment as completed
from ShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        Inputs: (all loaded from the parameter YAML file)
        world_map - a P by 4 numpy array specifying the location, orientation,
            and identification of all the markers/AprilTags in the world. The
            format of each row is (x,y,theta,id) with x,y giving 2D position,
            theta giving orientation, and id being an integer specifying the
            unique identifier of the tag.
        occupancy_map - an N by M numpy array of boolean values (represented as
            integers of either 0 or 1). This represents the parts of the map
            that have obstacles. It is mapped to metric coordinates via
            x_spacing and y_spacing
        pos_init - a 3 by 1 array specifying the initial position of the robot,
            formatted as usual as (x,y,theta)
        pos_goal - a 3 by 1 array specifying the final position of the robot,
            also formatted as (x,y,theta)
        max_speed - a parameter specifying the maximum forward speed the robot
            can go (i.e. maximum control signal for v)
        max_omega - a parameter specifying the maximum angular speed the robot
            can go (i.e. maximum control signal for omega)
        x_spacing - a parameter specifying the spacing between adjacent columns
            of occupancy_map
        y_spacing - a parameter specifying the spacing between adjacent rows
            of occupancy_map
        t_cam_to_body - numpy transformation between the camera and the robot
            (not used in simulation)
        """

        # TODO for student: Comment this when running on the robot 
        #self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal, 
	#	                            max_speed, max_omega, x_spacing, y_spacing)
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # speed control variables
        self.v = 0.1 # allows persistent cmds through detection misses
        self.omega = -0.1 # allows persistent cmds through detection misses
        self.last_detect_time = rospy.get_time() #TODO on bot only
        self.missed_vision_debounce = 1

        self.start_time = 0

        # generate the path assuming we know our start location, goal, and environment
        self.path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
        self.path_idx = 0
        self.mission_complete = False
        self.carrot_distance = 0.22

        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and esimtaiton
        are done. This function is called at 60Hz
        """

        print(' ')

        # TODO for student: Comment this when running on the robot 
        #meas = self.robot_sim.get_measurements()
        #imu_meas = self.robot_sim.get_imu()

        # TODO for student: Use this when transferring code to robot
        meas = self.ros_interface.get_measurements()
        imu_meas = self.ros_interface.get_imu()

        # meas is the position of the robot with respect to the AprilTags
        # print(meas)

        # now that we have the measurements, update the predicted state
        self.kalman_filter.step_filter(self.v, imu_meas, meas)
        # print(self.kalman_filter.x_t)

        # TODO remove on bot, shows predicted state on simulator
        #self.robot_sim.set_est_state(self.kalman_filter.x_t)

        # pull the next path point from the list
        cur_goal = self.getCarrot()

        # cur_goal = self.path[self.path_idx]
        # TODO test to just go to a goal
        # cur_goal[0] = 0.43
        # cur_goal[1] = 2

        # calculate the control commands need to reach next path point
        #print('')
        #print('current goal:')
        #print(cur_goal)
        #print('current state:')
        #print(self.kalman_filter.x_t)

        control_cmd = self.diff_drive_controller.compute_vel(self.kalman_filter.x_t,cur_goal);
        self.v = control_cmd[0]
        self.omega = control_cmd[1]

        #print('control command:')
       # print(control_cmd)

        if self.mission_complete:
            self.v = 0
            self.omega = 0

        #print(control_cmd)
        if control_cmd[2]:
            if len(self.path) > (self.path_idx + 1):
                self.path_idx = self.path_idx + 1
                print('next goal')
            else:
                self.mission_complete = True
        
        #TODO calibration test on bot only for linear velocity
        '''
        if self.start_time - 0 < 0.0001:
            self.start_time = rospy.get_time()

        if(rospy.get_time() - self.start_time > 4):
            self.v = 0
            self.omega = 0
        else:
            self.v = 0.15
            self.omega = 0
        '''

        #TODO on bot only
        self.ros_interface.command_velocity(self.v,self.omega)
        
        #TODO for simulation
        #self.robot_sim.command_velocity(self.v,self.omega)

        return

    def getCarrot(self):
        '''
        getCarrot - generates an artificial goal location along a path out 
        infront of the robot.
        path - the set of points which make up the waypoints in the path
        position - the current position of the robot
        '''
        path = self.path
        idx = self.path_idx
        pos = self.kalman_filter.x_t

        # if the current line segment ends in the goal point, set that to the goal
        if self.path_idx + 1 == len(self.path):
            return self.path[(len(self.path) - 1)]
        else:   
            # find the point on the current line closest to the robot
            # calculate current line's slope and intercept
            pt1 = self.path[self.path_idx]
            pt2 = self.path[self.path_idx + 1]
            x_diff = pt2[0] - pt1[0]
            y_diff = pt2[1] - pt1[1]
            vert = abs(x_diff) < 0.001

            # using the current line's slope and intercept find the point on that
            # line closest to the robots current point
            # assumes all lines are either veritcal or horizontal
            x_bot = pos[0][0]
            y_bot = pos[1][0]
            if vert:
                x_norm = pt2[0]
                y_norm = y_bot
            else:
                x_norm = x_bot
                y_norm = pt2[1]

            # if the normal point is past the end point of this segment inc path idx
            # assumes all lines are either vertical or horizontal
            inc = False
            if vert:
                if (y_diff > 0 and y_norm > pt2[1]) or (y_diff < 0 and y_norm < pt2[1]):
                    inc = True
            else:
                if (x_diff > 0 and x_norm > pt2[0]) or (x_diff < 0 and x_norm < pt2[0]):
                    inc = True

            if(inc):
                self.path_idx = self.path_idx + 1
                print('increment path index')

            # find a point L distance infront of the normal point on this line
            # assumes all lines are either vertical or horizontal
            if vert:
                x_goal = pt2[0]
                if y_diff > 0:
                    y_goal = y_bot + self.carrot_distance
                else:
                    y_goal = y_bot - self.carrot_distance
            else:
                y_goal = pt2[1]
                if x_diff > 0:
                    x_goal = x_bot + self.carrot_distance
                else:
                    x_goal = x_bot - self.carrot_distance

            goal = np.array([x_goal, y_goal])

            #print(goal)

            return goal

def main(args):
    # Load parameters from yaml
    rospy.init_node('robot_control') # TODO for running on bot
    param_path = rospy.get_param("~param_path") # TODO for running on bot
    # param_path = 'params.yaml' # TODO for running on sim 
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map, occupancy_map, pos_init, pos_goal,
                                max_vel, max_omega, x_spacing, y_spacing,
                                t_cam_to_body)

    # TODO for student: Comment this when running on the robot 
    # Run the simulation
    '''
    while not robotControl.robot_sim.done and plt.get_fignums():
        robotControl.process_measurements()
        robotControl.robot_sim.update_frame()

    plt.ioff()
    plt.show()
    '''

    # TODO for student: Use this to run the interface on the robot
    # Call process_measurements at 60Hz    
    r = rospy.Rate(15) # was 60Hz orginial
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    main(sys.argv)


