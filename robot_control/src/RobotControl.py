#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 19 2016.
"""
# TODO for robot control only
# import rospy

import yaml
import numpy as np

import sys

# TODO for student: Comment this section when running on the robot 
from RobotSim import RobotSim
import matplotlib.pyplot as plt

# TODO for student: uncomment when changing to the robot
# from RosInterface import ROSInterface

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
        self.robot_sim = RobotSim(world_map, occupancy_map, pos_init, pos_goal,
                                  max_speed, max_omega, x_spacing, y_spacing)
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        # self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # speed control variables
        self.v = 0.1 # allows persistent cmds through detection misses
        self.omega = -0.1 # allows persistent cmds through detection misses
        # self.last_detect_time = rospy.get_time() #TODO on bot only
        self.missed_vision_debounce = 1

        # generate the path assuming we know our start location, goal, and environment
        #self.path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)

        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """
        YOUR CODE HERE
        Main loop of the robot - where all measurements, control, and esimtaiton
        are done. This function is called at 60Hz
        """

        # TODO for student: Comment this when running on the robot 
        meas = self.robot_sim.get_measurements()
        imu_meas = self.robot_sim.get_imu()

        # TODO for student: Use this when transferring code to robot
        # meas = self.ros_interface.get_measurements()
        # imu_meas = self.ros_interface.get_imu()

        # meas is the position of the robot with respect to the AprilTags
        # print(meas)

        # now that we have the measurements, update the predicted state
        self.kalman_filter.step_filter(self.v, imu_meas, meas)

        ''' AprilTag Chasing Code
        # if there is an AprilTag in view
        if meas is not None:
            # if the ID is 0, chase it
            cur_id = meas[0][3]
            if cur_id == 0:
                # found something, log the time
                self.last_detect_time = rospy.get_time()
                # set the goal to the position of the robot with respect to the AprilTag
                goal = np.array([[meas[0][0]], [-1*meas[0][1]]])
                # set the current robots position to the origin
                state = np.array([0,0,0])
                target_velocity = self.diff_drive_controller.compute_vel(state, goal)
                # set the commanded velocity to the target
                self.v = target_velocity[0]
                self.omega = target_velocity[1]
                # if done, stop
                if target_velocity[2]:
                    # print('SOOOO CLOSE: STOP')

                    self.v = 0
                    self.omega = 0
        else: # if we're not seeing anything check if its been awhile then zero out commands
            if (rospy.get_time() - self.last_detect_time) > self.missed_vision_debounce:
                self.v = 0
                self.omega = 0
                # print('TIMEOUT: STOP')

        # print(self.v)
        # print(self.omega)
        self.ros_interface.command_velocity(self.v,self.omega)
        '''

        #print(self.kalman_filter.x_t)
        self.robot_sim.set_est_state(self.kalman_filter.x_t)
        #self.robot_sim.set_est_state(np.array([[0.5],[1],[0]]))
        
        # for Kalman filter just derp around
        #self.ros_interface.command_velocity(0.1,-0.01) # forward with a slow right
        self.robot_sim.command_velocity(self.v,self.omega)

        return
    
def main(args):
    # Load parameters from yaml
    # rospy.init_node('robot_control') # TODO for running on bot
    # param_path = rospy.get_param("~param_path") # TODO for running on bot
    param_path = 'params.yaml' # TODO for running on sim 
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
    while not robotControl.robot_sim.done and plt.get_fignums():
        robotControl.process_measurements()
        robotControl.robot_sim.update_frame()

    plt.ioff()
    plt.show()

    # TODO for student: Use this to run the interface on the robot
    # Call process_measurements at 60Hz
    '''
    r = rospy.Rate(15) # was 60Hz orginial
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)
    '''
if __name__ == "__main__":
    main(sys.argv)


