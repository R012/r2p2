#!/usr/bin/env python

""" Module defining what a controller is from a data structure POV.

Aside from defining what the Controller class requires as an abstract class,
this module provides two example controllers, as well as an abstract factory
in order to ease instancing controllers using configuration files. Usage of
this abstract factory is mandatory. In order to use it, just create a factory
method that takes a dictionary as input and returns a subclass of Controller
as output, then register the result using register_controller_factory.
Modification of this module is heavily discouraged.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

__author__ = "Mario Cobos Maestre"
__authors__ = ["Mario Cobos Maestre"]
__contact__ = "mario.cobos@edu.uah.es"
__copyright__ = "Copyright 2019, UAH"
__credits__ = ["Mario Cobos Maestre"]
__date__ = "2019/03/18"
__deprecated__ = False
__email__ =  "mario.cobos@edu.uah.es"
__license__ = "GPLv3"
__maintainer__ = "Mario Cobos Maestre"
__status__ = "Development"
__version__ = "0.0.1"

from abc import ABC
import math
import numpy as np
import utils as u

import path_planning as pp

controller_factory = {}

def register_controller_factory(controller_type, factory):
    """
        Call this function to register your controller factory. It must be a previously defined function.
        Inputs:
            - controller_type: string identifying the type of controller this factory creates. It must be the same string used
            to define the type in configuration files.
            - factory: factory function pointer. The new function must expect a dictionary as input, and return a Controller
            object.
    """
    global controller_factory
    controller_factory[controller_type] = factory

class Controller(ABC):
    """
        Base controller class. DO NOT INSTANCE.
        All controllers must inherit from this class.
        Its abstract methods must be implemented. The purpose of registering the corresponding
        robot to it is making it possible to request some lower level data and functionalities.
    """
    def __init__(self, controller_type = ""):
        """
            Controller type stored as a string mainly for logging purposes. Might also be used
            by specific implementations in order to indentify variations in behavior.
        """
        self.type = controller_type

    def control(self, ang, dst):
        """
            Driver function to centralize and standardize the controller. Can be modified by child classes,
            provided that the result value always is a tuple of the form (angular velocity, linear velocity)
        """
        return 0, 0

    def register_robot(self, r):
        """
            Registers the robot with the controller. Can be used to issue specific instructions directly, or
            to read some odometry information from the physical hardware.
        """
        self.robot = r

    def write_info_to_log(self, log_file):
        """
            Writes info relevant to the controller at hand to the specified log file.
            Must be implemented by those controllers that might need to output relevant information.
        """
        pass

    def on_collision(self, pos):
        """
            Callback for collision management. Doesn't need to be overriden unless special measures are to be
            taken when the robot actually collides.
        """
        pass
    
    def handle_collision(self, col):
        """
            Placeholder function used to represent any collision handled policies implemented by a given controller.
            Necessary in order to avoid weird crashes in the simulation.
        """
        pass

    def has_edge_list(self):
        """
            Override and set to true if the controller being implemented holds a list of detected edges.
        """
        return False

    def has_cur_detected_edge_list(self):
        """
            Override and set to true if the controller being implemented holds a list of currently detected edges.
        """
        return False

    def goal_oriented(self):
        """
            Override and set to true if the controller being implemented operates over a set of goals.
        """
        return False

class Telecom_Controller(Controller):
    """
        Class representing a teleoperated controller. Simply detects input from keyboard and uses it to control
        the robot.
    """
    def __init__(self):
        """
            Good old controller. Doesn't need any input.
        """
        super(Telecom_Controller, self).__init__("TELECOM")
        self.detected_edges = []
        self.cur_detected_edges = []
        self.actual_sensor_angles = []
        self.cur_detected_edges_distances = []

    def control(self, ang, dst):
        """
            Driver function to centralize and standardize the controller.
        """
        self.update_sensor_angles(ang, dst)
        return self.choose_angle(ang, dst), self.choose_speed(ang, dst)

    def register_robot(self, r):
        """
            Registers the robot with the controller. Can be used to issue specific instructions directly, or
            to read some odometry information from the physical hardware.
        """
        self.robot = r

    def write_info_to_log(self, log_file):
        """
            Standard function. Given the kind of controller, it has nothing to do.
        """
        pass

    def choose_angle(self, ang, dst):
        """
            Sets the angular velocity to 25 degrees per second in the direction of the pressed arrow.
            If there are no side arrows pressed, it returns 0.
        """
        if 'left' in u.pressed:
            return -25
        elif 'right' in u.pressed:
            return 25
        return 0
    
    def choose_speed(self, ang, dst):
        """
            Alters the robot's acceleration based on the directional arrows pressed.
            If forward is pressed, it increases in 3 pixels per second squared.
            If backward is pressed, it decresses in 3 pixels per second squared.
            If none of them are pressed, the robot's acceleration is not modified.
        """
        if 'up' in u.pressed:
            return self.robot.speed + 3
        elif 'down' in u.pressed:
            return self.robot.speed - 3
        return 0

    def handle_collision(self, col):
        """
            Registers the currently detected edges so they can be represented by the simulator.
        """
        self.cur_detected_edges = col
        """for e in col:
            if e not in self.detected_edges:
                self.detected_edges.append(e)"""

    def has_edge_list(self):
        """
            Always returns true, given that the controller keeps track of an edge list.
        """
        return True

    def has_cur_detected_edge_list(self):
        """
            Always returns true, given that the controller keeps track of the currently detected edges.
        """
        return True

    def goal_oriented(self):
        """
            Always returns false, given that the controller doesn't have a list of goals to accomplish.
        """
        return False

    def update_sensor_angles(self, angles, distances):
        """
            Updates the list representing the actual orientation of the sensors so that the simulator may
            provide a proper representation. It also adjusts distances if needed.
        """
        self.cur_detected_edges_distances = distances
        self.actual_sensor_angles = angles
        for i in range(0, len(self.cur_detected_edges_distances)):
            if self.cur_detected_edges_distances[i] is math.inf:
                self.cur_detected_edges_distances[i] = 10000

def create_telecom_controller(f):
    """
        Factory for a Telecom controller.
        Inputs:
            - f: a dictionary representing all configuration parameters. Can be empty, and it will not cause errors.
            Kept in order to standardize factory input.
        Outputs:
            - A fully configured Telecom_Controller object.
    """
    return Telecom_Controller()

register_controller_factory("TELECOM", create_telecom_controller)


class Sequential_PID_Controller(Controller):
    def __init__(self, goal=[(0,0)], ap=0, ai=0, ad=0, lp=0, li=0, ld=0):
        """
            Constructor for the Sequential_PID_Controller class.
            Initializes its goal list and proportionality constants.
                - goal: list of target spots that must be visited by the robot.
                - ap, ai, ad: proportionality constants for angular velocity.
                - lp, li, ld: proportionality constants for linear acceleration.
        """
        super(Sequential_PID_Controller, self).__init__("SEQ_PID")
        self.goal = goal
        self.ap = ap
        self.ai = ai
        self.ad = ad
        self.lp = lp
        self.li = li
        self.ld = ld
        self.accumulated_angle_error = 0
        self.accumulated_distance_error = 0
        self.last_angle_error = 0
        self.last_distance_error = 0
        self.max_acceleration = 5
        self.detected_edges = []
        self.cur_detected_edges = []
        self.actual_sensor_angles = []
        self.cur_detected_edges_distances = []
        self.target_angle = 0
        self.state = 0

    def control(self, ang, dst):
        """
            Driver function to centralize and standardize the controller.
            A tad more complex than in other examples, as it implements the
            controller's obstacle avoidance policy in the form of a very simple
            state machine that needs to be managed.
        """
        self.update_sensor_angles(ang, dst)
        self.manage_state(dst)
        if self.state is 0:
            return self.control_advance(ang, dst)
        elif self.state is 1:
            return self.control_avoid(ang, dst)

    def control_advance(self, ang, dst):
        """
            Controller for a regular functioning state. Just keep moving towards the
            next goal.
        """
        if not self.is_done():
            angle = self.choose_angle(ang, dst)
            speed = self.robot.speed + self.choose_acceleration(ang, dst)
            if self.is_at_goal():
                self.switch_to_next_goal()
            return angle, speed
        else:
            return 0, 0

    def control_avoid(self, ang, dst):
        """
            Controller for a state in which avoiding an obstacle has become a necessity.
            Tries to turn a bit  and use that turning in order to avoid the obstacle in question.
        """
        self.calculate_target_angle()
        if dst[1] >= 7 and dst[2] >= 7:
            a = self.target_angle + 90
        elif dst[-1] >= 7 and dst[-2] >= 7:
            a = self.target_angle - 90
        else:
            a = self.target_angle + 120
        self.target_angle = a+self.robot.orientation
        #self.target_angle %= 360
        angle = self.calculate_angle_variation()
        speed = self.robot.speed + self.choose_acceleration(ang, dst)
        return angle, speed

    def write_info_to_log(self, log_file):
        """
            Dumps information about the controller on the selected log file. Said information encompasses:
            - List of goals left to reach.
            - Current state.
            - Current target angle.
            The main point of this information being debugging.
        """
        log_file.write("[GOAL: "+str(self.goal))
        if self.state is 0:
            log_file.write("; STATE: ADVANCING")
        else:
            log_file.write("; STATE: AVOIDING")
        log_file.write("; TARGET_ANGLE: "+str(self.target_angle)+"º]\n")

    def choose_angle(self, angles, distances):
        """
            Given the needs of a PID controller, this function is a tad more complex than other implementations.
            First, it needs to update the robot's target angle, in order to correct deviation.
            Then, it needs to apply the actual PID controller in order to calculate the target angular velocity.
        """
        self.calculate_target_angle()
        return self.calculate_angle_variation()

    def choose_acceleration(self, angles, distances):
        """
            As it happens with the angular velocity, calculating the desired acceleration at any given point in time
            is harder with a PID controller.
            First, the controller calculates a variation in acceleration, then accounts for the robot's current acceleration.
            Finally, it ensures that the absolute value of the new acceleration won't exceed a certain threshold.
        """
        acc = self.robot.acceleration + self.calculate_acceleration_variation()
        if acc > self.max_acceleration:
            return self.max_acceleration
        elif -acc < self.max_acceleration:
            return -self.max_acceleration
        else:
            return acc
    
    def handle_collision(self, col):
        """
            Just updates the list of edges currently being detected, and the list of all detected edges.
        """
        self.cur_detected_edges = col
        for e in col:
            if e not in self.detected_edges:
                self.detected_edges.append(e)

    def has_edge_list(self):
        """
            Always returns true, given that the controller keeps track of an edge list.
        """
        return True

    def has_cur_detected_edge_list(self):
        """
            Always returns true, given that the controller keeps track of the currently detected edges.
        """
        return True

    def goal_oriented(self):
        """
            Always returns true, as the controller does keep a list of goals to accomplish.
        """
        return True

    def update_sensor_angles(self, angles, distances):
        """
            Updates the list representing the actual orientation of the sensors so that the simulator may
            provide a proper representation. It also adjusts distances if needed.
        """
        self.cur_detected_edges_distances = distances
        self.actual_sensor_angles = angles
        for i in range(0, len(self.cur_detected_edges_distances)):
            if self.cur_detected_edges_distances[i] is math.inf:
                self.cur_detected_edges_distances[i] = 10000

    def switch_to_next_goal(self):
        """
            Actually removes the currently achieved goal from the list of objectives to accomplish.
        """
        self.goal.pop(0)

    def is_at_goal(self):
        """
            Determines if the actual robot is touching the goal. If so, the goal is considered to have been reached.
        """
        return np.linalg.norm((self.goal[0][0]-self.robot.x, self.goal[0][1]-self.robot.y)) <= self.robot.radius * 1.85

    def is_done(self):
        """
            Returns True only when all goals have been accomplished.
        """
        return not self.goal

    def manage_state(self, dst):
        """
            Simply checks whether the robot can move forward normally or needs to avoid an obstacle,
            and update state accordingly.
        """
        if dst[-1] < 7 or dst[0] < 7 or dst[1] < 7 or dst[2] < 7 or dst[-2] < 7:
            self.state = 1
            return
        self.state = 0

    def calculate_acceleration_variation(self):
        """
            Applies PID control over the distance from the current spot to the current goal in order
            to determine how acceleration should vary. It also includes a policy to ease turning around.
        """
        if abs(self.target_angle - self.robot.orientation) > 45 or self.is_done():
            self.accumulated_distance_error += self.robot.brake()
            return self.robot.brake()
        e = np.linalg.norm((self.goal[0][0] - self.robot.x, self.goal[0][1] - self.robot.y))
        if self.robot.acceleration < self.max_acceleration:
            self.accumulated_distance_error += e
        de = e - self.last_distance_error
        self.last_distance_error = e
        return self.lp * e + self.li * self.accumulated_distance_error + self.ld * de

    def calculate_angle_variation(self):
        """
            Applies PID control over the difference between the desired angle and the current orientation
            of the robot in order to determine angular velocity.
        """
        e = self.target_angle - self.robot.orientation
        self.accumulated_angle_error += e
        de = e - self.last_angle_error
        self.last_angle_error = e
        return self.ap * e + self.ai * self.accumulated_angle_error + self.ad * de

    def calculate_target_angle(self):
        """
            Simply uses vector math to determine what angle the robot should be oriented in next.
            Python's math library handles most of the heavylifting.
        """
        self.target_angle = math.degrees(math.atan2((self.goal[0][1] - self.robot.y), (self.goal[0][0] - self.robot.x)))
        #self.target_angle %= 360

def create_seq_pid_controller(f):
    """
        Factory for a sequential PID controller.
        Inputs:
            - f: dictionary containing all necessary configuration variables, namely a list of goals, and 6 proportionality
            constants (ap, ai, ad, lp, li, ld) which will be used to define the PID controller proper (a stands for angular,
            l for linear).
        Outputs:
            - A fully configured and ready to use Sequential_PID_Controller object.
    """
    return Sequential_PID_Controller(goal=f['goal'], ap=f['ap'],\
                                          ai=f['ai'], ad=f['ad'],\
                                          lp=f['lp'], li=f['li'],\
                                          ld=f['ld'])

register_controller_factory("SEQ_PID", create_seq_pid_controller)

def create_path_planning_controller(f):
    """
        Factory for a PID controller which uses path planning in order to determine what waypoints must be visited.
        Inputs:
            - f: dictionary containing all necessary configuration variables, namely a list of goals, and 6 proportionality
            constants (ap, ai, ad, lp, li, ld) which will be used to define the PID controller proper (a stands for angular,
            l for linear), as well as the algorithm to be used, the start and goal points, the heuristic to be used, and
            configuration variables linked to the specific algorithm. These configuration variables must be a list of waypoints
            if the variant of the algorithm uses a navigation mesh, or a number representing the number of divisions to perform
            on each axis of the map if it's grid-based.
        Outputs:
            - A fully configured and ready to use Sequential_PID_Controller object.
    """
    if f['start'] == f['goal']:
        raise ValueError('Start and goal are the same spot.')
    u.switch_show_robot(1)
    if 'mesh' in f['algorithm']:
        return Sequential_PID_Controller(goal=pp.run_path_planning_mesh(f['waypoints'], f['algorithm'], f['start'], f['goal'], f['heuristic']), ap=f['ap'],\
                                          ai=f['ai'], ad=f['ad'],\
                                          lp=f['lp'], li=f['li'],\
                                          ld=f['ld'])
    else:
        return Sequential_PID_Controller(goal=pp.run_path_planning(f['grid_size'], f['algorithm'], f['start'], f['goal'], f['heuristic']), ap=f['ap'],\
                                          ai=f['ai'], ad=f['ad'],\
                                          lp=f['lp'], li=f['li'],\
                                          ld=f['ld'])

register_controller_factory("PATH_PLANNING", create_path_planning_controller)

import naive_controller
import pddl_executor
import neurocontroller
