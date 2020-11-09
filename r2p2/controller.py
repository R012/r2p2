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

class Controller(ABC):
    """
        Base controller class. DO NOT INSTANCE.
        All controllers must inherit from this class.
        Its abstract methods must be implemented. The purpose of registering the corresponding
        robot to it is making it possible to request some lower level data and functionalities.
    """
    def __init__(self, controller_type = "", config = None):
        """
            Controller type stored as a string mainly for logging purposes. Might also be used
            by specific implementations in order to indentify variations in behavior.
        """
        self.type = controller_type
        self.cur_detected_edges_distances = []
        self.cur_detected_edges = []
        self.actual_sensor_angles = []
        self.ang = []
        self.dst = []
        self.config = config

    def control(self, dst):
        """
            Driver function to centralize and standardize the controller. Can be modified by child classes,
            provided that the result value always is a tuple of the form (angular velocity, linear velocity)
        """
        self.update_sensor_angles(self.ang, dst)
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
        return True

    def goal_oriented(self):
        """
            Override and set to true if the controller being implemented operates over a set of goals.
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

    def set_dst(self, dst):
        self.dst = dst

    def set_ang(self, ang):
        self.ang = ang

def upd_sensor_angles(function):
    """Decorator to simplify the implementation of "control" methods on Controller subclasses"""
    def wrapper(self, dst):
        self.update_sensor_angles(self.ang, dst)
        return function(self, dst)
    return wrapper
    return wrapper