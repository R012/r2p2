#!/usr/bin/env python

""" This module defines what a naive controller is, from a data-structure POV.

This module contains the definition and implementation of the Naive_Controller
class.

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
__date__ = "2019/03/29"
__deprecated__ = False
__email__ =  "mario.cobos@edu.uah.es"
__license__ = "GPLv3"
__maintainer__ = "Mario Cobos Maestre"
__status__ = "Development"
__version__ = "0.0.1"


from controller import Controller, upd_sensor_angles

class Naive_Controller(Controller):
    """
        Class implementing an extremely simple naive controller.
        Not really dependable, meant to serve as a simple example
        only.
    """
    def __init__(self, config):
        """
            Constructor for the Naive_Controller class.
            Outputs:
                - A configured Naive_Controller object.
        """
        super().__init__("NAIVE", config)

    @upd_sensor_angles
    def control(self, dst):
        """
            Driver function to centralize and standardize the controller. Can be modified by child classes,
            provided that the result value always is a tuple of the form (angular velocity, acceleration)
        """
        return 3, 0

