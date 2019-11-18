#!/usr/bin/env python

""" Main driver module.

Main driver module for the robot simulator. Acts as a façade to hide most of
the underlying complexity of the simulator itself.

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
__version__ = "1.0.0b"

import utils as u
    
def start_simulation(config='../conf/scenario-default.json'):
    """
        Launches a simulation using the given configuration file.
        Inputs:
            - config: path to the configuration file to be used.

    """
    u.load_simulation(config)

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(\
        description='Run a simulation using a specified scenario.')
    parser.add_argument('--version', action='store_true', \
            help='Displays the current version of the simulator')
    parser.add_argument('--scenario', metavar='S', nargs='?',\
        help='path to the configuration JSON in which the scenario is defined.')
    args = parser.parse_args()
    if args.version:
        print('R2P2 v.'+__version__)
        exit()
    if args.scenario:
        start_simulation(args.scenario)
    else:
        start_simulation()
