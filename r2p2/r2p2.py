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

import json
import utils as u

from config_manager import ConfigManager
    
def start_simulation(config='../conf/scenario-default.json'):
    """
        Launches a simulation using the given configuration file.
        Inputs:
            - config: path to the configuration file to be used.

    """
    u.load_simulation(config)

def args_list_to_dict(args: list):
    dictionary = {}
    for x in range(int(len(args)/2)):
        key_pos = x*2
        val_pos = x*2+1

        # Removing hiphens on the left
        key = args[key_pos].lstrip('-')
        # Casting the value as json to accept plain parameters, lists, objects, nested objects...
        val = json.loads(args[val_pos])
        dictionary[key] = val
    return dictionary

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(\
        description='Run a simulation using a specified scenario.',
        epilog='Any extra argument passed through the CLI will be used to overwrite scenario parameters. Do not enter spaces in the values or add quotes! Ex: --start [20,10] --goal "[21, 11]"')
    parser.add_argument('--version', action='store_true', \
            help='Displays the current version of the simulator.')
    parser.add_argument('--scenario', metavar='S', nargs='?',\
        help='Path to the configuration JSON in which the scenario is defined.')
    parser.add_argument('--controller', metavar='C', nargs='?',\
        help='Path to the controller that you want to use. This will override the scenario one if there was any.')
    args, unknown = parser.parse_known_args()
    unknown_args = args_list_to_dict(unknown)

    if args.version:
        print('R2P2 v.'+__version__)
        exit()
    
    if args.scenario:
        config_mgr = ConfigManager(scenario_config = args.scenario, controller_config = args.controller, params = unknown_args)
    else:
        config_mgr = ConfigManager(controller_config = args.controller, params = unknown_args)
    start_simulation(config_mgr)
