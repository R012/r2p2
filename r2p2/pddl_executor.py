#!/usr/bin/env python

""" This module defines what a pddl executor is, from a data-structure POV.

This module contains the definition and implementation of the PDDL_Executor
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


import time
import re
import controller as c
import utils as u
import math

class PDDL_Executor(c.Controller):
    """
        Class defining what a PDDL_Executor is.
        This class takes in a planning configuration, including the data used to generate said planning,
        and executes the result in order to display it graphically.
        Battery issues can be avoided simply y setting costs to 0 and battery to 1, making it unnecessary
        to recharge.
    """
    def __init__(self, filepath):
        """
            Constructor for the PDDL_Executor.
            Aside from a filepath to the planning to be executed, it takes in
            several parameters that help conceptualize the robot to be controlled.
            Inputs:
                - step: amount of pixels the robot moves in a single step.
                - battery: max battery, as well as initial battery level.
                - charging_rate: amount of battery recharged per time step recharging battery.
                - movement_cost: amount of battery that must be spent in order to move.
                - reading_cost: amount of battery that must be spent in order to perform a CO2
                reading.
                - picture_cost: amount of battery that must be spent in order to take a picture.
                - filepath: path to the file contaning the planning that will be executed.
        """
        super(PDDL_Executor, self).__init__("PDDL")
        self.map_size = u.npdata.shape
        self.tasks = self.__generate_task_list(filepath)
        self.log = open('../logs/planning_execution.log', 'w')
        self.actions = {}
        self.timer = 0.75
        self.__generate_actions()

    def control(self, ang, dst):
        """
            Executes the next task in queue. Always returns 0, 0 to the robot,
            instead handling the entirety of the execution of the task on its
            own.
        """
        if self.timer >= 0.75:
            self.__parse_task()
        self.timer -= u.delta
        if self.timer <= 0:
            self.timer = 0.75
        return 0, 0

    def move(self, spot):
        """
            Moves the robot to a specific spot. The actual coordinates are calculated
            as spot×step.
        """
        if self.__can_execute('move'):
            destination = (spot[0]*self.robot.step_x, spot[1] * self.robot.step_y)
            print(destination)
            self.robot.orientation = math.degrees(math.atan2(destination[1] - self.robot.y,
                                                             destination[0] - self.robot.x))
            self.__move(destination)
            self.__deplete_battery('move')

    def move_north(self):
        """
            Moves the robot north, if there is enough battery for it.
        """
        if self.__can_execute('move'):
            destination = (self.robot.x, self.robot.y - self.robot.step_y)
            self.robot.orientation = 270
            self.__move(destination)
            self.__deplete_battery('move')

    def move_south(self):
        """
            Moves the robot south, if there is enough battery for it.
        """
        if self.__can_execute('move'):
            destination = (self.robot.x, self.robot.y + self.robot.step_y)
            self.robot.orientation = 90
            self.__move(destination)
            self.__deplete_battery('move')

    def move_east(self):
        """
            Moves the robot east, if there is enough battery for it.
        """
        if self.__can_execute('move'):
            destination = (self.robot.x + self.robot.step_x, self.robot.y)
            self.robot.orientation = 0
            self.__move(destination)
            self.__deplete_battery('move')

    def move_west(self):
        """
            Moves the robot west, if there is enough battery for it.
        """
        if self.__can_execute('move'):
            destination = (self.robot.x - self.robot.step_x, self.robot.y)
            self.robot.orientation = 180
            self.__move(destination)
            self.__deplete_battery('move')

    def get_co2_reading(self):
        """
            Gathers a reading of CO2 in the current position of the robot,
            if there is enough battery for it.
        """
        if self.__can_execute('reading'):
            self.__place_tag('get_co2_reading')
            r = u.get_reading(self.robot.x, self.robot.y)
            self.log.write("\t>> CO2 level: "+str(r)+"\n")
            print("\t>> CO2 level: "+str(r)+"\n")
            self.__deplete_battery('reading')
        

    def take_picture(self):
        """
            Takes a picture, if there is enough battery for it.
        """
        if self.__can_execute('picture'):
            pic_path = '../logs/'+str(int(time.time()))+'.png'
            self.__place_tag('take_picture')
            u.plt.savefig(pic_path)
            self.log.write("\t>> Picture saved to "+pic_path+"\n")
            print("\t>> Picture saved to "+pic_path+"\n")
            self.__deplete_battery('picture')

    def recharge_batteries(self):
        """Recharges the robot's batteries."""
        self.__place_tag('recharge_batteries')
        self.robot.battery += self.robot.charging_rate
        if self.robot.battery > self.robot.max_battery:
            self.robot.battery = self.robot.max_battery
        self.log.write("\t>> Battery level: "+str(self.robot.battery)+\
                       "/"+str(self.robot.max_battery)+"\n")
        print("\t>> Battery level: "+str(self.robot.battery)+\
                       "/"+str(self.robot.max_battery)+"\n")

    def generic_action(self, action_name):
        if self.__can_execute('generic'):
            self.__place_tag(action_name)
            self.log.write(action_name+"\n")
            print(action_name+"\n")
            self.__deplete_battery('generic')

    def __parse_task(self):
        if self.tasks:
            task = self.tasks.pop(0)
            if type(task) is tuple:
                self.log.write(task[0].lower()+"\n")
                self.actions[task[0]](self, task[1])
            elif task in self.actions:
                self.log.write(task.lower()+"\n")
                self.actions[task.lower()](self)
            else:
                self.log.write(task.lower()+"\n")
                self.actions['generic'](self, task)
        elif not self.log.closed:
            self.log.close()

    def __generate_task_list(self, filepath):
        tasks = []
        for line in open(filepath):
            print(line)
            if re.match(r'^\d(\.|\d)*: \(\w(\w|_)*(\ *(\w(\w|_)*)?)*\)', line):
                print("It's a match. Creating action.")
                split_line = line.split()
                tasks.append(split_line[1].replace('(', '').replace(')', ''))
                if tasks[-1].lower() == 'move':
                    coords = split_line[4].replace('p', '').replace(')', '')
                    tasks[-1] = (tasks[-1], (int(coords[:2]), int(coords[2:])))
        print("TASKS:")
        print(tasks)
        return tasks

    def __can_execute(self, action):
        return self.robot.battery >= self.robot.battery_cost[action]

    def __deplete_battery(self, action):
        self.robot.battery -= self.robot.battery_cost[action]
        if self.robot.battery < 0:
            self.robot.battery = 0
        self.log.write("\t>> Battery level: "+str(self.robot.battery)+\
                       "/"+str(self.robot.max_battery)+"\n")
        print("\t>> Battery level: "+str(self.robot.battery)+\
                       "/"+str(self.robot.max_battery)+"\n")

    def __move(self, spot):
        max_spot = self.__can_move(spot)
        if max_spot is None:
            max_spot = spot
        x = self.robot.radius if max_spot[0] - self.robot.radius < 0 else max_spot[0]
        x = self.map_size[0] - 1 - self.robot.radius if max_spot[0] + self.robot.radius > self.map_size[0] - 1 else x
        y = self.robot.radius if max_spot[1] - self.robot.radius < 0 else max_spot[1]
        y = self.map_size[1] - 1 - self.robot.radius if max_spot[1] + self.robot.radius > self.map_size[1] - 1 else y
        self.robot.x = x
        self.robot.y = y
        self.log.write("\t>> Moved to "+str((x, y))+"\n")
        print("\t>> Moved to "+str((x, y))+"\n")
        

    def __can_move(self, spot):
        return u.los_raycasting((self.robot.x, self.robot.y), spot, u.npdata)

    def __generate_actions(self):
        self.actions['move_north'] = PDDL_Executor.move_north
        self.actions['move_south'] = PDDL_Executor.move_south
        self.actions['move_west'] = PDDL_Executor.move_west
        self.actions['move_east'] = PDDL_Executor.move_east
        self.actions['get_co2_reading'] = PDDL_Executor.get_co2_reading
        self.actions['take_picture'] = PDDL_Executor.take_picture
        self.actions['recharge_batteries'] = PDDL_Executor.recharge_batteries
        self.actions['move'] = PDDL_Executor.move
        self.actions['generic'] = PDDL_Executor.generic_action

    def __place_tag(self, tag):
        if u.ax:
            u.ax.add_patch(u.plt.Circle((self.robot.x,
                                         self.robot.y), radius=self.robot.radius/2))
            u.ax.text(self.robot.x + self.robot.radius/2,
                      self.robot.y-self.robot.radius/2, tag)

def create_pddl_executor(f):
    """
        Factory to create PDDL_Executor objects.
        Inputs:
            - f: dictionary containing the following:
                - the parameters required to instance a PDDL_Executor. Refer to `PDDL_Executor.__init__` for more information.
                - co2_center: numeric 1D array of length two, defining the x, y center of the CO2 source for this scenario.
                - co2_radius: numeric value identifying the radius in which the CO2 extends.
        Outputs:
            - a fully configured PDDL_Executor object.
    """
    u.showFPS = False
    return PDDL_Executor(f['plan_path'])

c.register_controller_factory('PDDL', create_pddl_executor)
