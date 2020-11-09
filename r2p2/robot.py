#!/usr/bin/env python

""" This module defines what a robot is, from a data-structure POV.

This module contains the definition and implementation of the Robot class. It
is also in charge of defining the physical behavior of both the robot and its
sensors, and how to handle its general functioning. Do not modify it unless
you know what you are doing.

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
__version__ = "0.0.2"

import random
import math
import numpy as np
import matplotlib.colors as colors
from controller import Controller
import utils as u
import time
from threading import Lock

class Robot:
    """
        Class defining the properties of a robot, as well as its physical behavior. Used for simulation purposes.
        Aside from providing abstractions representing odometry, as well as other low level details (such as sensors,
        or movement systems), this class integrates with the Controller class. Both are part of a whole, and only
        kept separate for the sake of code clarity.
    """
    def __init__(self, identifier = -1, x = 0, y = 0, orientation = 0, speed = 0, max_speed = 2, acceleration = 0,\
                 sensors = 8, vision_range=(0, 10000), radius = 1,
                 color=(int(random.random()*255), int(random.random()*255), int(random.random()*255)),
                 controller = Controller(), name = None,
                 lock = Lock()):
        """
            Constructor function for the Robot class. Not only does it asign the corresponding values to all parameters,
            it also performs secondary initialization operations, such as registering the robot to its controller.
            Most parameters are self explanatory.
            sensors can either be an integer (representing how many sensors the robot has, assumed to be spaced out evenly),
            or a list of integers (representing the angles to which the robot's sensors are oriented, expressed in degrees.
        """
        self.lock = lock
        self.identifier = identifier
        self.x = x
        self.y = y
        self.last_pos = (x, y)
        self.orientation = orientation # Must be an angle in degrees
        self.speed = speed
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.angular_velocity = 0
        self.sensors = sensors
        self.vision_range = vision_range
        self.radius = radius
        self.controller = controller
        '''if type(color) is list:
            color = tuple(color)
        elif isinstance(color, str):
            print("Converting color string to color")
            color = tuple(colors.get_named_colors_mapping()[color])
            color = (int(255 * color[0]), int(255 * color[1]), int(255 * color[2]))
        self.color = color'''
        self.color=(int(random.random()*255), int(random.random()*255), int(random.random()*255))
        self.name = name
        if self.name is not None:
            self.log = open("../logs/"+str(self.name)+".log", "w+")
            self.sensors_output = open("../logs/"+str(self.name)+"_sensors.log", "w+")
            self.benchmark_log = open("../logs/"+str(self.name)+"_benchmark.log", "w+")

        else:
            self.log = open("../logs/robot_"+str(self.identifier)+".log", "w+")
            self.sensors_output = open("../logs/robot_"+str(self.identifier)+"_sensors.log", "w+")
            self.benchmark_log = open("../logs/robot_"+str(self.identifier)+"_benchmark.log", "w+")
        self.last_op_started = 0
        self.controller.register_robot(self)
        self.has_noise = True

    def set_position(self, x, y):
        self.lock.acquire()
        self.x = x
        self.y = y
        self.lock.release()
    
    def set_last_position(self, x, y):
        self.lock.acquire()
        self.last_pos = (x, y)
        self.lock.release()

    def set_color(self, color):
        if type(color) is list:
            color = tuple(color)
        elif isinstance(color, str):
            print("Converting color string to color")
            aux = colors.get_named_colors_mapping()[color]
            aux = aux.replace('#', '')
            aux = list(map(''.join, zip(*[iter(aux)]*2)))
            aux = [int(aux[i], 16) for i in range(len(aux))]
            color = tuple(aux)
            color = (int(color[0]), int(color[1]), int(color[2]))
        self.color = color

    def get_lock(self):
        return self.lock

    def insert_battery_details(self, step, battery, charging_rate,
                               movement_cost, reading_cost, picture_cost,
                               generic_cost):
        self.step_x = step
        self.step_y = step
        self.battery = battery
        self.max_battery = battery
        self.charging_rate = charging_rate
        self.battery_cost = {}
        self.battery_cost['move'] = movement_cost
        self.battery_cost['reading'] = reading_cost
        self.battery_cost['picture'] = picture_cost
        self.battery_cost['generic'] = generic_cost

    def __generate_noise(self):
        return np.random.normal(0, 0.5)

    def __write_to_log(self, text):
        self.log.write(text)

    def __benchmark_time(self, funcname):
        self.benchmark_log.write(funcname+","+str((time.time() - self.last_op_started)*1000)+'\n')

    def print_stats(self):
        """
            Auxiliary method that requests the robot to print its stats on stdout.
            DO NOT USE IN THE UPDATE LOOP. It will cause performance issues due to the console being busy and slow to write too.
        """
        self.last_op_started = time.time()
        print("[ROBOT ID: ", self.identifier, "; POSITION: (", self.x, ", ", self.y,"); ORIENTATION: ", self.orientation,"º; SPEED: ", \
              self.speed, "; MAX_SPEED: ", self.max_speed, "; ACCELERATION: ", self.acceleration, "]")
        self.__benchmark_time('print_stats')

    def write_stats_to_log(self):
        """
            Auxiliary method used to dump current robot stats into a log, effectively creating a snapshot of its current state.
            This proccess includes writing the robot's controller's current stats to the log, should they be relevant, as determined by the
            controller itself.
        """
        self.last_op_started = time.time()
        self.__write_to_log("\n[ROBOT ID: "+str(self.identifier)+"; POSITION: ("+str(self.x)+", "+str(self.y)+"); ORIENTATION: "+str(self.orientation)+"º; SPEED: "+\
              str(self.speed)+"; MAX_SPEED: "+str(self.max_speed)+"; ACCELERATION: "+str(self.acceleration)+"]\n")
        self.controller.write_info_to_log(self.log)
        self.__benchmark_time('write_stats_to_log')

    def collide(self, spot):
        """
            Method that asks the robot to register a collision at a specific spot (represented as a (x, y) tuple).
            Kept public for the sake of providing more possibilities to controllers, as a way, for instance, to penalize
            specific behaviors.
        """
        self.last_op_started = time.time()
        self.__write_to_log("[ROBOT ID "+str(self.identifier)+str(" COLLISION AT ("+str(spot[0])+", "+str(spot[1])+")"))
        self.controller.on_collision(spot)
        self.__benchmark_time('collide')

    def __crashing_radius(self, env):
        for i in range(0, 360):
            if env.item(int(self.x + math.cos(math.radians(i)) * self.radius), int(self.y + math.sin(math.radians(i)) * self.radius)) is 0:
                self.__benchmark_time('__crashing_radius')
                return True
        return False

    def __update_position(self, env, delta):
        """
            Auxiliary method used to modify the robot's position. Must only be called as part of Robot#update, hence why it is kept
            private.
            Aside from moving the robot according to its orientation and speed, this method also calculates and handles collisions on
            a physical level.
        """
        self.last_op_started = time.time()
        calculated_x = self.x + math.cos(math.radians(self.orientation)) * self.speed * delta
        calculated_y = self.y + math.sin(math.radians(self.orientation)) * self.speed * delta

        disp_vector = (calculated_x - self.x, calculated_y - self.y)

        angle = math.degrees(math.atan2(disp_vector[1], disp_vector[0]))
        angle %= 360
        
        max_spot = u.search_edge_in_angle_with_limit((self.x, self.y), math.radians(angle), env, np.linalg.norm(disp_vector)+self.radius)

        self.x = calculated_x
        self.y = calculated_y

        if max_spot != (-1, -1):
            max_disp_vector = (max_spot[0] - self.x - self.radius, max_spot[1] - self.y - self.radius)
            if env.item(int(calculated_x), int(calculated_y)) is 0:
                self.collide((self.x, self.y))
                if max_spot[0] - self.x > 0:
                    self.x = max_spot[0] + (self.radius + 1)
                elif max_spot[0] - self.x < 0:
                    self.x = max_spot[0] - (self.radius + 1)
                if max_spot[1] - self.y > 0:
                    self.y = max_spot[1] + (self.radius + 1)
                elif max_spot[1]- self.y < 0:
                    self.y = max_spot[1] - (self.radius + 1)
        
        if self.x + self.radius >= env.shape[0]:
            if self.y + self.radius < env.shape[1] and self.y - self.radius >= 0:
                self.collide((env.shape[0], self.y))
            elif self.y + self.radius >= env.shape[1]:
                self.collide(env.shape)
                self.y = env.shape[1] - 1 - self.radius
            else:
                self.collide((env.shape[0], 0))
                self.y = self.radius
            self.x = env.shape[0] - 1
        elif self.x - self.radius < 0:
            if self.y + self.radius < env.shape[1] and self.y - self.radius >= 0:
                self.collide((0, self.y))
            elif self.y + self.radius >= env.shape[1]:
                self.collide((0, env.shape[1]))
                self.y = env.shape[1] - 1 - self.radius
            else:
                self.collide((0, 0))
                self.y = self.radius
            self.x = 0
        elif self.y + self.radius > env.shape[1]:
            self.collide((self.x, env.shape[1]))
            self.y = env.shape[1] - 1 - self.radius
        elif self.y - self.radius < 0:
            self.collide((self.x, 0))
            self.y = self.radius

        if env.item(int(self.x), int(self.y)) is 50 or self.__crashing_radius(env):
            self.x = self.last_pos[0]
            self.y = self.last_pos[1]
            self.collide((self.x, self.y))
        else:
            self.last_pos = (self.x, self.y)
        self.__benchmark_time('__update_position')

    def __update_angle(self, delta):
        self.last_op_started = time.time()
        self.turn(self.angular_velocity * delta)
        self.__benchmark_time('__update_angle')

    def turn(self, angle = 0):
        """
            Instructs the robot to turn a specific angle. Best left alone unless you know exactly what you are doing.
        """
        self.orientation += angle
        #self.orientation %= 360

    def __updated_speed(self, delta):
        self.last_op_started = time.time()
        self.speed += self.acceleration * delta
        if self.speed > self.max_speed:
            self.speed = self.max_speed
        if self.speed < -self.max_speed:
            self.speed = -self.max_speed
        self.__benchmark_time('__updated_speed')

    def change_acceleration(self, acceleration):
        """
            Instructs the robot to change its acceleration to a specific value.
            It should be noted that this is the main way to control its speed.
        """
        self.last_op_started = time.time()
        self.acceleration = acceleration
        self.__benchmark_time('change_acceleration')

    def check_sensors(self, env):
        """
            Requests the robot to check its environment using its sensors, and then returns the result.
            Inputs:
                - env: Matrix data representing the environment. MUST BE AN NPARRAY.
            Outputs:
                - col: list of detected collision points.
                - dst: list of distances to the detected collision points.
                - ang: list of actual angles explored.
        """
        self.last_op_started = time.time()
        if type(self.sensors) is int:
            col, dst, ang = u.search_in_all_directions_with_step_and_offset_with_limit((round(self.x), round(self.y)), env, self.sensors, self.orientation, self.vision_range[1]+self.radius)
        elif type(self.sensors) is list:
            col, dst, ang =  u.search_in_all_directions_with_angles_and_offset_with_limit((round(self.x), round(self.y)), env, self.sensors, self.orientation, self.vision_range[1]+self.radius)
            
        to_pop = []
        for i in range(len(dst)):
            if (dst[i] < self.vision_range[0] + self.radius and dst[i] > self.vision_range[0]*0.125 + self.radius) or dst[i] > self.vision_range[1] + self.radius:
                dst[i] = self.vision_range[1] + self.radius
                to_pop.append(col[i])
            elif self.has_noise:
                dst[i] += self.__generate_noise()
                col[i] = (self.x + math.cos(math.radians(ang[i]))*dst[i], self.y + math.sin(math.radians(ang[i]))*dst[i])
        for i in to_pop:
            col.remove(i)
        for c in col:
            if c is (-1, -1):
                col.remove(c)
                
        self.__benchmark_time('check_sensors')
        return col, dst, ang

    def __manage_collisions(self, col):
        '''if (self.x, self.y) in col:
            self.collide((self.x, self.y))'''
        self.controller.handle_collision(col)

    def output_sensor_data(self, ang, dst):
        """
            Requests the robot to display its sensor data on stdout.
            DO NOT USE IN THE UPDATE LOOP. It will cause performance issues due to console being busy and slow to write to.
        """
        self.last_op_started = time.time()
        print("[ROBOT ID ", self.identifier, " SENSOR DATA:")
        for i in range(len(ang)):
            print("SENSOR: ", ang[i]-self.orientation, "º - DISTANCE: ", dst[i])
        print("]")
        self.__benchmark_time('output_sensor_data')

    def __output_sensor_data_to_log(self, ang, dst):
        self.last_op_started = time.time()
        text = "[ROBOT ID " + str(self.identifier) + " SENSOR DATA:\n"
        for i in range(len(ang)):
            text += "SENSOR: "+str(ang[i]-self.orientation)+"º - DISTANCE: "+str(dst[i])+"\n"
        text += "]"
        self.__write_to_log(text)
        self.__write_sensors_output(dst)
        self.__benchmark_time('__output_sensor_data_to_log')

    def __write_sensors_output(self, dst):
        text = ""
        for i in range(len(dst)):
            text += str(dst[i])+";" if i < len(dst)-1 else str(dst[i])
        text += "\n"
        self.sensors_output.write(text)

    def brake(self):
        """
            Instructs the robot to brake. This braking is proportional to its current acceleration and speed,
            and impacts only its acceleration.
        """
        neg = self.speed < 0
        new_speed = self.speed -self.acceleration - self.speed/3
        if (neg and new_speed > 0) or (not neg and new_speed < 0):
            self.stop()
            return - self.acceleration/5
        return - self.acceleration - self.speed/3

    def stop(self):
        """
            Instructs the robot to stop dead on its tracks. Essentially, the equivalent of making it
            forcibly stop its wheels.
        """
        self.speed = 0

    def control(self, env, delta):
        """
            Robot's main control loop. It always handles sensor checking before actually requesting instructions to its
            controller.
        """
        col, dst, ang = self.check_sensors(env)
        self.__output_sensor_data_to_log(ang, dst)
        self.__manage_collisions(col)
        #self.angular_velocity, acc = self.controller.control(ang, dst)
        #self.change_acceleration(acc)
        self.controller.set_dst(dst)
        self.controller.set_ang(ang)
        self.speed, self.angular_velocity = self.controller.control(dst)
        if abs(self.speed) > self.max_speed:
            self.speed = self.speed/abs(self.speed) * self.max_speed

    def update(self, env, delta, write_stats = False):
        """
            Controller driver.
            - env: environment data, represented as a matrix
            - delta: time ellapsed since the last update
        """
        self.lock.acquire()
        self.control(env, delta)
        self.__update_angle(delta)
        #self.__updated_speed(delta)
        self.__update_position(env, delta)
        if write_stats:
            self.write_stats_to_log()
        self.lock.release()
