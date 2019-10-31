#!/usr/bin/env python

""" Utility module. All auxiliary functions go here.

This module provides internal functionalities for everything else, aside from
handling the internal details of running the simulation itself. There is a
plethora of varied functions, so it's best left alone unless strictly necessary.
Do not modify this module unless you know exactly what you are doing.

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

from PIL import Image, ImageDraw, ImageColor
import numpy as np
import pygame
from scipy import ndimage as filters
from scipy.misc import imshow
from scipy.stats import norm
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import matplotlib as mpl
import time
import math
import json
import copy
from robot import Robot
from threading import Thread
from controller import Sequential_PID_Controller, Telecom_Controller
import controller

start_time = time.time()
last_call = time.time()
frames = 0
delta = 0
pressed = []
artists = []
co2_center = (0, 0)
scale = 1
xticks = []
xlabels = []
yticks = []
ylabels = []
ax = None
frozen_dist = None
fig = None
npdata = None
gui = True
show_robot = True
button = None
showFPS = false
run = True
screen = None
clock = None


def switch_show_robot(dummy):
    """
        Helper function that controls whether the robot should be displayed and updated.
        Mainly created as a callback for buttons to use, can be called before the
        simulation starts in order to get the corresponding button to show up.
        It requires a single positional argument due to Matplotlib constraints.
    """
    global show_robot
    show_robot = not show_robot

def press(event):
    """Helper function that handles key presses. Must be registered to a plot."""
    global pressed
    pressed.append(event.key)
    

def calculate_delta():
    """
    Helper function that calculates the delta value for any given update call.
    Necessary in order for the update to be performed somewhat smoothly.
    """
    '''global last_call
    new_call = time.time()
    delta = new_call - last_call
    last_call = new_call
    return delta'''
    global clock, delta
    delta = clock.tick(30)
    return delta

def generate_dist(size = 1):
    global frozen_dist, scale
    scale = 1/size
    norm.stats(scale=scale)
    frozen_dist = norm()

def get_reading(x, y):
    global frozen_dist
    if frozen_dist is None:
        generate_dist(1000)
    distance = np.linalg.norm((co2_center[0] - x, co2_center[1] - y))
    return frozen_dist.pdf(distance*scale)

def create_controller(json_file = '../conf/controller.json'):
    """
        Driver function to abstract the process of instancing a Controller object
        using factories.
        Inputs:
            - json_file: path to the JSON configuration file defining the controller
            in question.
        Outputs:
            - a fully configured Controller object.
    """
    global npdata
    with open(json_file, 'r') as fp:
        f = json.load(fp)
        c = controller.controller_factory[f['controller_type']](f)
        return c

def create_robot(json_file = '../conf/robot.json', controller = Telecom_Controller()):
    """
        Uses a json file to generate a fully configured Robot object.
        Inputs:
            - json_file: path to the JSON configuration file for the robot in question.
            - controller: pre-configured Controller object.
        Outputs:
            - fully configured Robot object.
    """
    with open(json_file, 'r') as fp:
        f = json.load(fp)
        if 'name' in f:
            r = Robot(identifier = f['id'], x = f['x'], y = f['y'],\
                      orientation = f['orientation'],\
                      vision_range = (f['sonar_range'][0], f['sonar_range'][1]),\
                      sensors = f['sonars'],\
                      radius = f['radius'], max_speed = f['max_speed'], controller=controller,
                      name=f['name'])
        else:
            r = Robot(identifier = f['id'], x = f['x'], y = f['y'],\
                      orientation = f['orientation'],\
                      vision_range = (f['sonar_range'][0], f['sonar_range'][1]),\
                      sensors = f['sonars'],\
                      radius = f['radius'], max_speed = f['max_speed'], controller=controller)
        if 'battery' in f:
            r.insert_battery_details(f['step'], f['battery'], f['charging_rate'],
                                     f['movement_cost'], f['reading_cost'],
                                     f['picture_cost'], f['generic_cost'])
        if 'color' in f:
            r.set_color(f['color'])
        return r

def load_simulation(json_file='../conf/config.json'):
    """
        Loads a simulation using a configuration file. For the time being, it limits itself to loading the corresponding map and robot.
        Inputs:
            - json_file: path to the configuration file describing the simulation to be loaded. This configuration file must be a JSON
            containing the following:
                * stage: string defining the path to the image file that represents the stage to be loaded.
                * robot: string defining the path to the configuration file of the robot that will be used.
    """
    global gui, npdata, co2_center
    with open(json_file, 'r') as fp:
        f = json.load(fp)
        npdata = load_image(f['stage'])
        if type(f['controller']) is list:
            c = []
            for path in f['controller']:
                c.append(create_controller(path))
        else:
            c = create_controller(f['controller'])
        gui = f['gui']
        if 'co2_center' in f:
            co2_center = f['co2_center']
        if 'co2_radius' in f:
            generate_dist(f['co2_radius'])
        if type(f['robot']) is list:
            r = []
            i = 0
            for path in f['robot']:
                if type(c) is list:
                    r.append(create_robot(path, c[i]))
                    i += 1
                    if i >= len(c):
                        i = 0
                else:
                    r.append(create_robot(path, copy.deepcopy(c)))
        else:
            r = create_robot(f['robot'], c)
        display_image(r)

def update_loop(robots, npdata):
    global delta, pressed, run
    while run:
        init_time = time.time()
        if gui:
            delta = calculate_delta()
        else:
            delta = 0.1
        for r in robots:
            r.get_lock().acquire()
            r.update(npdata, delta)
            r.write_stats_to_log()
            r.write_stats()
            r.get_lock().release()
        pressed.clear()
        time.sleep(1/80)

def update(robots, npdata):
    delta = calculate_delta()/1000
    for r in robots:
            r.get_lock().acquire()
            r.update(npdata, delta)
            #r.print_stats()
            r.write_stats_to_log()
            r.get_lock().release()

def animate(robots):
    """
        Update function. Updates internal world data, then prints it to a plot.
        Must be registered to said plot.
    """
    global start_time, frames, delta, show_robot, screen, clock
    if show_robot:
        for r in robots:
            r.get_lock().acquire()
            if r.controller.has_cur_detected_edge_list():
                '''for e in r.controller.detected_edges:
                    pygame.draw.circle(screen, r.color, (int(e[0]), int(e[1])), 1)'''
                for a in r.controller.actual_sensor_angles:
                    dstX = r.x + np.cos(np.radians(a)) * r.controller.cur_detected_edges_distances[r.controller.actual_sensor_angles.index(a)]
                    dstY = r.y + np.sin(np.radians(a)) * r.controller.cur_detected_edges_distances[r.controller.actual_sensor_angles.index(a)]
                    pygame.draw.line(screen, (255, 0, 255), (int(r.x), int(r.y)), (int(dstX), int(dstY)), 1)
                for e in r.controller.cur_detected_edges:
                    pygame.draw.circle(screen, (0, 255, 0), (int(e[0]), int(e[1])), int((100/np.linalg.norm((e[0]-r.x, e[1]-r.y)))/90))
                if type(r.color) is str:
                    r.color = list(mpl.colors.to_rgb(mpl.colors.get_named_colors_mapping()[r.color]))
                    r.color[0] *= 255
                    r.color[1] *= 255
                    r.color[2] *= 255
                    r.color = tuple(r.color)
            pygame.draw.circle(screen, r.color, (int(r.x), int(r.y)), int(r.radius))
            dstX = r.x + np.cos(np.radians(r.orientation)) * 2 * r.radius
            dstY = r.y + np.sin(np.radians(r.orientation)) * 2 * r.radius
            pygame.draw.line(screen, r.color, (int(r.x), int(r.y)), (int(dstX), int(dstY)), 1)
            r.get_lock().release()
    end_time = time.time()
    frames += 1
    if (end_time - start_time) >= 1:
        if showFPS:
            print("FPS: ", clock.get_fps())
        start_time = time.time()
        frames = 0

def load_image(infilename):
    """
        Helper function that loads an image in black and white, then returns it as an nparray.
    """
    img = Image.open(infilename).convert("L")
    img.load()
    data = np.asarray(img, dtype="int32")
    return data

def handle_close(evt):
    global run
    run = False

def display_image(r):
    """
        Driver function that starts the simulation, after being provided an image to use as stage.
    """
    global screen, npdata, show_robot, clock
    if show_robot:
        mpl.rcParams['toolbar'] = 'None'
    img = Image.fromarray(np.asarray(np.clip(npdata,0,255), dtype="uint8"), "L")
    img = img.convert("RGB")
    '''fig = plt.figure()
    fig.canvas.mpl_connect('close_event', handle_close)
    plt.gray()'''
    robots = []
    if type(r) is list:
        robots = r
    else:
        robots.append(r)
    for robot in robots:
        if robot.controller.goal_oriented():
            robot.x = robot.controller.goal[0][0]
            robot.y = robot.controller.goal[0][1]
    npdata = np.rot90(npdata)
    npdata = np.flipud(npdata)
    if gui:
        pygame.init()
        pygame.display.set_caption("R2P2")
        clock = pygame.time.Clock()
        size = img.size
        img = pygame.image.fromstring(img.tobytes("raw", 'RGB'), size, 'RGB')
        screen = pygame.display.set_mode(size)
        while True:
            #pressed.clear()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.KEYDOWN:
                    press(event)
                if event.type == pygame.KEYUP:
                    pressed.remove(event.key)
            
            update(robots, npdata)
            screen.fill((0, 0, 0))
            screen.blit(img, (0, 0))
            for robot in robots:
                if robot.controller.goal_oriented():
                    aux = [(robot.x, robot.y)]
                    for i in range(0, len(robot.controller.goal)):
                        aux.append(tuple(robot.controller.goal[i]))
                    for i in range(0, len(aux) - 1):
                        pygame.draw.line(screen,(155, 0, 100), aux[i], aux[i+1], 2)
                    for e in aux:
                        pygame.draw.circle(screen, (255, 0, 0), (int(e[0]), int(e[1])), 3)
            animate(robots)
            pygame.display.flip()
        '''fig.canvas.mpl_connect('key_press_event', press)
        fig.canvas.set_window_title(robots[-1].controller.type+"_"+str(robots[-1].identifier))
        ax = fig.gca()
        for robot in robots:
            if robot.controller.goal_oriented():
                img_d = ImageDraw.Draw(img)
                aux = [(robot.x, robot.y)]
                for i in range(0, len(robot.controller.goal)):
                    aux.append(tuple(robot.controller.goal[i]))
                img_d.line(aux, fill=(155, 0, 100), width=3)
                aux_1 = []
                for a in aux:
                    aux_1.append([(a[0]-3, a[1]-3), (a[0]+3, a[1]+3)])
                for a in aux_1:
                    img_d.ellipse(a, fill=(255, 0, 0))
                del img_d
        plt.imshow(img, interpolation='none')
        ani = animation.FuncAnimation(fig, animate, fargs=(fig, img, robots, npdata, ax, ), interval=0)
        if xlabels:
            plt.xticks(xticks, xlabels)
        if ylabels:
            plt.yticks(yticks, ylabels)
        if xlabels or ylabels:
            plt.grid(which='both', linewidth=1.5)
        if not show_robot:
            button = Button(plt.axes([0.7, 0.025, 0.25, 0.05]), 'Toggle play')
            button.on_clicked(switch_show_robot)
        t = Thread(target=update_loop, args=(robots, npdata))
        t.start()
        plt.show()
        t.join()'''
    else:
        update_loop(robots, npdata)

def search_edge_in_angle(origin, angle, image):
    """
        Basic raycasting function. It simply explores the given image in a specific angle until it finds a large enough contrast.
    """
    var_x = math.cos(angle)
    var_y = math.sin(angle)
    ref = image.item((int(origin[0]), int(origin[1])))
    while round(origin[0]) in range(0, image.shape[0]) and round(origin[1]) in range(0, image.shape[1]):
        if image.item(int(origin[0]), int(origin[1])) is 0:
            return origin
        origin = origin[0] + var_x, origin[1] + var_y
    return -1, -1

def search_edge_in_angle_with_limit(origin, angle, image, limit):
    """
        Basic raycasting function. It simply explores the given image in a specific angle until it finds a large enough contrast.
    """
    var_x = math.cos(angle)
    var_y = math.sin(angle)
    o = origin
    while round(origin[0]) in range(0, image.shape[0]) and round(origin[1]) in range(0, image.shape[1])\
        and np.linalg.norm((origin[0] - o[0], origin[1] - o[1])) < limit:
        if image.item(int(origin[0]), int(origin[1])) is 0:
            return origin
        origin = origin[0] + var_x, origin[1] + var_y
    return -1, -1

def search_in_all_directions_with_step_and_offset(origin, image, steps, offset):
    """
        Helper function that performs raycasting for a set of steps angles.
        These angles are spread an equal distance apart, and cover all directions
        from the origin spot.
    """
    collisions = []
    distances = []
    angles = []
    angle_step = math.floor(365/steps)
    for i in range(0, 360, angle_step):
        angles.append(i+offset)
        tmp = search_edge_in_angle(origin, math.radians(i+offset), image)
        if tmp != (-1, -1):
            collisions.append(tmp)
            dst = np.linalg.norm((tmp[0] - origin[0], tmp[1] - origin[1]))
            distances.append(dst)
        else:
            distances.append(math.inf)
            collisions.append((-1, -1))
    return collisions, distances, angles

def search_in_all_directions_with_step_and_offset_with_limit(origin, image, steps, offset, limit):
    """
        Helper function that performs raycasting for a set of steps angles.
        These angles are spread an equal distance apart, and cover all directions
        from the origin spot.
    """
    collisions = []
    distances = []
    angles = []
    angle_step = math.floor(365/steps)
    for i in range(0, 360, angle_step):
        angles.append(i+offset)
        tmp = search_edge_in_angle_with_limit(origin, math.radians(i+offset), image, limit)
        if tmp != (-1, -1):
            collisions.append(tmp)
            dst = np.linalg.norm((tmp[0] - origin[0], tmp[1] - origin[1]))
            distances.append(dst)
        else:
            distances.append(math.inf)
            collisions.append((-1, -1))
    return collisions, distances, angles

def search_in_all_directions_with_angles_and_offset(origin, image, angles, offset):
    """
        Helper function that performs raycasting in a specific set of angles.
    """
    collisions = []
    distances = []
    ang = []
    for a in angles:
        ang.append(a+offset)
        tmp = search_edge_in_angle(origin, math.radians(a+offset), image)
        if tmp != (-1, -1):
            collisions.append(tmp)
            dst = np.linalg.norm((tmp[0] - origin[0], tmp[1] - origin[1]))
            distances.append(dst)
        else:
            distances.append(math.inf)
            collisions.append((-1, -1))
    return collisions, distances, ang

def search_in_all_directions_with_angles_and_offset_with_limit(origin, image, angles, offset, limit):
    """
        Helper function that performs raycasting in a specific set of angles.
    """
    collisions = []
    distances = []
    ang = []
    for a in angles:
        ang.append(a+offset)
        tmp = search_edge_in_angle_with_limit(origin, math.radians(a+offset), image, limit)
        if tmp != (-1, -1):
            collisions.append(tmp)
            dst = np.linalg.norm((tmp[0] - origin[0], tmp[1] - origin[1]))
            distances.append(dst)
        else:
            distances.append(math.inf)
            collisions.append((-1, -1))
    return collisions, distances, ang

def los_raycasting(src, dst, img):
    """
        Function that allows for the calculation of a line of sight between two spots.
        Inputs:
            - src: point from which to check if there is line of sight.
            - dst: point towards which to check if there is line of sight.
            - img: map data to check if there is line of sight over.
        Outputs:
            - tuple representing the coordinates at which the first obstacle was found,
            or None if there is line of sight between both points.
    """
    dist = np.linalg.norm((dst[0] - src[0], dst[1] - src[1]))
    angle = math.degrees(math.atan2((dst[1] - src[1]), (dst[0] - src[0])))
    res = search_edge_in_angle_with_limit(src, math.radians(angle), img, dist)
    return res if res != (-1 , -1) else None
