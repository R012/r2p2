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
from matplotlib.pyplot import imshow
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
import controller
from controllers.controllers import get_controllers

start_time = time.time()
last_call = time.time()
frames = 0
delta = 0
pressed = []
labels = []
co2_center = (0, 0)
scale = 1
grid_size = []
ax = None
frozen_dist = None
fig = None
npdata = None
gui = True
show_robot = True
button = None
showFPS = False
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

def create_controllers():
    """
        Driver function to abstract the process of instancing a Controller object
        using factories.
        Inputs:
            - No input, everything comes from the config global variable
        Outputs:
            - a fully configured Controller object or a list of Controllers, depending on the config
    """
    global npdata
    if 'class' in config or 'controllers' in config:
        return get_controllers(config)
    else:
        raise KeyError("The configuration file received doesn't contain a \"class\" attribute")

def create_robot(json_file = '../conf/robot.json', controller = None):
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

def init_globals_from_config():
    """
    Initialize all global variables based on config
    TODO: Remove global variables and keep only the config dict, to at some point remove it too and pass it as reference if needed
    """
    showFPS = 'fps' in config
    gui = config['gui']
    co2_center = config['co2_center'] if 'co2_center' in config else (0, 0)


def create_robots(controllers):
    """
    Returns a list of robots or one robot depending on the config
    (TBC, Pedro) Why did we do a deepcopy of the controller config only when we received 1 controler, but not when multiple?
    If this is still a requirement, we need to add it to the r.append line
    """
    if type(config['robot']) is list:
        r = []
        for i, path in enumerate(config['robot']):
            if i >= len(controllers):
                r.append(create_robot(path, controllers[0]))
            else:
                r.append(create_robot(path, controllers[i]))
    else:
        r = create_robot(config['robot'], controllers[0])
    return r

def load_simulation(config_mgr):
    """
        Loads a simulation using a configuration file. For the time being, it limits itself to loading the corresponding map and robot.
        Inputs:
            - json_file: path to the configuration file describing the simulation to be loaded. This configuration file must be a JSON
            containing the following:
                * stage: string defining the path to the image file that represents the stage to be loaded.
                * robot: string defining the path to the configuration file of the robot that will be used.
    """
    global gui, npdata, co2_center, showFPS, config
    # Load the config in the global variable
    config = config_mgr.get_config()
    # Init global variables based on config dict
    # TODO: I think we should refactor everything to only use config as global (Pedro)
    init_globals_from_config()
    # Load the image used in the stage
    npdata = load_image(config['stage'])
    # Get the controller if only one or a list of controllers
    controllers = create_controllers()

    if 'co2_radius' in config:
        generate_dist(config['co2_radius'])
    
    robots = create_robots(controllers)
    display_image(robots)

def update_loop(robots, npdata):
    global delta, pressed, run
    while run:
        init_time = time.time()
        if gui:
            delta = calculate_delta()
        else:
            delta = 0.1
        for r in robots:
            r.update(npdata, delta, True)
        pressed.clear()
        time.sleep(1/80)

def update(robots, npdata):
    delta = calculate_delta()/1000
    for r in robots:
        r.update(npdata, delta, True)

def animate(robots):
    """
        Update function. Updates internal world data, then prints it to a plot.
        Must be registered to said plot.
    """
    global start_time, frames, delta, show_robot, screen, clock, grid_size
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
    global screen, npdata, show_robot, clock, labels, co2_center
    img = Image.fromarray(np.asarray(np.clip(npdata,0,255), dtype="uint8"), "L")
    img = img.convert("RGB")
    robots = []
    if type(r) is list:
        robots = r
    else:
        robots.append(r)
    for robot in robots:
        if robot.controller.goal_oriented():
            robot.set_position(robot.controller.goal[0][0], robot.controller.goal[0][1])

    npdata = np.rot90(npdata)
    npdata = np.flipud(npdata)
    if gui:
        pygame.init()
        pygame.display.set_caption("R2P2")
        clock = pygame.time.Clock()
        size = img.size
        img = pygame.image.fromstring(img.tobytes("raw", 'RGB'), size, 'RGB')
        screen = pygame.display.set_mode(size)
        font = pygame.font.SysFont("BitstreamVeraSans Roman", 23)
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.KEYDOWN:
                    press(event)
                if event.type == pygame.KEYUP:
                    if event.key in pressed:
                        pressed.remove(event.key)
            
            update(robots, npdata)
            screen.fill((0, 0, 0))
            screen.blit(img, (0, 0))
            if grid_size:
                grid_color = (150, 150, 150)
                font_size = round((grid_size[0]+grid_size[1]) // 2)
                if font_size > min(grid_size):
                    font_size = int(min(grid_size))
                label_font = pygame.font.SysFont("BitstreamVeraSans Roman", font_size)
                tolerance = 12
                offset_x = round(grid_size[0]/2)
                offset_y = round(grid_size[1]/4)
                for i in range(round(npdata.shape[0]/grid_size[1])+1):
                    liCoord = round(i * grid_size[0])
                    pygame.draw.line(screen, grid_color,
                                     (0, liCoord), (screen.get_width(), liCoord))
                    if ((grid_size[0] < tolerance and i%5 == 0)\
                    or grid_size[0] >= tolerance):
                        if int(i * grid_size[0] + grid_size[0]//2) - 2 <npdata.shape[0]:
                            if npdata.item(int(i * grid_size[0] + grid_size[0]//2) - 2,
                                       int(grid_size[1]//2) - 2) is 0:
                                font_color = (255, 255, 255)
                            else:
                                font_color = (0, 0, 0)
                        else:
                            font_color = (0, 0, 0)
                        text = label_font.render(str(i), True, font_color)
                        text_rect = text.get_rect()
                        text_rect.centerx = int(i * grid_size[1] + offset_x)
                        text_rect.top = int(offset_y)
                        screen.blit(text, text_rect)
                
                for j in range(round(npdata.shape[1]/grid_size[0])+1):
                    colCoord = round(j * grid_size[1])
                    pygame.draw.line(screen, grid_color,
                                     (colCoord, 0), (colCoord, screen.get_height()))
                    if ((grid_size[1] < tolerance and j%5 == 0)\
                    or grid_size[1] >= tolerance):
                        if int(grid_size[0]/2) < npdata.shape[0] and\
                        int(grid_size[1]*j+grid_size[1]/2)-20 < npdata.shape[1]:
                            if npdata.item(int(grid_size[0]/2) - 2,
                                       int(grid_size[1]*j+grid_size[1]/2) - 20) is 0:
                                font_color = (255, 255, 255)
                            else:
                                font_color = (0, 0, 0)
                        else:
                            font_color = (0, 0, 0)
                        text = label_font.render(str(j), True, font_color)
                        text_rect = text.get_rect()
                        text_rect.top = int(j * grid_size[0] + offset_y)
                        text_rect.centerx = int(offset_x)
                        screen.blit(text, text_rect)
            if npdata.item(int(co2_center[0]), int(co2_center[1])) is not 0:
                pygame.draw.line(screen, (125, 0, 0), (co2_center[0]-5, co2_center[1]-5),
                                 (co2_center[0]+5, co2_center[1]+5), 2)
                pygame.draw.line(screen, (125, 0, 0), (co2_center[0]-5, co2_center[1]+5),
                                 (co2_center[0]+5, co2_center[1]-5), 2)
            for robot in robots:
                if robot.controller.goal_oriented():
                    aux = [(robot.x, robot.y)]
                    for i in range(0, len(robot.controller.goal)):
                        aux.append(tuple(robot.controller.goal[i]))
                    for i in range(0, len(aux) - 1):
                        pygame.draw.line(screen,(155, 0, 100), aux[i], aux[i+1], 2)
                    for e in aux:
                        pygame.draw.circle(screen, (255, 0, 0), (int(e[0]), int(e[1])), 3)
            for label in labels:
                pos = label[0]
                text = label[1]
                text = font.render(text, True, (0, 0, 0))
                text_rect = text.get_rect()
                text_rect.left = pos[0] + 5
                text_rect.centery = pos[1]
                pygame.draw.circle(screen, (0, 0, 0), (int(pos[0]), int(pos[1])), 1)
                screen.blit(text, text_rect)
            animate(robots)
            pygame.display.flip()
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
