# External dependencies
import json
import numpy as np
import pygame
import time

from PIL import Image
from scipy.stats import norm
from copy import deepcopy
from matplotlib import colors as mpl_colors

# Internal dependencies
from controllers.controllers import get_controllers
from robot import Robot
from constants import simulator, pddl_exec

# CONSTANTS
DEFAULT_DELTA     = simulator['DEFAULT_DELTA']
DEFAULT_FRAMERATE = simulator['DEFAULT_FRAMERATE']
R2P2_SCREEN_TITLE = simulator['SCREEN_TITLE']
DELTA_FACTOR      = simulator['DELTA_FACTOR']
TOLERANCE         = simulator['TOLERANCE']
FONT_SIZE         = simulator['FONT_SIZE']
FONT_TYPE         = simulator['FONT_TYPE']
SHOW_FPS          = simulator['SHOW_FPS']
DEFAULT_STEP      = pddl_exec['DEFAULT_STEP']

class Simulator:
  def __init__(self, config_file = '../conf/scenario-default.json'):
    self.start_time = time.time()
    self.config_file = config_file
    self.cfg = self.__load_config(config_file)
    self.ctr_cfg = self.__load_controllers_config()
    self.rob_cfg = self.__load_robots_config()
    self.clock = pygame.time.Clock()
    self.co2_center = self.cfg['co2_center'] if 'co2_center' in self.cfg else (0, 0)
    self.show_robot = True
    self.pressed = []
    self.labels = []

    if 'co2_radius' in self.cfg:
      self.__generate_dist(self.cfg['co2_radius'])
    
  def start_simulation(self):
    self.np_img_data = self.__load_image(self.cfg['stage'])
    self.__init_sizes(self.np_img_data)
    controllers = self.__create_controllers(self.np_img_data)
    robots = self.__create_robots(controllers)
    self.__start(robots, self.np_img_data)

  # Internal methods
  def __init_sizes(self, img_data):
    img_w, img_h = img_data.shape
    if 'grid_size' in self.cfg:
      if isinstance(self.cfg['grid_size'], list):
        self.size = img_w/self.cfg['grid_size'][1], img_h/self.cfg['grid_size'][0]
      else:
        self.size = img_w/self.cfg['grid_size'], img_h/self.cfg['grid_size']
    elif (
      len(self.ctr_cfg) > 0      and
      'class' in self.ctr_cfg[0] and 
      self.ctr_cfg[0]['class'] == 'pddl_executor.PDDL_Executor' # I don't like how I'm checking if this is using the PDDL Executor assuming only 1 controller
    ):
      self.size = img_w/DEFAULT_STEP, img_h/DEFAULT_STEP

    self.grid_size = list(self.size) # Just to keep compatibility with old code, we should use self.size
      


  def __create_controllers(self, img_data):
    """
        Driver function to abstract the process of instancing a Controller object
        using factories.
        Inputs:
            - img_data: Numpy Array with the image grayscale
        Outputs:
            - a fully configured Controller object or a list of Controllers, depending on the config
    """
    if 'class' in self.cfg or len(self.ctr_cfg) > 0:
      return get_controllers(self)
    else:
      raise KeyError("The configuration file received doesn't contain a \"class\" attribute or controllers")

  def __load_config(self, config_file='../conf/config.json'):
      """
          Return a configuration dictionary with all the parameters coming from the scenario.
          - "controllers" will be a list of dictionaries with the configuration of the controllers that were included in the scenario file
          Example:
          {
            config_param: value,
            ...
            controllers: [
              { config_param: value ... }
              ...
            ]
          }
      """
      with open(config_file, 'r') as scen_cfg:
          config = json.load(scen_cfg)
      return config

  def __load_controllers_config(self):
    controllers_config = []
    if 'controller' in self.cfg:
      # If we get a list of controllers
      if type(self.cfg['controller']) is list:
        for path in self.cfg['controller']:
          with open(path, 'r') as conf_file:
            controllers_config.append(json.load(conf_file))
      # If we only get one controller
      else:
        with open(self.cfg['controller'], 'r') as conf_file:
          controllers_config.append(json.load(conf_file))
    return controllers_config

  def __load_robots_config(self):
    """Returns a list of Robot instances based in the config file"""
    robots_cfg = []
    if type(self.cfg['robot']) is list:
      for i, path in enumerate(self.cfg['robot']):
        with open(path, 'r') as rob_file:
          robots_cfg.append(json.load(rob_file))
    else:
      with open(self.cfg['robot'], 'r') as rob_file:
        robots_cfg.append(json.load(rob_file))
    return robots_cfg
    
  def __create_robots(self, controllers):
    robots = []
    for i, r_cfg in enumerate(self.rob_cfg):
      if i >= len(controllers):
        robots.append(self.__create_robot(r_cfg, deepcopy(controllers[0])))
      else:
        robots.append(self.__create_robot(r_cfg, controllers[i]))
    return robots

  def __create_robot(self, r_cfg, controller = None):
      """
          Uses a robot configuration to generate a fully configured Robot object.
          Inputs:
              - r_cfg: configuration dictionary with the parameters of the robot
              - controller: pre-configured Controller object.
          Outputs:
              - fully configured Robot object.
      """
      robot_name = r_cfg['name'] if 'name' in r_cfg else None
      robot = Robot(identifier = r_cfg['id'],
                    x = r_cfg['x'],     y = r_cfg['y'],
                    orientation = r_cfg['orientation'],   sensors = r_cfg['sonars'],
                    vision_range = tuple(r_cfg['sonar_range']),
                    radius = r_cfg['radius'], max_speed = r_cfg['max_speed'], controller=controller, name = robot_name)
      if 'battery' in r_cfg:
        robot.insert_battery_details(r_cfg['step'],           r_cfg['battery'],       r_cfg['charging_rate'],
                                     r_cfg['movement_cost'],  r_cfg['reading_cost'],  r_cfg['picture_cost'],
                                     r_cfg['generic_cost'])
      if 'color' in r_cfg:
        robot.set_color(r_cfg['color'])

      if robot.controller.goal_oriented():
        robot.x = robot.controller.goal[0][0]
        robot.y = robot.controller.goal[0][1]
      
      return robot

  def __load_image(self, image_path):
    """
        Helper function that loads an image in black and white, then returns it as an nparray.
    """
    img = Image.open(image_path).convert("L")
    img.load()
    data = np.asarray(img, dtype="int32")
    return data

  
  def __generate_dist(self, size = 1):
    self.scale = 1/size
    norm.stats(scale=self.scale)
    self.frozen_dist = norm()
  

  def __start(self, robots, img_data):
    """
        Driver function that starts the simulation, after being provided an image to use as stage.
    """
    self.run = True

    rot_img = np.flipud(np.rot90(img_data))

    if 'gui' in self.cfg and self.cfg['gui']:
      self.__init_gui(robots, self.__rgb_from_grayscale(img_data))
    else:
      self.__update_loop(robots, rot_img)

  def __rgb_from_grayscale(self, img_data):
    """Convert the numpy array of data we got from the grayscale image (1 channel) into RGB format (3 channels)"""
    img = Image.fromarray(np.asarray(np.clip(img_data,0,255), dtype="uint8"), "L")
    img = img.convert("RGB")
    return img


  def __calculate_delta(self):
    """
    Helper function that calculates the delta value for any given update call.
    Necessary in order for the update to be performed somewhat smoothly.
    """
    delta = DEFAULT_DELTA
    if 'gui' in self.cfg and self.cfg['gui']:
      delta = self.clock.tick(DEFAULT_FRAMERATE)
    return delta

  def __update_loop(self, robots, img_data):
    while self.run:
      delta = self.__calculate_delta()
      for r in robots:
          r.get_lock().acquire()
          r.update(img_data, delta)
          r.write_stats_to_log()
          r.get_lock().release()
      self.pressed.clear()
      time.sleep(1/80)

  def __setup_screen(self, img):
    pygame.init()
    pygame.display.set_caption(R2P2_SCREEN_TITLE)
    size = img.size
    img = pygame.image.fromstring(img.tobytes("raw", 'RGB'), size, 'RGB')
    screen = pygame.display.set_mode(size)
    return screen
  
  def __get_fonts(self):
    """Returns the different fonts required"""
    lbl_font_size = round((self.size[0]+self.size[1]) // 2)
    if lbl_font_size > min(list(self.size)):
        lbl_font_size = int(min(list(self.size)))
    font = pygame.font.SysFont(FONT_TYPE, FONT_SIZE)
    label_font = pygame.font.SysFont(FONT_TYPE, lbl_font_size)
    return (font, label_font)

  def __process_events(self):
    """
    Updates the "pressed" list with the events received from pygame.
    Returns True if the loop must continue or False if not
    """
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
          pygame.quit()
          return False
      if event.type == pygame.KEYDOWN:
          self.pressed.append(event.key)
      if event.type == pygame.KEYUP:
          self.pressed.remove(event.key)
    return True

  def __update(self, robots, npdata):
    delta = self.__calculate_delta() * DELTA_FACTOR
    for r in robots:
      r.get_lock().acquire()
      r.update(npdata, delta)
      r.write_stats_to_log()
      r.get_lock().release()
  
  def __draw_labels(self, font, screen):
    for label in self.labels:
      pos = label[0]
      text = label[1]
      text = font.render(text, True, (0, 0, 0))
      text_rect = text.get_rect()
      text_rect.left = pos[0] + 5
      text_rect.centery = pos[1]
      pygame.draw.circle(screen, (0, 0, 0), (int(pos[0]), int(pos[1])), 1)
      screen.blit(text, text_rect)

  def __animate(self, robots, screen, frames = 0):
    """
        Update function. Updates internal world data, then prints it to a plot.
        Must be registered to said plot.
    """
    if self.show_robot:
        for r in robots:
            r.get_lock().acquire()
            if r.controller.has_cur_detected_edge_list():
                for a in r.controller.actual_sensor_angles:
                    dstX = r.x + np.cos(np.radians(a)) * r.controller.cur_detected_edges_distances[r.controller.actual_sensor_angles.index(a)]
                    dstY = r.y + np.sin(np.radians(a)) * r.controller.cur_detected_edges_distances[r.controller.actual_sensor_angles.index(a)]
                    pygame.draw.line(screen, (255, 0, 255), (int(r.x), int(r.y)), (int(dstX), int(dstY)), 1)
                for e in r.controller.cur_detected_edges:
                    pygame.draw.circle(screen, (0, 255, 0), (int(e[0]), int(e[1])), int((100/np.linalg.norm((e[0]-r.x, e[1]-r.y)))/90))
                if type(r.color) is str:
                    r.color = list(mpl_colors.to_rgb(mpl_colors.get_named_colors_mapping()[r.color]))
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
    if (end_time - self.start_time) >= 1:
        if SHOW_FPS:
            print("FPS: ", self.clock.get_fps())
        self.start_time = time.time()
        frames = 0
    return frames

  def __init_gui(self, robots, img_data):
    screen = self.__setup_screen(img_data)
    font, label_font = self.__get_fonts()
    w, h = self.size
    img_w, img_h = img_data.size

    running = True
    frames = 0

    while running:
      running = self.__process_events()
      self.__update(robots, img_data)

      screen.fill((0, 0, 0))
      screen.blit(img_data, (0, 0))

      if self.grid_size:
        grid_color = (150, 150, 150)
        offset_x = round(w/2)
        offset_y = round(h/4)
        for i in range(round(img_w/h)+1):
            li_coord = round(i * w)
            pygame.draw.line(screen, grid_color,
                              (0, li_coord), (screen.get_width(), li_coord))
            if ((w < TOLERANCE and i%5 == 0)\
            or w >= TOLERANCE):
                if int(i * w + w//2) - 2 <img_w:
                    if img_data.item(int(i * w + w//2) - 2,
                                int(h//2) - 2) == 0:
                        font_color = (255, 255, 255)
                    else:
                        font_color = (0, 0, 0)
                else:
                    font_color = (0, 0, 0)
                text = label_font.render(str(i), True, font_color)
                text_rect = text.get_rect()
                text_rect.centerx = int(i * h + offset_x)
                text_rect.top = int(offset_y)
                screen.blit(text, text_rect)
        
        for j in range(round(img_h/w)+1):
            col_coord = round(j * h)
            pygame.draw.line(screen, grid_color,
                              (col_coord, 0), (col_coord, screen.get_height()))
            if ((h < TOLERANCE and j%5 == 0)\
            or h >= TOLERANCE):
                if int(w/2) < img_w and\
                int(h*j+h/2)-20 < img_h:
                    if img_data.item(int(w/2) - 2,
                                int(h*j+h/2) - 20) == 0:
                        font_color = (255, 255, 255)
                    else:
                        font_color = (0, 0, 0)
                else:
                    font_color = (0, 0, 0)
                text = label_font.render(str(j), True, font_color)
                text_rect = text.get_rect()
                text_rect.top = int(j * w + offset_y)
                text_rect.centerx = int(offset_x)
                screen.blit(text, text_rect)
    if img_data.item(int(self.co2_center[0]), int(self.co2_center[1])) != 0:
        pygame.draw.line(screen, (125, 0, 0), (self.co2_center[0]-5, self.co2_center[1]-5),
                          (self.co2_center[0]+5, self.co2_center[1]+5), 2)
        pygame.draw.line(screen, (125, 0, 0), (self.co2_center[0]-5, self.co2_center[1]+5),
                          (self.co2_center[0]+5, self.co2_center[1]-5), 2)
    for robot in robots:
        if robot.controller.goal_oriented():
            aux = [(robot.x, robot.y)]
            for i in range(0, len(robot.controller.goal)):
                aux.append(tuple(robot.controller.goal[i]))
            for i in range(0, len(aux) - 1):
                pygame.draw.line(screen,(155, 0, 100), aux[i], aux[i+1], 2)
            for e in aux:
                pygame.draw.circle(screen, (255, 0, 0), (int(e[0]), int(e[1])), 3)
    self.__draw_labels(font, screen)
    frames = self.__animate(robots, screen, frames)
    pygame.display.flip()