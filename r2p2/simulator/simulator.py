# External dependencies
import json
import numpy as np
import pygame

from PIL import Image
from scipy.stats import norm
from copy import deepcopy

# Internal dependencies
from controllers.controllers import get_controllers
from robot import Robot

class Simulator:
  def __init__(self, config_file = '../conf/scenario-default.json'):
    self.config_file = config_file
    self.cfg = self.__load_config(config_file)
    self.ctr_cfg = self.__load_controllers_config()
    self.rob_cfg = self.__load_robots_config()
    if 'co2_radius' in self.cfg:
      self.__generate_dist(self.cfg['co2_radius'])
    
  def start_simulation(self):
    np_img_data = self.__load_image(self.cfg['stage'])
    controllers = self.__create_controllers(np_img_data)
    robots = self.__create_robots(controllers)
    self.__display_image(robots, np_img_data)

  # Internal methods

  def __create_controllers(self, img_data):
    """
        Driver function to abstract the process of instancing a Controller object
        using factories.
        Inputs:
            - img_data: Numpy Array with the image grayscale
        Outputs:
            - a fully configured Controller object or a list of Controllers, depending on the config
    """
    if 'class' in self.cfg or 'controllers' in self.cfg:
        return self.get_controllers(self.cfg)
    else:
        raise KeyError("The configuration file received doesn't contain a \"class\" attribute")

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
    img = Image.fromarray(np.asarray(np.clip(img_data,0,255), dtype="uint8"), "L")
    img = img.convert("RGB")

    for robot in robots:
      if robot.controller.goal_oriented():
        robot.x = robot.controller.goal[0][0]
        robot.y = robot.controller.goal[0][1]

    img_data = np.flipud(np.rot90(img_data))

    if 'gui' in self.cfg and self.cfg['gui']:
      self.__init_gui()
    else:
      self.__update_loop(robots, img_data)


  def update_loop(self, robots, img_data):
    global delta, pressed, run
    while run:
      init_time = time.time()
      if gui:
          delta = calculate_delta()
      else:
          delta = 0.1
      for r in robots:
          r.get_lock().acquire()
          r.update(img_data, delta)
          r.write_stats_to_log()
          r.get_lock().release()
      pressed.clear()
      time.sleep(1/80)


  def __init_gui(self):
    if 'gui' in self.cfg and self.cfg['gui']:
        pygame.init()
        pygame.display.set_caption("R2P2")
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