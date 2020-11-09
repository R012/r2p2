import json

class ConfigManager():
  def __init__(self, scenario_config = '../conf/scenario-default.json', controller_config = None, params = None):
    self.config = self.__get_json(scenario_config)
    self.override_with_cli_params(params)
    print(self.config)

    if controller_config:
      print("Using controller from Command Line args")
      self.__load_cli_controller(controller_config)
    else:
      self.__load_controller_conf()
    # Override controllers' params if they are found in the scenario too
    self.__override_with_scen()

  def override_with_cli_params(self, params):
    self.config = {**self.config, **params}

  def get_config(self):
    return self.config
  
  def __get_json(self, path):
    """ Returns a dictionary with the parameters included in the JSON filepath given as argumetn """
    with open(path, 'r') as f:
      return json.load(f)

  def __load_controller_conf(self):
    """ Load the controller or controllers specified in the scenario config """
    if 'controller' in self.config:
      # If we get a list of controllers
      if type(self.config['controller']) is list:
        for path in self.config['controller']:
          if 'controllers' in self.config:
            self.config['controllers'].append(self.__get_json(path))
          else:
            self.config['controllers'] = [self.__get_json(path)]
      # If we only get one controller
      else:
        self.config['controllers'] = [self.__get_json(self.config['controller'])]
    
  def __load_cli_controller(self, path):
    """ Load the controller passed through command line (only 1) """
    self.config['controllers'] = [self.__get_json(path)]
  
  def __override_with_scen(self):
    """
    Overrides the controller setup parameters with any parameter found on the scenario
    Example:
      If you are using a PathPlanning scenario pointing to a path_panning controller setup and you
      have the controller parameter "grid_size" in both, the scenario and the controller, the
      scenario one will be used.
      If more than one controller was specified, it will be overriden in all of them.
    """
    # Not doing a merge of dictionaries to add warning messages of overriding and not adding extra params, which could lead to loops
    for controller in self.config['controllers']:
      for key in controller:
        if key in self.config:
          print("WARNING: The parameter \"{}\" has been provided on Scenario and Controller config. Scenario value \"{}\" will be used.".format(key, self.config[key]))
          controller[key] = self.config[key]
