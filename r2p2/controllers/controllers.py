import sys

class add_path():
    '''
        Wrapper class to automatically handle adding the local controller
        directory to the simulator's execution path.
    '''
    def __init__(self, path):
        self.path = path

    def __enter__(self):
        sys.path.insert(0, self.path)

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            sys.path.remove(self.path)
        except ValueError:
            pass

def load_controller(name):
    with add_path('controllers'):
        components = name.split('.')
        controller = __import__(components[0])
        for comp in components[1:]:
            controller = getattr(controller, comp)
    return controller

def get_controllers(simulator):
    """Create controllers based on the configuration received"""
    if len(simulator.ctr_cfg) > 0:
        controllers = []
        for ctrl in simulator.ctr_cfg:
            # Create the controller passing the config as arg to the constructor
            controller = load_controller(ctrl['class'])(ctrl, simulator)
            controllers.append(controller)
        return controllers
    elif 'controllers' in simulator.cfg and len(simulator.cfg['controllers']) > 0:
        # Create the controller passing the config as arg to the constructor
        return load_controller(simulator.cfg['class'])(simulator.cfg, simulator)



