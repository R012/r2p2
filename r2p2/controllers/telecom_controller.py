import pygame
import utils as u
from controller import Controller

pygame.joystick.init()
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

class Telecom_Controller(Controller):
    """
        Class representing a teleoperated controller. Simply detects input from keyboard and uses it to control
        the robot.
    """
    def __init__(self, config):
        """
            Good old controller. Doesn't need any input.
        """
        super().__init__("TELECOM", config)
        self.detected_edges = []
        self.cur_detected_edges = []
        self.actual_sensor_angles = []
        self.cur_detected_edges_distances = []

    def control(self, dst):
        """
            Driver function to centralize and standardize the controller.
        """
        super(Telecom_Controller, self).control(dst)
        return self.choose_speed(self.ang, dst), self.choose_angle(self.ang, self.dst)

    def register_robot(self, r):
        """
            Registers the robot with the controller. Can be used to issue specific instructions directly, or
            to read some odometry information from the physical hardware.
        """
        self.robot = r

    def write_info_to_log(self, log_file):
        """
            Standard function. Given the kind of controller, it has nothing to do.
        """
        pass

    def choose_angle(self, ang, dst):
        """
            Sets the angular velocity to 25 degrees per second in the direction of the pressed arrow.
            If there are no side arrows pressed, it returns 0.
        """
        global joystick
        if pygame.joystick.get_count() > 0 and joystick.get_init():
            x_diff = joystick.get_axis(0)
            if abs(x_diff) >= 0.01: # Anything under that threshold can be dismissed as noise
                return 90 * x_diff
        if pygame.K_LEFT in u.pressed:
            return -25
        elif pygame.K_RIGHT in u.pressed:
            return 25
        return 0
    
    def choose_speed(self, ang, dst):
        """
            Alters the robot's acceleration based on the directional arrows pressed.
            If forward is pressed, it increases in 3 pixels per second squared.
            If backward is pressed, it decresses in 3 pixels per second squared.
            If none of them are pressed, the robot's acceleration is not modified.
        """
        if pygame.joystick.get_count() > 0 and joystick.get_init():
            y_diff = joystick.get_axis(1)
            if abs(y_diff) >= 0.01: # Anything under that threshold can be dismissed as noise
                return self.robot.max_speed * -1 * y_diff
        if pygame.K_UP in u.pressed:
            return self.robot.speed + 3
        elif pygame.K_DOWN in u.pressed:
            return self.robot.speed - 3
        return 0

    def handle_collision(self, col):
        """
            Registers the currently detected edges so they can be represented by the simulator.
        """
        self.cur_detected_edges = col
        """for e in col:
            if e not in self.detected_edges:
                self.detected_edges.append(e)"""

    def has_edge_list(self):
        """
            Always returns true, given that the controller keeps track of an edge list.
        """
        return True

    def has_cur_detected_edge_list(self):
        """
            Always returns true, given that the controller keeps track of the currently detected edges.
        """
        return True

    def goal_oriented(self):
        """
            Always returns false, given that the controller doesn't have a list of goals to accomplish.
        """
        return False
