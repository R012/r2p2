import math
import numpy as np
import utils as u
import path_planning as pp
import json
from controller import Controller

class Sequential_PID_Controller(Controller):
    def __init__(self, config):
        """
            Constructor for the Sequential_PID_Controller class.
            Initializes its goal list and proportionality constants.
                - goal: list of target spots that must be visited by the robot.
                - ap, ai, ad: proportionality constants for angular velocity.
                - lp, li, ld: proportionality constants for linear acceleration.
        """
        super().__init__("SEQ_PID", config)
        with open('../conf/controller-PID.json', 'r') as fp:
            f = json.load(fp)
            self.goal = f['goal']
            self.ap = f['ap']
            self.ai = f['ai']
            self.ad = f['ad']
            self.lp = f['lp']
            self.li = f['li']
            self.ld = f['ld']
            self.accumulated_angle_error = 0
            self.accumulated_distance_error = 0
            self.last_angle_error = 0
            self.last_distance_error = 0
            self.max_acceleration = 5
            self.detected_edges = []
            self.cur_detected_edges = []
            self.actual_sensor_angles = []
            self.cur_detected_edges_distances = []
            self.target_angle = 0
            self.state = 0
            self.backing = False

    def control(self, dst):
        """
            Driver function to centralize and standardize the controller.
            A tad more complex than in other examples, as it implements the
            controller's obstacle avoidance policy in the form of a very simple
            state machine that needs to be managed.
        """
        super().control(dst)
        if self.backing:
            self.backing -= 1
            return self.robot.orientation, -self.robot.max_speed
        self.manage_state(self.dst)
        if self.state is 0 or self.state is 2:
            return self.control_advance(self.ang, dst)
        elif self.state is 1:
            return self.control_avoid(self.ang, dst)

    def control_advance(self, ang, dst):
        """
            Controller for a regular functioning state. Just keep moving towards the
            next goal.
        """
        if not self.is_done():
            angle = self.choose_angle(ang, dst)
            speed = self.robot.speed + self.choose_acceleration(ang, dst)
            if self.is_at_goal():
                self.switch_to_next_goal()
            return speed, angle
        else:
            return 0, 0

    def control_avoid(self, ang, dst):
        """
            Controller for a state in which avoiding an obstacle has become a necessity.
            Tries to turn a bit  and use that turning in order to avoid the obstacle in question.
        """
        corr_ang = 90
        risk = 1
        for i in range(len(dst)):
            risk *= (dst[i]/self.robot.vision_range[-1])
        if dst[1]+dst[2] < dst[-1]+dst[-2]:
            risk /= (self.robot.vision_range[-1]/dst[-1]) * (self.robot.vision_range[-1]/dst[-2])
        else:
            risk /= (self.robot.vision_range[-1]/dst[1])*(self.robot.vision_range[-1]/dst[2])
        if dst[0]/self.robot.vision_range[-1] < 0.45:
            risk *= dst[0]/self.robot.vision_range[-1]
        safety_distance = (self.robot.vision_range[-1]+self.robot.radius)* risk * 2
        if safety_distance <= self.robot.vision_range[0]:
            safety_distance = self.robot.vision_range[0] * 2
        if (dst[1] + dst[2]) > (dst[-1] + dst[-2]):
            a = self.target_angle%360 + corr_ang
        else:
            a = self.target_angle%360 - corr_ang
          
        x_var = safety_distance * np.cos(np.radians(self.target_angle-a))
        y_var = safety_distance * np.sin(np.radians(self.target_angle-a))
        if a%360 > 90 and a%360 < 270:
            x_var *= -1
        if a%360 > 180:
            y_var *= -1

        new_target = (self.robot.x + x_var,
                      self.robot.y + y_var)
        self.goal.insert(0, new_target)
        angle = self.calculate_angle_variation()
        self.target_angle = a
        speed = 0
        must_retreat = False
        for i in range(len(self.dst)):
            if self.dst[i]/self.robot.vision_range[-1] <\
            self.robot.vision_range[0]/self.robot.vision_range[-1] * 12:
                must_retreat = True
                break
        if must_retreat:
            speed = -self.robot.max_speed
        else:
            speed = self.robot.speed
        return speed, 0

    def write_info_to_log(self, log_file):
        """
            Dumps information about the controller on the selected log file. Said information encompasses:
            - List of goals left to reach.
            - Current state.
            - Current target angle.
            The main point of this information being debugging.
        """
        log_file.write("[GOAL: "+str(self.goal))
        if self.state is 0:
            log_file.write("; STATE: ADVANCING")
        else:
            log_file.write("; STATE: AVOIDING")
        log_file.write("; TARGET_ANGLE: "+str(self.target_angle)+"º]\n")

    def choose_angle(self, angles, distances):
        """
            Given the needs of a PID controller, this function is a tad more complex than other implementations.
            First, it needs to update the robot's target angle, in order to correct deviation.
            Then, it needs to apply the actual PID controller in order to calculate the target angular velocity.
        """
        self.calculate_target_angle()
        return self.calculate_angle_variation()

    def choose_acceleration(self, angles, distances):
        """
            As it happens with the angular velocity, calculating the desired acceleration at any given point in time
            is harder with a PID controller.
            First, the controller calculates a variation in acceleration, then accounts for the robot's current acceleration.
            Finally, it ensures that the absolute value of the new acceleration won't exceed a certain threshold.
        """
        acc = self.robot.acceleration + self.calculate_acceleration_variation()
        if acc > self.max_acceleration:
            return self.max_acceleration
        elif -acc < self.max_acceleration:
            return -self.max_acceleration
        else:
            return acc
    
    def handle_collision(self, col):
        """
            Just updates the list of edges currently being detected, and the list of all detected edges.
        """
        self.cur_detected_edges = col
        for e in col:
            if e not in self.detected_edges:
                self.detected_edges.append(e)

    def on_collision(self, pos):
        self.backing=10

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
            Always returns true, as the controller does keep a list of goals to accomplish.
        """
        return True

    def update_sensor_angles(self, angles, distances):
        """
            Updates the list representing the actual orientation of the sensors so that the simulator may
            provide a proper representation. It also adjusts distances if needed.
        """
        self.cur_detected_edges_distances = distances
        self.actual_sensor_angles = angles
        for i in range(0, len(self.cur_detected_edges_distances)):
            if self.cur_detected_edges_distances[i] is math.inf:
                self.cur_detected_edges_distances[i] = 10000

    def switch_to_next_goal(self):
        """
            Actually removes the currently achieved goal from the list of objectives to accomplish.
        """
        self.goal.pop(0)
        if self.state is 2:
            self.state = 0

    def is_at_goal(self):
        """
            Determines if the actual robot is touching the goal. If so, the goal is considered to have been reached.
        """
        if u.npdata.item((int(self.goal[0][0]), int(self.goal[0][1]))) is 0:
            return True
        return np.linalg.norm((self.goal[0][0]-self.robot.x, self.goal[0][1]-self.robot.y)) <= self.robot.radius * 1.85

    def is_done(self):
        """
            Returns True only when all goals have been accomplished.
        """
        return not self.goal

    def manage_state(self, dst):
        """
            Simply checks whether the robot can move forward normally or needs to avoid an obstacle,
            and update state accordingly.
        """
        rng = self.robot.vision_range[-1]
        tolerance = self.robot.vision_range[0]/rng * 13
        if (dst[-1]/rng < tolerance and dst[-2]/rng < tolerance) or\
           (dst[1]/rng < tolerance and dst[2]/rng < tolerance) or\
           dst[0]/rng < tolerance:
            if self.state == 0:
                self.state = 1
            else:
                self.state = 2
            return
        self.state = 0

    def calculate_acceleration_variation(self):
        """
            Applies PID control over the distance from the current spot to the current goal in order
            to determine how acceleration should vary. It also includes a policy to ease turning around.
        """
        if abs(self.target_angle - self.robot.orientation) > 45 or self.is_done():
            self.accumulated_distance_error += self.robot.brake()
            return self.robot.brake()
        e = np.linalg.norm((self.goal[0][0] - self.robot.x, self.goal[0][1] - self.robot.y))
        if self.robot.acceleration < self.max_acceleration:
            self.accumulated_distance_error += e
        de = e - self.last_distance_error
        self.last_distance_error = e
        return self.lp * e + self.li * self.accumulated_distance_error + self.ld * de

    def calculate_angle_variation(self):
        """
            Applies PID control over the difference between the desired angle and the current orientation
            of the robot in order to determine angular velocity.
        """
        e = self.target_angle - self.robot.orientation
        self.accumulated_angle_error += e
        de = e - self.last_angle_error
        self.last_angle_error = e
        return self.ap * e + self.ai * self.accumulated_angle_error + self.ad * de

    def calculate_target_angle(self):
        """
            Simply uses vector math to determine what angle the robot should be oriented in next.
            Python's math library handles most of the heavylifting.
        """
        self.target_angle = math.degrees(math.atan2((self.goal[0][1] - self.robot.y), (self.goal[0][0] - self.robot.x)))

    def register_robot(self, r):
        self.robot = r
        initial_point = self.goal[0]
        self.robot.set_position(initial_point[0], initial_point[1])
        self.robot.set_last_position(initial_point[0], initial_point[1])
        
def path_planning_controller(config):
    """
        Factory for a PID controller which uses path planning in order to determine what waypoints must be visited.
        Inputs:
            - f: dictionary containing all necessary configuration variables, namely a list of goals, and 6 proportionality
            constants (ap, ai, ad, lp, li, ld) which will be used to define the PID controller proper (a stands for angular,
            l for linear), as well as the algorithm to be used, the start and goal points, the heuristic to be used, and
            configuration variables linked to the specific algorithm. These configuration variables must be a list of waypoints
            if the variant of the algorithm uses a navigation mesh, or a number representing the number of divisions to perform
            on each axis of the map if it's grid-based.
        Outputs:
            - A fully configured and ready to use Sequential_PID_Controller object.
    """
    # Loads the controller config (only 1 expected) and overwrite the parameters with the ones in the scenario (it allows default values to be set at controller)
    f = {**config['controllers'][0], **config}
    if f['start'] == f['goal']:
        raise ValueError('Start and goal are the same spot.')
    controller = Sequential_PID_Controller(config)
    if 'mesh' in f['algorithm']:
        controller.goal=pp.run_path_planning_mesh(f['waypoints'], f['algorithm'], f['start'], f['goal'], f['heuristic'])
    else:
        controller.goal=pp.run_path_planning(f['grid_size'], f['algorithm'], f['start'], f['goal'], f['heuristic'])
        
    return controller
