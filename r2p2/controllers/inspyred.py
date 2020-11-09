import numpy as np
import math
import time
import json
import os

import controller as c
import utils as u

class Inspyred_controller(c.Controller):
    def __init__(self, config):
        super(Inspyred_controller, self).__init__("Inspyred", config)
        self.n_sonar = 0
        self.linCoefs = []
        self.angCoefs = []
        self.vlinmax = 0
        self.vangmax = 0
        self.distance = 0
        self.ranges = []
        self.odom = (-1, -1)
        self.origin = (-1, -1)
        self.distance = 0
        with open('../conf/controller-inspyred.json', 'r') as fp:
            f = json.load(fp)
            self.weights = f['weights']
            self.time = f['time']
            self.epoch_time = self.time * 1000
            self.evolve = f['evolve']
            self.set_network_params(self.weights)
        self.log = open("../logs/inspyred.log", 'w')
        self.cur_detected_edges = []
        self.actual_sensor_angles = []
        self.cur_detected_edges_distances = []

    def on_collision(self, pos):
        self.time = 0

    def handle_collision(self, col):
        """
            Registers the currently detected edges so they can be represented by the simulator.
        """
        self.cur_detected_edges = col

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

    def __output_fitness(self):
        f = open('../res/fitness.txt', 'w')
        f.write(str(self.fitness()))
        f.close()

    def control(self, dst):
        self.update_sensor_angles(self.ang, dst)
        self.distance += np.linalg.norm((self.robot.x - self.odom[0],\
                                        self.robot.y - self.odom[1]))
        self.odom = (self.robot.x, self.robot.y)
        if self.evolve:
            self.time -= u.delta
        if self.time <= 0:
            if self.evolve:
                self.__output_fitness()
                self.time = self.epoch_time
                self.odom = self.origin
                self.distance = 0
                self.robot.position(self.origin[0], self.origin[1])
                self.robot.orientation = 0
                self.robot.stop()
                self.robot.acceleration = 0

                self.__build_network()
                return 0, 0

        # Control logic here

        #print("Control with: ", self.weights)
        #print("Distance: ", dst)
        #print("LinCoefs:  ", self.linCoefs)
        #print("AngCoefs:  ", self.angCoefs)
        linVelocity = 0
        angVelocity = 0

        for distance, coef in zip(self.linCoefs, dst):
            linVelocity += coef * distance

        for distance, coef in zip(self.angCoefs, dst):
            angVelocity += coef * distance

        out = (linVelocity, angVelocity)
        self.log_step(dst, out)
        # print(dst, out)

        return out

    def log_step(self, dst, out):
        self.log.write("[" + str(time.time())+"]\tOdom: "+str(self.odom)+"\t-\tIn: "+str(dst)+"\t-\tOut: "+str(out)+"\n")

    def register_robot(self, r):
        super(Inspyred_controller, self).register_robot(r)
        self.odom = (self.robot.x, self.robot.y)
        self.origin = (self.robot.x, self.robot.y)
        self.__build_network()

    def __build_network(self):
        if type(self.robot.sensors) is list:
            self.n_sonar = len(self.robot.sensors)
        else:
            self.n_sonar = self.robot.sensors

        if self.evolve:
            self.__load_new_params()
        else:
            self.set_network_params(self.weights)

    def fitness(self):
        return self.distance + np.linalg.norm((self.odom[0] - self.origin[0],\
                                               self.odom[1] - self.origin[1]))

    def has_cur_detected_edge_list(self):
        """
            Always returns true, given that the controller keeps track of the currently detected edges.
        """
        return True

    def set_network_params(self, chromosome):
        # Interpret chromosome
        # Weights: [l1, l2, l3, ..., ln, a1, a2, a3, ..., an, vmaxlin, vmaxang]

        print("Assessing chromosome: ", chromosome, len(chromosome))
        if len(chromosome) % 2 != 0 or len(chromosome) <= 4:
            print ("ERROR: incorrect weights length")

        self.weights = chromosome
        self.vlinmax = chromosome[-2]
        self.vangmax = chromosome[-1]
        self.linCoefs = chromosome[:int((len(chromosome) / 2) - 1)]
        self.angCoefs = chromosome[int((len(chromosome) / 2)) - 1:-2]

    def __load_new_params(self):
        original_time = os.path.getmtime('../res/weights.json')
        while os.path.getmtime('../res/weights.json') == original_time:
            time.sleep(0.1)
        try:
            f = open('../res/weights.json', 'r')
            self.set_network_params(json.load(f)['params'])
            f.close()
        except Exception as e:
            print(e)
            self.__load_new_params()
