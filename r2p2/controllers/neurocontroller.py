import numpy as np
import math
import time
import json
import os

from sklearn.neural_network import MLPClassifier

import controller as c
import utils as u

class Neuro_controller(c.Controller):
    def __init__(self, config):
        super(Neuro_controller, self).__init__("NEURO", config)
        self.n_sonar = 0
        self.ann = 0
        self.distance = 0
        self.ranges = []
        self.odom = (-1, -1)
        self.origin = (-1, -1)
        self.distance = 0
        with open('../conf/controller-neuro.json', 'r') as fp:
            f = json.load(fp)
            self.weights = f['weights']
            self.hidden_layer = f['hidden_layer']
            self.activation = f['activation']
            self.time = f['time'] * 1000
            self.epoch_time = self.time
            self.evolve = f['evolve']
        self.log = open("../logs/neuro.log", 'w')
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

    def __normalize(self, dst):
        r = []
        for i in range(len(dst)):
            r.append(self.robot.vision_range[1] / dst[i])
        r = np.asarray(r).reshape(1, self.input_layer_size)
        return r

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
                self.robot.positon(self.origin[0], self.origin[1])
                self.robot.orientation = 0
                self.robot.stop()
                self.robot.acceleration = 0
                self.__build_network()
                return 0, 0
        inpt = self.__normalize(dst)
        pred = self.ann.predict(inpt)
        prob = self.ann.predict_proba(inpt)
        ang = prob[0][0]
        if ang > 180:
            ang = ang - 360
        spd = prob[0][1]
        out = (ang, spd)
        self.log_step(inpt, out)
        return out

    def log_step(self, dst, out):
        self.log.write("[" + str(time.time())+"]\tOdom: "+str(self.odom)+"\t-\tIn: "+str(dst)+"\t-\tOut: "+str(out)+"\n")

    def register_robot(self, r):
        super(Neuro_controller, self).register_robot(r)
        self.odom = (self.robot.x, self.robot.y)
        self.origin = (self.robot.x, self.robot.y)
        self.__build_network()

    def __build_network(self):
        if type(self.robot.sensors) is list:
            self.n_sonar = len(self.robot.sensors)
        else:
            self.n_sonar = self.robot.sensors
        self.input_layer_size = self.n_sonar
        hidden_layer_size = tuple(self.hidden_layer) # Layers are fully connected
                                # (1) defines one hidden layer with one neuron
        init_data = np.asarray([0 for _ in range(self.input_layer_size)])
        init_data = init_data.reshape(1, self.input_layer_size)
        self.ann = MLPClassifier(hidden_layer_sizes = hidden_layer_size,
                                 activation=self.activation, # You can try another activation function
                                 solver='adam', # This is not used at all
                                 warm_start = True,
                                 max_iter = 1)
        self.ann.fit(init_data, [[360, self.robot.max_speed]])
        self.ann.out_activation_ = 'identity'

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

    def set_network_params(self, weights):
        shapes = self.__calculate_shape(self.ann.coefs_)
        print(shapes)
        cutoff = []
        for i in range(len(shapes)):
            if cutoff:
                cutoff.append(shapes[i][0] * shapes[i][1] + cutoff[-1])
            else:
                cutoff.append(shapes[i][0] * shapes[i][1])
        print("Cutoff: {}".format(cutoff))

        w = []
        b = []
        for i in range(len(shapes)):
            b.append(np.asarray([0 for _ in range(shapes[i][1])]))
            if i > 0: # General case
                w.append(np.asarray(weights[cutoff[i-1]:cutoff[i]]).reshape(shapes[i]))
            else: # First position
                w.append(np.asarray(weights[:cutoff[i]]).reshape(shapes[i]))
            
        self.ann.coefs_ = w
        self.ann.intercepts_ = b

    def __calculate_shape(self, arr):
        s = []
        for a in arr:
            s.append(a.shape)
        return tuple(s)

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
