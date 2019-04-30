#!/usr/bin/env python

""" Helper benchmarking module. Standalone.

Standalone used to help benchmark the simulation's behavior based on logged data.
It can be used for online or offline benchmarking, as it regularly reads new data
and updates itself accordingly. Not the fastest, but a good, simple tool to help
debug.

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
__date__ = "2019/03/18"
__deprecated__ = False
__email__ =  "mario.cobos@edu.uah.es"
__license__ = "GPLv3"
__maintainer__ = "Mario Cobos Maestre"
__status__ = "Development"
__version__ = "0.0.1"

import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

labels = []


def read(logfile):
    global labels
    with open(logfile) as f:
        data = []
        labels = []
        while True:
            line = f.readline()
            time.sleep(0.1)
            if line:
                labels.append(line.split(',')[0])
                data.append(float(line.split(',')[1]))
                yield data

def trim_zeroes(values):
    global labels
    for i in range(len(values)):
        if values[i] <= 0.0:
            labels.pop(i)
            values.pop(i)
    return values

def animate(values):
    x = list(range(len(trim_zeroes(values))))
    line.set_data(x, values)
    ax.set_xlim(x[0], x[-1])
    ax.set_ylim(min(values), max(values))
    for i in zip(x, [values[j] for j in range(len(x))]):
        ax.annotate(labels[x.index(i[0])], xy=i, textcoords='data')
    return line,

fig, ax = plt.subplots()
line, = ax.plot([])


ani = FuncAnimation(fig, animate, frames=read('../logs/robot_0_benchmark.log'), interval=10)
plt.show()
