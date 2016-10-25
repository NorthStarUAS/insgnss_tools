#!/usr/bin/python

import math
import numpy as np
import matplotlib.pyplot as plt
import random

def f(coord, x):
    r = 1.0
    sx = math.sin(x)
    sx2 = math.sin(x*x)
    cx = math.cos(x)
    cx2 = math.cos(x*x)
    result = coord[0] * sx + coord[1] * cx + coord[2] * sx * cx + coord[3] * sx2 + coord[4] * cx2
    return result + random.random() * r

# generate 'noisy' sample data
xdata = []
ydata = []
coord = [ 5, 4, 3, 2, 1 ]
for x in np.linspace(0, 10, 1000):
    y = f(coord, x)
    print x, y
    xdata.append(x)
    ydata.append(y)

from scipy.optimize import minimize

plt.plot(xdata, ydata)
plt.ylabel('some function')
plt.show()
