from math import *
import numpy as np

L = 0.325
theta = 85.0 / 180 * pi
b = 60.0 / 1000

coor = [b+L*cos(pi/2 - theta)+b*sin(pi/2 - theta), -L + L*sin(pi/2 - theta) - b*cos(pi/2 - theta)]

print coor
