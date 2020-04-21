from math import *
import numpy as np 

r = 100.0 / 1000
theta = 110.0
L = 325.0 / 1000

theta = theta / 180.0 * pi 


new_pos = [r/2 + L*cos(theta - pi/2) - r/2*sin(theta-pi/2), -L - L*sin(theta - pi/2) - r/2*cos(theta-pi/2)]

print new_pos 
