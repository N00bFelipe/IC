import numpy as np

WIDTH = 800
HEIGHT = 600
FPS = 90
NUMROB = 100
SIZEROBOT = 5
DELTAT = 0.1 #1
MARGIN = 100 #200
TURN_FACTOR = 1 #1
VELMAX = 30 #15
SENSOR_ANGLE = np.deg2rad(60) #180
SENSOR_RANGE_MAX = 60 #75
SENSOR_RANGE_MIN = 20
BIAS = 0.07
CENTERING_FACTOR = 0.0008 #0.005
MATCHING_FACTOR = 0.03 #0.05
AVOID_FACTOR = 0.08 #0.05