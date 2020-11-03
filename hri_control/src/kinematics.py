import numpy as np
import math


def c(number):
    return math.cos(number)

def s(number):
    return math.sin(number)

def angleToCP(angles):
    tB1 = np.array([[c(angles[0]), -s(angles[0]), 0, 0],
                    [-s(angles[0]), -c(angles[0]), 0, 0],
                    [0, 0, -1, 0.1564],
                    [0, 0, 0, 1]])

    t12 = np.array([[c(angles[1]), -s(angles[1]), 0, 0],
                    [0, 0, -1, 0.0054],
                    [s(angles[1]), c(angles[1]), 0, -0.1284],
                    [0, 0, 0, 1]])

    t23 = np.array([[c(angles[2]), -s(angles[2]), 0, 0],
                    [0, 0, 1, -0.2104],
                    [-s(angles[2]), -c(angles[2]), 0, -0.0064],
                    [0, 0, 0, 1]])

    t34 = np.array([[c(angles[3]), -s(angles[3]), 0, 0],
                    [0, 0, -1, -0.0064],
                    [s(angles[3]), c(angles[3]), 0, -0.2104],
                    [0, 0, 0, 1]])

    t45 = np.array([[c(angles[4]), -s(angles[4]), 0, 0],
                    [0, 0, 1, -0.2084],
                    [-s(angles[4]), -c(angles[4]), 0, -0.0064],
                    [0, 0, 0, 1]])

    t56 = np.array([[c(angles[5]), -s(angles[5]), 0, 0],
                    [0, 0, -1, 0],
                    [s(angles[5]), c(angles[5]), 0, -0.1059],
                    [0, 0, 0, 1]])

    t67 = np.array([[c(angles[6]), -s(angles[6]), 0, 0],
                    [0, 0, 1, -0.1059],
                    [-s(angles[6]), -c(angles[6]), 0, 0],
                    [0, 0, 0, 1]])

    t7E = np.array([[1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, -0.0615],
                    [0, 0, 0, 1]])

    tBE = tB1.dot(t12).dot(t23).dot(t34).dot(t45).dot(t56).dot(t67).dot(t7E)

    return tBE[0:3,-1]


