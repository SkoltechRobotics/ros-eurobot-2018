# Point is always the x, y coordinate of object and anlge of rotation of its frame
import numpy as np

def cvt_local2global(local_point, sc_point):
    point = np.zeros(3)
    x, y, a = local_point
    X, Y, A = sc_point
    point[0] = x * np.cos(A) - y * np.sin(A) + X
    point[1] = x * np.sin(A) + y * np.cos(A) + Y
    point[2] = a + A
    return point


class EncoderIntegrator(object):
    def __init__(self, start_point):
        self.last_point = start_point

    def integrate(self, dpoint):
        self.last_point = cvt_local2global(dpoint, self.last_point)
        return self.last_point
