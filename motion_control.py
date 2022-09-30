"""
PURPOSE: given the object's relative location in the x(horizontal), y(vertical), and z(distance), return the desired
velocity of the drone in these three dimensions
"""

import numpy as np
from math import radians, tan


def cot(x):
    return 1/tan(x)


FRAME_WIDTH = 960
FRAME_HEIGHT = 720

Kx = np.array([0.7, 0.5, 0.20]) * 1  # P, I, D constants, 1/0 is on/off switch
Ky = np.array([2.0, 0.5, 0.25]) * 1  # P, I, D constants
Kz = np.array([1.0, 0.5, 0.2]) * 1  # P, I, D constants
# side note: i dont remember why I made the Derivative constants negative... it makes more sense if it was positive
# more experimentation needed with positive D constant

MAX_SPEED = 100  # max speed of the drone that will be assigned

FADE_COEFFICIENT = 1 / 3  # the previous frame is weighted 1/3 of the current


class MotionController:
    def clear_data(self):
        self.x = self.y = self.z = 0.0
        self.dx = self.dy = self.dz = 0.0
        self.ix = self.iy = self.iz = 0.0

    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.dx = self.dy = self.dz = 0.0
        self.ix = self.iy = self.iz = 0.0
        self.DIST_TO_PIX = 1 / 12  # the ratio between distance (cm) and the number of pixels
        self.I_MAX = 25 / self.DIST_TO_PIX / Kx[1]  # maximum value for the integral component to prevent blow-ups

    def __update_params(self, x, y, z, dx, dy, dz, ix, iy, iz):
        self.x = x
        self.y = y
        self.z = z
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.ix = ix
        self.iy = iy
        self.iz = iz

    def __update_params_tuple(self, xyz, ixyz, dxyz):
        self.x, self.y, self.z = tuple(xyz)
        self.dx, self.dy, self.dz = tuple(dxyz)
        self.ix, self.iy, self.iz = tuple(ixyz)

    def add_location(self, rect, dtime):
        if rect is None or len(rect) == 0:
            return

        # rect_np is the numpy array of the object's location in the current frame
        rect_np = np.array(rect)
        rect_np[0] = rect_np[0] - FRAME_WIDTH / 2
        rect_np[1] = rect_np[1] - FRAME_HEIGHT / 2
        rect_np[2] = rect_np[2] * (cot(radians(82.6) * rect_np[2] / 960) - cot(radians(10)))
        # angle is the field of view in radian * proportion of the object on screen
        # the use of cotangent is explained more in the email I sent to Dr Fu in the beginning of the summer

        # the xyz's are first set to the location of the object in the previous frame
        xyz = np.array([self.x, self.y, self.z])
        dxyz = np.array([self.dx, self.dy, self.dz])
        ixyz = np.array([self.ix, self.iy, self.iz])

        # merges the locations from the previous frame with the current
        if not self.x == self.y == self.z == 0.0:
            dxyz = (FADE_COEFFICIENT*dxyz + (rect_np - xyz) / dtime) / (1+FADE_COEFFICIENT)

        xyz = (FADE_COEFFICIENT * xyz + rect_np) / (1 + FADE_COEFFICIENT)

        ixyz = np.add(ixyz, rect_np * dtime)
        ixyz = np.clip(ixyz, -self.I_MAX, self.I_MAX)

        self.__update_params_tuple(xyz, ixyz, dxyz)

    # returns the desired speed in the form of a tuple (x, y, z)
    def instruct(self, diagnostic=False):
        dx_drone = float(np.sum(np.multiply(Kx, np.array([self.x, self.ix, self.dx]))))
        dy_drone = float(np.sum(np.multiply(Ky, np.array([self.y, self.iy, self.dy]))))
        dz_drone = float(np.sum(np.multiply(Kz, np.array([self.z, self.iz, self.dz]))))

        if diagnostic:
            print('PID', self.x, self.ix, self.dx)
            print(list(np.multiply(Kx, np.array([self.x, self.ix, self.dx]))))
        ret = np.array([dx_drone, -dy_drone, dz_drone]) * self.DIST_TO_PIX
        ret = np.clip(ret, -MAX_SPEED, MAX_SPEED)
        return ret.astype(int)

    def get_obj_displacement(self):
        return np.array([self.x, self.y, self.z])

    def get_obj_velocity(self):
        return np.array([self.dx, self.dy, self.dz])

    def get_obj_integral(self):
        return np.array([self.ix, self.iy, self.iz])

    def get_x_info(self):
        return np.array([self.x, self.ix, self.dx])

    def get_y_info(self):
        return np.array([self.y, self.iy, self.dy])

    def get_z_info(self):
        return np.array([self.z, self.iz, self.dz])
