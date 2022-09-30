from tkinter import *  # don't delete this (necessary for matplotlib)
import matplotlib.pyplot as plt
import numpy as np

MAX_SIZE = 100


class Logger:
    def __init__(self, obj_plot=True, drone_plot=True):
        tmp = np.zeros((MAX_SIZE, 3))
        self.obj_x_info, self.obj_y_info, self.obj_z_info = tmp.copy(), tmp.copy(), tmp.copy()
        tmp = np.zeros((MAX_SIZE, 2))
        self.dr_x_info, self.dr_y_info, self.dr_z_info = tmp.copy(), tmp.copy(), tmp.copy()
        self.t = np.linspace(-MAX_SIZE, 0, num=MAX_SIZE, endpoint=False)

        self.OBJ_PLOT = obj_plot
        self.DR_PLOT = drone_plot

        num_cols = int(self.OBJ_PLOT) + int(self.DR_PLOT)
        if self.OBJ_PLOT:
            self.obj_x_plot = plt.subplot(3, num_cols, 1)
            self.obj_y_plot = plt.subplot(3, num_cols, 1+num_cols)
            self.obj_z_plot = plt.subplot(3, num_cols, 1+num_cols*2)
            self.obj_x_plot.set_title('OBJ X')
            self.obj_y_plot.set_title('OBJ Y')
            self.obj_z_plot.set_title('OBJ Z')
            self.update_obj_graph()
        if self.DR_PLOT:
            self.drone_x_plot = plt.subplot(3, num_cols, 1 * (1+int(self.OBJ_PLOT)))
            self.drone_y_plot = plt.subplot(3, num_cols, 2 * (1+int(self.OBJ_PLOT)))
            self.drone_z_plot = plt.subplot(3, num_cols, 3 * (1+int(self.OBJ_PLOT)))
            self.drone_x_plot.set_title('DRONE X')
            self.drone_y_plot.set_title('DRONE Y')
            self.drone_z_plot.set_title('DRONE Z')
            self.update_drone_graph()

    def update_drone(self, x, y, z):
        self.dr_x_info[:-1] = self.dr_x_info[1:]
        self.dr_x_info[-1] = x
        self.dr_y_info[:-1] = self.dr_y_info[1:]
        self.dr_y_info[-1] = y
        self.dr_z_info[:-1] = self.dr_z_info[1:]
        self.dr_z_info[-1] = z
        if self.DR_PLOT:
            self.update_drone_graph()

    def update_obj(self, x, y, z):
        self.obj_x_info[:-1] = self.obj_x_info[1:]
        self.obj_x_info[-1] = x
        self.obj_y_info[:-1] = self.obj_y_info[1:]
        self.obj_y_info[-1] = y
        self.obj_z_info[:-1] = self.obj_z_info[1:]
        self.obj_z_info[-1] = z
        if self.OBJ_PLOT:
            self.update_obj_graph()

    def update_drone_graph(self):
        if not self.DR_PLOT:
            return
        xt, yt, zt = self.dr_x_info.T, self.dr_y_info.T, self.dr_z_info.T
        self.drone_x_plot.clear()
        self.drone_y_plot.clear()
        self.drone_z_plot.clear()
        for i, mode in enumerate(['desired', 'actual']):
            self.drone_x_plot.plot(self.t, xt[i], label=mode)
            self.drone_y_plot.plot(self.t, yt[i], label=mode)
            self.drone_z_plot.plot(self.t, zt[i], label=mode)
        self.drone_x_plot.legend()
        self.drone_y_plot.legend()
        self.drone_z_plot.legend()
        plt.draw()
        plt.pause(0.001)

    def update_obj_graph(self):
        if not self.OBJ_PLOT:
            return
        xt, yt, zt = self.obj_x_info.T, self.obj_y_info.T, self.obj_z_info.T
        self.obj_x_plot.clear()
        self.obj_y_plot.clear()
        self.obj_z_plot.clear()
        for i, mode in enumerate(['P', 'I', 'D']):
            self.obj_x_plot.plot(self.t, xt[i], label=mode)
            self.obj_y_plot.plot(self.t, yt[i], label=mode)
            self.obj_z_plot.plot(self.t, zt[i], label=mode)
        self.obj_x_plot.legend()
        self.obj_y_plot.legend()
        self.obj_z_plot.legend()
        plt.draw()
        plt.pause(0.001)
