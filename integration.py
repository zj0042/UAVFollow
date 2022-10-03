from djitellopy import Tello
import cv2
import pygame
import pandas as pd
import numpy as np
import os
import time

from imutils.video import fps

from yolo_detection import YoloTracker
from hsv_detection import HsvTracker
from motion_control import MotionController
from data_logger import Logger
from configparser import ConfigParser

config = ConfigParser()
config.read('config.ini')

# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
PYGAME_FPS = int(config['control']['PYGAME_FPS'])
LOG = config.getboolean('control', 'LOG')
CONTROL_METHOD = config['control']['CONTROL_METHOD']  # 'yolo' or 'hsv'

SAVE_FRAMES = False
MAX_CSV_LENGTH = int(config['data_collection']['MAX_CSV_LENGTH'])
SAVE_DIR = config['data_collection']['SAVE_DIR']


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - RETURN: Takeoff
            - SPACE: Land
    """
    handControl = True

    def __init__(self):
        # Init pygame
        pygame.init()

        # Create pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([int(960 * 0.9), int(720 * 0.9)])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.vz = 0
        self.vx = 0
        self.vy = 0
        self.vyaw = 0
        self.v = 0.0, 0.0, 0.0  # yaw, up/down, forward/backward
        self.speed = 10  # do not change this
        self.recording = False

        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // PYGAME_FPS)

        if CONTROL_METHOD == 'yolo':
            self.yolo_tracker = YoloTracker(config['yolo_tracker'])
        else:
            self.hsv_tracker = HsvTracker(config['hsv_tracker'])

        self.motion_controller = MotionController()

        if LOG:
            self.logger = Logger(obj_plot=False, drone_plot=False)

        if SAVE_FRAMES:
            if not os.path.isdir(SAVE_DIR):
                os.mkdir(SAVE_DIR)
            if not os.path.isdir(SAVE_DIR + '/img'):
                os.mkdir(SAVE_DIR + '/img')
            self.img_counter = max(
                [int(e.split('.')[0]) for e in os.listdir(SAVE_DIR + '/img') if e[0] != '.'] + [0]) + 1
            self.counter_start = self.img_counter

    def process_frame(self, frame):
        # Displaying battery
        battery_text = "Battery: {}%".format(self.tello.get_battery())
        frame = cv2.putText(frame, battery_text, (5, 720 - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        frame = cv2.putText(frame, "velocities: " + str(self.v), (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2)
        frame = cv2.putText(frame, f'saving frames: {SAVE_FRAMES}', (5, 720-45), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        frame = np.rot90(frame)
        frame = np.flipud(frame)

        frame = cv2.resize(frame, (int(frame.shape[1] * 0.9), int(frame.shape[0] * 0.9)))
        frame = pygame.surfarray.make_surface(frame)
        return frame

    def run(self):
        global SAVE_FRAMES
        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()
        # self.tello.send_keepalive()
        frame_read = self.tello.get_frame_read()

        should_stop = False

        dat_arr = []

        while not should_stop:
            last_time = time.time()
            if frame_read.stopped:
                break

            for event in pygame.event.get():
                if event.type == pygame.USEREVENT + 1:
                    self.update()
                elif event.type == pygame.QUIT:
                    break
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_RETURN:
                        self.tello.takeoff()
                        self.motion_controller.clear_data()
                        self.send_rc_control = True
                    elif event.key == pygame.K_SPACE:
                        should_stop = True
                    elif event.key == pygame.K_r:
                        SAVE_FRAMES = not SAVE_FRAMES
                        if not SAVE_FRAMES:
                            pd.DataFrame(dat_arr, columns=['img name', 'control method', 'x', 'y', 'radius', 'vx', 'vy',
                                                           'vz']).to_csv(
                                f'{SAVE_DIR}/{self.counter_start}-{self.img_counter}.csv')
                            dat_arr = []
                            self.counter_start = self.img_counter

            self.screen.fill([0, 0, 0])

            orig_frame = frame_read.frame
            if CONTROL_METHOD == 'yolo':
                frame, rect = self.yolo_tracker.get_rect(orig_frame)
            else:
                frame, rect = self.hsv_tracker.get_rect(orig_frame)

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Commented out to color correct

            dtime = time.time() - last_time
            self.motion_controller.add_location(rect, dtime)
            self.v = self.motion_controller.instruct(diagnostic=False)

            if self.tello.is_flying:
                self.v = list(map(int, self.v))
                self.vx, self.vy, self.vz = self.v

            if SAVE_FRAMES:
                if rect is not None and len(rect) != 0:
                    cv2.imsave(SAVE_DIR + f'/img/{self.img_counter}.jpg', frame)
                    arr = [f'{self.img_counter}.jpg', CONTROL_METHOD, rect[0], rect[1], rect[2], self.vx, self.vy,
                           self.vz]
                    dat_arr.append(arr)
                    if len(dat_arr) == MAX_CSV_LENGTH:
                        pd.DataFrame(dat_arr, columns=['img name', 'control method', 'x', 'y', 'radius', 'vx', 'vy',
                                                       'vz']).to_csv(
                            f'{SAVE_DIR}/{self.counter_start}-{self.img_counter}.csv')
                        dat_arr = []
                        self.counter_start = self.img_counter + 1
                    self.img_counter += 1

            # logging data
            if LOG:
                self.logger.update_drone(np.array([self.vx, self.tello.get_speed_x()]),
                                         np.array([self.vy, self.tello.get_speed_y()]),
                                         np.array([self.vz, self.tello.get_speed_z()]))
                self.logger.update_obj(self.motion_controller.get_x_info(),
                                       self.motion_controller.get_y_info(),
                                       self.motion_controller.get_z_info())

            frame = self.process_frame(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()
            time.sleep((1 / PYGAME_FPS) - (time.time() - last_time))

        self.tello.land()
        self.send_rc_control = False
        self.tello.end()

    def update(self):
        if not self.send_rc_control:
            return
        self.tello.send_rc_control(self.vx, self.vz,
                                   self.vy, self.vyaw)


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
