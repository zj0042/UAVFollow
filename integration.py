from djitellopy import Tello
import cv2
import pygame
import numpy as np
import os
import time

from imutils.video import fps

from yolo_detection import YoloTracker
from bangbang_motion_control import MotionController as NewMotionController
from pid_motion_control import MotionController as NewMotionController
from old_motion_control import MotionController as OldMotionController
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

        self.notecard_tracker = YoloTracker(config['yolo_tracker'])

        self.new_motion_controller = NewMotionController()
        self.old_motion_controller = OldMotionController()

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
        self.tello.connect()
        self.tello.set_speed(self.speed)

        # In case streaming is on. This happens when we quit this program without the escape key.
        self.tello.streamoff()
        self.tello.streamon()
        # self.tello.send_keepalive()
        frame_read = self.tello.get_frame_read()

        should_stop = False

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

            self.screen.fill([0, 0, 0])

            frame = frame_read.frame
            frame, rect = self.notecard_tracker.get_rect(frame)
            print(f"obj pixels: x:{rect[0]}\t y:{rect[1]}\t size:{rect[2]})")

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Commented out to color correct

            dtime = time.time() - last_time
            v = self.new_motion_controller.instruct(rect, dtime)
            old_v = self.old_motion_controller.instruct(rect, time)
            for i in range(len(v)):
                if v[i] is None:
                    v[i] = old_v[i]

            print(f"left/right: {v[0]}\t up/down: {v[1]}\t forward/back: {v[2]}")

            if self.tello.is_flying:
                v = list(map(int, v))
                self.vx, self.vy, self.vz = v

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
