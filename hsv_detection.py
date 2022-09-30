# USAGE
# python object_tracking.py --video object_tracking_example.mp4
# python object_tracking.py

# import the necessary packages
from collections import deque
import numpy as np
from hsv_helper import HsvSetter
import cv2
import imutils
import time


class HsvTracker:
    def __init__(self, config):
        self.circle = ()
        self.hsv_helper = HsvSetter(config['HSV_DIR'])

    def get_rect(self, frame):
        hsv_value = self.hsv_helper.get_hsv()

        if frame is None:
            pass
        # resize the frame, blur it, and convert it to the HSV
        # color space
        # frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv_temp = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv_temp, hsv_value[0], hsv_value[1])
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            self.circle = (x, y, radius * 2)
            # only proceed if the radius meets a minimum size
            if radius > 10:
                frame = cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

        return frame, self.circle
