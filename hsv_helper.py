from typing import BinaryIO

import cv2
import numpy as np


def nothing(x):
    pass


class HsvSetter:
    hsv = np.asarray([])

    def __init__(self, hsv_dir):
        self.hsv = np.load('hsv_value.npy')
        print('hsv: ' + str(self.hsv))
        cv2.namedWindow("Trackbars")
        # Now create 6 trackbars that will control the lower and upper range of
        # H,S and V channels. The Arguments are like this: Name of trackbar,
        # window name, range,callback function. For Hue the range is 0-179 and
        # for S,V its 0-255.
        cv2.createTrackbar("L - H", "Trackbars", self.hsv[0][0], 179, nothing)
        cv2.createTrackbar("L - S", "Trackbars", self.hsv[0][1], 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", self.hsv[0][2], 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", self.hsv[1][0], 179, nothing)
        cv2.createTrackbar("U - S", "Trackbars", self.hsv[1][1], 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", self.hsv[1][2], 255, nothing)

    def get_hsv(self):
        # Flip the frame horizontally (Not required)
        # frame = cv2.flip(frame, 1)

        # Get the new values of the trackbar in real time as the user changes
        # them
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        # Set the lower and upper HSV range according to the value selected
        # by the trackbar
        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])
        self.hsv = np.asarray([lower_range, upper_range])
        np.save('hsv_value', self.hsv)

        return self.hsv

    def display_preview(self, frame):
        # Convert the BGR image to HSV image.
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Filter the image and get the binary mask, where white represents
        # your target color
        lower_range, upper_range = self.hsv[0], self.hsv[1]
        mask = cv2.inRange(hsv_image, lower_range, upper_range)

        # You can also visualize the real part of the target color (Optional)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Converting the binary mask to 3 channel image, this is just so
        # we can stack it with the others
        mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # stack the mask, orginal frame and the filtered result
        stacked = np.hstack((mask_3, res))
        cv2.imshow('Trackbars', cv2.resize(stacked, None, fx=0.4, fy=0.4))

        key = cv2.waitKey(1)
        if key == ord('h'):
            print("hsv_arr saved!")
            np.save('hsv_value', self.hsv)