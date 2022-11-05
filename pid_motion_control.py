import numpy as np


class MotionController:
    def __init__(self):
        self.error = 0  # proportional
        self.error_sum = 0  # integral
        self.error_delta = 0  # derivative

    def instruct(self, rect, dtime):
        # -480 <= obj_x <= 480, negative: left side of the frame, positive: right side of the frame

        error = rect[0] - 960/2
        error = error / 480

        '''
        -1 <= error <= 1, 
        -1 implies that the object is at the left edge of the frame while 1 implies that the object is at the right edge
        '''

        self.error_sum = self.error_sum + error * dtime
        self.error_delta = (self.error - error) / dtime
        self.error = error

        '''
        Calculate vx using either the error, the delta of the error, or the sum of the error, or all of them
        '''
        vx = 100 * error

        return np.array([vx, None, None]).astype(int)