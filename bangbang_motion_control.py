import numpy as np


class MotionController:
    def instruct(self, rect, dtime):
        # -480 <= obj_x <= 480, negative: left side of the frame, positive: right side of the frame
        error_x = rect[0] - 960/2

        # -360 <= obj_y <= 360, negative: bottom side of the frame, positive: top side of the frame
        error_y = rect[1] - 720/2

        # distance from the corners to the center, in number of pixels
        error_size = rect[2]

        # these variables represent the velocities of the drone
        vx = 0  # -100 <= vx <= 100, negative:left, positive:right
        vy = 0  # -100 <= vy <= 100, negative:up,   positive:down
        vz = 0  # -100 <= vz <= 100, negative:back, positive:forward

        '''
        your task is to modify the vx, vy, vz variables based on obj_x, obj_y, obj_size
        there are many ways you can do this, but some common ways include using 
        INSERT YOUR CODE BELOW:
        '''


        return np.array([vx, vy, vz]).astype(int)
