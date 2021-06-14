import time
import numpy as np
import math
import enum

class ActionTypes(enum.Enum):
    MOVE_COARSE = 0,
    MOVE_FINE = 1,
    MOVE_REL = 2,
    NET = 3,
    LOAD = 4,
    PLACE = 5,
    TRAY_INIT = 6, 
    LOAD_COMPLETE = 7,
    ESTOP = 8,
    WAIT_FOR_LOCALIZATION = 9, 
    MOVE_CONST_VEL = 10,
    CLEAR_ERROR = 11,
    NONE = 12,
    SET_POSE = 13,
    MOVE_WITH_VISION = 14,
    TOGGLE_VISION_DEBUG = 15,
    START_CAMERAS = 16,
    STOP_CAMERAS = 17,

class NonBlockingTimer:

    def __init__(self, trigger_time):
        self.start_time = time.time()
        self.trigger_time = trigger_time

    def check(self):
        if time.time() - self.start_time > self.trigger_time:
            return True 
        else:
            return False

def write_file(filename, text):
    with open(filename, 'w+') as f:
        f.write(text)


# Pos and offset expected as 2 long numpy vector, Angle is expected in degrees
# Returns 2 length numpy vector
def TransformPos(pos_2d, frame_offset_2d, frame_angle):

    frame_angle = math.radians(frame_angle)
    # Make a transform
    tf = np.array([[ np.cos(frame_angle), -np.sin(frame_angle), frame_offset_2d[0] ],
                   [ np.sin(frame_angle),  np.cos(frame_angle), frame_offset_2d[1] ],
                   [0, 0, 1 ]])

    pos_2d = np.concatenate((pos_2d,[1])).reshape((3,))
    new_pos = np.matmul(tf, pos_2d)
    return new_pos[:2].reshape((2,))


class ActionValidator:

    def __init__(self):
        self.expected_action = str(ActionTypes.NONE)
        self.action_validated = False

    def update_expected_action(self, expected_action):
        self.expected_action = str(expected_action)
        self.action_validated = False

    def update_action_validation(self, metric):
        if metric is not None and \
           'in_progress' in metric and \
           'current_action' in metric and \
            metric['current_action'] == self.expected_action:
            if self.expected_action in [ActionTypes.START_CAMERAS, ActionTypes.STOP_CAMERAS]:
                self.action_validated = True
            elif metric['in_progress'] == True:
                self.action_validated = True
        return self.action_validated




if __name__ == '__main__':

    test_pos = np.array([1,1])
    test_offset = np.array([2,2])
    test_angle = 90
    expected_pos = np.array([1,3])
    new_pos = TransformPos(test_pos, test_offset, test_angle)
    print("Actual: {}, expected: {}".format(new_pos, expected_pos))