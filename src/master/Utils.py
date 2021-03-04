import time
import numpy as np

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


def TransformPos(pos_2d, frame_offset_2d, frame_angle):

    frame_angle = frame_angle * np.pi/180.0
    # Make a transform
    tf = np.array([[ np.cos(frame_angle), -np.sin(frame_angle), frame_offset_2d[0] ],
                   [ np.sin(frame_angle),  np.cos(frame_angle), frame_offset_2d[1] ],
                   [0, 0, 1 ]])

    pos_2d = np.concatenate((pos_2d,[1])).reshape((3,))
    new_pos = np.matmul(tf, pos_2d)
    return new_pos[:2].reshape((2,))



if __name__ == '__main__':

    test_pos = np.array([1,1])
    test_offset = np.array([2,2])
    test_angle = 90
    expected_pos = np.array([1,3])
    new_pos = TransformPos(test_pos, test_offset, test_angle)
    print("Actual: {}, expected: {}".format(new_pos, expected_pos))