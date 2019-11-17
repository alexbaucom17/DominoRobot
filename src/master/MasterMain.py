import FieldPlanner
import config
import os
import pickle
from MarvelMindHandler import RobotPositionHandler
import time
from RobotClient import RobotClient
import msvcrt
import copy
import sys


def doWaypointGeneration(cfg, draw = False):
    print('Generating domino field from image...',end='',flush=True)
    field = FieldPlanner.generateField(cfg)
    print('done.')

    print('Generating tiles from field...',end='',flush=True)
    tiles = field.generateTiles()
    print('done.')

    print('Generating robot paths...',end='',flush=True)
    waypoints = FieldPlanner.generateWaypoints(tiles, cfg)
    print('done.')

    print('Generating tile build instructions...',end='',flush=True)
    #build_instructions = tiles.generateBuildInstructions()
    print('done.')

    if draw:
        print('Generating output diagrams:')
        #field.printStats()
        #field.show()
        #tiles.draw()
        #tiles.show_ordering()
        waypoints.drawWaypoints(40)

    return waypoints

class Rate:

    def __init__(self, rate):
        """
        Smart sleeping to maintain rate (Hz)
        """
        self.sleep_time = 1.0/rate
        self.prev_time = None

    def sleep(self):
        if self.prev_time:
            dt = time.monotonic() - self.prev_time
            des_sleep_time = self.sleep_time - dt
            if des_sleep_time > 0:
                time.sleep(des_sleep_time)
        
        self.prev_time = time.monotonic()


class InputHandler:

    def __init__(self):
        self.in_str = ""
        self.ready = False

    def service(self):
        if msvcrt.kbhit():
            new_key = msvcrt.getche()
            # print("Got: {}".format(new_key))
            if new_key == b'\r':
                self.ready = True                
            else:
                self.in_str += new_key.decode('UTF-8')

    def get_data(self):
        if self.ready:
            out_str = copy.copy(self.in_str)
            self.in_str = ""
            self.ready = False
        else:
            out_str = ""
        return out_str

    

class Master:

    def __init__(self, cfg):

        print("Initializing Master")

        # Handle initial generation or loading of waypoints
        if os.path.exists(cfg.plan_file):
            with open(cfg.plan_file, 'rb') as f:
                self.waypoints = pickle.load(f)
                print("Loaded waypoint data from {}".format(cfg.plan_file))
        else:
            self.waypoints = doWaypointGeneration(cfg)
            with open(cfg.plan_file, 'wb') as f:
                pickle.dump(self.waypoints, f)
                print("Saved waypoint data to {}".format(cfg.plan_file))

        # robot ID - TODO handle multiple
        robot_id = 1

        # Setup marvelmind devices and position handler
        self.pos_handler = RobotPositionHandler()
        self.pos_handler.wake_robot(robot_id)

        # Setup robot clients and communication
        self.robot_client = RobotClient(robot_id)

        # Setup other miscellaneous variables
        self.rate = Rate(20)
        self.input_handler = InputHandler()
        self.cfg = cfg

        # Wait a little while for setup to finish
        sleep_time = 30
        print('Waiting {} seconds for beacons to fully wake up'.format(sleep_time))
        time.sleep(sleep_time)

        print("Starting loop")

    def cleanup(self):

        print("Cleaning up")
        self.pos_handler.sleep_robot(1)
        self.pos_handler.close()

    def check_input(self):

        self.input_handler.service()
        data = self.input_handler.get_data()
        if data:
            if "move" in data:
                start_idx = data.find('[')
                end_idx = data.find(']')
                vals = data[start_idx+1:end_idx].split(',')
                vals = [x.strip() for x in vals]
                print("Got move command with parameters [{},{},{}]".format(vals[0], vals[1], vals[2]))
                self.robot_client.move(float(vals[0]), float(vals[1]), float(vals[2]))
            elif "quit" in data:
                self.cleanup()
                sys.exit()


    def loop(self):

        while True:
            self.check_input()
            #self.pos_handler.service_queues()
            self.rate.sleep()





if __name__ == '__main__':
    cfg = config.Config()
    m = Master(cfg)
    m.loop()

