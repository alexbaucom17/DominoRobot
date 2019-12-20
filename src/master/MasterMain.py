import os
import pickle
import time
import copy
import sys
import PySimpleGUI as sg
import config
import FieldPlanner
from MarvelMindHandler import RobotPositionHandler
from RobotClient import RobotClient


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
        # self.pos_handler = RobotPositionHandler(cfg)
        # self.pos_handler.wake_robot(robot_id)

        # Setup robot clients and communication
        #self.robot_client = RobotClient(cfg, robot_id)

        # Setup other miscellaneous variables
        self.rate = Rate(20)
        self.cfg = cfg

        # Wait a little while for setup to finish
        # sleep_time = 30
        # print('Waiting {} seconds for beacons to fully wake up'.format(sleep_time))
        # time.sleep(sleep_time)

        # Make GUI
        sg.change_look_and_feel('BlueMono')

        layout = [[sg.Text('Previous command:'), sg.Text(size=(12,1), key='_OUTPUT_')],
                [sg.Text('Command:'), sg.Input(key='_IN_')],
                [sg.Button('Send Command'), sg.Button('Exit')]]

        self.window = sg.Window('Robot Controller', layout, return_keyboard_events=True)

        print("Starting loop")

    def cleanup(self):

        print("Cleaning up")
        # self.pos_handler.sleep_robot(1)
        # self.pos_handler.close()

    def checkNetworkStatus(self):
        net_status = self.robot_client.net_status()
        if net_status:
            print("Network status of Robot 1 is GOOD")
        else:
            print("Network status of Robot 1 is BAD")

    def handle_input(self, data):

        if "move" in data:
            start_idx = data.find('[')
            end_idx = data.find(']')
            vals = data[start_idx+1:end_idx].split(',')
            vals = [x.strip() for x in vals]
            print("Got move command with parameters [{},{},{}]".format(vals[0], vals[1], vals[2]))
            #self.robot_client.move(float(vals[0]), float(vals[1]), float(vals[2]))
        elif "net" in data:
            #self.checkNetworkStatus()
            pass
        else:
            print("Unknown command: {}".format(data))

    def update_gui(self):

        event, values = self.window.read()
        #print(event, values)
        if event is None or event == 'Exit':
            return True
        if event in ('Send Command', '\r', '\n'): #, '\r', QT_ENTER_KEY1, QT_ENTER_KEY2):
            command = values['_IN_']
            self.window['_OUTPUT_'].update(command)
            self.window['_IN_'].update("")
            self.handle_input(command)

        return False


    def loop(self):

        while True:
            
            #self.check_input()
            #self.pos_handler.service_queues()
            
            done = self.update_gui()
            if done:
                break

            self.rate.sleep()

        self.cleanup()
        self.window.close()





if __name__ == '__main__':
    cfg = config.Config()
    m = Master(cfg)
    m.loop()

