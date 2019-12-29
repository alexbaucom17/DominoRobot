import os
import pickle
import time
import copy
import sys
import math
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

class CmdGui:

    def __init__(self):
        sg.change_look_and_feel('Dark Blue 3')

        col1 = [[sg.Text('Command:'), sg.Input(key='_IN_')], [sg.Button('Send'), sg.Button('Exit')]]
        col2 = [[sg.Text("Robot 1 status:")],
               [sg.Text("Robot 1 offline",size=(30, 10),relief=sg.RELIEF_RAISED,key='_R1STATUS_')]]
#sg.Output(size=(100, 15)),
        layout = [[sg.Graph(canvas_size=(750,750),graph_bottom_left=(0,0), graph_top_right=(10, 10), key="_GRAPH_", background_color="white")  ],
                   [sg.Column(col1), sg.Column(col2)] ]

        self.window = sg.Window('Robot Controller', layout, return_keyboard_events=True)
        self.window.finalize()

        self.draw_robot(3,3,0)


    def update(self):

        event, values = self.window.read(timeout=20)
        command = None
        if event is None or event == 'Exit':
            return True, None
        if event in ('Send Command', '\r', '\n'): # Catch enter as well
            command = values['_IN_']
            self.window['_IN_'].update("")

        return False, command

    def update_robot_status(self, status_dict):
        status_str = "Cannot get robot status"
        if status_dict:
            try:
                status_str = ""
                status_str += "Position: [{}, {}, {}]\n".format(status_dict['pos_x'],status_dict['pos_y'], status_dict['pos_a'])
                status_str += "Velocity: [{}, {}, {}]\n".format(status_dict['vel_x'],status_dict['vel_y'], status_dict['vel_a'])
                status_str += "Controller timing: {}\n".format(status_dict['controller_loop_ms'])
                status_str += "Position timing:   {}\n".format(status_dict['position_loop_ms'])
                status_str += "Counter:   {}\n".format(status_dict['counter'])
                status_str += "Free memory:   {}\n".format(status_dict['free_memory'])

                # Also update the visualization position
                self.update_robot_viz_position(status_dict['pos_x'],status_dict['pos_y'], status_dict['pos_a'])
            except Exception:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_R1STATUS_'].update(status_str)

    def update_robot_viz_position(self, x, y, a):
        if self.r1_figs:
            for f in self.r1_figs:
                self.window['_GRAPH_'].DeleteFigure(f)
        
        self.draw_robot(x, y, a)

    def draw_robot(self, x, y, a):
        robot_length = 1
        robot_width = 1
        front_point = [x + robot_length/2 * math.cos(a), y + robot_length/2 * math.sin(a)]
        back_point = [x - robot_length/2 * math.cos(a), y - robot_length/2 * math.sin(a)]
        ortho_angle = a + math.pi/2
        back_left_point = [back_point[0] + robot_width/2 * math.cos(ortho_angle), back_point[1] + robot_width/2 * math.sin(ortho_angle)]
        back_right_point = [back_point[0] - robot_width/2 * math.cos(ortho_angle), back_point[1] - robot_width/2 * math.sin(ortho_angle)]

        self.r1_figs = []
        self.r1_figs.append(self.window['_GRAPH_'].DrawLine(point_from=front_point, point_to=back_left_point, color='black'))
        self.r1_figs.append(self.window['_GRAPH_'].DrawLine(point_from=back_left_point, point_to=back_right_point, color='black'))
        self.r1_figs.append(self.window['_GRAPH_'].DrawLine(point_from=back_right_point, point_to=front_point, color='black'))
        self.r1_figs.append(self.window['_GRAPH_'].DrawLine(point_from=[x,y], point_to=front_point, color='red'))


    def close(self):
        self.window.close()
    

class Master:

    def __init__(self, cfg):

        # Init GUI
        self.cmd_gui = CmdGui()

        # Setup other miscellaneous variables
        self.cfg = cfg

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

        # Robot and marvelmind interfaces
        self.pos_handler = None
        self.robot_client = None
        self.online = False
        self.bring_online(1) # robot_id = 1 - handle multiple in the future

        print("Starting loop")

    def bring_online(self, robot_id):
        try:
            print("Attempting to bring robot {} online".format(robot_id))
            # Setup marvelmind devices and position handler
            self.pos_handler = RobotPositionHandler(self.cfg)
            self.pos_handler.wake_robot(robot_id)

            # Setup robot clients and communication
            self.robot_client = RobotClient(self.cfg, robot_id) # TODO Add a ping check here maybe to verify it is reachable, possible add retry logic too

            # Wait a little while for setup to finish
            sleep_time = 30
            print('Waiting {} seconds for beacons to fully wake up'.format(sleep_time))
            time.sleep(sleep_time)
            self.online = True
            print("Robot {} online".format(robot_id))

        except Exception as e:
            print("Unable to bring robot {} online. Reason: {}".format(robot_id, repr(e)))

    def cleanup(self):

        print("Cleaning up")
        if self.online:
            self.pos_handler.sleep_robot(1)
            self.pos_handler.close()
            pass

    def checkNetworkStatus(self):
        net_status = self.robot_client.net_status()
        if net_status:
            print("Network status of Robot 1 is GOOD")
        else:
            print("Network status of Robot 1 is BAD")

    def run_command(self, data):

        if "move" in data:
            start_idx = data.find('[')
            end_idx = data.find(']')
            vals = data[start_idx+1:end_idx].split(',')
            vals = [x.strip() for x in vals]
            print("Got move command with parameters [{},{},{}]".format(vals[0], vals[1], vals[2]))
            if self.online:
                self.robot_client.move(float(vals[0]), float(vals[1]), float(vals[2]))
        elif "net" in data:
            self.checkNetworkStatus()
        elif "status" in data:
            print("Got status command")
            status_dict = {}
            if self.online:
                status_dict = self.robot_client.request_status()
            self.cmd_gui.update_robot_status(status_dict)
        elif "pos" in data:
            start_idx = data.find('[')
            end_idx = data.find(']')
            vals = data[start_idx+1:end_idx].split(',')
            vals = [x.strip() for x in vals]
            print("Got pos command with parameters [{},{},{}]".format(vals[0], vals[1], vals[2]))
            self.cmd_gui.update_robot_viz_position(float(vals[0]), float(vals[1]), float(vals[2]))
            if self.online:
                status = self.robot_client.send_position(x,y,a)
        elif "online" in data:
            print("Got online command")
            self.bring_online(1)  # robot_id = 1
        elif "test" in data:
            print("Got test status")
        else:
            print("Unknown command: {}".format(data))


    def loop(self):
        last_status_time = 0

        while True:
            
            if self.online:

                # Service marvelmind queues
                robots_updated = self.pos_handler.service_queues()
                for robot in robots_updated:
                    pos = self.pos_handler.get_position(robot)
                    self.robot_client.send_position(pos[0], pos[1], pos[2]) # TODO: update for multiple robots

                # Update staus in gui
                if time.time() - last_status_time > 1:
                    status = self.robot_client.request_status()
                    if status == None or status == "":
                        status = "Robot status not available!"
                    self.cmd_gui.update_robot_status(status)
                    last_status_time = time.time()
            
            # Handle any input from gui
            done, command_str = self.cmd_gui.update()
            if done:
                break
            if command_str:
                self.run_command(command_str)


        # Clean up whenever loop exits
        self.cleanup()
        self.cmd_gui.close()





if __name__ == '__main__':
    cfg = config.Config()
    m = Master(cfg)
    m.loop()

