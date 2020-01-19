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


class NonBlockingTimer:

    def __init__(self, trigger_time):
        self.start_time = time.time()
        self.trigger_time = trigger_time

    def check(self):
        if time.time() - self.start_time > self.trigger_time:
            return True 
        else:
            return False

class CmdGui:

    def __init__(self):
        sg.change_look_and_feel('Dark Blue 3')

        col1 = [[sg.Text('Command:'), sg.Input(key='_IN_')], [sg.Button('Send'), sg.Button('Exit')], [sg.Button('Exit keep MM')]]
        col2 = [[sg.Text("Robot 1 status:")],
               [sg.Text("Robot 1 offline",size=(40, 15),relief=sg.RELIEF_RAISED,key='_R1STATUS_')]]
#sg.Output(size=(100, 15)),
        layout = [[sg.Graph(canvas_size=(600,600),graph_bottom_left=(0,0), graph_top_right=(10, 10), key="_GRAPH_", background_color="white")  ],
                   [sg.Column(col1), sg.Column(col2)] ]

        self.window = sg.Window('Robot Controller', layout, return_keyboard_events=True)
        self.window.finalize()

        self.draw_robot(3,3,0)


    def update(self):

        event, values = self.window.read(timeout=20)
        command = None
        if event is None or event == 'Exit':
            return True, None
        if event == "Exit keep MM":
            return True, 'exit_dbg'
        if event in ('Send Command', '\r', '\n'): # Catch enter as well
            command = values['_IN_']
            self.window['_IN_'].update("")

        return False, command

    def update_robot_status(self, status_dict, msg_metrics):
        status_str = "Cannot get robot status"
        if status_dict:
            try:
                status_str = ""
                status_str += "Position: [{0:.3f} m, {1:.3f} m, {2:.3f} rad]\n".format(status_dict['pos_x'],status_dict['pos_y'], status_dict['pos_a'])
                status_str += "Velocity: [{0:.3f} m/s, {1:.3f} m/s, {2:.3f} rad/s]\n".format(status_dict['vel_x'],status_dict['vel_y'], status_dict['vel_a'])
                status_str += "Confidence: [{0:.2f} %, {1:.2f} %, {2:.2f} %]\n".format(status_dict['confidence_x']/2.55,status_dict['confidence_y']/2.55, status_dict['confidence_a']/2.55)
                status_str += "Controller timing: {} ms\n".format(status_dict['controller_loop_ms'])
                status_str += "Position timing:   {} ms\n".format(status_dict['position_loop_ms'])
                status_str += "Motion in progress: {}\n".format(status_dict["in_progress"])
                status_str += "Counter:   {}\n".format(status_dict['counter'])
                status_str += "Free memory:   {} bytes\n".format(status_dict['free_memory'])
                status_str += "Marvelmind message stats:\n  Dropped: {0:.1f}%\n  Dropped dist: {1:.1f}%\n  Dropped time {2:.1f}%\n  Last sent: {3:.2f}s".format(
                    msg_metrics['frac_dropped_total']*100, msg_metrics['frac_dropped_dist']*100,
                    msg_metrics['frac_dropped_time']*100, msg_metrics['time_since_last_sent'])

                # Also update the visualization position
                self.update_robot_viz_position(status_dict['pos_x'],status_dict['pos_y'], status_dict['pos_a'])
            except Exception as e:
                status_str = "Bad dicts: " + str(status_dict) + str(msg_metrics)
                print("Message exception: " + repr(e))

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


class CmdGenerator:
    """ Generate commands for testing"""
    def __init__(self, steps=None):
        self.cur_step = 0
        self.step_timer = NonBlockingTimer(1)
        # Number is wait for that long, string is command, -1 is done, -2 is repeat
        if steps:
            self.steps = steps
        else:
            self.steps = [1, "move[2.75,2.75,0]", 3, "fine[3,3,0]", 2, "fine[2.75,2.75,0]", 1, "move[1,2,0]", -2]
        self.done = False

    def next_step(self, in_progress):
        cmd = None
        if self.done:
            return cmd

        if self.step_timer.check() and in_progress is False:
            new_cmd = self.steps[self.cur_step]
            self.cur_step += 1
            if isinstance(new_cmd, str):
                cmd = new_cmd
                print("Command generator executing command: {}".format(new_cmd))
                self.step_timer = NonBlockingTimer(2)
            elif isinstance(new_cmd, int):
                if new_cmd == -1:
                    print("Command generator done")
                    self.done = True
                elif new_cmd == -2:
                    print("Repeating command generator")
                    self.step_timer = NonBlockingTimer(1)
                    self.cur_step = 0
                else:
                    print("Command generator waiting for {} seconds".format(new_cmd))
                    self.step_timer = NonBlockingTimer(new_cmd)

        return cmd


def write_file(filename, text):
    with open(filename, 'w+') as f:
        f.write(text)

def get_file_bool(filename):
    try:
        with open(filename, 'r+') as f:
            txt = f.readline()
            if txt == 'True':
                return True
            else:
                return False
    except FileNotFoundError:
        return False
    

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
        self.initialized = False
        self.init_timer = None
        self.bring_online(1) # robot_id = 1 - handle multiple in the future
        self.cmd_generator = None
        self.in_progress = False

        print("Starting loop")

    def bring_online(self, robot_id):
        try:
            # Setup robot clients and communication
            print("Attempting to connect to robot {} over wifi".format(robot_id))
            self.robot_client = RobotClient(self.cfg, robot_id) # TODO Add a ping check here maybe to verify it is reachable, possible add retry logic too
        except Exception as e:
            print("Couldn't connect to robot {} online. Reason: {}".format(robot_id, repr(e)))
            return

        try:
            # Setup marvelmind devices and position handler
            print("Attempting to enable marvelmind beacons on robot {} ".format(robot_id))
            self.pos_handler = RobotPositionHandler(self.cfg)
            self.pos_handler.wake_robot(robot_id)
            self.online = True
            self.initialized = False
            if get_file_bool(self.cfg.mm_beacon_state_file):
                self.init_timer = NonBlockingTimer(2)
                print("Beacons already awake, begninning position transmission immediately")
            else:
                self.init_timer = NonBlockingTimer(30)
                print("Beacons enabled, will wait for 30 seconds to let them fully wake up")
        except Exception as e:
            print("Couldn't enable beacons on robot {}. Reason: {}".format(robot_id, repr(e)))


    def cleanup(self, keep_mm_awake):

        print("Cleaning up")
        if self.online:
            if not keep_mm_awake:
                self.pos_handler.sleep_robot(1)
                write_file(self.cfg.mm_beacon_state_file, 'False')
            self.pos_handler.close()

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
        elif "rel" in data:
            start_idx = data.find('[')
            end_idx = data.find(']')
            vals = data[start_idx+1:end_idx].split(',')
            vals = [x.strip() for x in vals]
            print("Got move_rel command with parameters [{},{},{}]".format(vals[0], vals[1], vals[2]))
            if self.online:
                self.robot_client.move_rel(float(vals[0]), float(vals[1]), float(vals[2]))
        elif "fine" in data:
            start_idx = data.find('[')
            end_idx = data.find(']')
            vals = data[start_idx+1:end_idx].split(',')
            vals = [x.strip() for x in vals]
            print("Got move_fine command with parameters [{},{},{}]".format(vals[0], vals[1], vals[2]))
            if self.online:
                self.robot_client.move_fine(float(vals[0]), float(vals[1]), float(vals[2]))
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
        elif "auto" in data:
            self.cmd_generator = CmdGenerator()
        elif "stop" in data:
            self.cmd_generator = None
        elif "exit_dbg" in data:
            pass
        else:
            print("Unknown command: {}".format(data))


    def loop(self):
        last_status_time = 0

        while True:
            
            if self.online:

                metrics = {'frac_dropped_total':0, 'frac_dropped_dist':0, 'frac_dropped_time': 0, 'time_since_last_sent': 999}

                if self.initialized:

                    # Service marvelmind queues
                    robots_updated = self.pos_handler.service_queues()
                    for robot in robots_updated:
                        pos = self.pos_handler.get_position(robot)
                        self.robot_client.send_position(pos[0], pos[1], pos[2]) # TODO: update for multiple robots

                    # get metrics
                    metrics = self.pos_handler.get_metrics()

                elif self.init_timer.check():
                    print("Beacons awake. Beginning position transmission")
                    write_file(self.cfg.mm_beacon_state_file, 'True')
                    self.initialized = True

                # Update staus in gui
                if time.time() - last_status_time > 0.5:
                    status = self.robot_client.request_status()
                    if status == None or status == "":
                        status = "Robot status not available!"
                    else:
                        try:
                            self.in_progress = status["in_progress"]
                        except Exception:
                            print("Status miissing expected in_progress field")
                    self.cmd_gui.update_robot_status(status, metrics)
                    last_status_time = time.time()
            
            # Handle any input from gui
            done, command_str = self.cmd_gui.update()

            # Handle any input from cmd generator
            if self.cmd_generator:
                command_str = self.cmd_generator.next_step(self.in_progress)

            if done:
                break
            if command_str:
                self.run_command(command_str)


        # Clean up whenever loop exits
        keep_mm_awake = False
        if command_str == "exit_dbg":
            keep_mm_awake = True;
        self.cleanup(keep_mm_awake)
        self.cmd_gui.close()





if __name__ == '__main__':
    cfg = config.Config()
    m = Master(cfg)
    m.loop()

