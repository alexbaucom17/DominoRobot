import time
import math
import copy
import config
import os
import sys
import pickle
import logging
import PySimpleGUI as sg

from FieldPlanner import Plan, ActionTypes, Action, MoveAction, TestPlan, MoveConstVelAction
from Runtime import RuntimeManager, OFFLINE_TESTING, SKIP_MARVELMIND


def status_panel(name):
    width = 40
    height = 10
    return [[sg.Text("{} status".format(name))], [sg.Text("{} offline".format(name), size=(width, height), relief=sg.RELIEF_RIDGE, key='_{}_STATUS_'.format(name.upper())) ]]

def setup_gui_layout(panel_names, target_names):
    # Left hand column with status panels
    col1 = []
    for name in panel_names:
        col1 += status_panel(name)

    # Middle column with plot and buttons
    target_element = [ [sg.Text("Target: ")], [sg.Combo(target_names, key='_TARGET_')] ]

    actions = [a for a in ActionTypes]
    action_element = [ [sg.Text("Action: ")], [sg.Combo(actions, key='_ACTION_')] ]

    data_element = [ [sg.Text('Data:')], [sg.Input(key='_ACTION_DATA_')] ]

    run_button = [[sg.Button('Run Plan', button_color=('white','green'), size=[20,5], pad=(5,30)) ]]
    estop_button = [[sg.Button('ESTOP', button_color=('white','red'), size=[20,5], pad=(5,30)) ]]
    manual_button = [[sg.Button('Send Command', button_color=('white','blue'), size=[20,5], pad=(5,30)) ]]

    col2 = [[sg.Graph(canvas_size=(600,600), graph_bottom_left=(-5,-5), graph_top_right=(5, 5), key="_GRAPH_", background_color="grey") ],
            [sg.Column(target_element), sg.Column(action_element), sg.Column(data_element)],
            [sg.Column(run_button), sg.Column(estop_button), sg.Column(manual_button)]  ]

    # Right hand column with text ouput
    col3 = [[sg.Output(size=(50, 50))]]
    
    return [[ sg.Column(col1), sg.Column(col2), sg.Column(col3)]]


class CmdGui:

    def __init__(self, config):

        self.config = config
        
        sg.change_look_and_feel('DarkBlack')

        panel_names = ["{}".format(n) for n in self.config.ip_map]
        panel_names += ['base', 'plan', 'pos']
        target_names = copy.deepcopy(panel_names)
        target_names.remove('plan')
        layout = setup_gui_layout(panel_names, target_names)

        self.window = sg.Window('Robot Controller', layout, return_keyboard_events=True)
        self.window.finalize()

        self.viz_figs = {}

    def close(self):
        self.window.close()


    def update(self):

        event, values = self.window.read(timeout=20)

        # At exit, check if we should keep marvelmind on
        if event is None or event == 'Exit':
            if OFFLINE_TESTING or SKIP_MARVELMIND:
                return 'Exit', None
            else:
                clicked_value = sg.popup_yes_no('Do you want to keep the Marvelmind running')
                if clicked_value == "Yes":
                    return "ExitMM", None
                else:
                    return 'Exit', None
        
        # Sending a manual action (via button or pressing enter)
        if event in ('Send Command', '\r', '\n'):
            manual_action = self._parse_manual_action(values)
            self.window['_ACTION_DATA_'].update("")
            return 'Action', manual_action

        # Pressing the run plan button
        if event == "Run Plan":
            return "Run", None

        if event == "ESTOP":
            return "ESTOP", None

        return None, None

    def _parse_manual_action(self, values):
        target = values['_TARGET_']
        action_type = ActionTypes(values['_ACTION_'])
        data_str = values['_ACTION_DATA_']
        name = 'ManualAction'

        action = None
        if action_type in [ActionTypes.MOVE_COARSE, ActionTypes.MOVE_REL, ActionTypes.MOVE_FINE]:
            data = data_str.split(',')
            data = [x.strip() for x in data]
            action = MoveAction(action_type, name, data[0], data[1], data[2])
        elif action_type in [ActionTypes.MOVE_CONST_VEL]:
            data = data_str.split(',')
            data = [x.strip() for x in data]
            action = MoveConstVelAction(action_type, name, data[0], data[1], data[2], data[3])
        else:
            action = Action(action_type, name)

        return (target, action)

    def update_status_panels(self, metrics):
        for key, metric in metrics.items():
            if key == 'pos':
                self._update_pos_panel(metric)
            elif key == 'plan':
                self._update_plan_panel(metric)
            elif key == 'base':
                self._update_base_panel(metric)
            else:
                self._update_robot_panel(key, metric) 


    def _update_pos_panel(self, status_dict):
        status_str = "Cannot get marvelmind status"
        if status_dict:
            try:
                status_str = ""
                status_str += "Marvelmind message stats:\n  Dropped: {0:.1f}%\n  Dropped dist: {1:.1f}%\n  Dropped time {2:.1f}%\n  Last sent: {3:.2f}s".format(
                    status_dict['frac_dropped_total']*100, status_dict['frac_dropped_dist']*100,
                    status_dict['frac_dropped_time']*100, status_dict['time_since_last_sent'])
            except Exception as e:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_POS_STATUS_'].update(status_str)

    def _update_plan_panel(self, status_dict):
        status_str = "Plan is not running"
        if status_dict:
            try:
                status_str = ""
                for id, data in status_dict.items():
                    status_str += "{}\n".format(id)
                    status_str += "  Cycle: {}\n".format(data["cycle"])
                    status_str += "  Action: {}\n".format(data["action"])
            except Exception as e:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_PLAN_STATUS_'].update(status_str)

    def _update_base_panel(self, status_dict):
        status_str = "Cannot get base status"
        if status_dict:
            try:
                status_str = ""
                status_str += "Sensors: [{}, {}, {}, {}]\n".format(
                    status_dict['sensor_1'],status_dict['sensor_2'],status_dict['sensor_3'],status_dict['sensor_4'])
                status_str += "Action in Progress: {}\n".format(status_dict['in_progress'])
                status_str += "Counter: {}\n".format(status_dict['counter'])
            except Exception as e:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_BASE_STATUS_'].update(status_str)

    def _update_robot_panel(self, robot_id, status_dict):
        status_str = "Cannot get {} status".format(robot_id)
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

                # Also update the visualization position
                self._update_robot_viz_position(robot_id, status_dict['pos_x'],status_dict['pos_y'], status_dict['pos_a'])
            except Exception as e:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_{}_STATUS_'.format(robot_id.upper())].update(status_str)

    def _update_robot_viz_position(self, robot_id, x, y, a):
        if robot_id in self.viz_figs:
            for f in self.viz_figs[robot_id]:
                self.window['_GRAPH_'].DeleteFigure(f)
        
        self.viz_figs[robot_id] = self._draw_robot(x, y, a)

    def _draw_robot(self, x, y, a):
        robot_length = 1
        robot_width = 1
        front_point = [x + robot_length/2 * math.cos(a), y + robot_length/2 * math.sin(a)]
        back_point = [x - robot_length/2 * math.cos(a), y - robot_length/2 * math.sin(a)]
        ortho_angle = a + math.pi/2
        back_left_point = [back_point[0] + robot_width/2 * math.cos(ortho_angle), back_point[1] + robot_width/2 * math.sin(ortho_angle)]
        back_right_point = [back_point[0] - robot_width/2 * math.cos(ortho_angle), back_point[1] - robot_width/2 * math.sin(ortho_angle)]

        figs = []
        figs.append(self.window['_GRAPH_'].DrawLine(point_from=front_point, point_to=back_left_point, color='black'))
        figs.append(self.window['_GRAPH_'].DrawLine(point_from=back_left_point, point_to=back_right_point, color='black'))
        figs.append(self.window['_GRAPH_'].DrawLine(point_from=back_right_point, point_to=front_point, color='black'))
        figs.append(self.window['_GRAPH_'].DrawLine(point_from=[x,y], point_to=front_point, color='red'))
        return figs
    

class Master:

    def __init__(self, cfg, gui_handle):

        self.cfg = cfg
        self.plan_cycle_number = 0
        self.plan = None
        self.load_plan()
        # For debugging, comment out plan lines above
        # self.plan = TestPlan()
        self.cmd_gui = gui_handle
        self.plan_running = False

        logging.info("Initializing Master")
        self.runtime_manager = RuntimeManager(self.cfg)
        self.runtime_manager.initialize()
        self.initialized = False


    def load_plan(self):

        # If we don't already have a plan, load it or generate it
        if not self.plan:
            if os.path.exists(self.cfg.plan_file):
                with open(self.cfg.plan_file, 'rb') as f:
                    self.plan = pickle.load(f)
                    logging.info("Loaded plan from {}".format(self.cfg.plan_file))
            else:
                self.plan = Plan(self.cfg)
                with open(self.cfg.plan_file, 'wb') as f:
                    pickle.dump(self.plan, f)
                    logging.info("Saved plan to {}".format(self.cfg.plan_file))


    def loop(self):

        keep_mm_running = False
        while True:

            # Only do some stuff once we are initialized
            if not self.initialized:
                if self.runtime_manager.get_initialization_status() != RuntimeManager.STATUS_FULLY_INITIALIZED:
                    pass
                else:
                    self.initialized = True
                    logging.info("Init completed, starting main loop")

            else:
            
                # If we have an idle robot, send it the next cycle to execute
                if self.plan_running and self.runtime_manager.any_idle_bots():
                    logging.info("Sending cycle {} for execution".format(self.plan_cycle_number))
                    next_cycle = self.plan.get_cycle(self.plan_cycle_number)
                    
                    # If we get none, that means we are done with the plan
                    if next_cycle is None:
                        self.plan_running = False
                        self.plan_cycle_number = 0
                        logging.info("Completed plan!")
                    else:
                        self.plan_cycle_number += 1
                        self.runtime_manager.assign_new_cycle(next_cycle)

                # Run updates for the runtime manager
                self.runtime_manager.update()
            
                # Get metrics and update the gui
                metrics = self.runtime_manager.get_all_metrics()
                self.cmd_gui.update_status_panels(metrics)


            # Handle any input from gui
            event, manual_action = self.cmd_gui.update()
            if event == "Exit":
                break
            if event == "ExitMM":
                keep_mm_running = True
                break
            if event == "Run":
                self.plan_running = True
            if event == "Action":
                self.runtime_manager.run_manual_action(manual_action)
            if event == "ESTOP":
                self.runtime_manager.estop()

        # Clean up whenever loop exits
        self.runtime_manager.shutdown(keep_mm_running)
        self.cmd_gui.close()


def configure_logging(path):

    rootLogger = logging.getLogger()
    rootLogger.setLevel(logging.INFO)

    fileHandler = logging.FileHandler(os.path.join(path,"master.log"), 'w+')
    fileFormatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    fileHandler.setFormatter(fileFormatter)
    rootLogger.addHandler(fileHandler)

    consoleHandler = logging.StreamHandler()
    consoleFormatter = logging.Formatter("%(message)s")
    consoleHandler.setFormatter(consoleFormatter)
    rootLogger.addHandler(consoleHandler)


if __name__ == '__main__':
    # Setup config and gui
    cfg = config.Config()
    gui = CmdGui(cfg)
    # Need to setup gui before logging to ensure that output pane captures logs correctly
    configure_logging(cfg.log_folder)
    # Startup master and loop forever
    m = Master(cfg, gui)
    m.loop()

