import time
import math
import copy
import config
import os
import sys
import pickle
import logging
import PySimpleGUI as sg
import traceback

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
    target_element = [ [sg.Text("Target: ")], [sg.Combo(target_names, key='_TARGET_', default_value='robot1')] ]

    actions = [a for a in ActionTypes]
    action_element = [ [sg.Text("Action: ")], [sg.Combo(actions, key='_ACTION_')] ]

    data_element = [ [sg.Text('Data:')], [sg.Input(key='_ACTION_DATA_')] ]

    plan_button_size = [10,2]
    plan_button_pad = (2, 10)
    load_plan_button = sg.Button('Load Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_LOAD_PLAN_') 
    run_plan_button = sg.Button('Run Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_RUN_PLAN_', disabled=True) 
    pause_plan_button = sg.Button('Pause Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_PAUSE_PLAN_', disabled=True) 
    abort_plan_button = sg.Button('Abort Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_ABORT_PLAN_', disabled=True)
    plan_buttons = [[sg.Column([[load_plan_button], [run_plan_button]]), sg.Column([[pause_plan_button], [abort_plan_button]])]]

    button_size = [20,6]
    button_pad = (2,10)
    estop_button = [[sg.Button('ESTOP', button_color=('white','red'), size=button_size, pad=button_pad) ]]
    manual_button = [[sg.Button('Send Command', button_color=('white','green'), size=button_size, pad=button_pad) ]]

    col2 = [[sg.Graph(canvas_size=(700,700), graph_bottom_left=(-5,-5), graph_top_right=(5, 5), key="_GRAPH_", background_color="grey") ],
            [sg.Column(target_element), sg.Column(action_element), sg.Column(data_element)],
            [sg.Column(plan_buttons), sg.Column(estop_button), sg.Column(manual_button)]  ]

    # Right hand column with text ouput
    col3 = [[sg.Output(size=(70, 50))]]
    
    return [[ sg.Column(col1), sg.Column(col2), sg.Column(col3)]]


class CmdGui:

    def __init__(self, config):

        self.config = config
        
        sg.change_look_and_feel('DarkBlack')

        panel_names = ["{}".format(n) for n in self.config.ip_map]
        panel_names += ['base', 'plan', 'mm']
        target_names = copy.deepcopy(panel_names)
        target_names.remove('plan')
        layout = setup_gui_layout(panel_names, target_names)

        self.window = sg.Window('Robot Controller', layout, return_keyboard_events=True)
        self.window.finalize()

        self.viz_figs = {}
        self.plan_state_str = ""

    def close(self):
        self.window.close()


    def update(self):

        event, values = self.window.read(timeout=20)
        # if event != "__TIMEOUT__":
        #     print(event)
        #     print(values)

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
        if event == "_RUN_PLAN_":
            clicked_value = sg.popup_yes_no('Ready to start plan?')
            if clicked_value == "Yes":
                return "Run", None

        if event == "_LOAD_PLAN_":
            return "Load", None

        if event == "_PAUSE_PLAN_":
            return "Pause", None

        if event == "_ABORT_PLAN_":
            clicked_value = sg.popup_yes_no('Abort plan? This will stop running the plan and lose any progress\n!!!Not Implemented Yet!!!')
            if clicked_value == "Yes":
                return "Abort", None

        if event == "ESTOP":
            return "ESTOP", None

        return None, None

    def update_plan_button_status(self, plan_state):
        self.plan_state_str = plan_state
        if plan_state == "None":
            self.window['_RUN_PLAN_'].update(disabled=True)
            self.window['_LOAD_PLAN_'].update(disabled=False)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=True)
        elif plan_state == "Loaded":
            self.window['_RUN_PLAN_'].update(disabled=False)
            self.window['_LOAD_PLAN_'].update(disabled=False)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=True)
        elif plan_state == "Running":
            self.window['_RUN_PLAN_'].update(disabled=True)
            self.window['_LOAD_PLAN_'].update(disabled=True)
            self.window['_PAUSE_PLAN_'].update(disabled=False)
            self.window['_ABORT_PLAN_'].update(disabled=False)
        elif plan_state == "Paused":
            self.window['_RUN_PLAN_'].update(disabled=False)
            self.window['_LOAD_PLAN_'].update(disabled=True)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=False)
        

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
            if key == 'mm':
                self._update_marvelmind_panel(metric)
            elif key == 'plan':
                self._update_plan_panel(metric)
            elif key == 'base':
                self._update_base_panel(metric)
            else:
                self._update_robot_panel(key, metric) 


    def _update_marvelmind_panel(self, status_dict):
        status_str = "Cannot get marvelmind status"
        if status_dict:
            try:
                status_str = ""
                status_str = "Connected devices:\n"
                for addr, data in status_dict:
                    status_str += "  Address: {} | Sleep: {}\n".format(addr, data['sleep'])
            except Exception as e:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_MM_STATUS_'].update(status_str)

    def _update_plan_panel(self, status_dict):
        status_str = "Plan is not running"
        if status_dict:
            try:
                status_str = ""
                status_str += "Plan state: {}\n".format(self.plan_state_str)
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
        if robot_id in self.viz_figs.keys():
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
        # For debugging, comment out plan lines above
        # self.plan = TestPlan()
        self.cmd_gui = gui_handle
        self.plan_status = "None"

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

            self.plan_status = "Loaded"


    def loop(self):

        keep_mm_running = False
        while True:

            # Only do some stuff once we are initialized
            if not self.initialized:
                if self.runtime_manager.get_initialization_status() != RuntimeManager.STATUS_FULLY_INITIALIZED:
                    self.runtime_manager.initialize()
                else:
                    self.initialized = True
                    logging.info("Init completed, starting main loop")

            else:
            
                # If we have an idle robot, send it the next cycle to execute
                if self.plan_status == "Running" and self.runtime_manager.any_idle_bots():
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
                self.plan_status = "Running"
            if event == "Load":
                # TODO: Acc option to save/load various plan files
                # TODO: Enable ability to load new plan even if another plan is already loaded (with proper confirmation)
                self.load_plan()
            if event == "Pause":
                # TODO: Actually pause plan
                self.plan_status = "Paused"
            if event == "Abort":
                # TODO: Acutally abort plan
                # TODO: Maybe add option to save state of plan when aborted/crashed so it is possible to reload
                self.plan_status = "None"
            if event == "Action":
                self.runtime_manager.run_manual_action(manual_action)
            if event == "ESTOP":
                self.runtime_manager.estop()

            self.cmd_gui.update_plan_button_status(self.plan_status)

        # Clean up whenever loop exits
        self.runtime_manager.shutdown(keep_mm_running)
        self.cmd_gui.close()


def configure_logging(path):

    rootLogger = logging.getLogger()
    rootLogger.setLevel(logging.INFO)

    if not os.path.isdir(path):
        os.mkdir(path)

    fileHandler = logging.FileHandler(os.path.join(path,"master.log"), 'w+')
    fileFormatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    fileHandler.setFormatter(fileFormatter)
    rootLogger.addHandler(fileHandler)

    consoleHandler = logging.StreamHandler()
    consoleFormatter = logging.Formatter("%(message)s")
    consoleHandler.setFormatter(consoleFormatter)
    rootLogger.addHandler(consoleHandler)


if __name__ == '__main__':
    try:
        # Setup config and gui
        cfg = config.Config()
        gui = CmdGui(cfg)
        # Need to setup gui before logging to ensure that output pane captures logs correctly
        configure_logging(cfg.log_folder)
        # Startup master and loop forever
        m = Master(cfg, gui)
        m.loop()
    except Exception:
        logging.exception("Unhandled exception")


