import time
import math
import copy
import config
import os
import sys
import logging
import PySimpleGUI as sg
import traceback

from FieldPlanner import *
from Runtime import RuntimeManager, PlanStatus


STATUS_PANEL_OK_COLOR = "green"
STATUS_PANEL_BAD_COLOR = "red"

def status_panel(name):
    width = 40
    height = 10
    return [[sg.Text("{} status".format(name))], [sg.Text("{} offline".format(name), size=(width, height), \
        relief=sg.RELIEF_RIDGE, key='_{}_STATUS_'.format(name.upper()), background_color=STATUS_PANEL_BAD_COLOR) ]]

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
    plan_file_field = sg.Input(key='_PLAN_FILE_', visible=False, enable_events=True)
    load_plan_button = sg.FileBrowse(button_text='Load Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, \
        key='_LOAD_PLAN_', file_types=(('Robot Plans', ('*.json','*.p')),)) 
    run_plan_button = sg.Button('Run Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_RUN_PLAN_', disabled=True) 
    pause_plan_button = sg.Button('Pause Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_PAUSE_PLAN_', disabled=True) 
    abort_plan_button = sg.Button('Abort Plan', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_ABORT_PLAN_', disabled=True)
    plan_buttons = [[sg.Column([[plan_file_field, load_plan_button], [run_plan_button]]), sg.Column([[pause_plan_button], [abort_plan_button]])]]

    button_size = [20,6]
    button_pad = (2,10)
    estop_button = [[sg.Button('ESTOP', button_color=('white','red'), size=button_size, pad=button_pad) ]]
    manual_button = [[sg.Button('Send Command', button_color=('white','green'), size=button_size, pad=button_pad) ]]

    col2 = [[sg.Graph(canvas_size=(700,700), graph_bottom_left=(-1,-1), graph_top_right=(10, 10), key="_GRAPH_", background_color="grey") ],
            [sg.Column(target_element), sg.Column(action_element), sg.Column(data_element)],
            [sg.Column(plan_buttons), sg.Column(estop_button), sg.Column(manual_button)]  ]

    # Right hand column with text ouput
    col3 = [[sg.Output(size=(70, 50), echo_stdout_stderr=True)]]
    
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

    def close(self):
        self.window.close()


    def update(self):

        event, values = self.window.read(timeout=20)
        # if event != "__TIMEOUT__":
        #     print(event)
        #     print(values)

        # At exit, check if we should keep marvelmind on
        if event is None or event == 'Exit':
            if self.config.OFFLINE_TESTING or self.config.SKIP_MARVELMIND:
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

        if event == "_PLAN_FILE_":
            return "Load", values["_PLAN_FILE_"]

        if event == "_PAUSE_PLAN_":
            return "Pause", None

        if event == "_ABORT_PLAN_":
            clicked_value = sg.popup_yes_no('Abort plan?')
            if clicked_value == "Yes":
                return "Abort", None

        if event == "ESTOP":
            return "ESTOP", None

        return None, None
    
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

    def _update_marvelmind_panel(self, status_dict):
        status_str = "Cannot get marvelmind status"
        color_str = STATUS_PANEL_BAD_COLOR
        if status_dict:
            try:
                status_str = ""
                status_str = "Connected devices:\n"
                for addr, data in status_dict:
                    status_str += "  Address: {} | Sleep: {}\n".format(addr, data['sleep'])
                color_str = STATUS_PANEL_OK_COLOR
            except Exception as e:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_MM_STATUS_'].update(status_str, background_color=color_str)

    def _update_plan_button_status(self, plan_status):
        if plan_status == PlanStatus.NONE or plan_status == PlanStatus.ABORTED:
            self.window['_RUN_PLAN_'].update(text='Run', disabled=True)
            self.window['_LOAD_PLAN_'].update(disabled=False)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=True)
        elif plan_status == PlanStatus.LOADED or plan_status == PlanStatus.DONE:
            self.window['_RUN_PLAN_'].update(text='Run', disabled=False)
            self.window['_LOAD_PLAN_'].update(disabled=False)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=True)
        elif plan_status == PlanStatus.RUNNING:
            self.window['_RUN_PLAN_'].update(text='Run', disabled=True)
            self.window['_LOAD_PLAN_'].update(disabled=True)
            self.window['_PAUSE_PLAN_'].update(disabled=False)
            self.window['_ABORT_PLAN_'].update(disabled=False)
        elif plan_status == PlanStatus.PAUSED:
            self.window['_RUN_PLAN_'].update(text='Resume', disabled=False)
            self.window['_LOAD_PLAN_'].update(disabled=True)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=False)
        else:
            logging.warning("Unhandled plan statusfor button state: {}".format(plan_status))

    def _update_plan_panel(self, status_dict):
        status_str = "Plan is not running"
        color_str = STATUS_PANEL_BAD_COLOR
        if status_dict:
            try:
                self._update_plan_button_status(status_dict['status'])

                status_str = ""
                plan_state_str = "{}".format(status_dict['status']).split('.')[1]
                status_str += "Plan status: {}\n".format(plan_state_str)
                status_str += "Plan filename: {}\n".format(status_dict['filename'])
                status_str += "Next cycle num: {}\n".format(status_dict['next_cycle_number'])
                status_str += "Idle bots: {}\n".format(status_dict['idle_bots'])
                for id, data in status_dict['robots'].items():
                    needs_restart_str = ''
                    if data['needs_restart']:
                        needs_restart_str = "(Needs Restart)"
                    status_str += "{}{}:\n".format(id, needs_restart_str)
                    status_str += "  Cycle id: {}\n".format(data["cycle_id"])
                    status_str += "  Action id: {}\n".format(data["action_id"])
                    status_str += "  Action name: {}\n".format(data["action_name"])
                
                # Set panel coloring based on state
                if plan_state_str == "PAUSED" or plan_state_str == "ABORTED":
                    color_str = STATUS_PANEL_BAD_COLOR
                elif plan_state_str != "NONE":
                    color_str = STATUS_PANEL_OK_COLOR
            except Exception as e:
                status_str = "Bad dict: " + str(status_dict)

        self.window['_PLAN_STATUS_'].update(status_str, background_color=color_str)

    def _update_base_panel(self, status_dict):
        status_str = "Cannot get base status"
        color_str = STATUS_PANEL_BAD_COLOR
        if status_dict:
            try:
                status_str = ""
                status_str += "Sensors: [{}, {}, {}, {}]\n".format(
                    status_dict['sensor_1'],status_dict['sensor_2'],status_dict['sensor_3'],status_dict['sensor_4'])
                status_str += "Action in Progress: {}\n".format(status_dict['in_progress'])
                status_str += "Counter: {}\n".format(status_dict['counter'])
                color_str = STATUS_PANEL_OK_COLOR
            except Exception as e:
                if "offline" in str(status_dict):
                    status_str = str(status_dict)
                else:
                    status_str = "Bad dict: " + str(status_dict)

        self.window['_BASE_STATUS_'].update(status_str, background_color=color_str)

    def _update_robot_panel(self, robot_id, status_dict):
        status_str = "Cannot get {} status".format(robot_id)
        color_str = STATUS_PANEL_BAD_COLOR
        if status_dict:
            try:
                status_str = ""
                status_str += "Position: [{0:.3f} m, {1:.3f} m, {2:.3f} rad]\n".format(status_dict['pos_x'],status_dict['pos_y'], status_dict['pos_a'])
                status_str += "Velocity: [{0:.3f} m/s, {1:.3f} m/s, {2:.3f} rad/s]\n".format(status_dict['vel_x'],status_dict['vel_y'], status_dict['vel_a'])
                status_str += "Localization Confidence: {0:.1f}%\n".format(status_dict['localization_confidence']*100)
                status_str += "Controller timing: {} ms\n".format(status_dict['controller_loop_ms'])
                status_str += "Position timing:   {} ms\n".format(status_dict['position_loop_ms'])
                status_str += "Motion in progress: {}\n".format(status_dict["in_progress"])
                status_str += "Has error: {}\n".format(status_dict["error_status"])
                status_str += "Counter:   {}\n".format(status_dict['counter'])

                # Also update the visualization position
                self._update_robot_viz_position(robot_id, status_dict['pos_x'],status_dict['pos_y'], status_dict['pos_a'])
                color_str = STATUS_PANEL_OK_COLOR
            except Exception as e:
                if "offline" in str(status_dict):
                    status_str = str(status_dict)
                else:
                    status_str = "Bad dict: " + str(status_dict)

        self.window['_{}_STATUS_'.format(robot_id.upper())].update(status_str, background_color=color_str)

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
        self.cmd_gui = gui_handle
        self.keep_mm_running = False

        logging.info("Initializing Master")
        self.runtime_manager = RuntimeManager(self.cfg)
        self.runtime_manager.initialize()

    def loop(self):

        ready_to_exit = False
        while not ready_to_exit:
            self.runtime_manager.update()
            ready_to_exit = self.update_gui_and_handle_input()

        # Clean up whenever loop exits
        self.runtime_manager.shutdown(self.keep_mm_running)
        self.cmd_gui.close()

    def update_gui_and_handle_input(self):
        # Handle any input from gui
        event_type, event_data = self.cmd_gui.update()
        if event_type == "Exit":
            return True
        if event_type == "ExitMM":
            self.keep_mm_running = True
            return True
        if event_type == "Run":
            if self.runtime_manager.get_plan_status() == PlanStatus.PAUSED:
                logging.info("PLAN RESUMED")
            self.runtime_manager.set_plan_status(PlanStatus.RUNNING)
        if event_type == "Load":
            if event_data is not '':
                self.runtime_manager.load_plan(event_data)
        if event_type == "Pause":
            logging.info("PLAN PAUSED")
            self.runtime_manager.estop()
            self.runtime_manager.set_plan_status(PlanStatus.PAUSED)
        if event_type == "Abort":
            logging.warning("PLAN ABORTED")
            self.runtime_manager.estop()
            self.runtime_manager.set_plan_status(PlanStatus.ABORTED)
        if event_type == "Action":
            self.runtime_manager.run_manual_action(event_data)
        if event_type == "ESTOP":
            self.runtime_manager.estop()
            if self.runtime_manager.get_plan_status() == PlanStatus.RUNNING:
                self.runtime_manager.set_plan_status(PlanStatus.PAUSED)
                logging.warning("Pausing plan due to ESTOP event")

        # Get metrics and update the displayed info
        metrics = self.runtime_manager.get_all_metrics()
        self.cmd_gui.update_status_panels(metrics)

        return False



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
    # Setup config and gui
    cfg = config.Config()
    gui = CmdGui(cfg)
    # Need to setup gui before logging to ensure that output pane captures logs correctly
    configure_logging(cfg.log_folder)
    # Startup master and loop forever
    m = Master(cfg, gui)
    m.loop()


