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
import Utils


STATUS_PANEL_OK_COLOR = "green"
STATUS_PANEL_BAD_COLOR = "red"

def status_panel(name):
    width = 40
    height = 20
    return [[sg.Text("{} status".format(name))], [sg.Text("{} offline".format(name), size=(width, height), \
        relief=sg.RELIEF_RIDGE, key='_{}_STATUS_'.format(name.upper()), background_color=STATUS_PANEL_BAD_COLOR) ]]


def setup_gui_layout(config, panel_names, target_names):
    plan_button_size = [10,2]
    plan_button_pad = (2, 10)

    # Left hand column with status panels
    col1 = []
    for name in panel_names:
        col1 += status_panel(name)

    # Plan cycle/action modification buttons
    incremenet_cycle_button = sg.Button('Cycle +', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_INC_CYCLE_', disabled=True) 
    decremenet_cycle_button = sg.Button('Cycle -', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_DEC_CYCLE_', disabled=True) 
    incremenet_action_button = sg.Button('Action +', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_INC_ACTION_', disabled=True) 
    decremenet_action_button = sg.Button('Action -', button_color=('white','blue'), size=plan_button_size, pad=plan_button_pad, key='_DEC_ACTION_', disabled=True) 
    cycle_buttons = [[sg.Column([[incremenet_cycle_button], [decremenet_cycle_button]]), sg.Column([[incremenet_action_button], [decremenet_action_button]])]]
    col1 += cycle_buttons

    # Middle column with plot and buttons
    target_element = [ [sg.Text("Target: ")], [sg.Combo(target_names, key='_TARGET_', default_value='robot1')] ]

    actions = [a for a in ActionTypes]
    action_element = [ [sg.Text("Action: ")], [sg.Combo(actions, key='_ACTION_')] ]

    data_element = [ [sg.Text('Data:')], [sg.Input(key='_ACTION_DATA_')] ]

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

    col2 = [[sg.Graph(canvas_size=(700,700), graph_bottom_left=config.graph_bottom_left, graph_top_right=config.graph_top_right, float_values=True, key="_GRAPH_", background_color="light grey") ],
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
        panel_names += ['plan']
        target_names = copy.deepcopy(panel_names)
        target_names.remove('plan')
        layout = setup_gui_layout(config, panel_names, target_names)

        self.window = sg.Window('Robot Controller', layout, return_keyboard_events=True)
        self.window.finalize()

        self.viz_figs = {}

        self._draw_environment()

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
            if manual_action is not None:
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

        if event in ["_INC_CYCLE_","_DEC_CYCLE_","_INC_ACTION_","_DEC_ACTION_"]:
            return event, None

        if event == "ESTOP":
            return "ESTOP", None

        return None, None
    
    def update_status_panels(self, metrics):
        for key, metric in metrics.items():
            if key == 'mm' or key == 'base':
                continue
            elif key == 'plan':
                self._update_plan_panel(metric)
            else:
                self._update_robot_panel(key, metric) 
        

    def _parse_manual_action(self, values):
        target = values['_TARGET_']
        action_type = ActionTypes(values['_ACTION_'])
        data_str = values['_ACTION_DATA_']
        name = 'ManualAction'

        action = None
        if action_type in [ActionTypes.MOVE_COARSE, ActionTypes.MOVE_REL, ActionTypes.MOVE_FINE, ActionTypes.MOVE_WITH_DISTANCE]:
            data = data_str.split(',')
            data = [x.strip() for x in data]
            if len(data) != 3:
                logging.warning("Invalid data: {}".format(data))
                return None
            action = MoveAction(action_type, name, data[0], data[1], data[2])
        elif action_type in [ActionTypes.MOVE_CONST_VEL]:
            data = data_str.split(',')
            data = [x.strip() for x in data]
            if len(data) != 4:
                logging.warning("Invalid data: {}".format(data))
                return None
            action = MoveConstVelAction(action_type, name, data[0], data[1], data[2], data[3])
        elif action_type in [ActionTypes.SET_POSE]:
            data = data_str.split(',')
            data = [x.strip() for x in data]
            if len(data) != 3:
                logging.warning("Invalid data: {}".format(data))
                return None
            action = SetPoseAction(action_type, name, data[0], data[1], data[2])
        else:
            action = Action(action_type, name)

        return (target, action)

    def _udpate_cycle_button_status(self, disabled):
        self.window['_INC_CYCLE_'].update(disabled=disabled)
        self.window['_DEC_CYCLE_'].update(disabled=disabled)
        self.window['_INC_ACTION_'].update(disabled=disabled)
        self.window['_DEC_ACTION_'].update(disabled=disabled)

    def _update_plan_button_status(self, plan_status):
        if plan_status == PlanStatus.NONE:
            self.window['_RUN_PLAN_'].update(text='Run', disabled=True)
            self.window['_LOAD_PLAN_'].update(disabled=False)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=True)
            self._udpate_cycle_button_status(disabled=True)
        elif plan_status == PlanStatus.LOADED or plan_status == PlanStatus.DONE:
            self.window['_RUN_PLAN_'].update(text='Run', disabled=False)
            self.window['_LOAD_PLAN_'].update(disabled=False)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=True)
            self._udpate_cycle_button_status(disabled=False)
        elif plan_status == PlanStatus.RUNNING:
            self.window['_RUN_PLAN_'].update(text='Run', disabled=True)
            self.window['_LOAD_PLAN_'].update(disabled=True)
            self.window['_PAUSE_PLAN_'].update(disabled=False)
            self.window['_ABORT_PLAN_'].update(disabled=False)
            self._udpate_cycle_button_status(disabled=True)
        elif plan_status == PlanStatus.PAUSED:
            self.window['_RUN_PLAN_'].update(text='Resume', disabled=False)
            self.window['_LOAD_PLAN_'].update(disabled=True)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=False)
            self._udpate_cycle_button_status(disabled=False)
        elif plan_status == PlanStatus.ABORTED:
            self.window['_RUN_PLAN_'].update(text='Restart', disabled=False)
            self.window['_LOAD_PLAN_'].update(disabled=False)
            self.window['_PAUSE_PLAN_'].update(disabled=True)
            self.window['_ABORT_PLAN_'].update(disabled=True)
            self._udpate_cycle_button_status(disabled=False)
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

    def _update_robot_panel(self, robot_id, status_dict):
        status_str = "Cannot get {} status".format(robot_id)
        color_str = STATUS_PANEL_BAD_COLOR
        if status_dict:
            try:
                robot_pose = [status_dict['pos_x'], status_dict['pos_y'], math.degrees(status_dict['pos_a'])]
                status_str = ""
                status_str += "Position: [{0:.3f} m, {1:.3f} m, {2:.2f} deg]\n".format(robot_pose[0], robot_pose[1], robot_pose[2])
                status_str += "Velocity: [{0:.3f} m/s, {1:.3f} m/s, {2:.2f} deg/s]\n".format(status_dict['vel_x'],status_dict['vel_y'], math.degrees(status_dict['vel_a']))
                status_str += "Distance Pose : [{0:.3f} m, {1:.3f} m, {2:.2f} deg]\n".format(status_dict['dist_x'],status_dict['dist_y'], math.degrees(status_dict['dist_a']))
                status_str += "Raw Distance: FL: {0:.3f} m, FR: {1:.3f} m\n                      SF: {2:.3f} m, SB: {3:.3f} m\n".format(\
                    status_dict['dist_fl'],status_dict['dist_fr'], status_dict['dist_sf'], status_dict['dist_sb'])
                status_str += "Localization total confidence: {:.1f}%\n".format(status_dict['localization_total_confidence']*100)
                status_str += "  Axes confidence: [{:.1f}%, {:.1f}%, {:.1f}%]\n".format(
                    status_dict['localization_confidence_x']*100,status_dict['localization_confidence_y']*100,status_dict['localization_confidence_a']*100)
                status_str += "Localization position uncertainty: {:.2f}\n".format(status_dict['last_position_uncertainty'])
                status_str += "Controller timing: {} ms\n".format(status_dict['controller_loop_ms'])
                status_str += "Position timing:   {} ms\n".format(status_dict['position_loop_ms'])
                status_str += "Distance timing:   {} ms\n".format(status_dict['distance_loop_ms'])
                status_str += "Current action:   {}\n".format(status_dict['current_action'].split('.')[-1])
                status_str += "Motion in progress: {}\n".format(status_dict["in_progress"])
                status_str += "Has error: {}\n".format(status_dict["error_status"])
                status_str += "Counter:   {}\n".format(status_dict['counter'])

                # Also update the visualization position
                self._update_robot_viz_position(robot_id, robot_pose)
                # If there is target position data populated, draw the target too
                if 'current_move_data' in status_dict.keys():
                    self._update_target_viz_position(robot_id, robot_pose, status_dict['current_move_data'])
                color_str = STATUS_PANEL_OK_COLOR

            except Exception as e:
                if "offline" in str(status_dict):
                    status_str = str(status_dict)
                else:
                    status_str = "Bad dict: " + str(status_dict)

        self.window['_{}_STATUS_'.format(robot_id.upper())].update(status_str, background_color=color_str)

    def _update_robot_viz_position(self, robot_id, robot_pose):
        if robot_id in self.viz_figs.keys():
            for f in self.viz_figs[robot_id]:
                self.window['_GRAPH_'].DeleteFigure(f)
        self.viz_figs[robot_id] = self._draw_robot(robot_pose, use_target_color=False)

    def _update_target_viz_position(self, robot_id, robot_pose, target_pose):
        viz_key = "{}_target".format(robot_id)
        if viz_key in self.viz_figs.keys():
            for f in self.viz_figs[viz_key]:
                self.window['_GRAPH_'].DeleteFigure(f)

        if robot_pose and target_pose:
            target_line = self.window['_GRAPH_'].draw_line(point_from=robot_pose[:2], point_to=target_pose[:2], color='yellow', width=2)
            self.viz_figs[viz_key] = self._draw_robot(target_pose, use_target_color=True)
            self.viz_figs[viz_key].append(target_line)  

    def _draw_robot(self, robot_pose, use_target_color):

        robot_line_color = 'black'
        robot_line_thickness = 2
        direction_line_color = 'red'
        direction_line_thickness = 1
        if use_target_color:
            robot_line_color = 'green'
            robot_line_thickness = 2
            direction_line_color = 'tomato'
            direction_line_thickness = 1


        robot_global_pos_2d = np.array([robot_pose[0],robot_pose[1]])
        robot_angle = robot_pose[2]
        robot_fig_handles = []
        
        # Robot chassis points in robot frame
        chassis_fl = np.array([self.config.robot_rotation_center_offset,  self.config.robot_chassis_size/2.0])
        chassis_fr = np.array([self.config.robot_rotation_center_offset, -self.config.robot_chassis_size/2.0])
        chassis_bl = np.array([self.config.robot_rotation_center_offset - self.config.robot_chassis_size,  self.config.robot_chassis_size/2.0])
        chassis_br = np.array([self.config.robot_rotation_center_offset - self.config.robot_chassis_size, -self.config.robot_chassis_size/2.0])

        # Tile points in robot frame
        tile_bl = -self.config.tile_to_robot_offset
        tile_br = tile_bl + np.array([0, -self.config.tile_size_width_meters])
        tile_fl = tile_bl + np.array([self.config.tile_size_height_meters, 0])
        tile_fr = tile_br + np.array([self.config.tile_size_height_meters, 0])

        # Robot direction indicator
        direction_pt = np.array([self.config.robot_direction_indicator_length, 0])
        direction_arrow_x = self.config.robot_direction_indicator_arrow_length*math.cos(math.radians(self.config.robot_direction_indicator_arrow_angle))
        direction_arrow_y = self.config.robot_direction_indicator_arrow_length*math.sin(math.radians(self.config.robot_direction_indicator_arrow_angle))
        direction_arrow_left =  direction_pt + np.array([-direction_arrow_x,  direction_arrow_y])
        direction_arrow_right = direction_pt + np.array([-direction_arrow_x, -direction_arrow_y])

        # Collect, transform, and draw chassis points
        chassis_points_robot_frame = [chassis_fl, chassis_fr, chassis_br, chassis_bl] # order matters for drawing
        chassis_points_global_frame = []
        for pt in chassis_points_robot_frame:
            global_pt = Utils.TransformPos(pt, robot_global_pos_2d, robot_angle)
            chassis_points_global_frame.append(global_pt)
        for i in range(4):
            start_pt = list(chassis_points_global_frame[i])
            end_pt = list(chassis_points_global_frame[(i+1)%4])
            robot_fig_handles.append(self.window['_GRAPH_'].draw_line(point_from=start_pt, point_to=end_pt, color=robot_line_color, width = robot_line_thickness))

        # Collect, transform, and draw tile points
        tile_points_robot_frame = [tile_bl, tile_br, tile_fr, tile_fl] # order matters for drawing
        tile_points_global_frame = []
        for pt in tile_points_robot_frame:
            global_pt = Utils.TransformPos(pt, robot_global_pos_2d, robot_angle)
            tile_points_global_frame.append(global_pt)
        for i in range(4):
            start_pt = list(tile_points_global_frame[i])
            end_pt = list(tile_points_global_frame[(i+1)%4])
            robot_fig_handles.append(self.window['_GRAPH_'].draw_line(point_from=start_pt, point_to=end_pt, color=robot_line_color, width = robot_line_thickness))

        # Draw angle indicator line
        global_direction_pt = Utils.TransformPos(direction_pt, robot_global_pos_2d, robot_angle)
        robot_fig_handles.append(self.window['_GRAPH_'].draw_line(point_from=list(robot_global_pos_2d), point_to=list(global_direction_pt), color=direction_line_color, width=direction_line_thickness))
        global_direction_arrow_left = Utils.TransformPos(direction_arrow_left, robot_global_pos_2d, robot_angle)
        robot_fig_handles.append(self.window['_GRAPH_'].draw_line(point_from=list(global_direction_pt), point_to=list(global_direction_arrow_left), color=direction_line_color, width=direction_line_thickness))
        global_direction_arrow_right = Utils.TransformPos(direction_arrow_right, robot_global_pos_2d, robot_angle)
        robot_fig_handles.append(self.window['_GRAPH_'].draw_line(point_from=list(global_direction_pt), point_to=list(global_direction_arrow_right), color=direction_line_color, width=direction_line_thickness))

        return robot_fig_handles

    def _draw_environment(self):

        # Robot boundaries
        self.viz_figs["boundaires"] = self.window['_GRAPH_'].draw_rectangle(self.config.robot_boundaries[0], self.config.robot_boundaries[1], line_color='red')

        # Domino field boundaries
        bottom_left = self.config.domino_field_origin
        rect_width_height = Utils.TransformPos(np.array([self.config.field_width, self.config.field_height]), [0,0], self.config.domino_field_angle)
        top_left = (bottom_left[0], bottom_left[1]+rect_width_height[1])
        bottom_right = (bottom_left[0] + rect_width_height[0], bottom_left[1])
        self.viz_figs["field"] = self.window['_GRAPH_'].draw_rectangle(top_left, bottom_right, line_color='green')

        # Base station
        base_station_top_left = (self.config.base_station_boundaries[0][0], self.config.base_station_boundaries[1][1])
        base_station_bottom_right = (self.config.base_station_boundaries[1][0], self.config.base_station_boundaries[0][1])
        self.viz_figs["base"] = self.window['_GRAPH_'].draw_rectangle(base_station_top_left, base_station_bottom_right, line_color='blue')

        # X axis
        left_side = (self.config.graph_bottom_left[0], 0)
        right_side = (self.config.graph_top_right[0], 0)
        self.viz_figs["xaxis"] = self.window['_GRAPH_'].draw_line(left_side, right_side, color="black", width=2)
        x_text_location = (right_side[0] - self.config.axes_label_offset, right_side[1] + self.config.axes_label_offset)
        self.viz_figs["xaxis_label"] = self.window['_GRAPH_'].draw_text("X", x_text_location)
        x_ticks = [i for i in range(0, int(right_side[0]), self.config.axes_tick_spacing)] + [i for i in range(0, int(left_side[0]), -self.config.axes_tick_spacing)]
        x_ticks = np.sort(np.unique(np.array(x_ticks)))
        for val in x_ticks:
            bottom = (val, -self.config.axes_tick_size/2)
            top = (val, self.config.axes_tick_size/2)
            self.viz_figs["xtick_{}".format(val)] = self.window['_GRAPH_'].draw_line(bottom, top, color="black")
            label_location = (bottom[0] + self.config.axes_label_offset, bottom[1])
            self.viz_figs["xtick_label_{}".format(val)] = self.window['_GRAPH_'].draw_text("{}".format(val), label_location)

        # Y axis
        bottom_side = (0, self.config.graph_bottom_left[1])
        top_side = (0, self.config.graph_top_right[1])
        self.viz_figs["yaxis"] = self.window['_GRAPH_'].draw_line(bottom_side, top_side, color="black", width=2)
        y_text_location = (top_side[0] + self.config.axes_label_offset, top_side[1] - self.config.axes_label_offset)
        self.viz_figs["yaxis_label"] = self.window['_GRAPH_'].draw_text("Y", y_text_location)
        y_ticks = [i for i in range(0, int(top_side[1]), self.config.axes_tick_spacing)] + [i for i in range(0, int(bottom_side[1]), -self.config.axes_tick_spacing)]
        y_ticks = np.sort(np.unique(np.array(y_ticks)))
        for val in y_ticks:
            left = (-self.config.axes_tick_size/2, val)
            right = (self.config.axes_tick_size/2, val)
            self.viz_figs["ytick_{}".format(val)] = self.window['_GRAPH_'].draw_line(left, right, color="black")
            label_location = (right[0], right[1]+self.config.axes_label_offset)
            self.viz_figs["ytick_label_{}".format(val)] = self.window['_GRAPH_'].draw_text("{}".format(val), label_location)


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
        if event_type == "_INC_CYCLE_":
            self.runtime_manager.increment_robot_cycle("robot1")
        if event_type == "_DEC_CYCLE_":
            self.runtime_manager.decrement_robot_cycle("robot1")
        if event_type == "_INC_ACTION_":
            self.runtime_manager.increment_robot_action("robot1")
        if event_type == "_DEC_ACTION_":
            self.runtime_manager.decrement_robot_action("robot1")
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


