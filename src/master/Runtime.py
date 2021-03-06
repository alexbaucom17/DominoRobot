import time
import logging
import enum
import pprint
import pickle
import os
import json
import copy
import math

from numpy.core.defchararray import add

from MarvelMindHandler import MarvelmindWrapper, MockMarvelmindWrapper
from RobotClient import RobotClient, BaseStationClient, MockRobotClient, MockBaseStationClient
from FieldPlanner import ActionTypes, Action, TestPlan, RunFieldPlanning
from Utils import write_file, NonBlockingTimer, ActionValidator

class RobotInterface:
    def __init__(self, config, robot_id):
        self.comms_online = False
        self.robot_client = None
        self.config = config
        self.robot_id = robot_id
        self.last_status_time = 0
        self.last_status = None
        self.simple_action_map = {}
        self.current_action = ActionTypes.NONE
        self.current_move_data = []
        self.current_action_timer = None
        self.wait_timer = None
        self.waiting_active = False

    def bring_online(self, use_mock=False):
        self._bring_comms_online(use_mock)

    def check_online(self):
        return self.comms_online

    def get_robot_metrics(self):
        metrics = copy.copy(self.last_status)
        if metrics and type(metrics) is dict:
            metrics['current_action'] = str(self.current_action)
            metrics['current_move_data'] = self.current_move_data
        if metrics and self.waiting_active:
            metrics['in_progress'] = True
        return metrics

    def _setup_action_map(self):
        self.simple_action_map = {
            ActionTypes.LOAD: self.robot_client.load,
            ActionTypes.PLACE: self.robot_client.place,
            ActionTypes.TRAY_INIT: self.robot_client.tray_init,
            ActionTypes.LOAD_COMPLETE: self.robot_client.load_complete,
            ActionTypes.ESTOP: self.robot_client.estop,
            ActionTypes.CLEAR_ERROR: self.robot_client.clear_error,
            ActionTypes.WAIT_FOR_LOCALIZATION: self.robot_client.wait_for_localization,
            ActionTypes.TOGGLE_VISION_DEBUG: self.robot_client.toggle_vision_debug,
            ActionTypes.START_CAMERAS: self.robot_client.start_cameras,
            ActionTypes.STOP_CAMERAS: self.robot_client.stop_cameras,
        }

    def _bring_comms_online(self, use_mock=False):
        try:
            logging.info("Attempting to connect to {} over wifi".format(self.robot_id))

            if use_mock:
                self.robot_client = MockRobotClient(self.config, self.robot_id)
            else:
                self.robot_client = RobotClient(self.config, self.robot_id)

            if self.robot_client.net_status():
                self.comms_online = True
                self._setup_action_map()
                logging.info("Connected to {} over wifi".format(self.robot_id))
        except Exception as e:
            logging.info("Couldn't connect to {} over wifi".format(self.robot_id))
            return 

    def _get_status_from_robot(self):
        self.last_status = self.robot_client.request_status()
        self.last_status_time = time.time()
        
        # Reset current action and move data if the robot finished an action
        if self.current_action != ActionTypes.NONE and self.last_status is not None and \
            'in_progress' in self.last_status.keys() and not self.last_status['in_progress'] and self.current_action_timer.check():
            self.current_action = ActionTypes.NONE
            self.current_move_data = []


    def update(self):

        if self.comms_online:

            # Request a status update if needed
            if time.time() - self.last_status_time > self.config.robot_status_wait_time:
                try:
                    self._get_status_from_robot()
                except RuntimeError:
                    logging.info("Network connection with {} lost".format(self.robot_id))
                    self.comms_online = False
                    self.last_status = "{} offline".format(self.robot_id)

            if self.waiting_active:
                if self.wait_timer.check():
                    self.waiting_active = False
                    self.wait_timer = None

    def run_action(self, action):
        
        if not self.comms_online:
            return

        logging.info("Running {} on {}".format(action.action_type, self.robot_id))

        try:
            if action.action_type is not None:
                self.current_action = action.action_type
            
            if action.action_type == ActionTypes.MOVE_COARSE:
                self.robot_client.move(action.x, action.y, action.a)
                self.current_move_data = [action.x, action.y, action.getAngleDegrees()]
            elif action.action_type == ActionTypes.MOVE_REL:
                self.robot_client.move_rel(action.x, action.y, action.a)
            elif action.action_type == ActionTypes.MOVE_REL_SLOW:
                self.robot_client.move_rel_slow(action.x, action.y, action.a)
            elif action.action_type == ActionTypes.MOVE_FINE:
                self.robot_client.move_fine(action.x, action.y, action.a)
                self.current_move_data = [action.x, action.y, action.getAngleDegrees()]
            elif action.action_type == ActionTypes.MOVE_FINE_STOP_VISION:
                self.robot_client.move_fine_stop_vision(action.x, action.y, action.a)
                self.current_move_data = [action.x, action.y, action.getAngleDegrees()]
            elif action.action_type == ActionTypes.MOVE_WITH_VISION:
                self.robot_client.move_with_vision(action.x, action.y, action.a)
            elif action.action_type == ActionTypes.MOVE_CONST_VEL:
                self.robot_client.move_const_vel(action.vx, action.vy, action.va, action.t)
            elif action.action_type == ActionTypes.SET_POSE:
                self.robot_client.set_pose(action.x, action.y, action.a)
            elif action.action_type == ActionTypes.NET:
                status = self.robot_client.net_status()
                logging.info("Robot {} network status is: {}".format(self.robot_id, status))
            elif action.action_type == ActionTypes.WAIT:
                self.wait_timer = NonBlockingTimer(action.time)
                self.waiting_active = True
            elif action.action_type in self.simple_action_map.keys():
                self.simple_action_map[action.action_type]()
            else:
                logging.info("Unknown action: {}".format(action.action_type))

            self.current_action_timer = NonBlockingTimer(self.config.robot_next_action_wait_time)
        except RuntimeError:
            logging.info("Network connection with {} lost".format(self.robot_id))
            self.comms_online = False
            self.last_status = "{} offline".format(self.robot_id)



class BaseStationInterface:

    def __init__(self, config):
        self.config = config
        self.client = None
        self.comms_online = False
        self.last_status = None
        self.last_status_time = 0

    def bring_online(self, use_mock=False):
        logging.info("Bringing BaseStation comms online")
        if use_mock:
            self.client = MockBaseStationClient(self.config)
        else:
            self.client = BaseStationClient(self.config)

        if self.client.net_status():
            self.comms_online = True

    def get_last_status(self):
        return self.last_status

    def check_online(self):
        return self.comms_online

    def update(self):
        # Request a status update if needed
        if time.time() - self.last_status_time > self.config.base_station_status_wait_time:
            try:
                self._get_status_from_base_station()
            except RuntimeError:
                logging.info("Network connection with base station lost")
                self.comms_online = False
                self.last_status = "Base station offline"

    def _get_status_from_base_station(self):
        self.last_status = self.client.request_status()
        if self.last_status == None or self.last_status == "":
            self.last_status = "Base station status not available!"
        self.last_status_time = time.time()

    def run_action(self, action):

        if not self.comms_online:
            return

        try:
            if action.action_type == ActionTypes.ESTOP:
                self.client.estop()
            elif action.action_type == ActionTypes.NET:
                status = self.client.net_status()
                logging.info("Base station network status is: {}".format(status))
            elif action.action_type == ActionTypes.LOAD:
                self.client.load()
            else:
                logging.info("Unknown action: {}".format(action.action_type))

        except RuntimeError:
            logging.info("Network connection with base station lost")
            self.comms_online = False
            self.last_status = "Base station offline"

class PlanStatus(enum.Enum):
    NONE = 0,
    LOADED = 1,
    RUNNING = 2,
    PAUSED = 3,
    ABORTED = 4,
    DONE = 5,

class RuntimeManager:

    STATUS_NOT_INITIALIZED = 0
    STATUS_PARTIALLY_INITIALIZED = 1
    STATUS_FULLY_INITIALIZED = 2

    def __init__(self, config):

        self.config = config

        self.robots = {id: RobotInterface(config, id) for id in self.config.ip_map.keys()}
        self.base_station = BaseStationInterface(config)

        self.initialization_status = RuntimeManager.STATUS_NOT_INITIALIZED

        self.component_initialization_status = {id: False for id in self.config.ip_map.keys()}
        self.component_initialization_status['mm'] = False
        self.component_initialization_status['base_station'] = False

        if self.config.OFFLINE_TESTING or self.config.SKIP_MARVELMIND:
            self.mm_wrapper = MockMarvelmindWrapper(config)
        else:
            self.mm_wrapper = MarvelmindWrapper(config)

        self.last_metrics = {}
        self.idle_bots = set([n for n in self.robots.keys()])
        self.initialization_timer = None

        self.cycle_tracker = {n: {'action_id': None, 'cycle_id': None, 'timer': None, 'needs_restart': False, 'action_validator': ActionValidator()} for n in self.robots.keys()}
        self.next_cycle_number = 0
        self.plan = None
        self.plan_path = ""
        self.plan_status = PlanStatus.NONE
        if self.config.USE_TEST_PLAN:
            self._load_plan_from_file('')
        elif self.config.REGEN_PLAN:
            logging.info("Regenerating and loading plan")
            plan = RunFieldPlanning(autosave=True)
            self._load_plan_from_object(plan, os.path.join(self.config.plans_dir, "autosaved.p"))
        elif self.config.AUTO_LOAD_PLAN and self.config.AUTO_LOAD_PLAN_NAME:
            plan_path = os.path.join(self.config.plans_dir, self.config.AUTO_LOAD_PLAN_NAME)
            self._load_plan_from_file(plan_path)

    def initialize(self):

        if self.initialization_timer and not self.initialization_timer.check():
            return

        # Handle testing/mock case
        use_base_station_mock = False
        use_robot_mock = False
        if self.config.OFFLINE_TESTING:
            use_base_station_mock = True
            use_robot_mock = True
        if self.config.SKIP_BASE_STATION:
            use_base_station_mock = True
        
        # Bring everything online if needed
        if not self.component_initialization_status['base_station']:
            self.base_station.bring_online(use_base_station_mock)
        if not self.component_initialization_status['mm']:
            self.mm_wrapper.wake_all_devices_only_if_needed()
        for id, robot in self.robots.items():
            if not self.component_initialization_status[id]:
                robot.bring_online(use_robot_mock)
        
        # Check the status of initialization
        self._check_initialization_status()
        if self.initialization_status != RuntimeManager.STATUS_FULLY_INITIALIZED:
            self.initialization_timer = NonBlockingTimer(3)
            logging.info("Unable to fully initialize RuntimeManager, will try again in 3 seconds")
            logging.info("Current componenent initialization status:")
            logging.info(pprint.pformat(self.component_initialization_status, indent=2, width=10))


    def shutdown(self, keep_mm_awake):
        logging.info("Shutting down")
        if self.initialization_status != RuntimeManager.STATUS_NOT_INITIALIZED:
            if not keep_mm_awake:
                self.mm_wrapper.sleep_all_devices()

    def get_initialization_status(self):
        self._check_initialization_status()
        return self.initialization_status

    def get_all_metrics(self):
        return self.last_metrics

    def update(self):
        self._check_initialization_status()
        if self.get_initialization_status != RuntimeManager.STATUS_FULLY_INITIALIZED:
            self.initialize()

        self.base_station.update()
        for robot in self.robots.values():
            robot.update()

        self._update_all_metrics()

        if self.plan_status == PlanStatus.RUNNING:
            self._update_plan()
            self._update_cycle_actions()
            self._cycle_state_to_file()

    def run_manual_action(self, manual_action):
        target = manual_action[0]
        action = manual_action[1]
        self._run_action(target, action)

    def estop(self):
        logging.warning("ESTOP")
        self._run_action('base', Action(ActionTypes.ESTOP, 'ESTOP'))
        for robot in self.robots.values():
            robot.run_action(Action(ActionTypes.ESTOP, 'ESTOP'))

    def load_plan(self, plan_file):
        if plan_file and os.path.basename(plan_file).split('.')[1] == 'json':
            self._load_cycle_state_from_file(plan_file)
        else:
            self._load_plan_from_file(plan_file)        

    def set_plan_status(self, status):
        
        # Clean up if we don't want to save the plan state
        if status == PlanStatus.ABORTED:
            self.next_cycle_number = 0
            self.cycle_tracker = {n: {'action_id': None, 'cycle_id': None, 'timer': None, 'needs_restart': False, 'action_validator': ActionValidator()} for n in self.robots.keys()}
            self.idle_bots = set([n for n in self.robots.keys()])
        elif status == PlanStatus.PAUSED:
            for data in self.cycle_tracker.values():
                data['needs_restart'] = True

        self.plan_status = status

    def increment_robot_cycle(self, target):
        self._modify_cycle_state(target, add_cycle_id=1)
    
    def decrement_robot_cycle(self, target):
        self._modify_cycle_state(target, add_cycle_id=-1)

    def increment_robot_action(self, target):
        self._modify_cycle_state(target, add_action_id=1)

    def decrement_robot_action(self, target):
        self._modify_cycle_state(target, add_action_id=-1)

    def set_cycle(self, target, cycle_num):
        self._modify_cycle_state(target, add_cycle_id=cycle_num, absolute=True)

    def set_action(self, target, action_num):
        self._modify_cycle_state(target, add_action_id=action_num, absolute=True)

    def get_plan_status(self):
        return self.plan_status

    def get_plan_info(self):
        if not self.plan:
            return None
        else:
            return (self.plan, self.plan_status, self.next_cycle_number-1)

    def _load_plan_from_object(self, plan, plan_path=""):
        self.plan = plan
        self.plan_status = PlanStatus.LOADED
        self.plan_path = plan_path
        for key in self.robots.keys():
            self.set_cycle(key, 0)
            self.set_action(key, 0)

    def _load_plan_from_file(self, plan_file):
        if self.config.USE_TEST_PLAN or plan_file == "testplan":
            self._load_plan_from_object(TestPlan(), "testplan")
        else:
            with open(plan_file, 'rb') as f:
                plan = pickle.load(f)
                self._load_plan_from_object(plan, plan_file)
                logging.info("Loaded plan from {}".format(plan_file))  
        for key in self.robots.keys():
            self.set_cycle(key, 0)
            self.set_action(key, 0)

    def _update_plan(self):
        # If we have an idle robot, send it the next cycle to execute
        if self.plan_status == PlanStatus.RUNNING and self._any_idle_bots():
            next_cycle = self.plan.get_cycle(self.next_cycle_number)
            
            # If we get none, that means we are done with the plan
            if next_cycle is None:
                self.next_cycle_number = 0
                logging.info("Completed plan!")
                self.plan_status = PlanStatus.DONE
                self._erase_cycle_state_file()
            else:
                logging.info("Sending cycle {} for execution".format(self.next_cycle_number))
                self.next_cycle_number += 1
                self._assign_new_cycle(next_cycle)

    def _any_idle_bots(self):
        return len(self.idle_bots) > 0

    def _assign_new_cycle(self, cycle):
        expected_robot = cycle.robot_id
        cycle_id = cycle.id
        if expected_robot not in self.idle_bots:
            raise ValueError("Expected cycle {} to be run with {} but only available robots are {}".format(cycle_id, expected_robot, self.idle_bots))
        else:
            logging.info("Assigning cycle {} to {}".format(cycle_id, expected_robot))
            self.cycle_tracker[expected_robot]['cycle_id'] = cycle_id
            self.cycle_tracker[expected_robot]['action_id'] = None

    def _run_action(self, target, action):
        if action.action_type == ActionTypes.PAUSE_PLAN:
            self.set_plan_status(PlanStatus.PAUSED)
            self.increment_robot_action(target)
        elif target == 'base':
            self.base_station.run_action(action)
        elif 'robot' in target:
            self.robots[target].run_action(action)
            if target in self.idle_bots:
                self.idle_bots.remove(target)
        else:
            logging.info("Unknown target: {}".format(target))

    def _check_initialization_status(self):
        self.component_initialization_status['base_station'] = self.base_station.check_online()
        self.component_initialization_status['mm'] = self.mm_wrapper.check_all_devices_status()
        for id,robot in self.robots.items():
            self.component_initialization_status[id] = robot.check_online()
        
        ready = [i for i in self.component_initialization_status.values()]
        if all(ready):
            self.initialization_status = RuntimeManager.STATUS_FULLY_INITIALIZED
        elif any(ready):
            self.initialization_status = RuntimeManager.STATUS_PARTIALLY_INITIALIZED

    def _update_all_metrics(self):
        self.last_metrics = {}
        self.last_metrics['mm'] = self.mm_wrapper.get_metrics()
        self.last_metrics['base'] = self.base_station.get_last_status()
        self.last_metrics['plan'] = self._get_plan_metrics()
        for robot in self.robots.values():
            robot_metrics = robot.get_robot_metrics()
            self.last_metrics[str(robot.robot_id)] = robot_metrics

            # Check if the robot has an error that would require pausing the plan
            robot_has_error = robot_metrics and "error_status" in robot_metrics and robot_metrics["error_status"]
            plan_running = self.plan_status == PlanStatus.RUNNING
            if plan_running and robot_has_error:
                logging.warning("Pausing plan due to error on {}. Please address before proceeding.".format(robot.robot_id))
                self.set_plan_status(PlanStatus.PAUSED)

    def _get_plan_metrics(self):
        plan_metrics = {}
        plan_metrics['status'] = self.plan_status
        plan_metrics['filename'] = os.path.basename(self.plan_path)
        plan_metrics['next_cycle_number'] = self.next_cycle_number
        plan_metrics['idle_bots'] = list(self.idle_bots)
        robot_metrics = {}
        for id, data in self.cycle_tracker.items():
            robot_metrics[id] = {}
            robot_metrics[id] = {'cycle_id': 'None', 'action_name': 'None', 'action_id': 'None', 'vision_offset': 'None','tile_coordinate': 'None'}
            if data['cycle_id'] is not None:
                robot_metrics[id]['cycle_id'] = data['cycle_id'] 
                robot_metrics[id]['tile_coordinate'] = self.plan.field.tiles[data['cycle_id']].coordinate               
            if data['action_id'] is not None:
                robot_metrics[id]['action_id'] = data['action_id']
                action = self.plan.get_action(data['cycle_id'], data['action_id'])
                if action:
                    robot_metrics[id]['action_name'] = action.name
                if action and action.action_type == ActionTypes.MOVE_WITH_VISION:
                    robot_metrics[id]['vision_offset'] = (action.x, action.y, math.degrees(action.a))
                else:
                    robot_metrics[id]['vision_offset'] = 'None'
            robot_metrics[id]['needs_restart'] = data['needs_restart']

        plan_metrics['robots'] = robot_metrics
        return plan_metrics

    def _update_cycle_actions(self):

        # Use metrics to update in progress actions
        for robot_id, metric in self.last_metrics.items():
            # No updates needed for pos tracker, bbse station, or plan
            if robot_id in ['mm', 'plan', 'base']:
                continue

            if metric is None or 'in_progress' not in metric.keys():
                continue

            # Figure out if we need to do any updates on the running actions
            tracker_data = self.cycle_tracker[robot_id]
            if tracker_data['cycle_id'] is not None:
                cycle = self.plan.get_cycle(tracker_data['cycle_id'])

                # Check if this action needs to be restarted due to being paused or interrupted
                if tracker_data['needs_restart']:
                    tracker_data['timer'] = NonBlockingTimer(self.config.robot_next_action_wait_time)
                    action = cycle.action_sequence[tracker_data['action_id']]
                    logging.info("Re-starting action {} ({}) on {}".format(tracker_data['action_id'], action.name, robot_id))
                    self._run_action(robot_id, action)
                    tracker_data['action_validator'].update_expected_action(action.action_type)
                    tracker_data['needs_restart'] = False

                # The timer check here is to give a delay for the robot to actually start the action 
                # before master checks if it is finished
                action_timer_ready = tracker_data['timer'] is None or tracker_data['timer'].check()

                # If we got a new cycle but haven't started an action yet, start the first action
                start_next_action = False
                if tracker_data['action_id'] is None:
                    start_next_action = True

                # If the robot was doing an action and is now finished, start the next one
                action_assigned = tracker_data['action_id'] is not None
                action_finished = not metric['in_progress']
                previous_action_validated = tracker_data['action_validator'].update_action_validation(metric)
                if action_assigned and action_finished and action_timer_ready:
                    start_next_action = True
                    if not previous_action_validated:
                        logging.warning("Everything else finished but previous action not validated")

                if start_next_action:
                    # Check if there is a new action to run for this cycle, if not, end the cycle
                    if tracker_data['action_id'] is not None and (tracker_data['action_id'] + 1) >= len(cycle.action_sequence):
                        logging.info("{} finished cycle {}".format(robot_id, cycle.id))
                        tracker_data['cycle_id'] = None
                        tracker_data['action_id'] = None

                    else:
                        # If there is a new action to run, start it
                        if tracker_data['action_id'] is None:
                            tracker_data['action_id'] = 0
                        else:
                            tracker_data['action_id'] += 1
                        tracker_data['timer'] = NonBlockingTimer(self.config.robot_next_action_wait_time)
                        next_action = cycle.action_sequence[tracker_data['action_id']]
                        logging.info("Starting action {} ({}) on {}".format(tracker_data['action_id'], next_action.name, robot_id))
                        self._run_action(robot_id, next_action)
                        tracker_data['action_validator'].update_expected_action(next_action.action_type)

        # Update if any robots are idle
        for robot, data in self.cycle_tracker.items():
            if data['action_id'] == None and data['cycle_id'] == None:
                self.idle_bots.add(robot)

    def _modify_cycle_state(self, target, add_cycle_id=None, add_action_id=None, absolute=False):
        if self.plan_status == PlanStatus.RUNNING:
            logging.warning("Cannot update cycle state while plan is running.")
            return
        if target not in self.cycle_tracker.keys():
            logging.warning("Invalid target: {}".format(target))
            return
        
        cur_cycle_id = self.cycle_tracker[target]["cycle_id"]
        if cur_cycle_id is None:
            cur_cycle_id = 0
        new_cycle_id = cur_cycle_id
        if add_cycle_id is not None:
            if absolute:
                new_cycle_id = add_cycle_id
            else:                
                new_cycle_id = cur_cycle_id + add_cycle_id
            if self.plan and new_cycle_id >= len(self.plan.cycles) or new_cycle_id < 0:
                logging.warning("Cycle id {} out of bounds".format(new_cycle_id))
                return
        
        cur_action_id = self.cycle_tracker[target]["action_id"]
        if cur_action_id is None:
            cur_action_id = 0
        new_action_id = cur_action_id
        if add_action_id is not None:
            if absolute:
                new_action_id = add_action_id
            else:
                new_action_id = cur_action_id + add_action_id    
            if self.plan and new_action_id >= len(self.plan.get_cycle(new_cycle_id).action_sequence) or new_action_id < 0:
                logging.warning("Action id {} out of bounds".format(new_action_id))
                return

        data = self.cycle_tracker[target]
        data['action_id'] = new_action_id
        data['cycle_id'] = new_cycle_id
        data['timer'] = None
        data['needs_restart'] = True
        data['action_validator'] = ActionValidator()
        self.next_cycle_number = new_cycle_id + 1
        try:
            self.idle_bots.remove(target)
        except:
            pass


    def _cycle_state_to_file(self):
        if self.plan_status == PlanStatus.RUNNING:
            # Copy data and prepare to write
            robot_cycle_states = copy.deepcopy(self.cycle_tracker)
            for tracker_data in robot_cycle_states.values():
                del tracker_data['timer']
                del tracker_data['action_validator']

            data_to_dump = {}
            data_to_dump['plan_path'] = self.plan_path
            data_to_dump['next_cycle_number'] = self.next_cycle_number
            data_to_dump['robots'] = robot_cycle_states

            with open(self.config.cycle_state_file, 'w') as f:
                json.dump(data_to_dump, f)

    def _load_cycle_state_from_file(self, filepath):
        logging.info("Loading cycle state information from {}".format(filepath))
        with open(filepath, 'r') as f:
            loaded_data = json.load(f)
        
        # Set the various bits of data from the file
        self.plan_path = loaded_data['plan_path']
        self.next_cycle_number = loaded_data['next_cycle_number']
        self.cycle_tracker = loaded_data['robots']

        # Need to handle robot states carefully
        for robot, data in self.cycle_tracker.items():
            # Need to re-add timer state and tell the controller the action needs to be restarted
            data['timer'] = None
            data['needs_restart'] = True
            data['action_validator'] = ActionValidator()

            # Need to remove the robot from the idle list if it was actually doing something at the time it stopped
            if data['action_id'] is not None or data['cycle_id'] is not None:
                self.idle_bots.remove(robot)
        
        # Reload the plan data and start the status as PAUSED so that it can be resumed
        self._load_plan_from_file(self.plan_path)
        self.plan_status = PlanStatus.PAUSED

    def _erase_cycle_state_file(self):
        fname = self.config.cycle_state_file       
        if os.path.exists(fname):
            os.remove(fname)



        

