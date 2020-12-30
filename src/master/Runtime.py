import time
import logging

from MarvelMindHandler import MarvelmindWrapper, MockMarvelmindWrapper
from RobotClient import RobotClient, BaseStationClient, MockRobotClient, MockBaseStationClient
from FieldPlanner import ActionTypes, Action
from Utils import write_file, NonBlockingTimer
import pprint

# Debugging flags
OFFLINE_TESTING = False
SKIP_BASE_STATION = True
SKIP_MARVELMIND = True

class RobotInterface:
    def __init__(self, config, robot_id):
        self.comms_online = False
        self.robot_client = None
        self.config = config
        self.robot_id = robot_id
        self.last_status_time = 0
        self.last_status = None

    def bring_online(self, use_mock=False):
        self._bring_comms_online(use_mock)

    def check_online(self):
        return self.comms_online

    def get_last_status(self):
        return self.last_status

    def _bring_comms_online(self, use_mock=False):
        try:
            logging.info("Attempting to connect to {} over wifi".format(self.robot_id))

            if use_mock:
                self.robot_client = MockRobotClient(self.config, self.robot_id)
            else:
                self.robot_client = RobotClient(self.config, self.robot_id)

            if self.robot_client.net_status():
                self.comms_online = True
                logging.info("Connected to {} over wifi".format(self.robot_id))
        except Exception as e:
            logging.info("Couldn't connect to {} over wifi".format(self.robot_id))
            return 

    def _get_status_from_robot(self):
        self.last_status = self.robot_client.request_status()
        self.last_status_time = time.time()

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

    def run_action(self, action):
        
        if not self.comms_online:
            return

        logging.info("Running {} on {}".format(action.action_type, self.robot_id))

        try:
            if action.action_type == ActionTypes.MOVE_COARSE:
                self.robot_client.move(action.x, action.y, action.a)
            elif action.action_type == ActionTypes.MOVE_REL:
                self.robot_client.move_rel(action.x, action.y, action.a)
            elif action.action_type == ActionTypes.MOVE_FINE:
                self.robot_client.move_fine(action.x, action.y, action.a)
            elif action.action_type == ActionTypes.NET:
                status = self.robot_client.net_status()
                logging.info("Robot {} network status is: {}".format(self.robot_id, status))
            elif action.action_type == ActionTypes.LOAD:
                self.robot_client.load()
            elif action.action_type == ActionTypes.PLACE:
                self.robot_client.place()
            elif action.action_type == ActionTypes.TRAY_INIT:
                self.robot_client.tray_init()
            elif action.action_type == ActionTypes.LOAD_COMPLETE:
                self.robot_client.load_complete()
            elif action.action_type == ActionTypes.ESTOP:
                self.robot_client.estop()
            elif action.action_type == ActionTypes.MOVE_CONST_VEL:
                self.robot_client.move_const_vel(action.vx, action.vy, action.va, action.t)
            elif action.action_type == ActionTypes.CLEAR_ERROR:
                self.robot_client.clear_error()
            else:
                logging.info("Unknown action: {}".format(action.action_type))
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

        if OFFLINE_TESTING or SKIP_MARVELMIND:
            self.mm_wrapper = MockMarvelmindWrapper(config)
        else:
            self.mm_wrapper = MarvelmindWrapper(config)

        self.last_metrics = {}
        self.cycle_tracker = {n: {'action': None, 'cycle': None, 'step': 0, 'timer': None} for n in self.robots.keys()}
        self.idle_bots = set([n for n in self.robots.keys()])
        self.initialization_timer = None

    def initialize(self):

        if self.initialization_timer and not self.initialization_timer.check():
            return

        # Handle testing/mock case
        use_base_station_mock = False
        use_robot_mock = False
        if OFFLINE_TESTING:
            use_base_station_mock = True
            use_robot_mock = True
        if SKIP_BASE_STATION:
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
            self.initialization_timer = NonBlockingTimer(10)
            logging.info("Unable to fully initialize RuntimeManager, will try again in 10 seconds")
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
        self._update_cycle_actions()

    def run_manual_action(self, manual_action):
        target = manual_action[0]
        action = manual_action[1]
        self._run_action(target, action)

    def estop(self):
        logging.warn("ESTOP")
        self._run_action('base', Action(ActionTypes.ESTOP, 'ESTOP'))
        for robot in self.robots.values():
            robot.run_action(Action(ActionTypes.ESTOP, 'ESTOP'))

    def any_idle_bots(self):
        return len(self.idle_bots) > 0

    def assign_new_cycle(self, new_cycle):
        expected_robot = new_cycle.robot_id
        if expected_robot not in self.idle_bots:
            raise ValueError("Expected cycle {} to be run with {} but only available robots are {}".format(new_cycle.id, expected_robot, self.idle_bots))
        else:
            logging.info("Assigning cycle {} to {}".format(new_cycle.id, expected_robot))
            self.cycle_tracker[expected_robot]['cycle'] = new_cycle
            self.cycle_tracker[expected_robot]['step'] = 0
            self.cycle_tracker[expected_robot]['action'] = None

    def _run_action(self, target, action):
        if target == 'base':
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
            self.last_metrics[str(robot.robot_id)] = robot.get_last_status()

    def _get_plan_metrics(self):
        plan_metrics = {}
        for id, data in self.cycle_tracker.items():
            if data['cycle']:
                plan_metrics[id] = {'cycle': data['cycle'].id}
            else:
                plan_metrics[id] = {'cycle': 'None'}
            if data['action']:
                plan_metrics[id]['action'] = data['action'].name
            else:
                plan_metrics[id]['action'] = 'None'

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
            if tracker_data['cycle'] is not None:
                start_next_action = False

                # The timer check here is to give a delay for the robot to actually start the action 
                # before master checks if it is finished
                action_timer_ready = tracker_data['timer'] is None or tracker_data['timer'].check()

                # If we got a new cycle but haven't started an action yet, start the first action
                if tracker_data['action'] is None:
                    start_next_action = True

                # If the robot was doing an action and is now finished, start the next one
                tracker_ok = tracker_data['action'] is not None
                metric_ok = not metric['in_progress']
                if tracker_ok and metric_ok and action_timer_ready:
                    tracker_data['step'] += 1
                    start_next_action = True

                if start_next_action:
                    # Check if there is a new action to run for this cycle, if not, end the cycle
                    if tracker_data['step'] >= len(tracker_data['cycle'].action_sequence):
                        logging.info("{} finished cycle {}".format(robot_id, tracker_data['cycle'].id))
                        tracker_data['cycle'] = None
                        tracker_data['step'] = 0
                        tracker_data['action'] = None

                    else:
                        # If there is a new action to run, start it
                        next_action = tracker_data['cycle'].action_sequence[tracker_data['step']]
                        tracker_data['action'] = next_action
                        tracker_data['timer'] = NonBlockingTimer(self.config.robot_next_action_wait_time)
                        logging.info("Starting action {} on {}".format(next_action.name, robot_id))
                        self._run_action(robot_id, next_action)

        # Update if any robots are idle
        for robot, data in self.cycle_tracker.items():
            if data['action'] == None and data['cycle'] == None:
                self.idle_bots.add(robot)
