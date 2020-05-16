import time
import logging

from MarvelMindHandler import RobotPositionHandler, MockRobotPositionHandler
from RobotClient import RobotClient, BaseStationClient, MockRobotClient, MockBaseStationClient
from FieldPlanner import ActionTypes, Action

# Debugging flags
OFFLINE_TESTING = True
SKIP_BASE_STATION = True
SKIP_MARVELMIND = True

class NonBlockingTimer:

    def __init__(self, trigger_time):
        self.start_time = time.time()
        self.trigger_time = trigger_time

    def check(self):
        if time.time() - self.start_time > self.trigger_time:
            return True 
        else:
            return False

def write_file(filename, text):
    with open(filename, 'w+') as f:
        f.write(text)

class RobotInterface:
    def __init__(self, config, robot_id):
        self.pos_handler = None
        self.comms_online = False
        self.position_online = False
        self.robot_client = None
        self.config = config
        self.robot_id = robot_id
        self.position_init_timer = None
        self.last_status_time = 0
        self.last_status = None

    def attach_pos_handler(self, pos_handler):
        self.pos_handler = pos_handler
        # If we haven't shut down from the previous time, we are ready to stream now
        self.position_online = self.pos_handler.still_running

    def bring_online(self, use_mock=False):
        self._bring_comms_online(use_mock)
        self._bring_position_online()

    def check_online(self):
        return self.comms_online and self.position_online

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
                logging.info("Connected to {} oover wifi".format(self.robot_id))
        except Exception as e:
            logging.info("Couldn't connect to {} over wifi. Reason: {}".format(self.robot_id , repr(e)))
            return 

    def _bring_position_online(self, quick=False):
        if not self.pos_handler:
            logging.info("No position handler attached for {}".format(self.robot_id))
            return
    
        if self.position_online:
            logging.info("Pos handler still running, skipping beacon wake up on {}".format(self.robot_id))
            return

        try:
            # Setup marvelmind devices and position handler
            logging.info("Attempting to enable marvelmind beacons on {} ".format(self.robot_id))
            self.pos_handler.wake_robot(self.robot_id)
            self.position_init_timer = NonBlockingTimer(30)
        except Exception as e:
            logging.info("Couldn't enable beacons on robot {}. Reason: {}".format(self.robot_id, repr(e)))

    def _get_status_from_robot(self):
        self.last_status = self.robot_client.request_status()
        self.last_status_time = time.time()

    def update(self):

        # Check if position is ready to stream
        if not self.position_online and self.position_init_timer and self.position_init_timer.check():
            self.position_online = True

        if self.comms_online:

            # Send robot position if available
            if self.position_online and self.pos_handler.new_data_ready(self.robot_id):
                pos = self.pos_handler.get_position(self.robot_id)
                self.robot_client.send_position(pos[0], pos[1], pos[2])

            # Request a status update if needed
            if time.time() - self.last_status_time > self.config.robot_status_wait_time:
                # Note that this causes a slight delay in the lifter stepper motors
                self._get_status_from_robot()

    def run_action(self, action):
        
        if not self.comms_online:
            return

        logging.info("Running {} on {}".format(action.action_type, self.robot_id))

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
        else:
            logging.info("Unknown action: {}".format(action.action_type))



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
            self._get_status_from_base_station()

    def _get_status_from_base_station(self):
        self.last_status = self.client.request_status()
        if self.last_status == None or self.last_status == "":
            self.last_status = "Base station status not available!"
        self.last_status_time = time.time()

    def run_action(self, action):

        if not self.comms_online:
            return
        
        # TODO: impliment commands


class RuntimeManager:

    STATUS_NOT_INITIALIZED = 0
    STATUS_PARTIALLY_INITIALIZED = 1
    STATUS_FULLY_INITIALIZED = 2

    def __init__(self, config):

        self.config = config
        self.initialization_status = RuntimeManager.STATUS_NOT_INITIALIZED

        self.robots = {n: RobotInterface(config, n) for n in self.config.ip_map.keys()}
        self.base_station = BaseStationInterface(config)

        if OFFLINE_TESTING or SKIP_MARVELMIND:
            self.pos_handler = MockRobotPositionHandler(config)
        else:
            self.pos_handler = RobotPositionHandler(config)

        self.last_metrics = {}
        self.cycle_tracker = {n: {'action': None, 'cycle': None, 'step': 0} for n in self.robots.keys()}
        self.idle_bots = set([n for n in self.robots.keys()])

    def initialize(self):

        # Handle testing/mock case
        use_base_station_mock = False
        use_robot_mock = False
        if OFFLINE_TESTING:
            use_base_station_mock = True
            use_robot_mock = True
        if SKIP_BASE_STATION:
            use_base_station_mock = True
        
        # Bring everything online
        self.base_station.bring_online(use_base_station_mock)
        for robot in self.robots.values():
            robot.attach_pos_handler(self.pos_handler)
            robot.bring_online(use_robot_mock)
        
        self._check_initialization_status()

    def shutdown(self, keep_mm_awake):

        logging.info("Shutting down")
        if self.initialization_status != RuntimeManager.STATUS_NOT_INITIALIZED:
            if not keep_mm_awake:
                for robot in self.robots.values():
                    self.pos_handler.sleep_robot(robot.robot_id)
                write_file(self.config.mm_beacon_state_file, 'False')
            self.pos_handler.close()

    def get_initialization_status(self):
        self._check_initialization_status()
        logging.info("Initialization status: {}".format(self.initialization_status))
        return self.initialization_status

    def get_all_metrics(self):
        return self.last_metrics

    def update(self):
        self._check_initialization_status()

        self.pos_handler.service_queues()

        self.base_station.update()
        for robot in self.robots.values():
            robot.update()

        self._update_all_metrics()
        self._update_cycle_actions()

    def run_manual_action(self, manual_action):
        target = manual_action[0]
        action = manual_action[1]
        self._run_action(target, action)

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
        ready = []
        ready.append(self.base_station.check_online())
        for robot in self.robots.values():
            ready.append(robot.check_online())
        
        if all(ready):
            self.initialization_status = RuntimeManager.STATUS_FULLY_INITIALIZED
            write_file(self.config.mm_beacon_state_file, 'True')
        elif any(ready):
            self.initialization_status = RuntimeManager.STATUS_PARTIALLY_INITIALIZED

    def _update_all_metrics(self):
        self.last_metrics = {}
        self.last_metrics['pos'] = self.pos_handler.get_metrics()
        self.last_metrics['base'] = self.base_station.get_last_status()
        self.last_metrics['plan'] = {} # TODO: Fill in data for plan exectution
        for robot in self.robots.values():
            self.last_metrics[str(robot.robot_id)] = robot.get_last_status()

    def _update_cycle_actions(self):

        # Use metrics to update in progress actions
        for robot_id, metric in self.last_metrics.items():
            # No updates needed for pos tracker, bbse station, or plan
            if robot_id in ['pos', 'plan', 'base']:
                continue

            # Figure out if we need to do any updates on the running actions
            tracker_data = self.cycle_tracker[robot_id]
            if tracker_data['cycle'] is not None:
                start_next_action = False
                
                # If we got a new cycle but haven't started an action yet, start the first action
                if tracker_data['action'] is None:
                    start_next_action = True

                # If the robot was doing an action and is now finished, start the next one
                if tracker_data['action'] is not None and not metric['in_progress']:
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
                        logging.info("Starting action {} on {}".format(next_action.name, robot_id))
                        self._run_action(robot_id, next_action)

        # Update if any robots are idle
        for robot, data in self.cycle_tracker.items():
            if data['action'] == None and data['cycle'] == None:
                self.idle_bots.add(robot)
