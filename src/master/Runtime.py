import time

from MarvelMindHandler import RobotPositionHandler
from RobotClient import RobotClient, BaseStationClient
from FieldPlanner import ActionTypes, Action

class NonBlockingTimer:

    def __init__(self, trigger_time):
        self.start_time = time.time()
        self.trigger_time = trigger_time

    def check(self):
        if time.time() - self.start_time > self.trigger_time:
            return True 
        else:
            return False


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

def write_file(filename, text):
    with open(filename, 'w+') as f:
        f.write(text)

class RobotInterface:
    def __init__(self, config, robot_number):
        self.pos_handler = None
        self.comms_online = False
        self.position_online = False
        self.robot_client = None
        self.config = config
        self.robot_number = robot_number
        self.position_init_timer = None
        self.last_status_time = 0
        self.last_status = None
        self.command_in_progress = False

    def attach_pos_handler(self, pos_handler):
        self.pos_handler = pos_handler
        # If we haven't shut down from the previous time, we are ready to stream now
        self.position_online = self.pos_handler.still_running

    def bring_online(self):
        self._bring_online()
        self._bring_position_online()

    def check_online(self):
        return self.comms_online and self.position_online

    def get_last_status(self):
        return self.last_status

    def get_in_progress(self):
        return self.command_in_progress

    def _bring_comms_online(self):
        try:
            print("Attempting to connect to robot {} over wifi".format(self.robot_number))
            self.robot_client = RobotClient(self.config, self.robot_number) 
            if self.robot_client.net_status()
                self.comms_online = True
        except Exception as e:
            print("Couldn't connect to robot {} online. Reason: {}".format(, repr(e)))
            return 

    def _bring_position_online(self, quick=False):
        if not self.pos_handler:
            print("No position handler attached for robot {}".format(self.robot_number))
            return
    
        if self.position_online:
            print("Pos handler still running, skipping beacon wake up on robot {}".format(self.robot_number))
            return

        try:
            # Setup marvelmind devices and position handler
            print("Attempting to enable marvelmind beacons on robot {} ".format(self.robot_number))
            self.pos_handler.wake_robot(self.robot_number)
            self.position_init_timer = NonBlockingTimer(30)
        except Exception as e:
            print("Couldn't enable beacons on robot {}. Reason: {}".format(self.robot_number, repr(e)))

    def _get_status_from_robot(self):
        self.last_status = self.robot_client.request_status()
        if self.last_status == None or self.last_status == "":
            self.last_status = "Robot status not available!"
        else:
            try:
                self.command_in_progress = self.last_status["in_progress"]
            except Exception:
                print("Status miissing expected in_progress field")
        self.last_status_time = time.time()

    def update(self):

        # Check if position is ready to stream
        if not self.position_online and self.position_init_timer and self.position_init_timer.check():
            self.position_online = True

        if self.comms_online:

            # Send robot position if available
            if self.position_online and self.pos_handler.new_data_ready(self.robot_number):
                pos = self.pos_handler.get_position(self.robot_number)
                self.robot_client.send_position(pos[0], pos[1], pos[2])

            # Request a status update if needed
            if time.time() - self.last_status_time > self.config.robot_status_wait_time:
                self._get_status_from_robot()

    def run_action(self, target, action):
        
        if not self.comms_online:
            return

        if target != self.robot_number:
            raise ValueError("Invalid target: {}. Expecting {}}".format(target, self.robot_number))

        if action.type == ActionTypes.MOVE_COARSE:
            self.robot_client.move(action.x, action.y, action.a)
        elif action.type == ActionTypes.MOVE_REL:
            self.robot_client.move_rel(action.x, action.y, action.a)
        elif action.type == ActionTypes.MOVE_FINE:
            self.robot_client.move_fine(action.x, action.y, action.a)
        elif action.type == ActionTypes.NET:
            status = self.robot_client.net_status()
            print("Robot {} network status is: {}".format(self.robot_number, status))
        else:
            print("Unknown command: {}".format(command.type))



class BaseStationInterface:

    def __init__(self, config):
        self.config = config
        self.client = None
        self.comms_online = False
        self.last_status = None
        self.last_status_time = 0

    def bring_online(self):
        print("Bringing BaseStation comms online")
        self.client = BaseStationClient(self.config)
        if self.client.net_status()
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
        else:
            try:
                self.command_in_progress = self.last_status["in_progress"]
            except Exception:
                print("Status miissing expected in_progress field")
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
        self.robots = {n: RobotInterface(config, int(n)) for n in self.config.ip_map.keys()}
        self.base_station = BaseStationInterface(config)
        self.pos_handler = RobotPositionHandler(config)
        self.initialization_status = STATUS_NOT_INITIALIZED

        self.last_metrics = {}
        self.cycle_tracker = {n: {'action': None, 'cycle': None, 'step': 0} for n in self.robots.keys()}
        self.idle_bots = set([n for n in self.robots.keys])

    def initialize(self):

        # Bring everything online
        self.base_station.bring_online()
        for robot in self.robots.values():
            robot.attach_pos_handler(self.pos_handler)
            robot.bring_online()
        
        self.check_status()

    def shutdown(self, keep_mm_awake):

        print("Shutting down")
        if self.initialization_status != STATUS_NOT_INITIALIZED:
            if not keep_mm_awake:
                for robot in self.robots.values():
                    self.pos_handler.sleep_robot(robot.robot_number)
                write_file(self.cfg.mm_beacon_state_file, 'False')
            self.pos_handler.close()

    def get_initialization_status(self):
        return self.initialization_status

    def get_all_metrics(self):
        return self.last_metrics

    def update(self):
        self._check_initialization_status()

        self.pos_handler.service_queues()

        self.base_station.update()
        for robot in self.robots:
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
            raise ValueError("Expected cycle {} to be run with robot {} but only available robots are {}".format(new_cycle.id, expected_robot, self.idle_bots))
        else:
            self.cycle_tracker[expected_robot]['cycle'] = new_cycle
            self.cycle_tracker[expected_robot]['steps'] = 0
            self.cycle_tracker[expected_robot]['action'] = None


    def _run_action(self, target, action):
        if target = 'base':
            self.base_station.run_action(action)
        elif type(target) is str or type(target) is str
            self.robots[str(target)].run_action(action)
        else:
            print("Unknown target: {}".format(target))

    def _check_initialization_status(self):
        ready = []
        ready.append(self.base_station.check_online())
        for robot in self.robots.values():
            ready.append(robot.check_online())
        
        if all(ready):
            self.status = STATUS_FULLY_INITIALIZED
            write_file(self.config.mm_beacon_state_file, 'True')
        elif any(ready)
            self.status = STATUS_PARTIALLY_INITIALIZED

    def _update_all_metrics(self):
        # TODO: Add entry for plan execution
        self.last_metrics = {}
        self.last_metrics['pos'] = self.pos_handler.get_metrics()
        self.last_metrics['base'] = self.base_station.get_last_status()
        for robot in self.robots.values():
            self.last_metrics[str(robot.robot_number)] = robot.get_last_status()

    def _update_cycle_actions(self):

        # Use metrics to update in progress actions
        for key, val in self.last_metrics:
            # No updates needed for pos tracker or base station
            if key == 'pos' or key == 'base':
                continue

            # If the robot was doing an action and is now finished, try to start the next one
            tracker_data = self.cycle_tracker[key]
            if tracker_data['action'] is not None and not val['in_progress']:
                tracker_data['step'] += 1

                # Check if there is a new action to run for this cycle, if not, end the cycle
                if tracker_data['step'] > len(tracker_data['cycle'].action_sequence):
                    tracker_data['cycle'] = None
                    tracker_data['step'] = 0

                else:
                    # If there is a new action to run, start it
                    next_action = tracker_data['cycle'].action_sequence[tracker_data['step']]
                    tracker_data['action'] = next_action
                    self._run_action(next_action)

        # Update if any robots are idle
        for robot, data in self.cycle_tracker:
            if data['action'] == None and data['cycle'] == None:
                self.idle_bots.add(robot)
