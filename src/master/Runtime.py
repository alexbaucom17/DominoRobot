import time
import enum
from MarvelMindHandler import RobotPositionHandler
from RobotClient import RobotClient, BaseStationClient

class NonBlockingTimer:

    def __init__(self, trigger_time):
        self.start_time = time.time()
        self.trigger_time = trigger_time

    def check(self):
        if time.time() - self.start_time > self.trigger_time:
            return True 
        else:
            return False

class CommandTypes(enum.Enum):
    MOVE = 'move',
    MOVE_FINE = 'fine',
    MOVE_REL = 'rel',
    NET = 'net',
    LOAD = 'load',
    PLACE = 'place'

class Command:
    def __init__(self, target, cmd_type, data):
        self.target = target
        self.type = cmd_type
        self.data = data

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

    def run_command(self, command):
        
        if not self.comms_online:
            return

        if command.target != self.robot_number:
            raise ValueError("Invalid target: {}. Expecting {}}".format(command.target, self.robot_number))

        if command.type == CommandTypes.MOVE:
            self.robot_client.move(float(command.data[0]), float(command.data[1]), float(command.data[2]))
        elif command.type == CommandTypes.MOVE_REL:
            self.robot_client.move_rel(float(command.data[0]), float(command.data[1]), float(command.data[2]))
        elif command.type == CommandTypes.MOVE_FINE:
            self.robot_client.move_fine(float(command.data[0]), float(command.data[1]), float(command.data[2]))
        elif command.type == CommandTypes.NET":
            status = self.robot_client.net_status()
            print("Robot {} network status is: {}".format(self.robot_number, status))
        else:
            print("Unknown command: {}, data: {}".format(command.type, command.data))



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

    def run_command(self, command):

        if not self.comms_online:
            return

        if command.target != 'base':
            raise ValueError("Invalid target: {}. Expecting base".format(command.target))


class RuntimeManager:

    STATUS_NOT_INITIALIZED = 0
    STATUS_PARTIALLY_INITIALIZED = 1
    STATUS_FULLY_INITIALIZED = 2

    def __init__(self, config):

        self.config = config
        self.robots = [RobotInterface(config, n) for n in config.ip_map.keys()]
        self.base_station = BaseStationInterface(config)
        self.pos_handler = RobotPositionHandler(config)
        self.initialization_status = STATUS_NOT_INITIALIZED

    def initialize(self):

        # Bring everything online
        self.base_station.bring_online()
        for robot in self.robots:
            robot.attach_pos_handler(self.pos_handler)
            robot.bring_online()
        
        self.check_status()

    def shutdown(self, keep_mm_awake):

        print("Shutting down")
        if self.initialization_status != STATUS_NOT_INITIALIZED:
            if not keep_mm_awake:
                for robot in self.robots:
                    self.pos_handler.sleep_robot(robot.robot_number)
                write_file(self.cfg.mm_beacon_state_file, 'False')
            self.pos_handler.close()

    def get_initialization_status(self):
        return self.initialization_status

    def check_initialization_status(self):
        ready = []
        ready.append(self.base_station.check_online())
        for robot in self.robots:
            ready.append(robot.check_online())
        
        if all(ready):
            self.status = STATUS_FULLY_INITIALIZED
            write_file(self.config.mm_beacon_state_file, 'True')
        elif any(ready)
            self.status = STATUS_PARTIALLY_INITIALIZED

    def get_all_metrics(self):
        metrics = {}
        metrics['pos'] = self.pos_handler.get_metrics()
        metrics['base'] = self.base_station.get_last_status()
        for robot in self.robots:
            metrics[robot.robot_number] = robot.get_last_status()

        return metrics

    def update(self):
        self.check_initialization_status()

        self.pos_handler.service_queues()

        self.base_station.update()
        for robot in self.robots:
            robot.update()

    def run_command(self, command):
        if command.target = 'base':
            self.base_station.run_command(command)
        elif type(command.target) is int:
            self.robots[command.target].run_command(command)
        else:
            print("Unknown target: {}".format(command.target))