import time
from MarvelMindHandler import RobotPositionHandler
from RobotClient import RobotClient

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

            # Request a position update if needed
            if time.time() - self.last_status_time > self.config.robot_status_wait_time:
                self._get_status_from_robot()

    def run_command(self, command, data=None):
        
        if not self.comms_online:
            return

        if command == "move":
            self.robot_client.move(float(data[0]), float(data[1]), float(data[2]))
        elif command == "rel":
            self.robot_client.move_rel(float(data[0]), float(data[1]), float(data[2]))
        elif command == "fine":
            self.robot_client.move_fine(float(data[0]), float(data[1]), float(data[2]))
        elif command == "net":
            status = self.robot_client.net_status()
            print("Robot {} network status is: {}".format(self.robot_number, status))
        else:
            print("Unknown command: {}, data: {}".format(command, data))



class BaseStationInterface:
    #TODO
    pass



class RuntimeManager:

    STATUS_NOT_INITIALIZED = 0
    STATUS_PARTIALLY_INITIALIZED = 1
    STATUS_FULLY_INITIALIZED = 2

    def __init__(self, config):

        self.config = config
        self.robots = [RobotInterface(config, n) for n in config.ip_map.keys()]
        self.base_station = BaseStationInterface(config)
        self.pos_handler = RobotPositionHandler(config)
        self.status = STATUS_NOT_INITIALIZED

    def initialize(self):

        # Bring everything online
        self.base_station.bring_online()
        for robot in self.robots:
            robot.attach_pos_handler(self.pos_handler)
            robot.bring_online()
        
        self.check_status()

    def check_status(self):
        ready = []
        ready.append(self.base_station.check_online())
        for robot in self.robots:
            ready.append(robot.check_online())
        
        if all(ready):
            self.status = STATUS_FULLY_INITIALIZED
        elif any(ready)
            self.status = STATUS_PARTIALLY_INITIALIZED

    def update(self):
        self.check_status()

        self.base_station.update()
        for robot in self.robots:
            robot.update()