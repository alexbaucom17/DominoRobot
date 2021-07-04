# This is a basic TCP client for a robot that wraps up sending
# and recieving various commands

import socket
import select
import json
import time
import logging
import copy

PORT = 8123
NET_TIMEOUT = 0.1 # seconds
START_CHAR = "<"
END_CHAR = ">"

class TcpClient:

    def __init__(self, ip, port, timeout):
        self.socket = socket.create_connection((ip, port), timeout)
        self.socket.setblocking(False)
        if not socket:
            return None

    def send(self, msg, print_debug=True):
        totalsent = 0
        if print_debug:
            logging.info("TX: " + msg)
        msg = START_CHAR + msg + END_CHAR
        msg_bytes = msg.encode()
        while totalsent < len(msg):
            sent = 0
            try:
                sent = self.socket.send(msg_bytes[totalsent:])
            except:
                pass
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def recieve(self, timeout=0.1, print_debug=True):

        # logging.info("Checking socket ready")
        # socket_ready, _, _ = select.select([self.socket], [], [])
        # if not socket_ready:
        #     logging.info("Socket not ready")
        #     return ""

        new_msg = ""
        new_msg_ready = False
        start_time = time.time()
        while not new_msg_ready and time.time() - start_time < timeout:
            # Get new data
            try:
                data = self.socket.recv(2048)
            except socket.error:
                continue
            if data == b'':
                raise RuntimeError("socket connection broken")

            # Decode data and parse into message
            new_str = data.decode(encoding='UTF-8',errors='strict')

            start_idx = new_str.find(START_CHAR)
            end_idx = new_str.find(END_CHAR)

            if start_idx != -1 and end_idx != -1:
                new_msg = new_str[start_idx+1:end_idx]
                new_msg_ready = True
            elif start_idx != -1:
                new_msg += new_str[start_idx+1:]
                msg_rcv_in_progress = True
            elif end_idx != -1:
                new_msg += new_str[:end_idx]
                new_msg_ready = True
            else:
                new_msg += new_str

        if print_debug and new_msg:
            logging.info("RX: " + new_msg)
        if not new_msg_ready:
            #logging.info("Socket timeout")
            new_msg = ""

        return new_msg

class ClientBase:

    def __init__(self, ip): 
        self.client = TcpClient(ip, PORT, NET_TIMEOUT)
        # Queue is used to help handle out of order messages recieved
        self.incoming_queue = []

    def wait_for_server_response(self, expected_msg_type, timeout=0.1, print_debug=True):
        """
        Waits for specified time to get a reply from the server
        Must set expected_msg_type to a string and this function will return the first message
        recieved matching that type. Other messages are added to the queue and checked the
        next time this function is called. If no matching message is recieved before timeout,
        this function returns None
        """

        # First, check the existing queue to see if the message we are looking for is there
        for ix in range(len(self.incoming_queue)):
            # Since type field existence is checked before insertion into queue, it should
            # always be safe to check the field directly
            if self.incoming_queue[ix]['type'] == expected_msg_type:
                msg_copy = copy.copy(self.incoming_queue[ix])
                del self.incoming_queue[ix]
                return msg_copy    

        # If there was nothing in the queue matching the expected type, try to recieve a message
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                incoming_msg = self.client.recieve(timeout=0.1, print_debug=print_debug)
            except socket.timeout:
                break
            if incoming_msg:
                try:
                    new_msg = json.loads(incoming_msg)
                    if 'type' not in new_msg:
                        logging.warning("Discarded msg for missing type field: {}".format(new_msg))
                        continue
                    elif new_msg['type'] == expected_msg_type:
                        return new_msg
                    else:
                        self.incoming_queue.append(new_msg)
                except:
                    logging.warning("Error decoding json: {}".format(incoming_msg))
                    break

        # Will get here if timeout is reached or decode error happens
        return None

    def send_msg_and_wait_for_ack(self, msg, print_debug=False, timeout=0.1):
        """ 
        Sends msg and ensures that the correct ack is returned
        Logs a warning if ack is not recieved or incorrect ack is recieved
        """

        self.client.send(json.dumps(msg,separators=(',',':')), print_debug=print_debug) # Make sure json dump is compact for transmission
        resp = self.wait_for_server_response(expected_msg_type='ack', print_debug=print_debug)
        if not resp:
            logging.warning('Did not recieve ack')
        else:
            if resp['type'] != 'ack':
                logging.warning('Expecting return type ack')
            elif resp['data'] != msg['type']:
                logging.warning('Incorrect ack type')
        
        return resp

    def net_status(self):
        """ Check if the network connection is ok"""
        msg = {'type': 'check'}
        status = False
        try:
            self.send_msg_and_wait_for_ack(msg)
            status = True
        except:
            pass
        finally:
            return status

    def request_status(self):
        """ Request status from server """
        msg = {'type' : 'status'}
        self.client.send(json.dumps(msg), print_debug=False)
        resp = self.wait_for_server_response(expected_msg_type='status', print_debug=False)
        if not resp:
            logging.warning("Did not recieve status response")
            return None
        return resp['data']

    def estop(self):
        """ Tell client to estop """
        msg = {'type': 'estop'}
        self.send_msg_and_wait_for_ack(msg)


class RobotClient(ClientBase):

    def __init__(self, cfg, robot_id):
        super().__init__(cfg.ip_map[robot_id])

        self.robot_id = robot_id
        self.cfg = cfg

    def move(self, x, y, a):
        """ Tell robot to move to specific location """
        msg = {'type': 'move', 'data': {'x': x, 'y': y, 'a': a}}
        self.send_msg_and_wait_for_ack(msg)

    def move_rel(self, x, y, a):
        """ Tell robot to move to a relative location """
        msg = {'type': 'move_rel', 'data': {'x': x, 'y': y, 'a': a}}
        self.send_msg_and_wait_for_ack(msg)

    def move_rel_slow(self, x, y, a):
        """ Tell robot to move to a relative location but slowly """
        msg = {'type': 'move_rel_slow', 'data': {'x': x, 'y': y, 'a': a}}
        self.send_msg_and_wait_for_ack(msg)

    def move_fine(self, x, y, a):
        """ Tell robot to move to a specific location with fine precision """
        msg = {'type': 'move_fine', 'data': {'x': x, 'y': y, 'a': a}}
        self.send_msg_and_wait_for_ack(msg)

    def move_with_vision(self, x, y, a):
        """ Tell robot to move to a location relative to the vision measurements from cameras"""
        msg = {'type': 'move_vision', 'data': {'x': x, 'y': y, 'a': a}}
        self.send_msg_and_wait_for_ack(msg)

    def move_const_vel(self, vx, vy, va, t):
        """ Tell robot to move at constant velocity for a specific amount of time"""
        msg = {'type': 'move_const_vel', 'data': {'vx': vx, 'vy': vy, 'va': va, 't': t}}
        self.send_msg_and_wait_for_ack(msg)

    def place(self):
        """ Tell robot to place pallet """
        msg = {'type': 'place'}
        self.send_msg_and_wait_for_ack(msg)
    
    def load(self):
        """ Tell robot to load pallet """
        msg = {'type': 'load'}
        self.send_msg_and_wait_for_ack(msg)

    def tray_init(self):
        """ Tell robot to initialize tray """
        msg = {'type': 'init'}
        self.send_msg_and_wait_for_ack(msg)

    def load_complete(self):
        """ Tell robot that base station load is complete """
        msg = {'type': 'lc'}
        self.send_msg_and_wait_for_ack(msg)

    def clear_error(self):
        """ Tell robot to clear an existing error """
        msg = {'type': 'clear_error'}
        self.send_msg_and_wait_for_ack(msg)

    def wait_for_localization(self):
        """ Tell robot to wait for localization"""
        msg = {'type': 'wait_for_loc'}
        self.send_msg_and_wait_for_ack(msg)

    def set_pose(self, x, y, a):
        """ Sets the pose of the robot explicity bypassing any localization code """
        msg = {'type': 'set_pose', 'data': {'x': x, 'y': y, 'a': a}}
        self.send_msg_and_wait_for_ack(msg)

    def toggle_vision_debug(self):
        """ Tell robot to toggle vision debug output"""
        msg = {'type': 'toggle_vision_debug'}
        self.send_msg_and_wait_for_ack(msg)

    def start_cameras(self):
        """ Tell robot to start the cameras"""
        msg = {'type': 'start_cameras'}
        self.send_msg_and_wait_for_ack(msg)

    def stop_cameras(self):
        """ Tell robot to stop the cameras"""
        msg = {'type': 'stop_cameras'}
        self.send_msg_and_wait_for_ack(msg)

class BaseStationClient(ClientBase):
    
    def __init__(self, cfg):
        super().__init__(cfg.base_station_ip)
        self.cfg = cfg

    def load(self):
        """ Tells the station to load the next set of dominos into the robot"""
        msg = {'type': 'load'}
        self.send_msg_and_wait_for_ack(msg)


# Hacky Mocks to use for testing
class MockRobotClient:
    def __init__(self, cfg, robot_id):
        super().__init__()
        self.robot_id = robot_id
        self.cfg = cfg

    def move(self, x, y, a):
        pass

    def move_rel(self, x, y, a):
        pass

    def move_rel_slow(self, x, y, a):
        pass

    def move_fine(self, x, y, a):
        pass

    def place(self):
        pass

    def net_status(self):
        return True

    def estop(self):
        pass

    def load(self):
        pass

    def request_status(self):
        return {"in_progress": False, "pos_x": 1, "pos_y": 2, "pos_a": 0}
    
    def clear_error(self):
        pass

    def wait_for_localization(self):
        pass

    def toggle_distance(self):
        pass

class MockBaseStationClient:
    
    def __init__(self, cfg):
        super().__init__()
        self.cfg = cfg

    def load(self):
        pass

    def net_status(self):
        return True
    
    def estop(self):
        pass

    def request_status(self):
        return {}
    

if __name__== '__main__':
    import config
    from MasterMain import configure_logging
    cfg = config.Config()
    configure_logging(cfg.log_folder)
    r = RobotClient(cfg, 'robot1')
    time.sleep(1)

    r.request_status()
    #r.request_status()
    time.sleep(1)
    #r.request_status()
    #time.sleep(1)
    
    # while(True):
    #     speed = input("Input move speed [x,y,a]: ").strip().split(',')
    #     if len(speed) != 3:
    #         logging.info("Need to provide comma separated values.")
    #     else:
    #         x = float(speed[0])
    #         y = float(speed[1])
    #         a = float(speed[2])
    #         r.move(x,y,a)
        

    #     time.sleep(3)