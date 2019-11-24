# This is a basic TCP client for a robot that wraps up sending
# and recieving various commands

import socket
import select
import json
import time

PORT = 1234
NET_TIMEOUT = 10 # seconds
START_CHAR = "<"
END_CHAR = ">"

class TcpClient:

    def __init__(self, ip, port, timeout):
        self.socket = socket.create_connection((ip, port), timeout)
        if not socket:
            return None

    def send(self, msg):
        totalsent = 0
        print("TX: " + msg)
        msg = START_CHAR + msg + END_CHAR
        msg_bytes = msg.encode()
        while totalsent < len(msg):
            sent = self.socket.send(msg_bytes[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def recieve(self, timeout=1):

        socket_ready, _, _ = select.select([self.socket], [], [])
        if not socket_ready:
            return ""

        new_msg = ""
        new_msg_ready = False
        msg_rcv_in_progress = False
        start_time = time.time()
        while not new_msg_ready and time.time() - start_time < timeout:
            # Get new data
            data = self.socket.recv(2048)
            if data == b'':
                raise RuntimeError("socket connection broken")

            # Decode data and parse into message
            new_str = data.decode(encoding='UTF-8',errors='strict')
            if not msg_rcv_in_progress:
                start_idx = new_str.find(START_CHAR)
                if start_idx != -1:
                    new_msg += new_str[start_idx+1:]
                    msg_rcv_in_progress = True
            else:
                end_idx = new_str.find(END_CHAR)
                if end_idx != -1:
                    new_msg += new_str[:end_idx]
                    msg_rcv_in_progress = False
                    new_msg_ready = True
                else:
                    new_msg += new_str

        print("RX: " + new_msg)
        if not new_msg_ready:
            print("Timeout")
            new_msg = ""

        return new_msg


class RobotClient:

    def __init__(self, cfg, robot_id):
        self.robot_id = robot_id
        self.client = TcpClient(cfg.ip_map[self.robot_id], PORT, NET_TIMEOUT)
        self.cfg = cfg

    def wait_for_server_response(self, timeout=3):
        """
        Waits for specified time to get a reply from the server
        Returns a dict with the message if one is recieved
        Returns None if no message is recieved
        """
          
        start_time = time.time()
        while time.time() - start_time < timeout:
            incoming_msg = self.client.recieve()
            if incoming_msg:
                return json.loads(incoming_msg)

        # Will get here if timeout is reached
        return None
                

    def send_msg_and_wait_for_ack(self, msg):
        """ 
        Sends msg and ensures that the correct ack is returned
        Raises an error if ack is not recieved pr incorrect ack is recieved
        """

        self.client.send(json.dumps(msg))
        resp = self.wait_for_server_response()
        if not resp:
            raise ValueError('Did not recieve reply')
        else:
            if resp['type'] != 'ack':
                raise ValueError('Error: Expecting return type ack')
            elif resp['data'] != msg['type']:
                raise ValueError('Error: Incorrect ack type')
            

    def move(self, x, y, a):
        """ Tell robot to move to specific location """
        msg = {'type': 'move', 'data': {'x': x, 'y': y, 'a': a}}
        self.send_msg_and_wait_for_ack(msg)

    def place(self):
        """ Tell robot to place pallet """
        msg = {'type': 'place'}
        self.send_msg_and_wait_for_ack(msg)

    def dock(self):
        """ Tell robot to dock for charging """
        msg = {'type': 'dock'}
        self.send_msg_and_wait_for_ack(msg)

    def undock(self):
        """ Tell robot to undock from charging """
        msg = {'type': 'undock'}
        self.send_msg_and_wait_for_ack(msg)

    def dropoff(self):
        """ Tell robot to dropoff empty pallet """
        msg = {'type': 'dropoff'}
        self.send_msg_and_wait_for_ack(msg)

    def pickup(self):
        """ Tell robot to pickup full pallet """
        msg = {'type': 'pickup'}
        self.send_msg_and_wait_for_ack(msg)

    def position(self, x, y, a, t):
        """ Send robot coordinates from marvelmind sensors """
        msg = {'type': 'move', 'data': {'x': x, 'y': y, 'a': a, 't': t}}
        self.send_msg_and_wait_for_ack(msg)

    def request_status(self):
        """ Request status from robot """
        msg = {'type' : 'status'}
        self.client.send(json.dumps(msg))
        status = self.wait_for_server_response()
        return status




if __name__== '__main__':
    r = RobotClient(1)
    time.sleep(2)
    
    while(True):
        speed = input("Input move speed [x,y,a]: ").strip().split(',')
        if len(speed) != 3:
            print("Need to provide comma separated values.")
        else:
            x = float(speed[0])
            y = float(speed[1])
            a = float(speed[2])
            r.move(x,y,a)
        

        time.sleep(3)