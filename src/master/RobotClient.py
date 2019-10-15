# This is a basic TCP client for a robot that wraps up sending
# and recieving various commands

import socket
import select
import json
import time

PORT = 1234
ID_TO_IP_DICT = {1: '192.168.1.13',
                 2: '192.168.1.14'}
NET_TIMEOUT = 10 # seconds

class TcpClient:

    def __init__(self, ip, port, timeout):
        self.socket = socket.create_connection((ip, port), timeout)
        if not socket:
            return None

    def send(self, msg):
        totalsent = 0
        msg = msg + '\0'
        msg_bytes = msg.encode()
        print("TX: " + msg)
        while totalsent < len(msg):
            sent = self.socket.send(msg_bytes[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def recieve(self):

        socket_ready, _, _ = select.select([self.socket], [], [])
        if not socket_ready:
            print("Not ready")
            return None

        chunks = []
        term_char_found = False
        while not term_char_found:
            chunk = self.socket.recv(2048)
            if chunk == b'':
                raise RuntimeError("socket connection broken")
            if chunk[-1] == 0:
                term_char_found = True
                chunk = chunk[:-1]
            chunks.append(chunk)

        msg_bytes = b''.join(chunks)
        msg_str = msg_bytes.decode()
        print("RX: " + msg_str)
        return msg_str


class RobotClient:

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.client = TcpClient(ID_TO_IP_DICT[self.robot_id], PORT, NET_TIMEOUT)

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
    
    r.dock()
    r.move(1,2,3)
    r.place()
    r.undock()
    r.dropoff()
    r.pickup()
    r.position(4,5,6,7)
    r.request_status()