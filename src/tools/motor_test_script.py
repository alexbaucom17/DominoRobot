import serial
import time

SERIAL_PORT = 'COM3'
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1

START_CHAR = "<"
END_CHAR = ">"


# Copied and modified from RobotClient.TcpClient
class SerialClient:

    def __init__(self, port, baud, timeout):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        if not self.ser:
            return None

    def send(self, msg):
        msg = START_CHAR + msg + END_CHAR
        msg_bytes = msg.encode()
        self.ser.write(msg_bytes)

    def recieve(self, timeout=0.1):

        new_msg = ""
        new_msg_ready = False
        start_time = time.time()
        while not new_msg_ready and time.time() - start_time < timeout:
            
            # Get new data
            data = self.ser.read()
            if data == b'':
                break

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

        if not new_msg_ready:
            new_msg = ""

        return new_msg

def send_vel(s, vel):
    if len(vel) != 3:
        raise ValueError("Velocity should be 3 elements long")

    msg = "{:.3f},{:.3f},{:.3f}".format(vel[0], vel[1], vel[2])
    print("SND: {}".format(msg))
    s.send(msg)

def check_response(s):
    msg = s.recieve()
    while msg:
        print("RCV: {}".format(msg))
        msg = s.recieve()

def move_vel_with_pause(s, vel, move_time):
    send_vel(s, vel)
    start_time = time.time()
    while time.time() - start_time < move_time:
        check_response(s)
        time.sleep(0.2)

def timed_move(s, vel, move_time, pause_time):
    print("Starting move")
    move_vel_with_pause(s, vel, move_time)
    print("Done with move")
    stop_vel = [0,0,0]
    move_vel_with_pause(s, stop_vel, pause_time)


if __name__ == '__main__':

    fwd_vel =  [ 0.1, 0, 0]
    bkwd_vel = [-0.1, 0, 0]
    move_time = 5
    pause_time = 2

    s = SerialClient(SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT)
    timed_move(s, fwd_vel, move_time, pause_time)
    timed_move(s, bkwd_vel, move_time, pause_time)

