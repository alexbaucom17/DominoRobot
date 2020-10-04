import serial
import time
import signal
import sys

SERIAL_PORT = 'COM7'
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

def power_on(s):
    msg = "Power:ON"
    s.send(msg)
    check_response(s)

def power_off(s):
    msg = "Power:OFF"
    s.send(msg)
    check_response(s)

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
    start_time = time.time()
    while time.time() - start_time < move_time:
        send_vel(s, vel)
        check_response(s)
        time.sleep(0.2)

def timed_move(s, vel, move_time, pause_time):
    print("Starting move")
    move_vel_with_pause(s, vel, move_time)
    print("Done with move")
    stop_vel = [0,0,0]
    move_vel_with_pause(s, stop_vel, pause_time)


if __name__ == '__main__':
    
    ser = SerialClient(SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT)

    def signal_handler(sig, frame):
        print('Gracefully powering down motors and exiting')
        power_off(ser)
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    fwd_vel =  [ 0.2, 0, 0]
    bkwd_vel = [-0.2, 0, 0]
    left_vel = [0, 0.2, 0]
    right_vel = [0, -0.2, 0]
    ccw_spin_vel = [0, 0, 0.2]
    cw_spin_vel = [0, 0, -0.2]
    diag_fwd_vel = [0.1, 0.173, 0]
    diag_rev_vel = [-0.1, -0.173, 0]
    move_time = 3
    pause_time = 2

    print("Motors on")
    power_on(ser)
    print("Move forward")
    timed_move(ser, fwd_vel, move_time, pause_time)
    print("Move backward")
    timed_move(ser, bkwd_vel, move_time, pause_time)
    print("Move left")
    timed_move(ser, left_vel, move_time, pause_time)
    print("Move right")
    timed_move(ser, right_vel, move_time, pause_time)
    print("Move CCW")
    timed_move(ser, ccw_spin_vel, move_time, pause_time)
    print("Move CW")
    timed_move(ser, cw_spin_vel, move_time, pause_time)
    print("Move Diag Fwd")
    timed_move(ser, diag_fwd_vel, move_time, pause_time)
    print("Move Diag Rev")
    timed_move(ser, diag_rev_vel, move_time, pause_time)
    print("Motors off")
    power_off(ser)

