import argparse
import os
import numpy as np
import json
import matplotlib.pyplot as plt

DEFAULT_LOG_PATH = "C:\\Users\\alexb\\Documents\\Github\\DominoRobot\\log"


def get_value(line, path):
    idx = 0
    for p in path:
        idx = line.find(p, idx)
        idx += len(p)

    data = ""
    for c in line[idx:]:
        if c in ['[',']',',','\n']:
            break
        elif c in [':',' ']:
            continue
        else:
            data += c        

    return float(data)

def get_array(line):
    start_idx = line.find('[')
    end_idx = line.find(']')
    array_txt = line[start_idx+1:end_idx]
    vals = array_txt.split(',')
    return [float(v.strip()) for v in vals]

class LogParser:

    def __init__(self, path):

        self.path = path
        self.time = np.zeros((1,1))
        self.target_pos = np.zeros((3,1))
        self.target_vel = np.zeros((3,1))
        self.cmd_vel = np.zeros((3,1))
        self.est_vel = np.zeros((3,1))
        self.est_pos = np.zeros((3,1))
        self.motor_cmd_vel = np.zeros((4,1))
        self.motor_est_vel = np.zeros((4,1))
        self.t_offset = 0

    def parse_logs(self):

        with open(self.path) as f:
            for line in f:

                if line.startswith("Target:"):
                    # time
                    self.time = np.append(self.time, get_value(line, ['Target', 'T']) + self.t_offset)
                    # target position
                    data = []
                    data.append(get_value(line, ['Target','Position','X']))
                    data.append(get_value(line, ['Target','Position','Y']))
                    data.append(get_value(line, ['Target','Position','A']))
                    arr = np.asarray(data).reshape((3,1))
                    self.target_pos = np.hstack((self.target_pos, arr))
                    # target velocity
                    data = []
                    data.append(get_value(line, ['Target','Velocity','X']))
                    data.append(get_value(line, ['Target','Velocity','Y']))
                    data.append(get_value(line, ['Target','Velocity','A']))
                    arr = np.asarray(data).reshape((3,1))
                    self.target_vel = np.hstack((self.target_vel, arr))
                elif line.startswith("CartVelCmd:"):
                    # command velocity
                    data = []
                    data.append(get_value(line, ['CartVelCmd','vx']))
                    data.append(get_value(line, ['CartVelCmd','vy']))
                    data.append(get_value(line, ['CartVelCmd','va']))
                    arr = np.asarray(data).reshape((3,1))
                    self.cmd_vel = np.hstack((self.cmd_vel, arr))
                elif line.startswith("MotorCommands:"):
                    # motor command vel
                    data = get_array(line)
                    arr = np.asarray(data).reshape((4,1))
                    self.motor_cmd_vel = np.hstack((self.motor_cmd_vel, arr))
                elif line.startswith("MotorMeasured:"):
                    # motor est vel
                    data = get_array(line)
                    arr = np.asarray(data).reshape((4,1))
                    self.motor_est_vel = np.hstack((self.motor_est_vel, arr))
                elif line.startswith("Est Vel:"):
                    # est velocity
                    data = []
                    data.append(get_value(line, ['Est Vel','X']))
                    data.append(get_value(line, ['Est Vel','Y']))
                    data.append(get_value(line, ['Est Vel','A']))
                    arr = np.asarray(data).reshape((3,1))
                    self.est_vel = np.hstack((self.est_vel, arr))
                elif line.startswith("Est Pos:"):
                    # est position
                    data = []
                    data.append(get_value(line, ['Est Pos','X']))
                    data.append(get_value(line, ['Est Pos','Y']))
                    data.append(get_value(line, ['Est Pos','A']))
                    arr = np.asarray(data).reshape((3,1))
                    self.est_pos = np.hstack((self.est_pos, arr))
                elif line.startswith("Reached goal"):
                    # Drop extra target line if needed
                    if self.time.shape[0] > self.est_pos.shape[1]:
                        self.time = self.time[:-1]
                        self.target_pos = self.target_pos[:,:-1]
                        self.target_vel = self.target_vel[:,:-1]

                    # Update time offset
                    self.t_offset += self.time[-1] + 0.5 #some extra spacing

                else:
                    continue
        
    
    def plot_pos(self):
        fig = plt.figure()
        fig.canvas.set_window_title('Cartesian Position')

        labels = ['x', 'y', 'a']
        for i in range(3):
            ax = fig.add_subplot(3,1,i+1)
            ax.plot(self.time,self.target_pos[i,:], 'b', self.time,self.est_pos[i,:], 'r')
            ax.legend(['Target', 'Estimate'])
            ax.set_ylabel("Position: {}".format(labels[i]))
        
        ax.set_xlabel('Time')

    def plot_vel(self):
        fig = plt.figure()
        fig.canvas.set_window_title('Cartesian Velocity')

        labels = ['x', 'y', 'a']
        for i in range(3):
            ax = fig.add_subplot(3,1,i+1)
            ax.plot(self.time,self.target_vel[i,:], 'b', self.time,self.est_vel[i,:], 'r', self.time,self.cmd_vel[i,:], 'g')
            ax.legend(['Target', 'Estimate', 'Command'])
            ax.set_ylabel("Velocity: {}".format(labels[i]))
        
        ax.set_xlabel('Time')

    def plot_motors(self):
        fig = plt.figure()
        fig.canvas.set_window_title('Motor Velocity')

        labels = ['0', '1', '2', '3']
        for i in range(4):
            ax = fig.add_subplot(4,1,i+1)
            ax.plot(self.time,self.motor_cmd_vel[i,:], 'b', self.time,self.motor_est_vel[i,:], 'r')
            ax.legend(['Target', 'Estimate'])
            ax.set_ylabel("Motor Velocity: {}".format(labels[i]))
        
        ax.set_xlabel('Time')
    
    def plot_logs(self):
        self.plot_pos()
        self.plot_vel()
        self.plot_motors()
        plt.show()



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot logfiles")
    parser.add_argument('filename', type=str)
    args = parser.parse_args()

    fpath = os.path.join(DEFAULT_LOG_PATH, args.filename)
    print("Loading logs from {}".format(fpath))

    lp = LogParser(fpath)
    lp.parse_logs()
    lp.plot_logs()
