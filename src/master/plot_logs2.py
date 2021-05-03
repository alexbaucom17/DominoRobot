import paramiko
from scp import SCPClient
import os
import matplotlib.pyplot as plt
import config
import csv
import numpy as np
import datetime

possible_rows = [
    'time',
    'pos_x',
    'pos_y',
    'pos_a',
    'vel_x',
    'vel_y',
    'vel_a',
    'target_pos_x',
    'target_pos_y',
    'target_pos_a',
    'target_vel_x',
    'target_vel_y',
    'target_vel_a',
    'control_vel_x',
    'control_vel_y',
    'control_vel_a'
]

def scp_last_motion_log(ip, remote_path, local_path):
    ssh_client = paramiko.SSHClient()
    ssh_client.load_system_host_keys()
    ssh_client.connect(ip, username="pi", password='dominobot')
    with SCPClient(ssh_client.get_transport()) as scp:
        scp.get(remote_path, local_path)
    if not os.path.exists(local_path):
        raise ValueError("SCP image did not complete successfully")

def init_data():
    data = {name:[] for name in possible_rows}
    data['title'] = ""
    data['start_time'] = None
    return data

def parse_log_file(path):

    data = init_data()
    with open(local_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=";")
        for id,row in enumerate(reader):
            if id == 0:
                continue
            elif id == 1:
                data['title'] = row[-1].replace('"','')
            else:
                parse_row(row, data)

    return data

def parse_row(row, data):
    entry_data = row[-1].replace('"','').split(",")
    entry_name = entry_data[0]
    entry_values = entry_data[1:]
    entry_time = datetime.datetime.strptime(row[1], '%H:%M:%S.%f')

    if entry_name == "time":
        if not data["start_time"]:
            data["start_time"] = entry_time
            data["time"].append(0)
        else:
            dt = (entry_time - data['start_time'])/ datetime.timedelta(seconds=1)
            data["time"].append(dt)

    else:
        for i,axis in enumerate(['x','y','a']):
            axis_name = entry_name+'_'+axis
            axis_value = float(entry_values[i])
            data[axis_name].append(axis_value)

def plot_data(data, rows_to_plot=None):
    if not rows_to_plot:
        rows_to_plot = possible_rows[1:]

    plt.figure()
    ax = plt.gca()

    for row in rows_to_plot:
        ax.plot('time', row, data=data, label=row)

    ax.legend()
    ax.set_title(data['title'])
    plt.xlabel('Time (s)')
    plt.show()


def plot_rows_axes(data, rows, axes):
    rows_to_plot = [row+'_'+axis for axis in axes for row in rows]
    plot_data(data, rows_to_plot)


if __name__ == '__main__':
    cfg = config.Config()

    GET_FILE = False
    EXISTING_LOCAL_FILENAME = "WiggleVision.csv"

    if GET_FILE:
        robot_ip = cfg.ip_map['robot1']
        remote_file = "/home/pi/DominoRobot/src/robot/log/last_motion_log.csv"
        local_file = os.path.join(cfg.log_folder, "last_motion_log.csv")
        scp_last_motion_log(robot_ip, remote_file, local_file)
    else:
        local_file = os.path.join(cfg.log_folder, EXISTING_LOCAL_FILENAME)

    parsed_data = parse_log_file(local_file)
    # plot_data(parsed_data, rows_to_plot=['pos_x','pos_y','pos_a'])
    plot_rows_axes(parsed_data, ['pos','vel','target_pos','target_vel','control_vel'], ['a'])