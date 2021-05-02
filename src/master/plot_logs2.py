import paramiko
from scp import SCPClient
import os
import matplotlib.pyplot as plt
import config
import csv
import numpy as np
import datetime

possible_rows = {'time':1,'pos':3,'vel':3,'target_pos':3,'target_vel':3,'control_vel':3}

def build_row_title_map():
    row_title_map = {}
    axes = ['x','y','a']
    counter = 0
    for row,count in possible_rows.items():
        if count == 1:
            row_title_map[row] = counter
            counter += 1
        elif count == 3:
            for axis_name in axes:
                row_title_map[row+"_"+axis_name] = counter
                counter += 1
    return row_title_map


row_title_map = build_row_title_map()
print(row_title_map)

def scp_last_motion_log(ip, remote_path, local_path):
    ssh_client = paramiko.SSHClient()
    ssh_client.load_system_host_keys()
    ssh_client.connect(ip, username="pi", password='dominobot')
    with SCPClient(ssh_client.get_transport()) as scp:
        scp.get(remote_path, local_path)
    if not os.path.exists(local_path):
        raise ValueError("SCP image did not complete successfully")

def init_data():
    n_rows = 0
    for count in possible_rows.values():
        n_rows += count
    data = {'title':"",'n_rows':n_rows,'start_time':None,'values':None}
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
    entry_time = datetime.datetime.strptime(row[1], '%H:%M:%S.%f')
    
    # If no data yet, start new column
    if data['values'] is None:
        data['values'] = np.zeros((data['n_rows'], 1))
        data['start_time'] = entry_time
        # print("start data")

    # If this entry is from a new time, start a new column
    dt = (entry_time - data['start_time']).microseconds / 1000000.0
    if dt > data['values'][row_title_map['time'],-1]:
        data['values'] = np.hstack((data['values'], np.zeros((data['n_rows'], 1))))
        data['values'][row_title_map['time'],-1] = dt
        # print("Add column at time {}".format(dt))
    
    # Find starting row to put data in
    row_ix = row_title_map[entry_data[0]+"_x"]
    
    # Put data in the array
    for i in range(3):
        # print("Adding data {} for row {}, {}".format(float(entry_data[i+1]),entry_data[0],i))
        data['values'][row_ix + i, -1] = float(entry_data[i+1])

def plot_data(data, rows=None, axes=None):
    if not rows:
        rows = possible_rows.keys()
    if not axes:
        axes = ['x','y','a']
    
    all_rows_to_plot = [row_name+"_"+axis_name  for row_name in rows if row_name is not 'time' for axis_name in axes]
    print(all_rows_to_plot)

    plt.figure()
    ax = plt.gca()

    plotting_data = data['values']
    x = plotting_data[row_title_map['time']]
    for row in all_rows_to_plot:
        y = plotting_data[row_title_map[row]]
        ax.plot(x,y,label=row)

    ax.legend()
    ax.set_title(data['title'])
    plt.show()



if __name__ == '__main__':
    cfg = config.Config()

    robot_ip = cfg.ip_map['robot1']
    remote_file = "/home/pi/DominoRobot/src/robot/log/last_motion_log.csv"
    local_file = os.path.join(cfg.log_folder, "last_motion_log.csv")

    scp_last_motion_log(robot_ip, remote_file, local_file)
    parsed_data = parse_log_file(local_file)
    plot_data(parsed_data, rows=['pos'], axes=['x'])