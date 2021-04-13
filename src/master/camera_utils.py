# Helper script to get camera images and debug info from raspi

import paramiko
from scp import SCPClient
import os
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import config

def scp_image(ip, remote_path, local_path):
    ssh_client = paramiko.SSHClient()
    ssh_client.load_system_host_keys()
    ssh_client.connect(ip, username="pi", password='dominobot')
    with SCPClient(ssh_client.get_transport()) as scp:
        scp.get(remote_path, local_path)
    if not os.path.exists(local_path):
        raise ValueError("SCP image did not complete successfully")

def display_debug_image(local_path):
    img = mpimg.imread(local_path)
    plt.imshow(img)
    figManager = plt.get_current_fig_manager()
    figManager.window.state('zoomed')
    plt.show()

if __name__ == '__main__':
    cfg = config.Config()

    robot_ip = cfg.ip_map['robot1']
    local_path = os.path.join(cfg.log_folder, "debug_img.jpg")
    remote_path = "/home/pi/debug_img.jpg"

    scp_image(robot_ip, remote_path, local_path)
    display_debug_image(local_path)

