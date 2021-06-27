# Helper script to get camera images and debug info from raspi

import paramiko
from scp import SCPClient
import os
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import config
from datetime import datetime

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

def get_and_display_multiple_images(cam_name, remote_ip, remote_path, local_path, img_data):
    fig, axes = plt.subplots(nrows=2, ncols=3)
    fig.suptitle("{} camera".format(cam_name), fontsize=16)
    ax = axes.ravel()

    for i,data in enumerate(img_data):
        cur_time_str = datetime.now().strftime("%Y%m%d%H%M%S")
        remote_path_to_file = remote_path + data['file']
        local_path_to_file = os.path.join(local_path, cur_time_str+"_"+cam_name+"_"+data['file'])
        scp_image(remote_ip, remote_path_to_file, local_path_to_file)
        img = mpimg.imread(local_path_to_file)
        if data["color"]:
            ax[i].imshow(img)
        else:
            ax[i].imshow(img, cmap='gray', vmin=0, vmax=255)
        ax[i].set_title(data['title'])
    
    plt.tight_layout()
    figManager = plt.get_current_fig_manager()
    figManager.window.state('zoomed')
    plt.show()

if __name__ == '__main__':
    cfg = config.Config()

    robot_ip = cfg.ip_map['robot1']
    remote_path = "/home/pi/images/debug/"
    local_path = cfg.log_folder
    img_data = []
    img_data.append({
        "file": "img_raw.jpg",
        "title": "raw",
        "color": False
    })
    img_data.append({
        "file": "img_undistorted.jpg",
        "title": "undistorted",
        "color": False
    })
    img_data.append({
        "file": "img_thresh.jpg",
        "title": "threshold",
        "color": False
    })
    img_data.append({
        "file": "img_keypoints.jpg",
        "title": "detections",
        "color": False
    })
    img_data.append({
        "file": "img_best_keypoint.jpg",
        "title": "best",
        "color": False
    })

    DISPLAY_SIDE = True;
    DISPLAY_REAR = True;

    if DISPLAY_SIDE:
        side_remote_path = remote_path + 'side/'
        get_and_display_multiple_images("side", robot_ip, side_remote_path, local_path, img_data)
    if DISPLAY_REAR:
        rear_remote_path = remote_path + 'rear/'
        get_and_display_multiple_images("rear", robot_ip, rear_remote_path, local_path, img_data)

