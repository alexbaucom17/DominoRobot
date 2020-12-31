import numpy as np
import os

class Config:

    # ====== PATHS ========
    
    root_path = "C:\\Users\\alexb\\Data\\Github\\DominoRobot\\"
    mm_api_relative_path = "marvelmind_SW_20202_04_19\\API\\api_windows_64bit\\dashapi.dll"
    config_dir_path = os.path.dirname(os.path.realpath(__file__))
    mm_api_path = os.path.join(root_path, mm_api_relative_path)
    log_folder = os.path.join(root_path, 'log')
    cycle_state_file = os.path.join(config_dir_path, 'plans', 'cycle_state.json')


    # ====== ROBOT CONFIG ========

    # Maps robot (or static) to sets of marvel mind beacons
    device_map = {
    "static": (1, 2),
    "robot1": (12, 13)
    }

    # Specifies which IP address each robot has
    ip_map = {'robot1': '192.168.1.5'}
    base_station_ip = '10.0.0.100'

    # ====== PLAN GENERATION ========

    # Image configuration
    image_name = os.path.join(config_dir_path, 'MR.jpg')
    desired_width = 200
    desired_height = 200
    dominos = np.array(
                [('black', (0,0,0)),
                ('red',   (1,0,0)),
                ('blue',  (0,0,1)),
                ('green', (0,1,0)),
                ('white', (1,1,1))
                ], dtype=object)

    # To get image to look right, need spacing to create square pixels for now
    # could fix this later by making image field creation take into account non-square pixels
    domino_width  = 0.024 # meters
    domino_height = 0.008 # meters
    domino_spacing_x = 0.008 # meters
    domino_spacing_y = 0.024 # meters

    # Spacing for drawing dominos as pixels instead of rectangles
    meters_per_pixel = 0.008
    domino_width_px = round(domino_width / meters_per_pixel)
    domino_height_px = round(domino_height / meters_per_pixel)
    domino_spacing_x_px = round(domino_spacing_x / meters_per_pixel)
    domino_spacing_y_px = round(domino_spacing_y / meters_per_pixel)

    # Tile configuration
    tile_width = 100
    tile_height = 100
    tile_background_color = (0.8, 0.8, 0.8)
    tile_edge_color = (0,0,1)
    tile_size_x_meters = tile_width * (domino_spacing_x + domino_width)
    tile_size_y_meters = tile_height * (domino_spacing_y + domino_height)


    # ====== ENVIRONMENT CONFIGURATION ========

    # Map configuration (distances in meters, angles in degrees)
    robot_boundaries = np.array([[0,0],[5,5]])
    base_station_boundaries = np.array([[0.5,0.5],[0.5,1.5]])
    base_station_target_pose = np.array([0, 1])
    base_station_coarse_pose_offset = np.array([0, -1])
    domino_field_origin = np.array([2,2])
    domino_field_angle = 90
    tile_placement_coarse_offset = np.array([0.2,0.2])
    prep_position_distance = 2 # How far out of field boundaries to do robot prep move
    exit_position_distance = 2 # How far out of the field boundaries to move to exit
    frame_to_robot_offset = np.array([-0.5, 0])

    # Computed - don't change
    field_width = tile_size_x_meters * desired_width/tile_width
    field_height = tile_size_y_meters * desired_height/tile_height
    domino_field_boundaries = np.array([domino_field_origin,domino_field_origin + np.array([field_width,field_height])])

    # ====== RUNTIME CONFIGURATION ========
    robot_status_wait_time = 0.5    # How many seconds to wait between status requests for each robot
    base_station_status_wait_time = 1 # How many seconds to wait between status requests for the base station
    robot_next_action_wait_time = 1.0 # How many seconds to wait before checking if robot is finished with current plan action
