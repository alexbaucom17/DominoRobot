import numpy as np
import os

class Config:

    # Debugging flags
    OFFLINE_TESTING = False
    SKIP_BASE_STATION = True
    SKIP_MARVELMIND = True
    USE_TEST_PLAN = False

    # ====== PATHS ========
    
    root_path = "C:\\Users\\alexb\\Data\\Github\\DominoRobot\\"   # Desktop
    #root_path = "C:\\Users\\alexb\\Documents\\Github\\DominoRobot\\"  # Laptop
    mm_api_relative_path = "marvelmind_SW_20202_04_19\\API\\api_windows_64bit\\dashapi.dll"
    config_dir_path = os.path.dirname(os.path.realpath(__file__))
    mm_api_path = os.path.join(root_path, mm_api_relative_path)
    log_folder = os.path.join(root_path, 'log')
    cycle_state_file = os.path.join(config_dir_path, 'plans', 'previous_plan_state.json')


    # ====== ROBOT CONFIG ========

    # Maps robot (or static) to sets of marvel mind beacons
    device_map = {
    "static": (11, 12),
    "robot1": (1, 2)
    }

    # Specifies which IP address each robot has
    #ip_map = {'robot1': '10.0.0.3'}   # Workshop
    ip_map = {'robot1': '192.168.1.5'}   # Home
    base_station_ip = '10.0.0.100'

    # ====== PLAN GENERATION ========

    # Image configuration
    image_name = os.path.join(config_dir_path, 'MR.jpg')
    desired_width_dominos = 30
    desired_height_dominos = 40
    dominos = np.array(
                [('black', (0,0,0)),
                ('red',   (1,0,0)),
                ('blue',  (0,0,1)),
                ('green', (0,1,0)),
                ('white', (1,1,1))
                ], dtype=object)

    # Physical dimensions of dominos
    domino_width  = 0.028 # meters
    domino_height = 0.0095 # meters
    domino_spacing_width = 0.024 # meters
    domino_spacing_height = 0.036 # meters

    # Spacing for drawing dominos as pixels instead of rectangles
    meters_per_pixel = 0.008
    domino_width_px = round(domino_width / meters_per_pixel)
    domino_height_px = round(domino_height / meters_per_pixel)
    domino_spacing_width_px = round(domino_spacing_width / meters_per_pixel)
    domino_spacing_height_px = round(domino_spacing_height / meters_per_pixel)

    # Tile configuration
    tile_width = 15
    tile_height = 20
    tile_background_color = (0.8, 0.8, 0.8)
    tile_edge_color = (0,0,1)
    tile_size_width_meters = tile_width * (domino_spacing_width + domino_width)
    tile_size_height_meters = tile_height * (domino_spacing_height + domino_height)


    # ====== ENVIRONMENT CONFIGURATION ========

    # Map configuration (distances in meters, angles in degrees)
    robot_boundaries = np.array([[0,0],[10,10]])                # Bottom left, top right, global frame
    base_station_boundaries = np.array([[0,1],[1,2]])           # Bottom left, top right, global frame
    base_station_target_pos = np.array([0.5, 1.5])              # Target position for robot to be under base station [x,y], in global frame
    base_station_target_angle = 180                             # Target angle (deg) for base station in global frame
    base_station_coarse_pose_offset = np.array([-1.5, 0])       # Offset from base station to use for apprach [x,y] in robot frame
    domino_field_origin = np.array([3,3])                       # Bottom left corner of domino field in global frame
    domino_field_angle = 90                                     # Domino field angle (deg), global frame
    tile_placement_coarse_offset = np.array([0.3,-0.3])         # Offset position for tile placement [x,y], in robot coordinate frame
    tile_to_robot_offset = np.array([tile_size_width_meters/2.0, -0.2])  # Offset from bottom left of tile to robot center [x,y], in robot coordinate frame
    prep_position_distance = 1                                  # How far out of field boundaries to do robot prep move
    exit_position_distance = 1                                  # How far out of the field boundaries to move to exit
    field_to_robot_frame_angle = 0                              # In case robot frame and field frame ever need to be rotated relative to each other

    # Computed - don't change
    field_width = tile_size_width_meters * desired_width_dominos/tile_width
    field_height = tile_size_height_meters * desired_height_dominos/tile_height
    domino_field_boundaries = np.array([domino_field_origin,domino_field_origin + np.array([field_width,field_height])])

    # ====== RUNTIME CONFIGURATION ========
    robot_status_wait_time = 0.5    # How many seconds to wait between status requests for each robot
    base_station_status_wait_time = 1 # How many seconds to wait between status requests for the base station
    robot_next_action_wait_time = 2.0 # How many seconds to wait before checking if robot is finished with current plan action
