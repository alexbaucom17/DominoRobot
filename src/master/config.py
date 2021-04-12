import numpy as np
import os
import Utils

class Config:

    # Various debug/test flags
    # Set to override config values for home network
    USING_HOME_NETWORK = False
    # Set to override config values for small scale testing
    USE_SMALL_TESTING_CONFIG = True
    # Set to skip connecting to robot
    OFFLINE_TESTING = False
    # Set to skip connecting to base station
    SKIP_BASE_STATION = True
    # Set to skip connecting to Marvelmind
    SKIP_MARVELMIND = True
    # Set to use fake plan instead of loading a generated one
    USE_TEST_PLAN = False
    # Set to auto-load this plan on master startup
    AUTO_LOAD_PLAN = False
    AUTO_LOAD_PLAN_NAME = "AccuracyTesting_3x3_withdistance_2axis.p"
    # Set to regenerate and auto load plan on master startup
    REGEN_PLAN = True

    # ====== PATHS ========
    
    root_path = "C:\\Users\\alexb\\Documents\\Github\\DominoRobot\\"  # Laptop
    if USING_HOME_NETWORK:
        root_path = "C:\\Users\\alexb\\Data\\Github\\DominoRobot\\"   # Desktop
    mm_api_relative_path = "marvelmind_SW_20202_04_19\\API\\api_windows_64bit\\dashapi.dll"
    config_dir_path = os.path.dirname(os.path.realpath(__file__))
    mm_api_path = os.path.join(root_path, mm_api_relative_path)
    log_folder = os.path.join(root_path, 'log')
    plans_dir = os.path.join(config_dir_path, 'plans')
    cycle_state_file = os.path.join(plans_dir, 'previous_plan_state.json')


    # ====== ROBOT CONFIG ========

    # Maps robot (or static) to sets of marvel mind beacons
    device_map = {
    "static": (11, 12),
    "robot1": (1, 2)
    }

    # Specifies which IP address each robot has
    ip_map = {'robot1': '10.0.0.3'}   # Workshop
    if USING_HOME_NETWORK:
        ip_map = {'robot1': '192.168.1.5'}   # Home
    base_station_ip = '10.0.0.100'

    # ====== PLAN GENERATION ========

    # Image configuration
    image_name = os.path.join(config_dir_path, 'MR.jpg')
    desired_width_dominos = 30
    desired_height_dominos = 40
    if USE_SMALL_TESTING_CONFIG:
        desired_width_dominos = 45
        desired_height_dominos = 60
    dominos = np.array(
                [('black', (0,0,0)),
                ('red',   (1,0,0)),
                ('blue',  (0,0,1)),
                ('green', (0,1,0)),
                ('white', (1,1,1))
                ], dtype=object)

    # Physical dimensions of dominos
    domino_width  = 0.025 # meters
    domino_height = 0.010 # meters
    domino_spacing_width = 0.037 # meters
    domino_spacing_height = 0.024 # meters

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
    robot_boundaries = np.array([[1,-11],[15,11]])                # Bottom left, top right, global frame
    base_station_boundaries = np.array([[0,1],[1,2]])           # Bottom left, top right, global frame
    base_station_target_pos = np.array([0.5, 1.5])              # Target position for robot to be under base station [x,y], in global frame
    base_station_target_angle = 180                             # Target angle (deg) for base station in global frame
    base_station_coarse_pose_offset = np.array([-1.5, 0])       # Offset from base station to use for apprach [x,y] in robot frame
    domino_field_top_left = np.array([3,3])                       # Top left corner of domino field in global frame
    domino_field_angle = 90                                     # Domino field angle (deg), global frame
    tile_placement_coarse_offset = np.array([0.3,-0.3])         # Offset position for tile placement [x,y], in robot coordinate frame
    tile_to_robot_offset = np.array([-0.3, -tile_size_width_meters/2.0])  # Offset from bottom left of tile to robot center [x,y], in robot coordinate frame
    distance_placement_pose = np.array([0.865,0.20,0])              # Target distance values for fine placement       
    prep_position_distance = 1                                  # How far out of field boundaries to do robot prep move
    exit_position_distance = 1                                  # How far out of the field boundaries to move to exit
    field_to_robot_frame_angle = 90                             # In case robot frame and field frame ever need to be rotated relative to each other

    if USE_SMALL_TESTING_CONFIG:  
        load_pose = np.array([9,-8,90])            
        domino_field_top_left = np.array([8.3,-4.1])  
        domino_field_angle = 0
        tile_placement_coarse_offset = np.array([-0.5,-0.5])
        tile_to_robot_offset = np.array([-0.3, -tile_size_width_meters/2.0 - 0.1])    # Testing a hack - x tile offset smaller so tray goes slightly over dominos                               

    # Computed - don't change
    field_width = tile_size_width_meters * desired_width_dominos/tile_width
    field_height = tile_size_height_meters * desired_height_dominos/tile_height
    domino_field_origin = domino_field_top_left + Utils.TransformPos(np.array([0,-field_height]), [0,0], domino_field_angle)
    domino_field_top_right = domino_field_origin + Utils.TransformPos(np.array([field_width,field_height]), [0,0], domino_field_angle)
    domino_field_boundaries = np.array([domino_field_origin, domino_field_top_right])  

    # ====== GUI CONFIGURATION ========
    graph_padding = 0.5                          # How many meters of padding to have around edge of graph
    axes_label_offset = 0.25                     # How many meters to offset labels on axes
    axes_tick_spacing = 5                        # How many meters of space between tick marks
    axes_tick_size = 0.5                         # How many meters long the tick marks should be
    robot_chassis_size = 0.6                     # How many meters wide and long the robot chassis is
    robot_rotation_center_offset = 0.245         # How many meters from front of robot to center of rotation
    robot_direction_indicator_length = 0.8       # How many meters long to make the direction line for the robot render
    robot_direction_indicator_arrow_length = 0.4  # How many meters long to make direction arrow lines
    robot_direction_indicator_arrow_angle = 25    # How many degrees to make arrow angle lines

    # Computed - don't change
    graph_bottom_left = np.array((
        min(robot_boundaries[0][0], 0) - graph_padding,
        robot_boundaries[0][1] - graph_padding
    ))
    graph_size = np.max(robot_boundaries[1] - robot_boundaries[0]) + 2*graph_padding
    graph_top_right = graph_bottom_left + np.array([graph_size,graph_size])


    # ====== RUNTIME CONFIGURATION ========
    robot_status_wait_time = 0.5    # How many seconds to wait between status requests for each robot
    base_station_status_wait_time = 1 # How many seconds to wait between status requests for the base station
    robot_next_action_wait_time = 2.0 # How many seconds to wait before checking if robot is finished with current plan action
