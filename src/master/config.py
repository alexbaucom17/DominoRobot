import numpy as np
import os
import Utils

class Config:

    # Various debug/test flags
    # Set to override config values for home network
    USING_HOME_NETWORK = False
    # Set for laptop vs desktop
    USING_DESKTOP = False
    # Set to override config values for small scale testing
    USE_SMALL_TESTING_CONFIG = False
    # Set to skip connecting to robot
    OFFLINE_TESTING = False
    # Set to skip connecting to base station
    SKIP_BASE_STATION = True
    # Set to skip connecting to Marvelmind
    SKIP_MARVELMIND = True
    # Set to use fake plan instead of loading a generated one
    USE_TEST_PLAN = False
    # MR LOGO plan
    MR_LOGO_PLAN = False
    # Set to auto-load this plan on master startup
    AUTO_LOAD_PLAN = False
    AUTO_LOAD_PLAN_NAME = "FullPlan_DominoBros.p"
    # Set to regenerate and auto load plan on master startup
    REGEN_PLAN = True
    # Set to true to use just a subsection of the overal plan
    USE_SUBSECTION = False

    # ====== PATHS ========
    
    root_path = "C:\\Users\\alexb\\Documents\\Github\\DominoRobot\\"  # Laptop
    if USING_DESKTOP:
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
    image_name = os.path.join(config_dir_path, 'DominoDesign-Questions.psd')
    num_tiles_width = 18
    num_tiles_height = 19
    dominos = np.array(
                [('black', (0,0,0)),
                ('red',   (1,0,0)),
                ('blue',  (0.188,0.5,0.886)),
                ('green', (0,1,0)),
                ('white', (1,1,1)),
                ('brown', (1,0.51,0)),
                ('yellow', (1,0.867,0)),
                ], dtype=object)

    if USE_SMALL_TESTING_CONFIG:  
        num_tiles_width = 2
        num_tiles_height = 4
    if MR_LOGO_PLAN:
        image_name = os.path.join(config_dir_path, 'logo.jpg')
        num_tiles_width = 5
        num_tiles_height = 5
        dominos = np.array(
                [('black', (0,0,0)),
                ('white', (1,1,1)),
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
    tile_background_color = (0.9,0.9,0.9)
    tile_edge_color = (0,0,1)
    tile_size_width_meters = 0.930 #tile_width * (domino_spacing_width + domino_width)
    tile_size_height_meters = 0.665 #tile_height * (domino_spacing_height + domino_height)
    desired_width_dominos = tile_width * num_tiles_width
    desired_height_dominos = tile_height * num_tiles_height

    # Map configureation
    grid_top_left = np.array([13.2, 6.9])
    grid_top_right = np.array([13.2, -10.07])
    grid_num_cols = 19
    grid_bottom_left = np.array([1.18,6.9])
    grid_bottom_right = np.array([1.18,-10.07])
    grid_num_rows = 19
    grid_tile_width = ((grid_top_left - grid_top_right)/grid_num_cols)[1]
    grid_tile_height = ((grid_top_left - grid_bottom_left)/grid_num_rows)[0]
    tile_size_width_meters = grid_tile_width
    # tile_size_height_meters = grid_tile_height

    # Vision offset configuration
    default_vision_offset = np.array((0,0,-1.5))
    vision_offset_file = os.path.join(plans_dir, 'vision_offsets_full_plan.csv')

    # ====== ENVIRONMENT CONFIGURATION ========

    # Map configuration (distances in meters, angles in degrees)
    robot_boundaries = np.array([[1,-11],[15,11]])              # Bottom left, top right, global frame
    highway_x = 3.5                                       # "Highway" coordinate
    load_waypoint = np.array([highway_x, 8.5])                    # xya (global frame) for waypoint to go to first before load prep
    highway_angle = 90

    base_station_boundaries = np.array([[2.5,10],[3.5,11]])         # Bottom left, top right, global frame
    base_station_target_angle = 175                             # Target angle (deg) for base station in global frame
    base_station_relative_offset = np.array([0.9, 0, 0])           # Relative position of base station from prep pos - robot frame (x,y,a)
    base_station_vision_offset = np.array([0.01,0.010,-0.5])     # Vision offset for base station alignment
    base_station_prep_pos = np.array([3.4, 9.5])                   # Pose outside of base station to align with before going in to dock
    base_station_prep_vision_offset = np.array([0,0.01,-1])      # Vision offset to use for base station prep pose

    robot_pose_top_left = grid_top_left                   # Robot pose in global frame for top left of tile position of domino field
    domino_field_angle = -90                                     # Domino field angle (deg), global frame
    tile_placement_fine_offset = np.array([0, 0.33])              # Offset position for fine tile placement [x,y], in robot coordinate frame (to avoid hitting next column)
    tile_placement_coarse_offset = np.array([-0.5,0.5])         # Offset position for tile placement [x,y], in robot coordinate frame
    tile_to_robot_offset = np.array([-0.3, -tile_size_width_meters/2.0])  # Offset from bottom left of tile to robot center [x,y], in robot coordinate frame     
    enter_position_distance = 1.5                                  # How far out of field boundaries to do robot prep move
    intermediate_entry_hz_y = 0                                 # Y coordinate for horizontal intermediate position
    intermediate_place_vt_x = 8                                 # X coordinate for vertical intermediate position
    field_to_robot_frame_angle = 90                             # In case robot frame and field frame ever need to be rotated relative to each other

    # Used for testing sub-sections of the larger pattern
    if USE_SUBSECTION:
        start_coords = (1,14)
        end_coords = (7,18)

    # Left side
    # if USE_SMALL_TESTING_CONFIG:  
    #     load_pose = np.array([7.5,7.5,0])            
    #     robot_pose_top_left = np.array([12.20,9.472])  
    #     domino_field_angle = -90
    #     tile_placement_coarse_offset = np.array([-0.5,-0.5])
    #     tile_to_robot_offset = np.array([-0.3, -tile_size_width_meters/2.0 ])                             

    # Right side
    if USE_SMALL_TESTING_CONFIG:  
        load_pose = np.array([8,-6.5,0])            
        robot_pose_top_left = np.array([12.74,-6.94])  
        domino_field_angle = -90
        tile_placement_coarse_offset = np.array([-0.5,-0.5])
        tile_to_robot_offset = np.array([-0.3, -tile_size_width_meters/2.0 ])    

    # Fine motion y offset adjustments
    # y_offset_cols = np.linspace(0.05, 0, num_tiles_height)
    y_offset_cols = np.linspace(0.0, 0, num_tiles_height)
    y_offset_rows = np.linspace(0, 0.0, num_tiles_width)
    x_offset_rows = np.linspace(0, 0.3, num_tiles_height)
    x_offset_rows[0:2] = 0
    # Angle adjustment for fine motion to try and prevent wheel from hitting
    angle_adjust_fine = 4   # degrees

    # Y offset
    # Pre col 4: 0.05
    # col 4: 0.08
    # col 5: 0.1
    # col 6: 0.12
    # col 7: 0.18
    # col 8: 0.20
    # col 9: 0.23
    # col 10: 0.26
    # col 11: 0.30
    # col 12: 0.33


    # Computed - don't change
    field_width = tile_size_width_meters * desired_width_dominos/tile_width
    field_height = tile_size_height_meters * desired_height_dominos/tile_height
    # Fix me
    domino_field_top_left = robot_pose_top_left + np.array([tile_size_height_meters-tile_to_robot_offset[0], -tile_to_robot_offset[1]]) \
         - np.array([0, 2*tile_size_width_meters])
        #Utils.TransformPos(np.array([tile_size_height_meters-tile_to_robot_offset[0], -tile_to_robot_offset[1]]), [0,0], domino_field_angle)
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
    robot_status_wait_time = 0.2    # How many seconds to wait between status requests for each robot
    base_station_status_wait_time = 1 # How many seconds to wait between status requests for the base station
    robot_next_action_wait_time = 2.0 # How many seconds to wait before checking if robot is finished with current plan action
