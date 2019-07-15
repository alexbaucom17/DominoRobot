import numpy as np

# Image configuration
image_name = 'MR.jpg'
desired_width = 200
desired_height = 200
dominoes = np.array(
            [('black', (0,0,0)),
             ('red',   (1,0,0)),
             ('blue',  (0,0,1)),
             ('green', (0,1,0)),
             ('white', (1,1,1))
             ])

# To get image to look right, need spacing to create square pixels for now
# could fix this later by making image field creation take into account non-square pixels
domino_width  = 0.024 # meters
domino_height = 0.008 # meters
domino_spacing_x = 0.008 # meters
domino_spacing_y = 0.024 # meters

# Spacing for drawing dominoes as pixels instead of rectangles
meters_per_pixel = 0.008
domino_width_px = round(domino_width / meters_per_pixel)
domino_height_px = round(domino_height / meters_per_pixel)
domino_spacing_x_px = round(domino_spacing_x / meters_per_pixel)
domino_spacing_y_px = round(domino_spacing_y / meters_per_pixel)

# Tile configuration
tile_width = 20
tile_height = 20
tile_background_color = (0.8, 0.8, 0.8)
tile_edge_color = (0,0,1)
tile_size_x_meters = tile_width * (domino_spacing_x + domino_width)
tile_size_y_meters = tile_height * (domino_spacing_y + domino_height)

# Map configuration (distances in meters, angles in degrees)
robot_boundaries = np.array([[0,0],[20,15]])
base_station_boundaries = np.array([[7,0],[14,2]])
charge_station_boundaries = np.array([[7,5],[8,8]])
tile_drop_location = np.array([8,1,-90]) # X, Y, theta
tile_pickup_location = np.array([13,1,-90]) # X, Y, theta
domino_field_origin = np.array([14,4])
field_width = tile_size_x_meters * desired_width/tile_width
field_height = tile_size_y_meters * desired_height/tile_height
domino_field_boundaries = np.array([domino_field_origin,domino_field_origin + np.array([field_width,field_height])])
prep_position_distance = 2 # How far out of field boundaries to do robot prep move
exit_position_distance = 2 # How far out of the field boundaries to move to exit

# Arm configuration
n_arms = 3

# Robot configuration
n_available_robots = 4
n_charging_ports = 2
frame_tile_to_robot = np.array([0.2, -0.25])