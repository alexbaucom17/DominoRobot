import numpy as np

# Image configuration
image_name = 'MR.jpg'
desired_width = 300
desired_height = 300
dominoes = np.array(
            [('black', (0,0,0)),
             ('red',   (1,0,0)),
             ('blue',  (0,0,1)),
             ('green', (0,1,0)),
             ('white', (1,1,1))
             ])

# Tile configuration
tile_width = 10
tile_height = 30
n_available_tiles = 10

# Arm configuration
n_arms = 3

# Robot configuration
n_available_robots = 4
n_charging_ports = 2

# Map configuration (distances in meters, angles in degrees)
robot_boundaries = np.array([[0,0],[20,15]])
base_station_boundaries = np.array([[7,0],[14,2]])
charge_station_boundaries = np.array([[7,5],[8,8]])
tile_drop_location = np.array([8,1,-90]) # X, Y, theta
tile_pickup_location = np.array([13,1,-90]) # X, Y, theta
domino_field_boundaries = np.array([[14,4],[19,14]])