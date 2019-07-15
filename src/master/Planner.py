import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import skimage.transform as sktf
import numpy as np
import matplotlib.patches as patches
import math


def generateField(cfg):
    # Load original image
    img = mpimg.imread(cfg.image_name)

    # Scaled image
    img_scaled = sktf.resize(img, (cfg.desired_height, cfg.desired_width), anti_aliasing=False)

    # Parse image into domino IDs by choosing 'closest' color
    img_parsed_color = np.zeros((cfg.desired_width * cfg.desired_height, 3))
    img_parsed_ids = np.zeros_like(img_parsed_color, dtype=np.int_)[:, 0]
    count = 0
    for row in img_scaled:
        for px in row:
            best_id = -1
            best_score = 9999999
            for id, val in enumerate(cfg.dominoes):
                color = val[1]
                score = np.linalg.norm(px - color)
                if score < best_score:
                    best_score = score
                    best_id = id
            img_parsed_ids[count] = best_id
            img_parsed_color[count] = cfg.dominoes[best_id][1]
            count = count + 1

    img_parsed_ids = img_parsed_ids.reshape((cfg.desired_height, cfg.desired_width))
    img_parsed_color = img_parsed_color.reshape((cfg.desired_height, cfg.desired_width, 3))

    return DominoField(cfg, img, img_scaled, img_parsed_ids, img_parsed_color)


def generateTileOrdering(num_x, num_y):

    coord = [num_x - 1, num_y - 1] # Starting at top right
    order = 0
    order_map = {tuple(coord): order}
    done = False
    next_pass = np.copy(coord)
    next_pass[1] = coord[1] - 1 # Next pass starts at far right, next row down
    order = order + 1
    while not done:
        # If you are at the top or left, go to next pass
        if coord[1] + 1 >= num_y or coord[0] - 1 < 0:

            coord = np.copy(next_pass)

            # Compute new value for next pass
            if next_pass[1] > 0:
                next_pass[1] = next_pass[1] - 1 # Next pass starts at far right, next row down
            elif next_pass[1] == 0:
                next_pass[0] = next_pass[0] - 1  # If we reach the bottom row, shift to the left instead
            else:
                raise ValueError("Whoops! Shouldn't ever get here!")

        else:
            coord[0] = coord[0] - 1 # Move up and left
            coord[1] = coord[1] + 1

        # Save the order in the map for this coordinate
        order_map[tuple(coord)] = order
        order = order + 1

        # If we got to 0,0 then we are done
        if coord[0] == 0 and coord[1] == 0:
            done = True

    return order_map


def generateWaypoints(tiles, cfg):
    # All waypoint positions are with respect to the marvelmind sensor frame

    counter = 0
    waypoints_by_tile_order = {}
    while counter < tiles.getNumberOfTiles():

        # Calculate tile placement position
        tile = tiles.getTileByOrder(counter)
        tile_pos_in_field_frame = np.array(tile.getPlacementPositionInMeters())
        tile_pos_in_global_frame = tile_pos_in_field_frame + cfg.domino_field_origin
        robot_pos_for_tile_placement = tile_pos_in_global_frame + cfg.frame_tile_to_robot
        waypoint_for_tile_placement = Waypoint(robot_pos_for_tile_placement[0], robot_pos_for_tile_placement[1], 90)

        # Prep position is where the robot lines up with the tile x position outside of the field boundaries
        # and prepares to drive forward to place the tile
        prep_x = waypoint_for_tile_placement.x
        prep_y = cfg.domino_field_origin[1] - cfg.prep_position_distance
        waypoint_for_prep_pos = Waypoint(prep_x, prep_y, 90)

        # Exit position is where the robot exits the field boundaries and drives to tile dropoff location
        exit_x = cfg.domino_field_origin[0] - cfg.exit_position_distance
        exit_y = waypoint_for_tile_placement.y
        exit_a = 180
        waypoint_for_exit_pos = Waypoint(exit_x, exit_y, exit_a)

        # Dropoff location
        waypoint_for_dropoff = Waypoint(cfg.tile_drop_location[0] + 0.5, cfg.tile_drop_location[1] + 0.5, 270)

        # Pickup location
        waypoint_for_pickup = Waypoint(cfg.tile_pickup_location[0] + 0.5, cfg.tile_pickup_location[1] + 0.5, 270)

        path = (waypoint_for_pickup, waypoint_for_prep_pos, waypoint_for_tile_placement, waypoint_for_exit_pos, waypoint_for_dropoff)
        waypoints_by_tile_order[counter] = path
        counter += 1

    return WaypointManager(cfg, waypoints_by_tile_order, tiles)

class DominoField:

    def __init__(self, cfg, img, img_scaled, img_parsed_ids, img_parsed_color):
        self.cfg = cfg
        self.img = img
        self.img_scaled = img_scaled
        self.img_parsed_ids = img_parsed_ids
        self.img_parsed_color = img_parsed_color

    def printStats(self):
        # Output some metrics
        print('Domino usage:')
        print('Total number of dominoes: ' + str(self.img_parsed_ids.size))
        print('Colors:')
        unique_colors, counts = np.unique(self.img_parsed_ids, return_counts=True)
        for i, id in enumerate(unique_colors):
            print('  ' + self.cfg.dominoes[id][0] + ': ' + str(counts[i]))

    def show(self):
        # Plot images
        fig, axes = plt.subplots(nrows=1, ncols=3)
        ax = axes.ravel()

        ax[0].imshow(self.img)
        ax[0].set_title('Original')
        ax[1].imshow(self.img_scaled)
        ax[1].set_title('Scaled')
        ax[2].imshow(self.img_parsed_color)
        ax[2].set_title('Dominoes')

        plt.tight_layout()
        figManager = plt.get_current_fig_manager()
        figManager.window.state('zoomed')
        plt.show()

    def generateTiles(self):

        # Check sizes and make sure things line up
        if self.cfg.desired_height % self.cfg.tile_height != 0:
            raise ValueError('Field height is not evenly divisible by tile height!')
        if self.cfg.desired_width % self.cfg.tile_width != 0:
            raise ValueError('Field width is not evenly divisible by tile width!')

        # Determine number of tiles needed in x and y
        n_tiles_x = int(self.cfg.desired_width / self.cfg.tile_width)
        n_tiles_y = int(self.cfg.desired_height / self.cfg.tile_height)

        order_map = generateTileOrdering(n_tiles_x, n_tiles_y)

        # Loop over tiles and assign id and colors to each
        tiles = TileCollection(self.cfg)
        for i in range(n_tiles_x):
            x_start_idx = i * self.cfg.tile_width
            x_end_idx = (i + 1) * self.cfg.tile_width

            for j in range(n_tiles_y):
                # Need to account for flipped y axis with array
                y_start_idx =  - j * self.cfg.tile_height - 1
                y_end_idx = - (j + 1) * self.cfg.tile_height - 1

                tile_values = np.copy(self.img_parsed_ids[y_start_idx:y_end_idx:-1, x_start_idx:x_end_idx])
                tile_coord = (i,j)
                tile_order = order_map[tile_coord]
                tiles.addTile(tile_coord, tile_values, tile_order)

        return tiles



class Tile:

    def __init__(self, cfg, coordinate, values, order):
        self.coordinate = coordinate
        self.values = values
        self.cfg = cfg
        self.order = order

    def getPlacementPositionInMeters(self):
        # Returns bottom left corner position of the tile relative to the origin of the field

        tile_start_x = self.coordinate[0] * self.cfg.tile_size_x_meters
        tile_start_y = self.coordinate[1] * self.cfg.tile_size_y_meters
        return (tile_start_x, tile_start_y)


    def draw(self, array):
        # Note that this function draws pixels assuming the array is indexed as x,y instead of rows and columns
        # the array is flipped to plot as an image in the parent function

        # Determine tile location
        tile_size_x_in_px = (self.cfg.domino_width_px + self.cfg.domino_spacing_x_px) * self.cfg.tile_width
        tile_size_y_in_px = (self.cfg.domino_height_px + self.cfg.domino_spacing_y_px) * self.cfg.tile_height
        tile_start_x_px = self.coordinate[0] * tile_size_x_in_px
        tile_start_y_px = self.coordinate[1] * tile_size_y_in_px
        tile_end_x_px = tile_start_x_px + tile_size_x_in_px
        tile_end_y_px = tile_start_y_px + tile_size_y_in_px

        # Fill in tile with background color
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px:tile_end_y_px, 0] = self.cfg.tile_background_color[0]
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px:tile_end_y_px, 1] = self.cfg.tile_background_color[1]
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px:tile_end_y_px, 2] = self.cfg.tile_background_color[2]

        # Fill in tile edge color (only have to do start locations since next tile over will fill in end locations)
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px, 0] = self.cfg.tile_edge_color[0]
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px, 1] = self.cfg.tile_edge_color[1]
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px, 2] = self.cfg.tile_edge_color[2]
        array[tile_start_x_px, tile_start_y_px:tile_end_y_px, 0] = self.cfg.tile_edge_color[0]
        array[tile_start_x_px, tile_start_y_px:tile_end_y_px, 1] = self.cfg.tile_edge_color[1]
        array[tile_start_x_px, tile_start_y_px:tile_end_y_px, 2] = self.cfg.tile_edge_color[2]

        # Draw dominoes
        for i in range(self.cfg.tile_width):
            domino_start_x = tile_start_x_px + self.cfg.domino_spacing_x_px + (self.cfg.domino_width_px + self.cfg.domino_spacing_x_px) * i
            domino_end_x = domino_start_x + self.cfg.domino_width_px
            for j in range(self.cfg.tile_height):
                domino_start_y = tile_start_y_px + self.cfg.domino_spacing_y_px + (self.cfg.domino_height_px + self.cfg.domino_spacing_y_px) * j
                domino_end_y = domino_start_y + self.cfg.domino_height_px
                domino_id = self.values[j, i]
                domino_color = self.cfg.dominoes[domino_id][1]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 0] = domino_color[0]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 1] = domino_color[1]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 2] = domino_color[2]


class TileCollection:

    def __init__(self, cfg):
        self.tiles = []
        self.cfg = cfg
        self.n_tiles_x = 0
        self.n_tiles_y = 0
        self.tile_order_map = {}

    def addTile(self, tile_coordinate, tile_values, tile_order):

        new_tile = Tile(self.cfg, tile_coordinate, tile_values, tile_order)
        self.tiles.append(new_tile)
        self.tile_order_map[tile_order] = new_tile

        if tile_coordinate[0] > self.n_tiles_x:
            self.n_tiles_x = tile_coordinate[0] + 1
        if tile_coordinate[1] > self.n_tiles_y:
            self.n_tiles_y = tile_coordinate[1] + 1

    def getTileByOrder(self, order):
        return self.tile_order_map[order]

    def getNumberOfTiles(self):
        return self.n_tiles_y * self.n_tiles_x

    def draw(self):

        # Allocate memory for image
        tile_size_x_in_px = (self.cfg.domino_width_px + self.cfg.domino_spacing_x_px) * self.cfg.tile_width
        tile_size_y_in_px = (self.cfg.domino_height_px + self.cfg.domino_spacing_y_px) * self.cfg.tile_height
        array_size_x = tile_size_x_in_px * self.n_tiles_x
        array_size_y = tile_size_y_in_px * self.n_tiles_y
        image_array = np.zeros((array_size_x, array_size_y, 3))

        # Generate image
        for tile in self.tiles:
            tile.draw(image_array)

        # Modify array to show image correctly
        image_array = np.transpose(image_array, (1, 0, 2))
        image_array = np.flip(image_array, 0)

        # Actually show image
        plt.imshow(image_array)
        figManager = plt.get_current_fig_manager()
        figManager.window.state('zoomed')
        plt.show()

    def show_ordering(self):

        # Build array of order to show and write
        order_array = np.zeros((self.n_tiles_x,self.n_tiles_y))
        for tile in self.tiles:
            order_array[tile.coordinate] = tile.order

        # Modify array to show image correctly
        order_array = np.transpose(order_array, (1, 0))

        # Show figure with colors
        fig = plt.figure()
        ax = fig.add_subplot(111)
        im = ax.imshow(order_array, origin='lower', cmap='cool', interpolation='None')

        for x in range(self.n_tiles_x):
            for y in range(self.n_tiles_y):
                label = int(order_array[y, x])
                ax.text(x, y, label, color='black', ha='center', va='center')

        fig.colorbar(im)
        plt.show()


class Waypoint:

    def __init__(self, x, y, a):
        # X position [m]
        # Y position [m]
        # Angle [deg]

        self.x = float(x)
        self.y = float(y)
        self.a = float(a)

    def getPos(self):
        return np.array([self.x, self.y])

    def getAngleDegrees(self):
        return self.a

    def getAngleRadians(self):
        return self.a * math.pi/180.0

    def __add__(self, other):
        self.x = self.x + other.x
        self.y = self.y + other.y
        self.a = self.a + other.a

    def __sub__(self, other):
        self.x = self.x - other.x
        self.y = self.y - other.y
        self.a = self.a - other.a

    def draw(self, ax):

       # Base triangle at 0 degrees
       scale = 0.3
       p1 = np.array([scale, 0])
       s = math.sin(45*math.pi/180.0)
       c = math.cos(45 * math.pi / 180.0)
       p2 = np.array([-scale*s, scale*c])
       p3 = np.array([-scale*s, -scale*c])
       points = np.vstack((p1, p2 ,p3))

       # Rotate for orientation
       s = math.sin(self.getAngleRadians())
       c = math.cos(self.getAngleRadians())
       R = np.array([[c, -s],[s, c]])

       for i in range(3):
           # Do local rotation
           points[i,:] = np.matmul(R, np.reshape(points[i,:],(2,1))).ravel()

           # Then offset for position
           points[i, :] = points[i, :] + self.getPos()

       ax.add_patch(patches.Polygon(points,
                                    fill=True,
                                    edgecolor='c',
                                    facecolor='c'))

class WaypointManager:

    def __init__(self, cfg, waypoints, tiles):
        self.cfg = cfg
        self.waypoints_by_tile_order = waypoints
        self.tiles = tiles

    def getWaypointsByOrder(self, order):
        return self.waypoints_by_tile_order[order]

    def drawWaypoints(self, order):

        # Draw overall map
        m = Map(self.cfg)
        ax = m.draw()

        # Draw waypoints
        for wpts in self.getWaypointsByOrder(order):
            wpts.draw(ax)

        # Configure limits and show
        ax.set_xlim(left=-1, right=self.cfg.robot_boundaries[1][0]+1)
        ax.set_ylim(bottom=-1, top=self.cfg.robot_boundaries[1][1]+1)
        ax.axis('equal')
        figManager = plt.get_current_fig_manager()
        figManager.window.state('zoomed')
        plt.show()




class Map:

    def __init__(self, cfg):
        self.cfg = cfg

    def draw(self):

        fig,ax = plt.subplots(1)

        # Draw overall map boundaries
        ax.add_patch(patches.Rectangle(self.cfg.robot_boundaries[0],
                                       self.cfg.robot_boundaries[1][0] - self.cfg.robot_boundaries[0][0],
                                       self.cfg.robot_boundaries[1][1] - self.cfg.robot_boundaries[0][1],
                                       fill=False,
                                       edgecolor='b'))

        # Draw field boundaries
        ax.add_patch(patches.Rectangle(self.cfg.domino_field_origin,
                                       self.cfg.field_width,
                                       self.cfg.field_height,
                                       fill=False,
                                       edgecolor='r'))

        # Draw base station
        ax.add_patch(patches.Rectangle(self.cfg.base_station_boundaries[0],
                                       self.cfg.base_station_boundaries[1][0] - self.cfg.base_station_boundaries[0][0],
                                       self.cfg.base_station_boundaries[1][1] - self.cfg.base_station_boundaries[0][1],
                                       fill=True,
                                       edgecolor='k',
                                       facecolor='k'))

        # Draw charging station
        ax.add_patch(patches.Rectangle(self.cfg.charge_station_boundaries[0],
                                       self.cfg.charge_station_boundaries[1][0] - self.cfg.charge_station_boundaries[0][0],
                                       self.cfg.charge_station_boundaries[1][1] - self.cfg.charge_station_boundaries[0][1],
                                       fill=True,
                                       edgecolor='g',
                                       facecolor='g'))

        # Draw pickup location
        ax.add_patch(patches.Rectangle(self.cfg.tile_pickup_location,
                                       1,
                                       1,
                                       fill=False,
                                       edgecolor='m'))

        # Draw dropoff location
        ax.add_patch(patches.Rectangle(self.cfg.tile_drop_location,
                                       1,
                                       1,
                                       fill=False,
                                       edgecolor='c'))

        return ax







