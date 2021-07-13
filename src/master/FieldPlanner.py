import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import skimage.transform as sktf
import numpy as np
import matplotlib.patches as patches
import math
import enum
import logging
import os
import Utils
from Utils import ActionTypes
import config
import pickle
import csv
import copy


def generateVisionOffsetMap(cfg, max_x, max_y):
    
    vision_offset_map = { (x,y): cfg.default_vision_offset for x in range(max_x) for y in range(max_y) }
    with open(cfg.vision_offset_file) as csvfile:
        reader = csv.reader(csvfile)
        for idx,row in enumerate(reader):
            if idx == 0:
                continue
            tile_x = int(row[0])
            tile_y = int(row[1])
            offset_x_meters = int(row[2]) / 1000.0
            offset_y_meters = int(row[3]) / 1000.0
            offset_a_degrees = float(row[4])
            add_to_default = bool(row[5])
            key = (tile_x,tile_y)
            value = (offset_x_meters,offset_y_meters,offset_a_degrees)
            if add_to_default:
                value = (   cfg.default_vision_offset[0] + offset_x_meters,
                            cfg.default_vision_offset[1] + offset_y_meters,
                            cfg.default_vision_offset[2] + offset_a_degrees
                        )
            vision_offset_map[key] = value

    return vision_offset_map


class DominoField:
    """
    Data and methods for parsing and image and generating a field of tiles of dominos
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.img = None
        self.img_scaled = None
        self.img_parsed_ids = None
        self.img_parsed_color = None
        self.n_tiles_x = 0
        self.n_tiles_y = 0
        self.tiles = []

    def generate(self):

        logging.info('Generating domino field from image...')
        self._generateField()
        logging.info('done.')

        logging.info('Generating tiles from field...')
        self._generateTiles()
        logging.info('done.')


    def printStats(self):
        # Output some metrics
        logging.info("Original image size: {}".format(self.img.shape[:2]))
        logging.info("Scaled image size: {}".format(self.img_scaled.shape[:2]))
        logging.info('Domino usage:')
        logging.info('Total number of dominos: ' + str(self.img_parsed_ids.size))
        logging.info('Colors:')
        unique_colors, counts = np.unique(self.img_parsed_ids, return_counts=True)
        for i, id in enumerate(unique_colors):
            logging.info('  ' + self.cfg.dominos[id][0] + ': ' + str(counts[i]))

    def show_image_parsing(self):
        # Plot images
        fig, axes = plt.subplots(nrows=1, ncols=3)
        ax = axes.ravel()

        ax[0].imshow(self.img)
        ax[0].set_title('Original')
        ax[1].imshow(self.img_scaled)
        ax[1].set_title('Scaled')
        ax[2].imshow(self.img_parsed_color)
        ax[2].set_title('Dominos')

        plt.tight_layout()
        figManager = plt.get_current_fig_manager()
        figManager.window.state('zoomed')
        plt.show()

    def render_domino_image_tiles(self):

        # Allocate memory for image
        tile_size_x_in_px = (self.cfg.domino_width_px + self.cfg.domino_spacing_width_px) * self.cfg.tile_width
        tile_size_y_in_px = (self.cfg.domino_height_px + self.cfg.domino_spacing_height_px) * self.cfg.tile_height
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

    def show_tile_ordering(self):

        # Build array of order to show and write
        order_array = np.zeros((self.n_tiles_x,self.n_tiles_y))
        # print(self.n_tiles_x)
        # print(self.n_tiles_y)
        for tile in self.tiles:
            # print(tile.coordinate)
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

    def _generateField(self):
        # Load original image
        img = mpimg.imread(self.cfg.image_name)

        # Skip A value for RGBA files
        if img.shape[2] is 4:
            img = img[:,:,:3]

        # Do some modifications to MR logo to get it the right shape
        if self.cfg.MR_LOGO_PLAN:

            # Crop vertical
            vert_size = img.shape[0]
            crop_factor = 0.4
            lb = int(vert_size/2 - crop_factor*vert_size)
            ub = int(vert_size/2 + crop_factor*vert_size)
            img = img[lb:ub,:,:]

            # Padd horizontal
            hz_size = img.shape[1]
            pad_amount = int(0.1 * hz_size)
            img = np.pad(img,  pad_width=[(0, 0), (pad_amount, pad_amount),(0, 0)], mode='constant')

        # Scaled image
        img_scaled = sktf.resize(img, (self.cfg.desired_height_dominos, self.cfg.desired_width_dominos), anti_aliasing=False)

        # Parse image into domino IDs by choosing 'closest' color
        img_parsed_color = np.zeros((self.cfg.desired_width_dominos * self.cfg.desired_height_dominos, 3))
        img_parsed_ids = np.zeros_like(img_parsed_color, dtype=np.int_)[:, 0]
        count = 0
        for row in img_scaled:
            for px in row:
                best_id = -1
                best_score = 9999999
                for id, val in enumerate(self.cfg.dominos):
                    color = val[1]
                    score = np.linalg.norm(px - color)
                    if score < best_score:
                        best_score = score
                        best_id = id
                img_parsed_ids[count] = best_id
                img_parsed_color[count] = self.cfg.dominos[best_id][1]
                count = count + 1

        img_parsed_ids = img_parsed_ids.reshape((self.cfg.desired_height_dominos, self.cfg.desired_width_dominos))
        img_parsed_color = img_parsed_color.reshape((self.cfg.desired_height_dominos, self.cfg.desired_width_dominos, 3))

        self.img = img
        self.img_scaled = img_scaled
        self.img_parsed_ids = img_parsed_ids
        self.img_parsed_color = img_parsed_color

    def _addTile(self, tile_coordinate, tile_values, tile_order, vision_offset_map):

        vision_offset = vision_offset_map[tile_coordinate]
        # print("Tile: order {}, coord {}, vision offset: {}".format(tile_order, tile_coordinate, vision_offset))
        new_tile = Tile(self.cfg, tile_coordinate, tile_values, tile_order, vision_offset)
        self.tiles.append(new_tile)

    def _generateTiles(self):

        # Check sizes and make sure things line up
        if self.cfg.desired_height_dominos % self.cfg.tile_height != 0:
            raise ValueError('Field height is not evenly divisible by tile height!')
        if self.cfg.desired_width_dominos % self.cfg.tile_width != 0:
            raise ValueError('Field width is not evenly divisible by tile width!')

        # Determine number of tiles needed in x and y
        self.n_tiles_x = int(self.cfg.desired_width_dominos / self.cfg.tile_width)
        self.n_tiles_y = int(self.cfg.desired_height_dominos / self.cfg.tile_height)
        print("Generating tiles {} x {}".format(self.n_tiles_x, self.n_tiles_y))

        order_map = self._generateTileOrdering(self.n_tiles_x, self.n_tiles_y)
        vision_offset_map = generateVisionOffsetMap(self.cfg, self.n_tiles_x, self.n_tiles_y)

        # Loop over tiles and assign id and colors to each
        for i in range(self.n_tiles_x):
            x_start_idx = i * self.cfg.tile_width
            x_end_idx = (i + 1) * self.cfg.tile_width

            for j in range(self.n_tiles_y):
                # Need to account for flipped y axis with array
                y_start_idx =  - j * self.cfg.tile_height - 1
                y_end_idx = - (j + 1) * self.cfg.tile_height - 1

                tile_values = np.copy(self.img_parsed_ids[y_start_idx:y_end_idx:-1, x_start_idx:x_end_idx])
                tile_coord = (i,j)
                tile_order = order_map[tile_coord]
                self._addTile(tile_coord, tile_values, tile_order, vision_offset_map)

        # Sort tile array so they are in order
        self.tiles = sorted(self.tiles, key = lambda tile: tile.order)

    @classmethod
    def _generateTileOrdering(cls, num_x, num_y):
        """
        Generates and ordering that maps x y tile coordinate -> order number
        """

        # return cls._generateTileOrderingDiagonal(num_x, num_y)
        return cls._generateTileOrderingColumns(num_x, num_y)

    @classmethod
    def _generateTileOrderingColumns(cls, num_x, num_y):
        def coordToOrderLR(coord, num_x, num_y):
            return coord[0]*num_y + num_y - coord[1] -1
        def coordToOrderRL(coord, num_x, num_y):
            return (num_x - coord[0] - 1)*num_y + num_y - coord[1] -1
        all_coords = [(x,y) for x in range(num_x) for y in range(num_y)]
        order_map = {coord: coordToOrderRL(coord, num_x, num_y) for coord in all_coords}
        return order_map

    @classmethod
    def _generateTileOrderingDiagonal(cls, num_x, num_y):

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


class Tile:
    """
    Holds info and useful methods related to an individual domino tile
    """

    def __init__(self, cfg, coordinate, values, order, vision_offset):
        self.coordinate = coordinate
        self.vision_offset = vision_offset
        self.values = values
        self.cfg = cfg
        self.order = order

    def getPlacementPositionInMeters(self):
        # Returns bottom left corner position of the tile relative to the origin of the field

        tile_start_x = self.coordinate[0] * self.cfg.tile_size_width_meters
        tile_start_y = self.coordinate[1] * self.cfg.tile_size_height_meters
        return (tile_start_x, tile_start_y)


    def draw(self, array):
        # Note that this function draws pixels assuming the array is indexed as x,y instead of rows and columns
        # the array is flipped to plot as an image in the parent function


        # Determine tile location
        tile_size_x_in_px = (self.cfg.domino_width_px + self.cfg.domino_spacing_width_px) * self.cfg.tile_width
        tile_size_y_in_px = (self.cfg.domino_height_px + self.cfg.domino_spacing_height_px) * self.cfg.tile_height
        tile_start_x_px = self.coordinate[0] * tile_size_x_in_px
        tile_start_y_px = self.coordinate[1] * tile_size_y_in_px
        tile_end_x_px = tile_start_x_px + tile_size_x_in_px
        tile_end_y_px = tile_start_y_px + tile_size_y_in_px

        self.draw_single(array, tile_start_x_px, tile_start_y_px, tile_end_x_px, tile_end_y_px)

    def draw_single(self, array, tile_start_x_px, tile_start_y_px, tile_end_x_px, tile_end_y_px, draw_edge=True):

        # Fill in tile with background color
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px:tile_end_y_px, 0] = self.cfg.tile_background_color[0]
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px:tile_end_y_px, 1] = self.cfg.tile_background_color[1]
        array[tile_start_x_px:tile_end_x_px, tile_start_y_px:tile_end_y_px, 2] = self.cfg.tile_background_color[2]

        # Fill in tile edge color (only have to do start locations since next tile over will fill in end locations)
        if draw_edge:
            array[tile_start_x_px:tile_end_x_px, tile_start_y_px, 0] = self.cfg.tile_edge_color[0]
            array[tile_start_x_px:tile_end_x_px, tile_start_y_px, 1] = self.cfg.tile_edge_color[1]
            array[tile_start_x_px:tile_end_x_px, tile_start_y_px, 2] = self.cfg.tile_edge_color[2]
            array[tile_start_x_px, tile_start_y_px:tile_end_y_px, 0] = self.cfg.tile_edge_color[0]
            array[tile_start_x_px, tile_start_y_px:tile_end_y_px, 1] = self.cfg.tile_edge_color[1]
            array[tile_start_x_px, tile_start_y_px:tile_end_y_px, 2] = self.cfg.tile_edge_color[2]

        # Draw dominos
        for i in range(self.cfg.tile_width):
            domino_start_x = tile_start_x_px + self.cfg.domino_spacing_width_px + (self.cfg.domino_width_px + self.cfg.domino_spacing_width_px) * i
            domino_end_x = domino_start_x + self.cfg.domino_width_px
            for j in range(self.cfg.tile_height):
                domino_start_y = tile_start_y_px + self.cfg.domino_spacing_height_px + (self.cfg.domino_height_px + self.cfg.domino_spacing_height_px) * j
                domino_end_y = domino_start_y + self.cfg.domino_height_px
                domino_id = self.values[j, i]
                domino_color = self.cfg.dominos[domino_id][1]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 0] = domino_color[0]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 1] = domino_color[1]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 2] = domino_color[2]


class Action:

    def __init__(self, action_type, name):
        self.action_type = action_type
        self.name = name

    def draw(self, ax, text=""):
        pass

class SetPoseAction(Action):
    def __init__(self, action_type, name, x, y, a):
        # action_type (enum)
        # string name
        # X position [m]
        # Y position [m]
        # Angle [deg]

        super().__init__(action_type, name)

        self.x = float(x)
        self.y = float(y)
        self.a = math.radians(float(a))

class WaitAction(Action):
    def __init__(self,action_type, name, time):
        super().__init__(action_type, name)
        self.time = time

class MoveConstVelAction(Action):

    def __init__(self, action_type, name, vx, vy, va, t):
        # action_type (enum)
        # string name
        # X velocity [m/s]
        # Y velocity [m/s]
        # Angle [rad/s]
        # time [sec]

        super().__init__(action_type, name)

        self.vx = float(vx)
        self.vy = float(vy)
        self.va = float(va)
        self.t = float(t)

class MoveAction(Action):

    def __init__(self, action_type, name, x, y, a):
        # action_type (enum)
        # string name
        # X position [m]
        # Y position [m]
        # Angle [deg]

        super().__init__(action_type, name)

        self.x = float(x)
        self.y = float(y)
        self.a = math.radians(float(a))

    def getPos(self):
        return np.array([self.x, self.y])

    def getAngleDegrees(self):
        return math.degrees(self.a)

    def getAngleRadians(self):
        return self.a

    def draw(self, ax, text="", show_label=True):

        if self.action_type in [ActionTypes.MOVE_WITH_VISION, ActionTypes.MOVE_REL, ActionTypes.MOVE_REL_SLOW]:
            return

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
        text_point = points[0]
        text_to_show = self.name
        if text is not "":
            text_to_show = text
        if show_label:
            ax.annotate(text_to_show, xy=text_point[:2], xytext=[1, 1], textcoords="offset points", fontsize=8, color="green")


def generate_full_action_sequence(cfg, tile):
    """
    Standard sequence:
    - Move to load
    - Do load
    - Move out of load
    - Move to field entry
    - Move to coarse drop off
    - Wait for localization
    - Move to fine drop off
    - Drop off
    - Move to coarse drop off
    - Move to field exit
    - Move to near load
    """

    # Setup positions in field frame
    tile_pos_in_field_frame = np.array(tile.getPlacementPositionInMeters())
    robot_placement_pos_field_frame = tile_pos_in_field_frame + Utils.TransformPos(cfg.tile_to_robot_offset, [0,0], cfg.field_to_robot_frame_angle)
    robot_placement_fine_pos_field_frame = robot_placement_pos_field_frame + Utils.TransformPos(cfg.tile_placement_fine_offset, [0,0], cfg.field_to_robot_frame_angle)
    robot_placement_coarse_pos_field_frame = robot_placement_fine_pos_field_frame + Utils.TransformPos(cfg.tile_placement_coarse_offset, [0,0], cfg.field_to_robot_frame_angle)

    # Convert positions to global frame
    robot_placement_coarse_pos_global_frame = Utils.TransformPos(robot_placement_coarse_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_placement_fine_pos_global_frame = Utils.TransformPos(robot_placement_fine_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_field_angle = cfg.domino_field_angle + cfg.field_to_robot_frame_angle

    # Setup various entry and intermediate positions
    intermediate_entry_pos_global_frame = np.array([cfg.highway_x, cfg.intermediate_entry_hz_y])
    entry_y = robot_placement_coarse_pos_global_frame[1]+cfg.enter_position_distance
    field_entry_pos_global_frame = np.array([cfg.highway_x, entry_y])
    intermediate_place_pos_global_frame = np.array([robot_placement_coarse_pos_global_frame[0], entry_y])

    # Figure out if intermediate steps are needed
    intermediate_hz = robot_placement_coarse_pos_global_frame[1] < cfg.intermediate_entry_hz_y - 1
    intermediate_vt = robot_placement_coarse_pos_global_frame[0] > cfg.intermediate_place_vt_x + 1

    # Tiles near the back wall don't have enough space for backwards offset
    relative_tile_offset = copy.deepcopy(cfg.tile_placement_coarse_offset)
    if robot_placement_coarse_pos_global_frame[0] < 1:
        robot_placement_coarse_pos_global_frame[0] = robot_placement_fine_pos_global_frame[0]
        relative_tile_offset = np.asarray([0, 1])

    actions = []

    name = "Move to load waypoint - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, cfg.load_waypoint[0], cfg.load_waypoint[1], cfg.base_station_target_angle))

    name = "Fake wait for loading"
    actions.append(WaitAction(ActionTypes.WAIT, name, 20))

    # name = "Wait for localization"
    # actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    # name = "Move to near load prep - coarse"
    # actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, cfg.base_station_prep_pos[0], cfg.base_station_prep_pos[1], cfg.base_station_target_angle))

    # name = "Start cameras"
    # actions.append(Action(ActionTypes.START_CAMERAS, name))

    # name = "Wait for localization"
    # actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    # name = "Move to load prep - fine"
    # actions.append(MoveAction(ActionTypes.MOVE_FINE, name, cfg.base_station_prep_pos[0], cfg.base_station_prep_pos[1], cfg.base_station_target_angle))

    # name = "Move to load prep - vision"
    # actions.append(MoveAction(ActionTypes.MOVE_WITH_VISION, name, cfg.base_station_prep_vision_offset[0], cfg.base_station_prep_vision_offset[1], cfg.base_station_prep_vision_offset[2]))
    
    # name = "Move to load - relative slow"
    # actions.append(MoveAction(ActionTypes.MOVE_REL_SLOW, name, cfg.base_station_relative_offset[0], cfg.base_station_relative_offset[1], cfg.base_station_relative_offset[2]))

    # name = "Align with load"
    # actions.append(MoveAction(ActionTypes.MOVE_WITH_VISION, name, cfg.base_station_vision_offset[0], cfg.base_station_vision_offset[1], cfg.base_station_vision_offset[2]))

    # name = "Stop cameras"
    # actions.append(Action(ActionTypes.STOP_CAMERAS, name))

    # name = "Load tile"
    # actions.append(Action(ActionTypes.LOAD, name))

    # name = "Move away from load - relative slow"
    # actions.append(MoveAction(ActionTypes.MOVE_REL_SLOW, name, cfg.base_station_relative_offset[0], cfg.base_station_relative_offset[1], cfg.base_station_relative_offset[2]))

    # name = "Move to load waypoint - coarse"
    # actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, cfg.load_waypoint[0], cfg.load_waypoint[1], cfg.highway_angle))

    # name = "Wait for localization"
    # actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    # if intermediate_hz:
    #     name = "Move to intermediate enter - coarse"
    #     actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, intermediate_entry_pos_global_frame[0], intermediate_entry_pos_global_frame[1], cfg.highway_angle))

    #     name = "Wait for motor cooldown (long)"
    #     actions.append(WaitAction(ActionTypes.WAIT, name, 20))

    name = "Move to enter - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, field_entry_pos_global_frame[0], field_entry_pos_global_frame[1], robot_field_angle))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    # name = "Wait for motor cooldown"
    # actions.append(WaitAction(ActionTypes.WAIT, name, 10))

    # if intermediate_vt:
    name = "Move to intermediate place - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, intermediate_place_pos_global_frame[0], intermediate_place_pos_global_frame[1], robot_field_angle))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    #     name = "Wait for motor cooldown"
    #     actions.append(WaitAction(ActionTypes.WAIT, name, 10))

    name = "Move to near place - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, robot_placement_coarse_pos_global_frame[0], robot_placement_coarse_pos_global_frame[1], robot_field_angle))

    name = "Start cameras"
    actions.append(Action(ActionTypes.START_CAMERAS, name))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    name = "Move to place - fine"
    actions.append(MoveAction(ActionTypes.MOVE_FINE_STOP_VISION, name, robot_placement_fine_pos_global_frame[0], robot_placement_fine_pos_global_frame[1], robot_field_angle))

    name = "Move to place - vision"
    actions.append(MoveAction(ActionTypes.MOVE_WITH_VISION, name, tile.vision_offset[0], tile.vision_offset[1], tile.vision_offset[2]))

    name = "Stop cameras"
    actions.append(Action(ActionTypes.STOP_CAMERAS, name))

    name = "Place tile"
    actions.append(Action(ActionTypes.PLACE, name))

    name = "Move away from place - relative slow"
    actions.append(MoveAction(ActionTypes.MOVE_REL_SLOW, name, relative_tile_offset[0], relative_tile_offset[1], 0))

    # if intermediate_vt:
    #     name = "Move to intermediate place - coarse"
    #     actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, intermediate_place_pos_global_frame[0], intermediate_place_pos_global_frame[1], robot_field_angle))

    #     name = "Wait for motor cooldown"
    #     actions.append(WaitAction(ActionTypes.WAIT, name, 10))

    name = "Move to exit - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, field_entry_pos_global_frame[0], field_entry_pos_global_frame[1], cfg.highway_angle))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    # name = "Wait for motor cooldown"
    # actions.append(WaitAction(ActionTypes.WAIT, name, 10))

    # if intermediate_hz:
    #     name = "Move to intermediate exit - coarse"
    #     actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, intermediate_entry_pos_global_frame[0], intermediate_entry_pos_global_frame[1], cfg.highway_angle))

    #     name = "Wait for motor cooldown (long)"
    #     actions.append(WaitAction(ActionTypes.WAIT, name, 20))

    return actions


def generate_small_testing_action_sequence(cfg, tile):
    """
    Short sequence for testing
    Load
    Move to near place - coarse
    Wait for localization
    Move to place - fine
    Place
    Move away from place - fine
    Move to load - coarse
    """

    # Setup positions in field frame
    tile_pos_in_field_frame = np.array(tile.getPlacementPositionInMeters())
    robot_placement_fine_pos_field_frame = tile_pos_in_field_frame + Utils.TransformPos(cfg.tile_to_robot_offset, [0,0], cfg.field_to_robot_frame_angle)
    robot_placement_coarse_pos_field_frame = robot_placement_fine_pos_field_frame + Utils.TransformPos(cfg.tile_placement_coarse_offset, [0,0], cfg.field_to_robot_frame_angle)
    enter_field_prep_pos_field_frame = [robot_placement_coarse_pos_field_frame[0], - cfg.prep_position_distance]
    exit_field_prep_pos_field_frame = [-cfg.exit_position_distance, robot_placement_coarse_pos_field_frame[1]]

    # Convert positions to global frame
    tile_pos_in_global_frame = Utils.TransformPos(tile_pos_in_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_placement_coarse_pos_global_frame = Utils.TransformPos(robot_placement_coarse_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_placement_fine_pos_global_frame = Utils.TransformPos(robot_placement_fine_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_field_angle = cfg.domino_field_angle + cfg.field_to_robot_frame_angle
    enter_field_prep_global_frame = Utils.TransformPos(enter_field_prep_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    exit_field_prep_global_frame = Utils.TransformPos(exit_field_prep_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)

    # print(tile.order)
    # print(tile.coordinate)
    # print(tile_pos_in_field_frame)
    # print(tile_pos_in_global_frame)
    # print(robot_placement_coarse_pos_global_frame)
    # print(robot_placement_fine_pos_global_frame)

    actions = []

    name = "Load tile"
    actions.append(Action(ActionTypes.LOAD, name))

    name = "Move to prep - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, enter_field_prep_global_frame[0], enter_field_prep_global_frame[1], robot_field_angle))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    name = "Move to near place - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, robot_placement_coarse_pos_global_frame[0], robot_placement_coarse_pos_global_frame[1], robot_field_angle))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    name = "Start cameras"
    actions.append(Action(ActionTypes.START_CAMERAS, name))

    name = "Move to place - fine"
    actions.append(MoveAction(ActionTypes.MOVE_FINE, name, robot_placement_fine_pos_global_frame[0], robot_placement_fine_pos_global_frame[1], robot_field_angle))

    name = "Move to place - vision"
    actions.append(MoveAction(ActionTypes.MOVE_WITH_VISION, name, tile.vision_offset[0], tile.vision_offset[1], tile.vision_offset[2]))

    name = "Stop cameras"
    actions.append(Action(ActionTypes.STOP_CAMERAS, name))

    name = "Place tile"
    actions.append(Action(ActionTypes.PLACE, name))

    name = "Move away from place - fine"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, robot_placement_coarse_pos_global_frame[0], robot_placement_coarse_pos_global_frame[1], robot_field_angle))

    name = "Move to exit - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, enter_field_prep_global_frame[0], enter_field_prep_global_frame[1], robot_field_angle))

    name = "Move to near load - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, cfg.load_pose[0], cfg.load_pose[1], cfg.load_pose[2]))

    return actions

def draw_env(cfg):
    """
    Draws a figure of the environment
    """

    fig,ax = plt.subplots(1)

    # Draw overall map boundaries
    ax.add_patch(patches.Rectangle(cfg.robot_boundaries[0],
                                    cfg.robot_boundaries[1][0] - cfg.robot_boundaries[0][0],
                                    cfg.robot_boundaries[1][1] - cfg.robot_boundaries[0][1],
                                    fill=False,
                                    edgecolor='b'))

    # Draw field boundaries
    ax.add_patch(patches.Rectangle(cfg.domino_field_origin,
                                    cfg.domino_field_top_right[0] - cfg.domino_field_origin[0],
                                    cfg.domino_field_top_right[1] - cfg.domino_field_origin[1],
                                    fill=False,
                                    edgecolor='r'))

    # Draw base station
    ax.add_patch(patches.Rectangle(cfg.base_station_boundaries[0],
                                    cfg.base_station_boundaries[1][0] - cfg.base_station_boundaries[0][0],
                                    cfg.base_station_boundaries[1][1] - cfg.base_station_boundaries[0][1],
                                    fill=True,
                                    edgecolor='k',
                                    facecolor='k'))


    ax.set_xlim(left=-1, right=cfg.robot_boundaries[1][0]+1)
    ax.set_ylim(bottom=-1, top=cfg.robot_boundaries[1][1]+1)
    ax.axis('equal')

    return ax


class Cycle:

    def __init__(self, id, cfg, robot_id, tile, action_sequence):
        self.id = id
        self.cfg = cfg
        self.robot_id = robot_id
        self.tile = tile
        self.action_sequence = action_sequence
        

    def draw_cycle(self, ax):
        for action in self.action_sequence:
            action.draw(ax)

    def draw_action(self, ax, idx, text=""):
        self.action_sequence[idx].draw(ax, text)


def generate_standard_cycles(cfg, field, cycle_generator_fn):
    start_num = 1
    robot_num = start_num
    n_robots = len(cfg.ip_map)
    cycles = []

    for tile in field.tiles:
        action_sequence = cycle_generator_fn(cfg, tile)
        cycles.append(Cycle(tile.order, cfg, "robot{}".format(robot_num), tile, action_sequence))
        robot_num += 1
        if robot_num > n_robots:
            robot_num = start_num

    return cycles


class BasePlan:

    def __init__(self, cycles):
        self.cycles = cycles

    def get_cycle(self, cycle_num):
        try:
            return self.cycles[cycle_num]
        except IndexError:
            return None

    def get_action(self, cycle_id, action_id):
        cycle = self.get_cycle(cycle_id)
        if cycle:
            try:
                return cycle.action_sequence[action_id]
            except IndexError:
                return None
        else:
            return None

class RealPlan(BasePlan):

    def __init__(self, cfg, field, cycles):
        self.cfg = cfg
        self.field = field
        super().__init__(cycles)

    def draw_cycle(self, cycle_num):
        ax = draw_env(self.cfg)
        self.get_cycle(cycle_num).draw_cycle(ax)
        plt.show()

    def find_pose_move_idx(self, cycle):
        # Figure out what id the tile pose is
        place_idx = -1
        for i,action in enumerate(cycle.action_sequence):
            if action.action_type == ActionTypes.PLACE:
                place_idx = i
                break
        if place_idx == -1:
            raise ValueError("Couldn't find placement index")
        tile_pose_move_idx = -1
        for j in range(place_idx, 0, -1):
            action = cycle.action_sequence[j]
            if action.action_type == ActionTypes.MOVE_FINE or action.action_type == ActionTypes.MOVE_COARSE :
                tile_pose_move_idx = j
                break
        if tile_pose_move_idx == -1:
            raise ValueError("Couldn't find movement index")

        return tile_pose_move_idx
            
    def draw_all_tile_poses(self):
        ax = draw_env(self.cfg)
        for cycle in self.cycles:
            tile_pose_move_idx = self.find_pose_move_idx(cycle)
            cycle.draw_action(ax, tile_pose_move_idx, text=cycle.tile.order)
        plt.show()

class Plan(RealPlan):

    def __init__(self, cfg, cycle_generator_fn):
        field = DominoField(cfg)
        field.generate()
        logging.info('Generating robot actions...')
        cycles = generate_standard_cycles(cfg, field, cycle_generator_fn)
        logging.info('done.')
        super().__init__(cfg, field, cycles)


class SubsectionPlan(RealPlan):

    def __init__(self, full_plan):
        self.full_plan = full_plan
        self.start_coords = full_plan.cfg.start_coords
        self.end_coords = full_plan.cfg.end_coords
        self.delta_coords = (self.end_coords[0] - self.start_coords[0],
                             self.end_coords[1] - self.start_coords[1] )
        new_field = DominoField(full_plan.cfg)
        new_field.n_tiles_x = self.delta_coords[0]
        new_field.n_tiles_y = self.delta_coords[1]
        counter = 0
        new_field.tiles = []
        new_cycles = []
        for i in range(len(full_plan.field.tiles)):
            tile = full_plan.field.tiles[i]
            cycle = full_plan.cycles[i]
            tile_coords = tile.coordinate
            if tile_coords[0] >= self.start_coords[0] and \
               tile_coords[0] <= self.end_coords[0] and \
               tile_coords[1] >= self.start_coords[1] and \
               tile_coords[1] <= self.end_coords[1]:
                # Make new tile
                new_tile = copy.deepcopy(tile)
                new_tile.order = counter
                new_field.tiles.append(new_tile)
                # Make new cycle
                new_cycle = copy.deepcopy(cycle)
                new_cycle.id = counter
                new_cycle.tile = new_tile
                new_cycles.append(new_cycle)
                counter += 1

        super().__init__(self.full_plan.cfg, new_field, new_cycles)



class TestPlan(BasePlan):
    """
    Test plan used for debugging and testing various action sequences
    """

    def __init__(self):
        actions = []
        actions.append(MoveAction(ActionTypes.MOVE_COARSE, "TestMoveCoarse", 0.5, 0.5, 0))
        actions.append(MoveAction(ActionTypes.MOVE_FINE, 'TestMoveFine', 1,1,0))
        actions.append(MoveAction(ActionTypes.MOVE_COARSE, 'Blah', 0,0,3.14))

        cycles = [Cycle(i,None,'robot1','TestCycle{}'.format(i), actions) for i in range(3)]
        super().__init__(cycles)


def RunFieldPlanning(autosave=False):
    cfg = config.Config()

    logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(os.path.join(cfg.log_folder,"planner.log")),
        logging.StreamHandler()
        ]
    )

    plan = None
    if cfg.USE_SMALL_TESTING_CONFIG:
        plan = Plan(cfg, generate_small_testing_action_sequence)
    else:
        plan = Plan(cfg, generate_full_action_sequence)

    if cfg.USE_SUBSECTION:
        plan = SubsectionPlan(plan)

    if autosave:
        fname = os.path.join(cfg.plans_dir,"autosaved.p")
        with open(fname, 'wb') as f:
            pickle.dump(plan, f)
            logging.info("Saved plan to {}".format(fname))

    return plan
    

def GeneratePDF(plan):

    from reportlab.lib.pagesizes import letter
    from reportlab.pdfgen import canvas
    from PIL import Image

    logging.info("Generating PDF")

    # Initialize pdf
    name = "domino_plan.pdf"
    if plan.cfg.MR_LOGO_PLAN:
        name = "domnino_plan_logo.pdf"
    full_path = os.path.join(plan.cfg.plans_dir, name)
    page_height, page_width = letter # Flipped to get landscape
    c = canvas.Canvas(full_path, pagesize=(page_width, page_height))

    # Pre allocate image array
    tile_size_x_in_px = (plan.cfg.domino_width_px + plan.cfg.domino_spacing_width_px) * plan.cfg.tile_width
    tile_size_y_in_px = (plan.cfg.domino_height_px + plan.cfg.domino_spacing_height_px) * plan.cfg.tile_height

    for i in range(len(plan.field.tiles)):
        # Get tile to draw on this page
        tile = plan.field.tiles[i]

        # Draw title
        text_width = 0.5 * page_width
        text_height = 0.9 * page_height
        text = "Tile {}, Coordinate: ({}, {})".format(i, tile.coordinate[0], tile.coordinate[1])
        c.setFont("Helvetica", 20)
        c.drawCentredString(text_width,text_height,text)

        # Draw orientation note
        text_width = 0.5 * page_width
        text_height = 0.1 * page_height
        text = "This side towards robot body"
        c.drawCentredString(text_width,text_height,text)

        # Draw image
        image_array = np.zeros((tile_size_x_in_px, tile_size_y_in_px, 3))
        tile.draw_single(image_array, 0, 0, tile_size_x_in_px, tile_size_y_in_px, draw_edge=False)
        image_array = np.transpose(image_array, (1, 0, 2))
        image_array = np.flip(image_array, 0)
        im = Image.fromarray(np.uint8(255*image_array), mode='RGB')
        image_fraction = 0.7
        start_width = (1-image_fraction)/2.0 * page_width
        start_height = (1-image_fraction)/2.0 * page_height
        image_width = image_fraction * page_width
        image_height = image_fraction * page_height
        c.drawInlineImage(im, start_width ,  start_height, width=image_width, height=image_height) 

        # Complete page
        c.showPage()

    c.save()




if __name__ == '__main__':

    import PySimpleGUI as sg

    plan = RunFieldPlanning(autosave=False)

    # plan.field.printStats()
    # plan.field.show_image_parsing()
    # plan.field.render_domino_image_tiles()
    # plan.field.show_tile_ordering()
    plan.draw_cycle(7)
    # plan.draw_cycle(17)
    # plan.draw_cycle(18)
    # plan.draw_all_tile_poses()

    # GeneratePDF(plan)

    # sg.change_look_and_feel('Dark Blue 3')
    # clicked_value = sg.popup_yes_no('Save plan to file?')
    # if clicked_value == "Yes":
    #     fname = sg.popup_get_file("Location to save", save_as=True)
    #     with open(fname, 'wb') as f:
    #         pickle.dump(plan, f)
    #         logging.info("Saved plan to {}".format(fname))

