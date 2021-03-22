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

        logging.info('Generating domino field from image...')
        self._generateField()
        logging.info('done.')

        logging.info('Generating tiles from field...')
        self._generateTiles()
        logging.info('done.')


    def printStats(self):
        # Output some metrics
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

    def _generateField(self):
        # Load original image
        img = mpimg.imread(self.cfg.image_name)

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

    def _addTile(self, tile_coordinate, tile_values, tile_order):

        new_tile = Tile(self.cfg, tile_coordinate, tile_values, tile_order)
        self.tiles.append(new_tile)

        if tile_coordinate[0] > self.n_tiles_x:
            self.n_tiles_x = tile_coordinate[0] + 1
        if tile_coordinate[1] > self.n_tiles_y:
            self.n_tiles_y = tile_coordinate[1] + 1

    def _generateTiles(self):

        # Check sizes and make sure things line up
        if self.cfg.desired_height_dominos % self.cfg.tile_height != 0:
            raise ValueError('Field height is not evenly divisible by tile height!')
        if self.cfg.desired_width_dominos % self.cfg.tile_width != 0:
            raise ValueError('Field width is not evenly divisible by tile width!')

        # Determine number of tiles needed in x and y
        n_tiles_x = int(self.cfg.desired_width_dominos / self.cfg.tile_width)
        n_tiles_y = int(self.cfg.desired_height_dominos / self.cfg.tile_height)

        order_map = self._generateTileOrdering(n_tiles_x, n_tiles_y)

        # Loop over tiles and assign id and colors to each
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
                self._addTile(tile_coord, tile_values, tile_order)

        # Sort tile array so they are in order
        self.tiles = sorted(self.tiles, key = lambda tile: tile.order)

    @classmethod
    def _generateTileOrdering(cls, num_x, num_y):
        """
        Generates and ordering that maps x y tile coordinate -> order number
        """

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

    def __init__(self, cfg, coordinate, values, order):
        self.coordinate = coordinate
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


class ActionTypes(enum.Enum):
    MOVE_COARSE = 0,
    MOVE_FINE = 1,
    MOVE_REL = 2,
    NET = 3,
    LOAD = 4,
    PLACE = 5,
    TRAY_INIT = 6, 
    LOAD_COMPLETE = 7,
    ESTOP = 8,
    WAIT_FOR_LOCALIZATION = 9, 
    MOVE_CONST_VEL = 10,
    CLEAR_ERROR = 11,
    NONE = 12,
    SET_POSE = 13,

class Action:

    def __init__(self, action_type, name):
        self.action_type = action_type
        self.name = name

    def draw(self, ax):
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
        text_point = points[0]
        ax.annotate(self.name, xy=text_point[:2], xytext=[1, 1], textcoords="offset points", fontsize=8, color="green")


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
    robot_placement_fine_pos_field_frame = tile_pos_in_field_frame + Utils.TransformPos(cfg.tile_to_robot_offset, [0,0], cfg.field_to_robot_frame_angle)
    robot_placement_coarse_pos_field_frame = robot_placement_fine_pos_field_frame + Utils.TransformPos(cfg.tile_placement_coarse_offset, [0,0], cfg.field_to_robot_frame_angle)
    enter_field_prep_pos_field_frame = [robot_placement_coarse_pos_field_frame[0], - cfg.prep_position_distance]
    exit_field_prep_pos_field_frame = [-cfg.exit_position_distance, robot_placement_coarse_pos_field_frame[1]]

    # Convert positions to global frame
    robot_placement_coarse_pos_global_frame = Utils.TransformPos(robot_placement_coarse_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_placement_fine_pos_global_frame = Utils.TransformPos(robot_placement_fine_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    enter_field_prep_global_frame = Utils.TransformPos(enter_field_prep_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    exit_field_prep_global_frame = Utils.TransformPos(exit_field_prep_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    base_station_coarse_pos = cfg.base_station_target_pos + Utils.TransformPos(cfg.base_station_coarse_pose_offset, [0,0], cfg.base_station_target_angle)
    robot_field_angle = cfg.domino_field_angle + cfg.field_to_robot_frame_angle

    actions = []

    name = "Move to load - fine"
    actions.append(MoveAction(ActionTypes.MOVE_FINE, name, cfg.base_station_target_pos[0], cfg.base_station_target_pos[1], cfg.base_station_target_angle))

    name = "Load tile"
    actions.append(Action(ActionTypes.LOAD, name))

    name = "Move away from load - fine"
    actions.append(MoveAction(ActionTypes.MOVE_FINE, name, base_station_coarse_pos[0], base_station_coarse_pos[1], cfg.base_station_target_angle))

    name = "Move to prep - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, enter_field_prep_global_frame[0], enter_field_prep_global_frame[1], robot_field_angle))

    name = "Move to near place - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, robot_placement_coarse_pos_global_frame[0], robot_placement_coarse_pos_global_frame[1], robot_field_angle))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    name = "Move to place - fine"
    actions.append(MoveAction(ActionTypes.MOVE_FINE, name, robot_placement_fine_pos_global_frame[0], robot_placement_fine_pos_global_frame[1], robot_field_angle))

    name = "Place tile"
    actions.append(Action(ActionTypes.PLACE, name))

    name = "Move away from place - fine"
    actions.append(MoveAction(ActionTypes.MOVE_FINE, name, robot_placement_coarse_pos_global_frame[0], robot_placement_coarse_pos_global_frame[1], robot_field_angle))

    name = "Move to exit - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, exit_field_prep_global_frame[0], exit_field_prep_global_frame[1], robot_field_angle))

    name = "Move to near load - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, base_station_coarse_pos[0], base_station_coarse_pos[1], cfg.base_station_target_angle))

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

    # Convert positions to global frame
    tile_pos_in_global_frame = Utils.TransformPos(tile_pos_in_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_placement_coarse_pos_global_frame = Utils.TransformPos(robot_placement_coarse_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_placement_fine_pos_global_frame = Utils.TransformPos(robot_placement_fine_pos_field_frame, cfg.domino_field_origin, cfg.domino_field_angle)
    robot_field_angle = cfg.domino_field_angle + cfg.field_to_robot_frame_angle

    print(tile.order)
    print(tile_pos_in_field_frame)
    print(tile_pos_in_global_frame)
    print(robot_placement_coarse_pos_global_frame)
    print(robot_placement_fine_pos_global_frame)

    actions = []

    name = "Load tile"
    actions.append(Action(ActionTypes.LOAD, name))

    name = "Move to near place - coarse"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, robot_placement_coarse_pos_global_frame[0], robot_placement_coarse_pos_global_frame[1], robot_field_angle))

    name = "Wait for localization"
    actions.append(Action(ActionTypes.WAIT_FOR_LOCALIZATION, name))

    name = "Move to place - fine"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, robot_placement_fine_pos_global_frame[0], robot_placement_fine_pos_global_frame[1], robot_field_angle))

    name = "Move to place - super fine"
    actions.append(MoveConstVelAction(ActionTypes.MOVE_CONST_VEL, name, 0.85, 0, 0, 0))

    name = "Place tile"
    actions.append(Action(ActionTypes.PLACE, name))

    name = "Move away from place - fine"
    actions.append(MoveAction(ActionTypes.MOVE_COARSE, name, robot_placement_coarse_pos_global_frame[0], robot_placement_coarse_pos_global_frame[1], robot_field_angle))

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
                                    cfg.field_width,
                                    cfg.field_height,
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

class Plan(BasePlan):

    def __init__(self, cfg, cycle_generator_fn):
        self.cfg = cfg
        self.field = DominoField(cfg)

        logging.info('Generating robot actions...')
        cycles = generate_standard_cycles(self.cfg, self.field, cycle_generator_fn)
        logging.info('done.')
        super().__init__(cycles)

    def draw_cycle(self, cycle_num):
        ax = draw_env(self.cfg)
        self.get_cycle(cycle_num).draw_cycle(ax)
        plt.show()


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



if __name__ == '__main__':

    import PySimpleGUI as sg
    import config
    import pickle
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

    # plan.field.printStats()
    # plan.field.show_image_parsing()
    # plan.field.render_domino_image_tiles()
    # plan.field.show_tile_ordering()
    plan.draw_cycle(2)


    sg.change_look_and_feel('Dark Blue 3')
    clicked_value = sg.popup_yes_no('Save plan to file?')
    if clicked_value == "Yes":
        fname = sg.popup_get_file("Location to save", save_as=True)
        with open(fname, 'wb') as f:
            pickle.dump(plan, f)
            logging.info("Saved plan to {}".format(fname))

