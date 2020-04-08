import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import skimage.transform as sktf
import numpy as np
import matplotlib.patches as patches
import math
import os
import pickle


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

        print('Generating domino field from image...',end='',flush=True)
        self._generateField()
        print('done.')

        print('Generating tiles from field...',end='',flush=True)
        self._generateTiles()
        print('done.')


    def printStats(self):
        # Output some metrics
        print('Domino usage:')
        print('Total number of dominos: ' + str(self.img_parsed_ids.size))
        print('Colors:')
        unique_colors, counts = np.unique(self.img_parsed_ids, return_counts=True)
        for i, id in enumerate(unique_colors):
            print('  ' + self.cfg.dominos[id][0] + ': ' + str(counts[i]))

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
        img_scaled = sktf.resize(img, (self.cfg.desired_height, self.cfg.desired_width), anti_aliasing=False)

        # Parse image into domino IDs by choosing 'closest' color
        img_parsed_color = np.zeros((self.cfg.desired_width * self.cfg.desired_height, 3))
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

        img_parsed_ids = img_parsed_ids.reshape((self.cfg.desired_height, self.cfg.desired_width))
        img_parsed_color = img_parsed_color.reshape((self.cfg.desired_height, self.cfg.desired_width, 3))

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
        if self.cfg.desired_height % self.cfg.tile_height != 0:
            raise ValueError('Field height is not evenly divisible by tile height!')
        if self.cfg.desired_width % self.cfg.tile_width != 0:
            raise ValueError('Field width is not evenly divisible by tile width!')

        # Determine number of tiles needed in x and y
        n_tiles_x = int(self.cfg.desired_width / self.cfg.tile_width)
        n_tiles_y = int(self.cfg.desired_height / self.cfg.tile_height)

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

        # Draw dominos
        for i in range(self.cfg.tile_width):
            domino_start_x = tile_start_x_px + self.cfg.domino_spacing_x_px + (self.cfg.domino_width_px + self.cfg.domino_spacing_x_px) * i
            domino_end_x = domino_start_x + self.cfg.domino_width_px
            for j in range(self.cfg.tile_height):
                domino_start_y = tile_start_y_px + self.cfg.domino_spacing_y_px + (self.cfg.domino_height_px + self.cfg.domino_spacing_y_px) * j
                domino_end_y = domino_start_y + self.cfg.domino_height_px
                domino_id = self.values[j, i]
                domino_color = self.cfg.dominos[domino_id][1]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 0] = domino_color[0]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 1] = domino_color[1]
                array[domino_start_x:domino_end_x, domino_start_y:domino_end_y, 2] = domino_color[2]



class Action:

    TYPE_MOVE_COARSE = 1
    TYPE_MOVE_FINE = 2
    TYPE_PICKUP = 3
    TYPE_PLACE = 4

    def __init__(self, action_type, name):
        self.action_type = action_type
        self.name = name

    def draw(self, ax):
        pass


class MoveAction(Action):

    def __init__(self, action_type, name, x, y, a):
        # action_type (enum)
        # string name
        # X position [m]py
        # Y position [m]
        # Angle [deg]

        super().__init__(action_type, name)

        self.x = float(x)
        self.y = float(y)
        self.a = float(a)

    def getPos(self):
        return np.array([self.x, self.y])

    def getAngleDegrees(self):
        return self.a

    def getAngleRadians(self):
        return self.a * math.pi/180.0

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


def generate_action_sequence(cfg, tile):
    """
    Standard sequence:
    - Move to pickup
    - Do pickup
    - Move out of pickup
    - Move to field entry
    - Move to coarse drop off
    - Move to fine drop off
    - Drop off
    - Move to coarse drop off
    - Move to field exit
    - Move to near pickup
    """

    # Setup positions
    tile_pos_in_field_frame = np.array(tile.getPlacementPositionInMeters())
    tile_pos_in_global_frame = tile_pos_in_field_frame + cfg.domino_field_origin
    robot_placement_fine_pose = tile_pos_in_global_frame + cfg.frame_to_robot_offset #TODO: make sure this work, might need to be a transform
    robot_placement_coarse_pose = robot_placement_fine_pose - cfg.tile_placement_coarse_offset
    base_station_coarse_pos = cfg.base_station_target_pose + cfg.base_station_coarse_pose_offset

    actions = []

    name = "Move to pickup - fine"
    actions.append(MoveAction(Action.TYPE_MOVE_FINE, name, cfg.base_station_target_pose[0], cfg.base_station_target_pose[1], cfg.domino_field_angle))

    name = "Pickup tile"
    actions.append(Action(Action.TYPE_PICKUP, name))

    name = "Move away from pickup - fine"
    actions.append(MoveAction(Action.TYPE_MOVE_FINE, name, base_station_coarse_pos[0], base_station_coarse_pos[1], cfg.domino_field_angle))

    name = "Move to prep - coarse"
    prep_x = robot_placement_coarse_pose[0]
    prep_y = cfg.domino_field_origin[1] - cfg.prep_position_distance
    actions.append(MoveAction(Action.TYPE_MOVE_COARSE, name, prep_x, prep_y, cfg.domino_field_angle))

    name = "Move to near place - coarse"
    actions.append(MoveAction(Action.TYPE_MOVE_COARSE, name, robot_placement_coarse_pose[0], robot_placement_coarse_pose[1], cfg.domino_field_angle))

    name = "Move to place - fine"
    actions.append(MoveAction(Action.TYPE_MOVE_FINE, name, robot_placement_fine_pose[0], robot_placement_fine_pose[1], cfg.domino_field_angle))

    name = "Place tile"
    actions.append(Action(Action.TYPE_PLACE, name))

    name = "Move away from place - fine"
    actions.append(MoveAction(Action.TYPE_MOVE_FINE, name, robot_placement_coarse_pose[0], robot_placement_coarse_pose[1], cfg.domino_field_angle))

    name = "Move to exit - coarse"
    exit_x = cfg.domino_field_origin[0] - cfg.exit_position_distance
    exit_y = robot_placement_coarse_pose[1]
    actions.append(MoveAction(Action.TYPE_MOVE_COARSE, name, exit_x, exit_y, cfg.domino_field_angle))

    name = "Move to near pickup - coarse"
    actions.append(MoveAction(Action.TYPE_MOVE_COARSE, name, base_station_coarse_pos[0], base_station_coarse_pos[1], cfg.domino_field_angle))

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

    def __init__(self, id, cfg, robot_id, tile):
        self.id = id
        self.cfg = cfg
        self.robot_id = robot_id
        self.tile = tile
        
        self.action_sequence = generate_action_sequence(cfg, tile)
        

    def draw_cycle(self, ax):
        for action in self.action_sequence:
            action.draw(ax)



class Plan:

    def __init__(self, cfg):
        self.cfg = cfg
        self.field = DominoField(cfg)
        self.cycles = []

        print('Generating robot actions...',end='',flush=True)
        self._generate_all_cycles()
        print('done.')

    def _generate_all_cycles(self):
        start_num = 1
        robot_num = start_num
        n_robots = len(self.cfg.ip_map)

        for tile in self.field.tiles:
            self.cycles.append(Cycle(tile.order, cfg, robot_num, tile))
            robot_num += 1
            if robot_num > n_robots:
                robot_num = start_num

    def draw_cycle(self, cycle_num):
        ax = draw_env(self.cfg)
        self.cycles[cycle_num].draw_cycle(ax)
        plt.show()


class PlanManager:

    def __init__(self, cfg):
        self.plan = None
        self.cfg = cfg
        self.cycle_num = 0
        # TODO: Fix up the runtime piece of this - maybe move out of this class? This is a bit of a mess
        self.progress_tracker = {n: {'action': None, 'cycle': None, 'step': 0} for n in self.cfg.ip_map.keys()}

    def get_plan(self):

        # If we don't already have a plan, load it or generate it
        if not self.plan:
            if os.path.exists(self.cfg.plan_file):
                with open(self.cfg.plan_file, 'rb') as f:
                    self.plan = pickle.load(f)
                    print("Loaded plan from {}".format(self.cfg.plan_file))
            else:
                self.plan = Plan(self.cfg)
                with open(self.cfg.plan_file, 'wb') as f:
                    pickle.dump(self.plan, f)
                    print("Saved plan to {}".format(self.cfg.plan_file))

        return self.plan

    def update_progress(self, progress_metrics):
        for key, val in progress_metrics:
            if key == 'pos' or key == 'base':
                continue
            if not val['in_progress'] and self.progress_tracker[key][action] is not None:
                # TODO: Make sure this doesn't create a race condition with starting the next action
                self.progress_tracker[key][action] = None

    def check_for_next_cycle(self):

        # Check if a robot is ready for the next cycle
        next_cycle = self.plan.cycles[self.cycle_num]
        next_robot = next_cycle.robot_id
        if self.progress_tracker[next_robot]['action'] == None and
           self.progress_tracker[next_robot]['cycle'] == None:

            self.progress_tracker[next_robot]['cycle'] = next_cycle
            self.cycle_num += 1

    def get_command(self):
        # Loop over all robots
        for robot, data in self.progress_tracker.items():
            # Check for a robot that isn't running an action
            if data['cycle'] and data['action'] == None:
                data['step'] += 1
                # Check if there is a new action to run, if not, end the cycle
                if data['step'] > len(data['cycle'].action_sequence):
                    data['cycle'] = None
                    data['step'] = 0
                else:
                    # If there is a new action to run, start it
                    next_action = data['cycle'].action_sequence[data['step']]
                    data['action'] = next_action
                    return next_action

        return None


if __name__ == '__main__':

    import config
    cfg = config.Config()
    pm = PlanManager(cfg)

    plan = pm.get_plan()

    plan.field.printStats()
    plan.field.show_image_parsing()
    plan.field.render_domino_image_tiles()
    plan.field.show_tile_ordering()
    plan.draw_cycle(5)

