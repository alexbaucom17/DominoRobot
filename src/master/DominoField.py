import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import skimage.transform as sktf
import numpy as np
import matplotlib.patches as patches


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

    def __init__(self, cfg, id, coordinate, values, order):
        self.id = id
        self.coordinate = coordinate
        self.values = values
        self.cfg = cfg
        self.order = order

    def draw(self, axes):
        # Determine tile location
        tile_size_x_in_meters = (self.cfg.domino_width + self.cfg.domino_spacing_x) * self.cfg.tile_width
        tile_size_y_in_meters = (self.cfg.domino_height + self.cfg.domino_spacing_y) * self.cfg.tile_height
        tile_start_x = self.coordinate[0] * tile_size_x_in_meters
        tile_start_y = self.coordinate[1] * tile_size_y_in_meters

        # Draw tile
        axes.add_patch(patches.Rectangle((tile_start_x,tile_start_y), tile_size_x_in_meters, tile_size_y_in_meters,
                                         edgecolor=self.cfg.tile_edge_color, facecolor=self.cfg.tile_background_color,
                                         zorder=1))

        # Draw dominoes
        for i in range(self.cfg.tile_width):
            domino_start_x = tile_start_x + 0.5 * self.cfg.domino_spacing_x + (self.cfg.domino_width + self.cfg.domino_spacing_x) * i
            for j in range(self.cfg.tile_height):
                domino_start_y = tile_start_y + 0.5 * self.cfg.domino_spacing_y + (self.cfg.domino_height + self.cfg.domino_spacing_y) * j
                domino_id = self.values[j,i]
                domino_color = self.cfg.dominoes[domino_id][1]
                axes.add_patch(patches.Rectangle((domino_start_x, domino_start_y), self.cfg.domino_width, self.cfg.domino_height,
                                      color=domino_color,zorder=2))

    def draw_px(self, array):
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
        self.next_id = 0
        self.cfg = cfg
        self.n_tiles_x = 0
        self.n_tiles_y = 0

    def addTile(self, tile_coordinate, tile_values, tile_order):

        new_tile = Tile(self.cfg, self.next_id, tile_coordinate, tile_values, tile_order)
        self.tiles.append(new_tile)

        self.next_id = self.next_id + 1
        if tile_coordinate[0] > self.n_tiles_x:
            self.n_tiles_x = tile_coordinate[0] + 1
        if tile_coordinate[1] > self.n_tiles_y:
            self.n_tiles_y = tile_coordinate[1] + 1

    def draw(self):
        plt.figure()
        axes = plt.gca()
        for tile in self.tiles:
            tile.draw(axes)

        axes.axis('equal')
        axes.autoscale()
        figManager = plt.get_current_fig_manager()
        figManager.window.state('zoomed')
        plt.show()

    def draw_px(self):

        # Allocate memory for image
        tile_size_x_in_px = (self.cfg.domino_width_px + self.cfg.domino_spacing_x_px) * self.cfg.tile_width
        tile_size_y_in_px = (self.cfg.domino_height_px + self.cfg.domino_spacing_y_px) * self.cfg.tile_height
        array_size_x = tile_size_x_in_px * self.n_tiles_x
        array_size_y = tile_size_y_in_px * self.n_tiles_y
        image_array = np.zeros((array_size_x, array_size_y, 3))

        # Generate image
        for tile in self.tiles:
            tile.draw_px(image_array)

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
        #order_array = np.flip(order_array, 0)

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


