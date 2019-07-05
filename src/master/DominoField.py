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
                tile_coord = [i,j]
                tiles.addTile(tile_coord, tile_values)

        return tiles



class Tile:

    def __init__(self, cfg, id, coordinate, values):
        self.id = id
        self.coordinate = coordinate
        self.values = values
        self.cfg = cfg

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


class TileCollection:

    def __init__(self, cfg):
        self.tiles = []
        self.next_id = 0
        self.cfg = cfg

    def addTile(self, tile_coordinate, tile_values):

        new_tile = Tile(self.cfg, self.next_id, tile_coordinate, tile_values)
        self.tiles.append(new_tile)

        self.next_id = self.next_id + 1

    def draw(self):
        plt.figure()
        axes = plt.gca()
        for tile in self.tiles:
            tile.draw(axes)

        axes.axis('equal')
        axes.autoscale()
        plt.show()
