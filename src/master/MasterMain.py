import Planner
import config as cfg

print('Generating domino field from image...',end='',flush=True)
field = Planner.generateField(cfg)
print('done.')

print('Generating tiles from field...',end='',flush=True)
tiles = field.generateTiles()
print('done.')

print('Generating robot paths...',end='',flush=True)
waypoints = Planner.generateWaypoints(tiles, cfg)
print('done.')

print('Generating tile build instructions...',end='',flush=True)
#build_instructions = tiles.generateBuildInstructions()
print('done.')

print('Generating output diagrams:')
field.printStats()
#field.show()
#tiles.draw()
#tiles.show_ordering()
