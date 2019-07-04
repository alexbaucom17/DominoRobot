import DominoField as df
import config as cfg

print('Generating domino field from image...',end='',flush=True)
field = df.generateField(cfg)
print('done.')

print('Generating tiles from field...',end='',flush=True)
tiles = field.generateTiles()
print('done.')

print('Generating tile build instructions...',end='',flush=True)
#build_instructions = tiles.generateBuildInstructions()
print('done.')

print('Generating robot paths and placement instructions...',end='',flush=True)
#placement_instructions = tiles.generatePlacementInstructions()
print('done.')

print('Generating output diagrams:')
field.printStats()
#field.show()
