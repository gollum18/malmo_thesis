# Describes the missions boundaries for the agent
mission_txt = (
    # Walk to goal mission
    './missions/pp_maze_one.txt',
    # Climb to goal mission
    './missions/pp_maze_two.txt',
    # Drop to goal mission
    './missions/pp_maze_three.txt',
    # Climb the big central pillar
    './missions/pp_maze_four.txt'
)

# Describes the mission itself in a way Malmo can interpret
mission_xml = (
    # Walk to goal mission
    './missions/pp_maze_one.xml',
    # Climb to goal mission
    './missions/pp_maze_two.xml',
    # Drop to goal mission
    './missions/pp_maze_three.xml',
    # Climb the big central pillar
    './missions/pp_maze_four.xml'
)

lower = ((-25, 54, -25), (-25, 54, -25), (-25, 52, -25), (-25, 54, -25))
upper = ((25, 70, 25), (25, 70, 25), (25, 70, 25), (25, 70, 25))

start = (0.5, 56, 24.5)
goal = ((0.5, 56, -24.5), (0.5, 58, -24.5), (0.5, 54, -24.5), (0.5, 56, 0.5))

obstacle = 'obs'
hazard = 'haz'
air = 'air'

sample_ellipsoidal = 1000
sample_uniform = 1001
sample_line_to = 1002
