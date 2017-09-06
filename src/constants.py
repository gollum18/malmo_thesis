# Describes the missions boundaries for the agent
mission_txt = (
    # Walk to goal mission
    './missions/pp_maze_one.txt',
    # Climb to goal mission
    './missions/pp_maze_two.txt',
    # Drop to goal mission
    './missions/pp_maze_three.txt',
    # Climb the big central pillar
    './missions/pp_maze_four.txt')

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

lower_dimensions = ((-25, 55, -25), (-25, 55, -25), (-25, 54, -25), (-25, 55, -25))
upper_dimensions = ((25, 70, 25), (25, 70, 25), (25, 70, 25), (25, 70, 25))

start = (0.5, 56, 24.5)
goal = ([0.5, 56, -24.5], [0.5, 58, -24.5], [0.5, 55, -24.5], [0.5, 61, 0.5], [0.5, 56, -28.5])

obstacle = 'obs'
hazard = 'haz'
air = 'air'