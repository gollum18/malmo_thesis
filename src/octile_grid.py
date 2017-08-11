import random

class OctileCell:

    def __init__(self, x, y, z, parent=None):
        self.pos = x, y, z # The position on the octile grid this cell represents.
        self.fscore = 0.0 # The combined g and h score of this cell.
        self.gscore = 0.0 # The cost to travel along the path so far.
        self.hscore = 0.0 # The guess of the cost remaining to the goal from this cell.
        self.parent = parent # The cell that created this cell.

    def __eq__(self, other):
        return self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1] and self.pos[2] == other.pos[2]

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.pos[0], self.pos[1], self.pos[2]))

    def __lt__(self, other):
        return self.fscore < other.fscore

    def __le__(self, other):
        return self.fscore <= other.fscore

    def __gt__(self, other):
        return self.fscore > other.fscore

    def __ge__(self, other):
        return self.fscore >= other.fscore

    def get_position(self):
        return self.pos[0], self.pos[1], self.pos[2]

    def get_x(self):
        return self.pos[0]

    def get_y(self):
        return self.pos[1]

    def get_z(self):
        return self.pos[2]

    def get_fscore(self):
        return self.fscore

    def set_fscore(self, score):
        self.fscore = score

    def get_gscore(self):
        return self.gscore

    def set_gscore(self, score):
        self.gscore = score

    def get_hscore(self):
        return self.hscore

    def set_hscore(self, score):
        self.hscore = score

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent

class OctileGrid:

    STATE_EMPTY = 0
    STATE_BLOCKED = 1
    STATE_GOAL = 2
    STATE_PROTECTED = 3
    STATE_START = 4
    STATE_PATH = 5

    Directions = {
    # Positions above and below player
    "DIR_ABOVE":[0, 0, 1],
    "DIR_BELOW":[0, 0, -1],

    # Positions surrounding player on the same level
    "DIR_NORTH":[0, -1, 0],
    "DIR_EAST":[1, 0, 0],
    "DIR_SOUTH":[0, 1, 0],
    "DIR_WEST":[-1, 0, 0],
    "DIR_SEAST":[1, 1, 0],
    "DIR_SWEST":[-1, 1, 0],
    "DIR_NEAST":[1, -1, 0],
    "DIR_NWEST":[-1, -1, 0],

    # Positions surrounding player on level above
    "DIR_ABOVE_NORTH":[0, -1, 1],
    "DIR_ABOVE_EAST":[1, 0, 0],
    "DIR_ABOVE_SOUTH":[0, 1, 1],
    "DIR_ABOVE_WEST":[-1, 0, 1],
    "DIR_ABOVE_SEAST":[1, 1, 1],
    "DIR_ABOVE_SWEST":[-1, 1, 1],
    "DIR_ABOVE_NEAST":[1, -1 ,1],
    "DIR_ABOVE_NWEST":[-1, -1, 1],

    # Positions surrounding player on level below
    "DIR_BELOW_NORTH":[0, -1, -1],
    "DIR_BELOW_EAST":[1, 0, -1],
    "DIR_BELOW_SOUTH":[0, 1, -1],
    "DIR_BELOW_WEST":[-1, 0, -1],
    "DIR_BELOW_SEAST":[1, 1, -1],
    "DIR_BELOW_SWEST":[-1, 1, -1],
    "DIR_BELOW_NEAST":[1, -1, -1],
    "DIR_BELOW_NWEST":[-1, -1, -1]
    }

    def __init__(self, x_dim, y_dim, z_dim, start=None, goal=None, percent_obs=0.30):
        if start and goal and start == goal:
            raise ValueError
        self.dimensions = x_dim, y_dim, z_dim
        self.start = start
        self.goal = goal
        self.grid_z = []

        # Generate a blank grid
        for z in range(z_dim):
            self.grid_y = []
            for y in range(y_dim):
                self.grid_x = []
                for x in range(x_dim):
                    self.grid_x.append(self.STATE_EMPTY)
                self.grid_y.append(self.grid_x)
            self.grid_z.append(self.grid_y)

        # Generate a start and goal
        if not start:
            self.start = self.rand3()
        if not goal:
            self.goal = self.rand3()
        while self.start == self.goal:
            self.start = self.rand3()

        # place the start and goal
        self.set_state(self.start[0], self.start[1], self.start[2], self.STATE_START)
        self.set_state(self.goal[0], self.goal[1], self.goal[2], self.STATE_GOAL)

        # Protect the start and goal
        self.protect_cell3(self.start[0], self.start[1], self.start[2])
        self.protect_cell3(self.goal[0], self.goal[1], self.goal[2])

        # Generate and place some obstacles
        obs = ((x_dim*y_dim*z_dim)-2)*percent_obs
        while obs > 0:
            pos = self.rand3()
            while (self.get_state(pos[0], pos[1], pos[2]) != self.STATE_EMPTY):
                pos = self.rand3()
            self.set_state(pos[0], pos[1], pos[2], self.STATE_BLOCKED)
            obs -= 1

    def __str__(self):
        s = "Printing map with metrics (x-dimension={0}, y-dimension={1}, z-dimension={2})\n".format(
            self.dimensions[0], self.dimensions[1], self.dimensions[2]
        )
        for z in range(self.dimensions[2]):
            s += "Level {0}:\n".format(z)
            for y in range(self.dimensions[1]):
                s += "|"
                for x in range(self.dimensions[0]):
                    s += "{0}|".format(self.state_string(self.get_state(x, y, z)))
                s += "\n"
            s += "\n"
        return s

    def get_goal(self):
        return self.goal

    def set_goal(self, x, y, z):
        if not self.is_valid_cell(x, y, z):
            raise ValueError
        self.goal = x, y, z

    def get_start(self):
        return self.start

    def set_start(self, x, y, z):
        if not self.is_valid_cell(x, y, z):
            raise ValueError
        self.start = x, y, z

    def get_state(self, x, y, z):
        return self.grid_z[z][y][x]

    def set_state(self, x, y, z, state):
        self.grid_z[z][y][x] = state

    def in_bounds(self, x, y, z):
        if x < 0 or x >= self.dimensions[0]:
            return False
        if y < 0 or y >= self.dimensions[1]:
            return False
        if z < 0 or z >= self.dimensions[2]:
            return False
        return True

    def generate_neighbors3(self, x, y, z):
        neighbors = []

        for direction in self.Directions.keys():
            neighbor = self.move(x, y, z, direction)
            if neighbor:
                neighbors.append(neighbor)

        return neighbors

    def is_goal_cell(self, cell):
        return cell.get_x() == self.goal[0] and cell.get_y() == self.goal[1] and cell.get_z() == self.goal[2]

    def is_valid_cell(self, x, y, z):
        return self.in_bounds(x, y, z) and self.get_state(x, y, z) != self.STATE_BLOCKED

    def move(self, x, y, z, direction):
        dx, dy, dz = self.Directions[direction]
        if self.is_valid_cell(x+dx, y+dy, z+dz):
            return x+dx, y+dy, z+dz
        return None

    def protect_cell2(self, x, y, z):
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                if self.in_bounds(x+dx, y+dy, z):
                    self.set_state(x+dx, y+dy, z, self.STATE_PROTECTED)

    def protect_cell3(self, x, y, z):
        for dz in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    if self.in_bounds(x+dx, y+dy, z+dz):
                        self.set_state(x+dx, y+dy, z+dz, self.STATE_PROTECTED)

    def rand2(self):
        return (random.randint(0, self.dimensions[0] - 1),
                random.randint(0, self.dimensions[1] - 1))

    def rand3(self):
        return (random.randint(0, self.dimensions[0]-1),
                random.randint(0, self.dimensions[1]-1),
                random.randint(0, self.dimensions[2]-1))

    def state_string(self, state):
        if state == self.STATE_EMPTY or state == self.STATE_PROTECTED:
            return "-"
        elif state == self.STATE_BLOCKED:
            return "B"
        elif state == self.STATE_START:
            return "S"
        elif state == self.STATE_GOAL:
            return "G"
        elif state == self.STATE_PATH:
            return "P"