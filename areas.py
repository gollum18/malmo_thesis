import random


class Cell:

    EMPTY = 0
    BLOCKED = 1
    AGENT = 2
    PATH = 3
    GOAL = 4
    PROTECTED = 5
    TRANSITION_UPPER = 6
    TRANSITION_LOWER = 7
    TRANSITION_NONE = 8

    def __init__(self, x, y, z, state, transition_type):
        self.x = x
        self.y = y
        self.z = z
        self.state = state
        self.transition_type = transition_type
        self.parent = None

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __str__(self):
        if self.state != Cell.AGENT:
            if self.transition_type == Cell.TRANSITION_UPPER:
                return "U"
            elif self.transition_type == Cell.TRANSITION_LOWER:
                return "L"
        if self.state == Cell.AGENT:
            return "A"
        elif self.state == Cell.BLOCKED:
            return "B"
        elif self.state == Cell.EMPTY or self.state == Cell.PROTECTED:
            return "-"
        elif self.state == Cell.PATH:
            return "P"
        elif self.state == Cell.GOAL:
            return "G"
        elif self.state == Cell.AGENT:
            return "A"

    def get_coordinates(self):
        return self.x, self.y, self.z

    def get_state(self):
        return self.state

    def get_transition_type(self):
        return self.transition_type

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent

    def set_state(self, state):
        self.state = state

    def set_transition_type(self, type):
        self.transition_type = type


class Area3D:

    def __init__(self, x_dim, y_dim, z_dim, start, goal, obs):
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim
        self.start = start
        self.goal = goal
        self.obs = obs
        self.map = {}

        # Generate the map
        for z in range(z_dim):
            row = []
            for x in range(x_dim):
                column = []
                for y in range(y_dim):
                    column.append(Cell(x, y, z, Cell.EMPTY, None))
                row.append(column)
            self.map[z] = row

        # Place the start and goal
        self.get_cell(start[0],start[1],start[2]).set_state(Cell.AGENT)
        self.get_cell(goal[0],goal[1],goal[2]).set_state(Cell.GOAL)
        self.protect_cell(start)
        self.protect_cell(goal)

        # Place the transitions
        for z in range(self.z_dim-1):
            x, y = random.randint(0, self.x_dim-1), random.randint(0, self.y_dim-1)
            while self.get_cell(x, y, z).get_state() != Cell.EMPTY:
                x, y = random.randint(0, self.x_dim-1), random.randint(0, self.y_dim-1)
            self.get_cell(x, y, z).set_transition_type(Cell.TRANSITION_UPPER)
            self.get_cell(x, y, z+1).set_transition_type(Cell.TRANSITION_LOWER)
            self.protect_cell((x, y, z))
            self.protect_cell((x, y, z+1))

        # Place the obstacles
        for i in range(self.obs):
            x, y, z = (random.randint(0, self.x_dim-1),
                       random.randint(0, self.y_dim-1),
                       random.randint(0, self.z_dim-1))
            while self.get_cell(x, y, z).get_state() != Cell.EMPTY:
                x, y, z = (random.randint(0, self.x_dim - 1),
                           random.randint(0, self.y_dim - 1),
                           random.randint(0, self.z_dim - 1))
            self.get_cell(x, y, z).set_state(Cell.BLOCKED)

    def __str__(self):
        s = ""
        for z in range(self.z_dim):
            s += "Level {0}:\n".format(z)
            for x in range(self.x_dim):
                s += "|"
                for y in range(self.y_dim):
                    s += str(self.get_cell(x, y, z)) + "|"
                s += "\n"
            s += "\n"
        return s

    def is_in_bounds(self, x, y, z):
        if x < 0 or x >= self.x_dim:
            return False
        if y < 0 or y >= self.y_dim:
            return False
        if z < 0 or z >= self.z_dim:
            return False
        return True

    def is_valid_cell(self, x, y, z):
        if self.is_in_bounds(x, y, z) and self.get_cell(x, y, z).get_state() != Cell.BLOCKED:
            return True
        return False

    def get_cell(self, x, y, z):
        if self.is_in_bounds(x, y, z):
            return self.map[z][x][y]
        raise ValueError

    def get_dimensions(self):
        return self.x_dim, self.y_dim, self.z_dim

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def get_num_obs(self):
        return self.obs

    def protect_cell(self, pos):
        x, y, z = pos
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue
                if self.is_in_bounds(x+dx,y+dy,z) and self.get_cell(x+dx,y+dy,z).get_state() == Cell.EMPTY:
                    self.get_cell(x+dx,y+dy,z).set_state(Cell.PROTECTED)

    def update_goal(self, pos):
        self.goal = pos

area = Area3D(10, 10, 3, (0, 0, 0), (5, 5, 2), 30)
print area
