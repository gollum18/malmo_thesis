import random


class Cell:

    OPEN = 10000
    BLOCKED = 10001
    AGENT = 10002
    GOAL = 10003
    TRANSITION_UPPER = 10004
    TRANSITION_LOWER = 10005

    def __init__(self, x, y, z, state):
        """
        Creates a cell in a 2D Area.
        :param x: The x coordinate of this cell.
        :param y: The y coordinate of this cell.
        :param z: The z coordinate of this cell.
        :param state: The state of this cell.
        """
        self.x = x
        self.y = y
        self.z = z
        self.state = state
        self.parent = None

    def __str__(self):
        if self.state == Cell.OPEN:
            return "-"
        elif self.state == Cell.BLOCKED:
            return "B"
        elif self.state == Cell.AGENT:
            return "A"
        elif self.state == Cell.GOAL:
            return "G"
        elif self.state == Cell.TRANSITION_UPPER:
            return "U"
        elif self.state == Cell.TRANSITION_LOWER:
            return "D"

    def get_parent(self):
        """
        Get the parent of this cell. Used in motion planning.
        :return: The parent of this cell, usually a neighbor but not always.
        """
        return self.parent

    def get_position(self):
        """
        Get the position of this cell.
        :return: The position of this cell as a tuple. i.e. (x, y, z).
        """
        return self.x, self.y, self.z

    def get_state(self):
        """
        Gets the state of this cell.
        :return: One of four possible values: {OPEN: 0, BLOCKED: 1, AGENT: 2, GOAL: 3}
        """
        return self.state

    def set_parent(self, parent):
        """
        Sets the parent of this cell.
        :param parent: The new parent of this cell.
        :return: Nothing
        """
        self.parent = parent

    def set_state(self, state):
        """
        Sets the state of this cell.
        :param state: The new state of this cell. Must be one of these: {OPEN: 0, BLOCKED: 1, AGENT: 2, GOAL: 3}
        :return: Nothing
        """
        self.state = state

    def line_of_sight(self, cell):
        """
        Determines whether there is line of sight between two cells.
        :param cell: The cell to check line of sight against.
        :return: True if there is line of sight, false otherwise.
        """
        # TODO: Determine if this cell has line of sight to another. See CSC 290 project for implementation in C#.
        raise NotImplementedError

class Area2D:

    def __init__(self, x_dim, y_dim, z):
        self.x_dim = x_dim # The length of this area
        self.y_dim = y_dim # The width of this area
        self.z = z # The level of this area
        self.area = []
        for x in range(x_dim):
            row = []
            for y in range(y_dim):
                row.append(Cell(x, y, z, Cell.OPEN))
            self.area.append(row)

    def __str__(self):
        s = ""
        for x in range(self.x_dim):
            s += "|"
            for y in range(self.y_dim):
                s += str(self.area[x][y]) + "|"
            s += "\n"
        return s

    def get_dimensions(self):
        return self.x_dim, self.y_dim

    def get_level(self):
        return self.z

    def get_state(self, pos):
        return self.area[pos[0]][pos[1]].get_state()

    def get_parent(self, pos):
        return self.area[pos[0]][pos[1]].get_parent()

    def get_position(self, pos):
        return self.area[pos[0]][pos[1]].get_position()

    def set_parent(self, pos, parent):
        self.area[pos[0]][pos[1]].set_parent(parent)

    def set_state(self, pos, state):
        self.area[pos[0]][pos[1]].set_state(state)


class Area3D:

    def __init__(self, x_dim, y_dim, z_dim, start, goal, num_obs):
        # Set the attributes of this object
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim
        self.start = start
        self.goal = goal
        self.num_obs = num_obs

        if goal[0] < 0 or goal[0] >= x_dim or goal[1] < 0 or goal[1] >= y_dim or goal[2] < 0 or goal[2] >= z_dim:
            raise ValueError

        if start[0] < 0 or start[0] >= x_dim or start[1] < 0 or start[1] >= y_dim or start[2] < 0 or start[2] >= z_dim:
            raise ValueError

        # Setup up all of the levels with open cells
        self.levels = {}
        for z in range(z_dim):
            self.levels[z] = Area2D(x_dim, y_dim, z)

        # Set the start and goal cells
        self.levels[start[2]].set_state((start[0], start[1]), Cell.AGENT)
        self.levels[goal[2]].set_state((goal[0], goal[1]), Cell.GOAL)

        # Generate some transitions, ecah level has two transitions, one above and one below.
        # These allow us to transition between levels in the 3d structure
        for z in range(z_dim):
            if z == z_dim-1:
                continue
            x, y = random.randint(0, self.x_dim-1), random.randint(0, self.y_dim-1)
            while self.levels[z].get_state((x, y)) != Cell.OPEN and self.levels[z+1].get_state((x, y)) != Cell.OPEN:
                x, y = random.randint(0, self.x_dim-1), random.randint(0, self.y_dim-1)
            self.levels[z].set_state((x, y), Cell.TRANSITION_UPPER)
            self.levels[z+1].set_state((x, y), Cell.TRANSITION_LOWER)

        # Generate some obstacles
        for i in range(num_obs):
            x, y, z = random.randint(0, self.x_dim-1), random.randint(0, self.y_dim-1), random.randint(0, self.z_dim-1)
            while self.levels[z].get_state((x, y)) != Cell.OPEN:
                x, y, z = random.randint(0, self.x_dim-1), random.randint(0, self.y_dim-1), random.randint(0, self.z_dim-1)
            self.levels[z].set_state((x, y), Cell.BLOCKED)

    def __str__(self):
        s = ""
        for z in range(self.z_dim):
            s += "Level: {0}\n{1}\n".format(z, self.levels[z])
        return s


    def get_parent(self, pos):
        x, y, z = pos
        if x < 0 or x >= self.x_dim:
            return ValueError
        elif y < 0 or y >= self.y_dim:
            return ValueError
        elif z < 0 or z >= self.z_dim:
            return ValueError
        return self.levels[z].get_parent(pos)

    def set_parent(self, pos, parent):
        x, y, z = pos
        if x < 0 or x >= self.x_dim:
            return ValueError
        elif y < 0 or y >= self.y_dim:
            return ValueError
        elif z < 0 or z >= self.z_dim:
            return ValueError
        self.levels[z].set_parent(pos, parent)

    def get_state(self, pos):
        x, y, z = pos
        if x < 0 or x >= self.x_dim:
            return ValueError
        elif y < 0 or y >= self.y_dim:
            return ValueError
        elif z < 0 or z >= self.z_dim:
            return ValueError
        return self.levels[z].get_state(pos)

    def set_state(self, pos, state):
        x, y, z = pos
        if x < 0 or x >= self.x_dim:
            return ValueError
        elif y < 0 or y >= self.y_dim:
            return ValueError
        elif z < 0 or z >= self.z_dim:
            return ValueError
        self.levels[z].set_state(pos, state)

area = Area3D(10, 10, 3, (0, 0, 0), (5, 5, 2), 50)
print area