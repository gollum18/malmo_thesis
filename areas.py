import random


class Cell:

    # The various transition types
    TRANSITION_NONE = 8000
    TRANSITION_UPPER = 8001
    TRANSITION_LOWER = 8002

    # The various movement types
    FOUR_WAY_MOVEMENT = 9001 # N, E, S, W
    EIGHT_WAY_MOVEMENT = 9002 # N, E, S, W + Intermediary Directions

    # The various cell types
    OPEN = 10000
    BLOCKED = 10001
    AGENT = 10002
    GOAL = 10003
    PATH = 10004

    def __init__(self, x, y, z, state, transition_type):
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
        self.transition_type = transition_type

    def __str__(self):
        if self.state != Cell.AGENT:
            if self.transition_type == Cell.TRANSITION_UPPER:
                return "U"
            elif self.transition_type == Cell.TRANSITION_LOWER:
                return "D"
        if self.state == Cell.OPEN:
            return "-"
        if self.state == Cell.BLOCKED:
            return "B"
        if self.state == Cell.AGENT:
            return "A"
        if self.state == Cell.GOAL:
            return "G"
        if self.state == Cell.PATH:
            return "P"

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

    def get_transition_type(self):
        return self.transition_type

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

    def set_transition_type(self, type):
        self.transition_type = type


class Area2D:

    def __init__(self, x_dim, y_dim, z, is_goal_area):
        self.x_dim = x_dim # The length of this area
        self.y_dim = y_dim # The width of this area
        self.z = z # The level of this area
        self.area = []
        self.is_goal_area = is_goal_area
        for x in range(x_dim):
            row = []
            for y in range(y_dim):
                row.append(Cell(x, y, z, Cell.OPEN, Cell.TRANSITION_NONE))
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

    def get_transition_type(self, pos):
        return self.area[pos[0]][pos[1]].get_transition_type()

    def set_parent(self, pos, parent):
        self.area[pos[0]][pos[1]].set_parent(parent)

    def set_state(self, pos, state):
        self.area[pos[0]][pos[1]].set_state(state)

    def set_transition_type(self, pos, type):
        self.area[pos[0]][pos[1]].set_transition_type(type)

    def is_goal_area(self):
        return self.is_goal_area

    def generate_neighbors_fourway(self, pos):
        neighbors = []
        if self.is_valid_cell((pos[0], pos[1]-1, pos[2])): # North
            neighbors.append((pos[0], pos[1]-1, pos[2]))
        if self.is_valid_cell((pos[0]+1, pos[1], pos[2])): # West
            neighbors.append((pos[0]+1, pos[1], pos[2]))
        if self.is_valid_cell((pos[0], pos[1]+1, pos[2])): # South
            neighbors.append((pos[0], pos[1]+1, pos[2]))
        if self.is_valid_cell((pos[0]-1, pos[1], pos[2])): # East
            neighbors.append((pos[0]-1, pos[1], pos[2]))
        if self.get_transition_type(pos) == Cell.TRANSITION_LOWER:
            neighbors.append((pos[0], pos[1], self.z - 1))
        elif self.get_transition_type(pos) == Cell.TRANSITION_UPPER:
            neighbors.append((pos[0], pos[1], self.z + 1))
        return neighbors

    def generate_neighbors_eightway(self, pos):
        neighbors = []
        for x in range(-1, 2):
            for y in range(-1, 2):
                if self.is_valid_cell((x, y)) and (x != 0 and y != 0):
                    neighbors.append((x, y, self.z))
        if self.get_transition_type(pos) == Cell.TRANSITION_LOWER:
            neighbors.append((pos[0], pos[1], self.z - 1))
        elif self.get_transition_type(pos) == Cell.TRANSITION_UPPER:
            neighbors.append((pos[0], pos[1], self.z + 1))
        return neighbors

    def is_valid_cell(self, pos):
        if pos[0] < 0 or pos[0] >= self.x_dim:
            return False
        elif pos[1] < 0 or pos[1] >= self.y_dim:
            return False
        elif self.get_state(pos) == Cell.BLOCKED:
            return False
        return True

    def line_of_sight(self, c0, c1):
        """
        Determines whether there is line of sight between two cells.
        :param c0: The left cell.
        :param c1: The right cell.
        :return: True if there is line of sight, false otherwise.
        """
        xO, yO = c0.get_coords()
        xT, yT = c1.get_coords()
        dX, dY = xT - xO, yT - yO
        sX, sY, f = 0, 0, 0

        if dY < 0:
            dY = -dY
            sY = -1
        else:
            sY = 1

        if dX < 0:
            dX = -dX
            sX = -1
        else:
            sX = 1

        if dX >= dY:
            while xO != xT:
                f = f + dY
                if f >= dX:
                    if not self.is_valid_cell((xO + ((sX - 1) / 2), yO + ((sY - 1) / 2))):
                        return False
                    yO = yO + sY
                    f = f - dX
                if (f != 0 and
                        not self.is_valid_cell((xO + ((sX - 1) / 2), yO + ((sY - 1) / 2)))):
                    return False
                if (dY == 0 and
                    not self.is_valid_cell((xO + ((sX - 1) / 2), yO)) and
                    not self.is_valid_cell((xO + ((sX - 1) / 2), yO - 1))):
                    return False
                xO = xO + sX
        else:
            while yO != yT:
                f = f + dX
                if f >= dY:
                    if not self.is_valid_cell((xO + ((sX - 1) / 2), yO + ((sY - 1) / 2))):
                        return False
                    xO = xO + sX
                    f = f - dY
                if (f != 0 and
                        not self.is_valid_cell((xO + ((sX - 1) / 2), yO + ((sY - 1) / 2)))):
                    return False
                if (dY == 0 and
                    not self.is_valid_cell((xO, yO + ((sY - 1) / 2))) and
                    not self.is_valid_cell((xO - 1, yO + ((sY - 1) / 2)))):
                    return False
                yO = yO + sY
        return True


class Area3D:

    def __init__(self, x_dim, y_dim, z_dim, start, goal, num_obs, movement_type):
        # Set the attributes of this object
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.z_dim = z_dim
        self.agent_level = start[2]
        self.agent_coords = start[0], start[1]
        self.goal = goal
        self.num_obs = num_obs
        self.movement_type = movement_type

        if goal[0] < 0 or goal[0] >= x_dim or goal[1] < 0 or goal[1] >= y_dim or goal[2] < 0 or goal[2] >= z_dim:
            raise ValueError

        if start[0] < 0 or start[0] >= x_dim or start[1] < 0 or start[1] >= y_dim or start[2] < 0 or start[2] >= z_dim:
            raise ValueError

        # Setup up all of the levels with open cells
        self.levels = {}
        for z in range(z_dim):
            self.levels[z] = Area2D(x_dim, y_dim, z, True if goal[2] == z else False)

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
            self.levels[z].set_transition_type((x, y), Cell.TRANSITION_UPPER)
            self.levels[z+1].set_transition_type((x, y), Cell.TRANSITION_LOWER)

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

    def get_agent_level(self):
        return self.agent_level

    def get_agent_coordinates(self):
        return self.agent_coords

    def get_agent_position(self):
        return self.agent_coords[0], self.agent_coords[1], self.agent_level

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

    def generate_neighbors(self, pos):
        if pos[2] >= 0 and pos[2] < self.z_dim:
            if self.movement_type == Cell.FOUR_WAY_MOVEMENT:
                return self.levels[pos[2]].generate_neighbors_fourway(pos)
            elif self.movement_type == Cell.EIGHT_WAY_MOVEMENT:
                return self.levels[pos[2]].generate_neighbors_eightway(pos)
        return None

    def is_goal_area(self, z):
        if z < 0 or z >= self.z_dim:
            return ValueError
        return self.levels[z].is_goal_area()

    def line_of_sight(self, c0, c1):
        """
        Will determine if there is line of sight between two cells on the same level.
        :param c0: The left cell.
        :param c1: The right cell.
        :return: True if there is line of sight, false if not. None if the two cells are not on the same level.
        """
        if c0.get_level() != c1.get_level():
            return None
        return self.levels[c0.get_level()].line_of_sight(c0, c1)