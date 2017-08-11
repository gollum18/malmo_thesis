import random

class OctileCell:

    def __init__(self, x, y, z, parent=None):
        """
        Creates a cell that represents a location in the search grid.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :param z: The z coordinate of the cell.
        :param parent: The parent who created this cell. It is not necessary to provide a parent on creation.
        """
        self.pos = x, y, z # The position on the octile grid this cell represents.
        self.fscore = 0.0 # The combined g and h score of this cell.
        self.gscore = 0.0 # The cost to travel along the path so far.
        self.hscore = 0.0 # The guess of the cost remaining to the goal from this cell.
        self.parent = parent # The cell that created this cell.

    def __eq__(self, other):
        """
        Determines if two cells are equal to each other.
        :param other: The other cell to compare against.
        :return: True if the cells share the same location, False otherwise.
        """
        return self.pos[0] == other.pos[0] and self.pos[1] == other.pos[1] and self.pos[2] == other.pos[2]

    def __ne__(self, other):
        """
        Determines if two cells are not equal to each other.
        :param other: The other cell to compare against.
        :return: This method passes the cell to the __eq__ method, negates the result, and returns it.
        """
        return not self.__eq__(other)

    def __hash__(self):
        """
        Creates a unique identifier for hashed data structures.
        :return: The hash is created from a tuple of the cells location. While not guaranteed to be unique, a proper
        implementation of a search agent will make this irrelevant.
        """
        return hash((self.pos[0], self.pos[1], self.pos[2]))

    def __lt__(self, other):
        """
        Determines when one of two cells is less than the others.
        :param other:  The other cell to compare against.
        :return: True if the callers f-score is less than the others.
        """
        return self.fscore < other.fscore

    def __le__(self, other):
        """
        Determines when one of two cells is less than or equal to the others.
        :param other: The other cell to compare against.
        :return: True if the callers f-score is less than or equal to the others.
        """
        return self.fscore <= other.fscore

    def __gt__(self, other):
        """
        Determines when one of two cells is greater than the other.
        :param other: The other cell to compare against.
        :return: True if the callers f-score is greater than the others.
        """
        return self.fscore > other.fscore

    def __ge__(self, other):
        """
        Determines when one of two cells is greater than or equal to the other.
        :param other: The other cell to compare against.
        :return: True if the callers f-score is greater than or equal to the others.
        """
        return self.fscore >= other.fscore

    def get_position(self):
        """
        Gets a tuple containing the x, y, and z coordinates of this cell.
        :return: Tuple (x, y, z).
        """
        return self.pos[0], self.pos[1], self.pos[2]

    def get_x(self):
        """
        Gets the x coordinate of this cell.
        :return: The x coordinate of the cell.
        """
        return self.pos[0]

    def get_y(self):
        """
        Gets the y coordinate of this cell.
        :return: The y coordinate of this cell.
        """
        return self.pos[1]

    def get_z(self):
        """
        Gets the z coordinate of this cell.
        :return: The z coordinate of this cell.
        """
        return self.pos[2]

    def get_fscore(self):
        """
        Gets the f-score of this cell.
        :return: The f-score of this cell.
        """
        return self.fscore

    def set_fscore(self, score):
        """
        Updates the f-score of this cell.
        :param score: The new f-score.
        :return: Nothing.
        """
        self.fscore = score

    def get_gscore(self):
        """
        Gets the g-score of this cell.
        :return: The g-score of this cell.
        """
        return self.gscore

    def set_gscore(self, score):
        """
        Updates the g-score of this cell.
        :param score: The new g-score of this cell.
        :return: Nothing.
        """
        self.gscore = score

    def get_hscore(self):
        """
        Gets the h-score of this cell.
        :return: The h-score of this cell.
        """
        return self.hscore

    def set_hscore(self, score):
        """
        Updates the h-score of this cell.
        :param score: The new h-score of this cell.
        :return: Nothing.
        """
        self.hscore = score

    def get_parent(self):
        """
        Gets the parent of this cell.
        :return: The parent of this cell.
        """
        return self.parent

    def set_parent(self, parent):
        """
        Updates the parent of this cell.
        :param parent: The new parent of this cell.
        :return: Nothing.
        """
        self.parent = parent


class OctileGrid:

    '''
    Represents states in the grid. A numerical constant holds less memory than a string thus we utilize numerical
    constants.
    '''
    STATE_EMPTY = 0
    STATE_BLOCKED = 1
    STATE_GOAL = 2
    STATE_PROTECTED = 3
    STATE_START = 4
    STATE_PATH = 5

    '''
    Maps directions to their dx, dy, and dz directional movement components. Used when generating neighbors.
    '''
    Directions = {
        # Positions above and below player
        "DIR_ABOVE": [0, 0, 1],
        "DIR_BELOW": [0, 0, -1],

        # Positions surrounding player on the same level
        "DIR_NORTH": [0, -1, 0],
        "DIR_EAST": [1, 0, 0],
        "DIR_SOUTH": [0, 1, 0],
        "DIR_WEST": [-1, 0, 0],
        "DIR_SOUTHEAST": [1, 1, 0],
        "DIR_SOUTHWEST": [-1, 1, 0],
        "DIR_NORTHEAST": [1, -1, 0],
        "DIR_NORTHWEST": [-1, -1, 0],

        # Positions surrounding player on level above
        "DIR_ABOVE_NORTH": [0, -1, 1],
        "DIR_ABOVE_EAST": [1, 0, 0],
        "DIR_ABOVE_SOUTH": [0, 1, 1],
        "DIR_ABOVE_WEST": [-1, 0, 1],
        "DIR_ABOVE_SOUTHEAST": [1, 1, 1],
        "DIR_ABOVE_SOUTHWEST": [-1, 1, 1],
        "DIR_ABOVE_NORTHEAST": [1, -1 ,1],
        "DIR_ABOVE_NORTHWEST": [-1, -1, 1],

        # Positions surrounding player on level below
        "DIR_BELOW_NORTH": [0, -1, -1],
        "DIR_BELOW_EAST": [1, 0, -1],
        "DIR_BELOW_SOUTH": [0, 1, -1],
        "DIR_BELOW_WEST": [-1, 0, -1],
        "DIR_BELOW_SOUTHEAST": [1, 1, -1],
        "DIR_BELOW_SOUTHWEST": [-1, 1, -1],
        "DIR_BELOW_NORTHEAST": [1, -1, -1],
        "DIR_BELOW_NORTHWEST": [-1, -1, -1]
    }

    def __init__(self, x_dim, y_dim, z_dim, start=None, goal=None, percent_obs=0.30):
        """
        Creates a three-dimensional grid world for testing three-dimensional search agents.
        :param x_dim: The maximum width(x) of the search grid.
        :param y_dim: The maximum length(y) of the search grid.
        :param z_dim: The maximum height(z) of the search grid.
        :param start: The starting position for the search agent. If one is not defined then one will be randomly
        generated for you.
        :param goal: The goal position for the search agent. If one is not defined then one will be randomly generated
        for you.
        :param percent_obs: The percent of the search grid that is unreachable by the agent. For best results try a range
        from 10% to 50% (.10 to .50). Exceeding 50% greatly increases the probability that the agent will not be able to
        find the goal.
        """
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
            while self.get_state(pos[0], pos[1], pos[2]) != self.STATE_EMPTY:
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
        """
        Gets the goal location for the search agent.
        :return: The goal location for the search agent.
        """
        return self.goal

    def set_goal(self, x, y, z):
        """
        Defines the goal location the search agent must try to reach.
        :param x: The x coordinate of the location.
        :param y: The y coordinate of the location.
        :param z: The z coordinate of the location.
        :return: Nothing.
        :raises: ValueError if the location is not valid.
        """
        if not self.is_valid_cell(x, y, z):
            raise ValueError
        self.goal = x, y, z

    def get_start(self):
        """
        Gets the starting location for the search agent.
        :return: The starting location for the search agent.
        """
        return self.start

    def set_start(self, x, y, z):
        """
        Defines the starting position for the search agent.
        :param x: The x coordinate of the location.
        :param y: The y coordinate of the location.
        :param z: The z coordinate of the location.
        :return: Nothing.
        :raises: ValueError if the starting location is not valid.
        """
        if not self.is_valid_cell(x, y, z):
            raise ValueError
        self.start = x, y, z

    def get_state(self, x, y, z):
        """
        Gets the state of the cell at the given location in the grid world.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :param z: The z coordinate of the cell.
        :return: The state of the cell at the given location if the location is valid.
        :raises: ValueError if the coordinates are not inside the grid world.
        """
        if self.in_bounds(x, y, z):
            return self.grid_z[z][y][x]
        raise ValueError

    def set_state(self, x, y, z, state):
        """
        Sets the state of the cell at the given location in the grid world.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :param z: The z coordinate of the cell.
        :param state: The state to update the cell to.
        :return: Nothing.
        :raises: ValueError if the coordinates are not inside the grid world.
        """
        if self.in_bounds(x, y, z):
            self.grid_z[z][y][x] = state
        raise ValueError

    def in_bounds(self, x, y, z):
        """
        Determines whether a location is inside the grid world.
        :param x: The x coordinate of the location.
        :param y: The y coordinate of the location.
        :param z: The z coordinate of the location.
        :return: True if the coordinate is in bounds, False otherwise.
        """
        if x < 0 or x >= self.dimensions[0]:
            return False
        if y < 0 or y >= self.dimensions[1]:
            return False
        if z < 0 or z >= self.dimensions[2]:
            return False
        return True

    def generate_neighbors3(self, x, y, z):
        """
        Will generate all adjacent neighbors to a given location in three-dimensions.
        :param x: The x coordinate of the location.
        :param y: The y coordinate of the location.
        :param z: The z coordinate of the location.
        :return: A list containing up to twenty-six valid neighbors.
        """
        neighbors = []

        for direction in self.Directions.keys():
            neighbor = self.move(x, y, z, direction)
            if neighbor:
                neighbors.append(neighbor)

        return neighbors

    def is_goal_cell(self, cell):
        """
        Determines if a given cell is a goal cell or not.
        :param cell: The cell to check goal state for.
        :return: True if the cell is the goal, False if not.
        """
        return cell.get_x() == self.goal[0] and cell.get_y() == self.goal[1] and cell.get_z() == self.goal[2]

    def is_valid_cell(self, x, y, z):
        """
        Determines whether a cell is valid or not. A cell is valid if it is within bounds and is not in a blocked
        state.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :param z: The z coordinate of the cell.
        :return: True if the cell is in bounds and not blocked, False otherwise.
        """
        return self.in_bounds(x, y, z) and self.get_state(x, y, z) != self.STATE_BLOCKED

    def move(self, x, y, z, direction):
        """
        Will return the location of an adjacent cell from the given coordinates with in the given direction.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :param z: The z coordinate of the cell.
        :param direction: The direction to move in.
        :return: The location of the adjacent cell if it is valid, None otherwise.
        """
        dx, dy, dz = self.Directions[direction]
        if self.is_valid_cell(x+dx, y+dy, z+dz):
            return x+dx, y+dy, z+dz
        return None

    def protect_cell2(self, x, y, z):
        """
        Protects cells directly adjacent to a cell that are on share the same z coordinate.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :param z: The z coordinate of the cell.
        :return: Nothing.
        """
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                if self.in_bounds(x+dx, y+dy, z):
                    self.set_state(x+dx, y+dy, z, self.STATE_PROTECTED)

    def protect_cell3(self, x, y, z):
        """
        Marks locations directly adjacent to a cell in three-dimensions.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :param z: The z coordinate of the cell.
        :return: Nothing.
        """
        for dz in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    if self.in_bounds(x+dx, y+dy, z+dz):
                        self.set_state(x+dx, y+dy, z+dz, self.STATE_PROTECTED)

    def rand2(self):
        """
        Generates a random two-dimensional point within the bounds of the grid world.
        :return: A tuple containing an x and y.
        """
        return (random.randint(0, self.dimensions[0] - 1),
                random.randint(0, self.dimensions[1] - 1))

    def rand3(self):
        """
        Generates a random three-dimensional point within the bounds of the grid world.
        :return: A tuple containing an x, y, and z.
        """
        return (random.randint(0, self.dimensions[0]-1),
                random.randint(0, self.dimensions[1]-1),
                random.randint(0, self.dimensions[2]-1))

    def reset_grid(self, path):
        """
        Resets the grid world back to its original state before the path was found.
        :param path: The path that was found.
        :return: Nothing.
        """
        for cell in path:
            self.set_state(cell.get_x(), cell.get_y(), cell.get_z(), self.STATE_EMPTY)

    def state_string(self, state):
        """
        Gets a string representation of a cell in the grid world.
        :param state: The state of the cell.
        :return: A human readable format for the cell.
        """
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
