from enum import Enum
import random


class State(Enum):
    OPEN = 0
    BLOCKED = 1
    AGENT = 2
    GOAL = 3
    PATH = 4
    PROTECTED = 5

    @staticmethod
    def str(state):
        if state == State.OPEN or state == State.PROTECTED:
            return "-"
        elif state == State.BLOCKED:
            return "B"
        elif state == State.AGENT:
            return "A"
        elif state == state.GOAL:
            return "G"
        elif state == State.PATH:
            return "P"


class TransitionType(Enum):
    TRANSITION_NONE = 0
    TRANSITION_UPPER = 1
    TRANSITION_LOWER = 2

    @staticmethod
    def str(type):
        if type == TransitionType.TRANSITION_UPPER:
            return "U"
        elif type == TransitionType.TRANSITION_LOWER:
            return "L"
        elif type == TransitionType.TRANSITION_NONE:
            return "-"


class World3D:

    #
    # Constructor
    #

    def __init__(self, xdim, ydim, zdim, start, goal, num_obs):
        self.dimensions = (xdim, ydim, zdim)
        self.start = start
        self.goal = goal
        self.num_obs = num_obs
        self.world = []
        for z in range(zdim):
            plane = []
            for y in range(ydim):
                row = []
                for x in range(xdim):
                    row.append([State.OPEN, TransitionType.TRANSITION_NONE])
                plane.append(row)
            self.world.append(plane)

        # Place the start and goal
        self.set_state(start[0], start[1], start[2], State.AGENT)
        self.set_state(goal[0], goal[1], goal[2], State.GOAL)
        self.protect_cell(start[0], start[1], start[2])
        self.protect_cell(goal[0], goal[1], goal[2])

        # Place the obstacles
        for i in range(self.num_obs):
            x, y, z = (random.randint(0, self.dimensions[0] - 1), random.randint(0, self.dimensions[1] - 1),
                       random.randint(0, self.dimensions[2] - 1))
            while self.get_state(x, y, z) != State.OPEN and self.get_state(x, y, z) == State.PROTECTED:
                x, y, z = (random.randint(0, self.dimensions[0] - 1), random.randint(0, self.dimensions[1] - 1),
                           random.randint(0, self.dimensions[2] - 1))
            self.set_state(x, y, z, State.BLOCKED)

        # Place the transitions
        for z in range(zdim):
            for i in range(2):
                x, y, = random.randint(0, self.dimensions[0] - 1), random.randint(0, self.dimensions[1] - 1)
                if z == 0:
                    while (self.get_state(x, y, z) != State.OPEN and
                                   self.get_state(x, y, z + 1) == State.OPEN and
                                   self.get_transition_type(x, y, z + 1) != TransitionType.TRANSITION_NONE):
                        x, y, = random.randint(0, self.dimensions[0] - 1), random.randint(0, self.dimensions[1] - 1)
                elif z == self.dimensions[2]-1:
                    while (self.get_state(x, y, z) != State.OPEN and
                                   self.get_state(x, y, z - 1) == State.OPEN and
                                   self.get_transition_type(x, y, z - 1) == TransitionType.TRANSITION_NONE):
                        x, y, = random.randint(0, self.dimensions[0] - 1), random.randint(0, self.dimensions[1] - 1)
                else:
                    while (self.get_state(x, y, z) != State.OPEN and
                                   self.get_state(x, y, z + 1) == State.OPEN and
                                   self.get_state(x, y, z - 1) == State.OPEN and
                                   self.get_transition_type(x, y, z + 1) != TransitionType.TRANSITION_NONE and
                                   self.get_transition_type(x, y, z - 1) != TransitionType.TRANSITION_NONE):
                        x, y, = random.randint(0, self.dimensions[0] - 1), random.randint(0, self.dimensions[1] - 1)
                if i == 0 and z > 0:
                    self.world[z][y][x][1] = TransitionType.TRANSITION_LOWER
                elif i == 1:
                    if z < self.dimensions[2] - 1:
                        self.world[z][y][x][1] = TransitionType.TRANSITION_UPPER
                self.protect_cell(x, y, z)

    #
    # Built-ins
    #

    def __str__(self):
        s = ""
        for z in range(self.dimensions[2]):
            s += "Level {0}:\n".format(z)
            for y in range(self.dimensions[1]):
                s += "|"
                for x in range(self.dimensions[0]):
                    if self.world[z][y][x][1] != TransitionType.TRANSITION_NONE:
                        s += TransitionType.str(self.world[z][y][x][1]) + "|"
                    else:
                        s += State.str(self.world[z][y][x][0]) + "|"
                s += "\n"
            s += "\n"
        return s

    #
    # Accessors/Mutators
    #

    def get_state(self, x, y, z):
        return self.world[z][y][x][0]

    def set_state(self, x, y, z, state):
        self.world[z][y][x][0] = state

    def get_transition_type(self, x, y, z):
        return self.world[z][y][x][1]

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    #
    # Methods
    #

    def is_goal_area(self, z):
        return self.goal[2] == z

    def is_goal_position(self, pos):
        x, y, z = pos[0], pos[1], pos[2]
        if not self.in_bounds(x, y, z):
            raise ValueError
        return self.get_state(x, y, z) == State.GOAL

    def in_bounds(self, x, y, z):
        if x < 0 or x >= self.dimensions[0]:
            return False
        if y < 0 or y >= self.dimensions[1]:
            return False
        if z < 0 or z >= self.dimensions[2]:
            return False
        return True

    def move_agent(self, old_pos, new_pos):
        if not self.in_bounds(old_pos[0], old_pos[1], old_pos[2]):
            raise ValueError
        if not self.in_bounds(new_pos[0], new_pos[1], new_pos[2]):
            raise ValueError
        self.set_state(old_pos[0], old_pos[1], old_pos[2], State.EMPTY)
        self.set_state(new_pos[0], new_pos[1], new_pos[2], State.AGENT)

    def protect_cell(self, x, y, z):
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                if self.in_bounds(x+dx, y+dy, z):
                    self.set_state(x+dx, y+dy, z, State.PROTECTED)