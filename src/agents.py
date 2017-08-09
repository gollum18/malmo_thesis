from enum import Enum
from collections import deque
import worlds

class AgentType(Enum):
    DISCRETE = 0
    CONTINUOUS = 1

class Cell3D:

    def __init__(self, x, y, z, parent):
        """
        Creates a cell. A cell is used by path planning algorithms to find paths.
        :param x: The x coordinate.
        :param y: The y coordinate.
        :param z: The z coordinate.
        :param parent: The parent. Used for generating the final path.
        """
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __str__(self):
        return "Cell3D: ({0}, {1}, {2})".format(self.x, self.y, self.z)

    def get_coordinates(self):
        """
        Gets a tuple containaing coordinates in the form (x, y, z)
        :return: A tuple containing the coordinates of this cell.
        """
        return self.x, self.y, self.z

    def get_x(self):
        """
        Gets the x coordinate of this cell.
        :return: The x coordinate.
        """
        return self.x

    def get_y(self):
        """
        Gets the y coordinate of this cell.
        :return: The y coordinate.
        """
        return self.y

    def get_z(self):
        """
        Gets the z coordinate of this cell.
        :return: The z coordinate.
        """
        return self.z

    def get_parent(self):
        """
        Gets the parent of this cell.
        :return: The parent.
        """
        return self.parent

    def set_parent(self, parent):
        """
        Sets the parent for this cell.
        :param parent: The new parent.
        :return: Does not return.
        """
        self.parent = parent


class Cell3DWithCosts(Cell3D):

    def __init__(self, x, y, z, parent):
        """
        Creates a 3d cell acceptable for use with an algorithm that uses costs/heuristics like A*.
        :param x:
        :param y:
        :param z:
        :param parent:
        """
        Cell3D.__init__(self, x, y, z, parent)
        self.f_score = 0.0
        self.g_score = 0.0
        self.h_score = 0.0

    def __str__(self):
        return "Cell3D: ({0}, {1}, {2}\nF: {3}, G: {4}, H: {5})".format(self.x, self.y, self.z,
                                        self.f_score, self.g_score, self.h_score)

    def get_fscore(self):
        return self.f_score

    def get_gscore(self):
        return self.g_score

    def get_hscore(self):
        return self.h_score

    def set_fscore(self, score):
        self.f_score = score

    def set_gscore(self, score):
        self.g_score = score

    def set_hscore(self, score):
        self.h_score = score


class Agent:

    def __init__(self, world, type):
        self.world = world
        self.type = type

    def get_agent_type(self):
        return type

    def get_world(self):
        return self.world

    def set_world(self, world):
        self.world = world

    def reconstruct_path(self, end):
        path = []

        node = end.get_parent()
        while node.get_parent():
            path.append(node)
            self.world.set_state(node.get_x(), node.get_y(), node.get_z(), worlds.State.PATH)
            node = node.get_parent()

        return path.reverse()

    def get_neighbors(self, cell):
        x, y, z = cell.x, cell.y, cell.z
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                if self.world.in_bounds(x + dx, y + dy, z) and self.world.get_state(x, y, z) != worlds.State.BLOCKED:
                    neighbors.append(Cell3D(x + dx, y + dy, z, cell))
        if self.world.get_transition_type(x, y, z) == worlds.TransitionType.TRANSITION_UPPER:
            neighbors.append(Cell3D(x, y, z + 1, cell))
        elif self.world.get_transition_type(x, y, z) == worlds.TransitionType.TRANSITION_LOWER:
            neighbors.append(Cell3D(x, y, z - 1, cell))
        return neighbors


class SearchAgent(Agent):

    def __init__(self, world, type):
        if type != AgentType.CONTINUOUS and type != AgentType.DISCRETE:
            raise ValueError
        Agent.__init__(self, world, type)
        self.open_list = None
        self.closed_list = None

    def get_open_list(self):
        return self.open_list

    def get_closed_list(self):
        return self.closed_list


class BreadthFirstSearchAgent(SearchAgent):

    def __init__(self, world, type):
        SearchAgent.__init__(self, world, type)
        self.open_list = deque()
        self.closed_list = set()

    def bfs(self):
        x, y, z = self.world.get_start()
        self.open_list.append(Cell3D(x, y, z, None))

        while self.open_list:
            current = self.open_list.pop()
            if self.world.is_goal_position(current.get_coordinates()):
                return self.reconstruct_path(current)
            if current not in self.closed_list:
                self.closed_list.add(current)
                for neighbor in self.get_neighbors(current):
                    self.open_list.append(neighbor)

        return None

class DepthFirstSearchAgent(SearchAgent):

    def __init__(self, world, type):
        SearchAgent.__init__(self, world, type)
        self.open_list = deque()
        self.closed_list = set()

    def dfs(self):
        x, y, z = self.world.get_start()
        self.open_list.appendleft(Cell3D(x, y, z, None))

        while self.open_list:
            current = self.open_list.pop()
            if self.world.is_goal_position(current.get_coordinates()):
                return self.reconstruct_path(current)
            if current not in self.closed_list:
                self.closed_list.add(current)
                for neighbor in self.get_neighbors(current):
                    self.open_list.appendleft(neighbor)

        return None

bfs_agent = BreadthFirstSearchAgent(worlds.World3D(10, 10, 3, (0, 0, 0), (5, 5, 2), 30), AgentType.DISCRETE)
dfs_agent = DepthFirstSearchAgent(worlds.World3D(10, 10, 3, (0, 0, 0), (5, 5, 2), 30), AgentType.DISCRETE)

bfs_agent.bfs()
dfs_agent.dfs()

print bfs_agent.get_world()
print dfs_agent.get_world()