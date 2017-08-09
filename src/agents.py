from enum import Enum
from collections import deque
from Queue import PriorityQueue
import worlds
import random
import math


# CONSTANTS
X_LIM = 10
Y_LIM = 10
Z_LIM = 5
NUM_OBS = 50


class AgentType(Enum):
    NON_HEURISTIC = 0
    HEURISTIC = 1


class Heuristics:

    @staticmethod
    def euclidean_squared_distance(p1, p2):
        """
        Determines the euclidean squared distance between two cells in two dimensions.
        :param p1: The first point.
        :param p2: The second point.
        :return: The euclidean squared distance in two dimensions.
        """
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        return dx*dx+dy*dy

    @staticmethod
    def euclidean3_squared_distance(p1, p2):
        """
        Determines the euclidean squared distance between two cells in three dimensions.
        :param p1: The first point.
        :param p2: The second  point.
        :return: The euclidean squared distance in three dimensions.
        """
        dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
        return dx*dx+dy*dy+dz*dz

    @staticmethod
    def euclidean_distance(p1, p2):
        """
        Determines the euclidean distance between two points in two dimensions.
        :param p1: The first point.
        :param p2: The second point.
        :return: The euclidean distance in two dimensions.
        """
        return math.sqrt(Heuristics.euclidean_squared_distance(p1, p2))

    @staticmethod
    def euclidean3_distance(p1, p2):
        """
        Determines the euclidean distance between two points in three dimensions.
        :param p1: The first point.
        :param p2: The second point.
        :return: The euclidean distance in three dimensions.
        """
        return math.sqrt(Heuristics.euclidean3_squared_distance(p1, p2))

    @staticmethod
    def manhattan_distance(p1, p2):
        """
        Calculates the manhattan distance between two points in two dimensions.
        :param p1: The first point.
        :param p2: The second point.
        :return: The manhattan distance in two dimensions.
        """
        return (p2[0] - p1[0]) + (p2[1] - p1[1])

    @staticmethod
    def manhattan3_distance(p1, p2):
        """
        Calculates the manhattan distance between two points in three dimensions.
        :param p1: The first point.
        :param p2: The second point.
        :return: The manhattan distance in three dimensions.
        """
        return (p2[0] - p1[0]) + (p2[1] - p1[1]) + (p2[2] - p1[2])

class Cell3D:

    def __init__(self, x, y, z, parent=None):
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

    def __ne__(self, other):
        return not self.x == other.x and self.y == other.y and self.z == other.z

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

    def __lt__(self, other):
        return self.f_score < other.f_score

    def __le__(self, other):
        return self.f_score <= other.f_score

    def __gt__(self, other):
        return self.f_score > other.f_score

    def __ge__(self, other):
        return self.f_score >= other.f_score

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
            if not node.get_coordinates() == self.get_world().get_start():
                self.world.set_state(node.get_x(), node.get_y(), node.get_z(), worlds.State.PATH)
            node = node.get_parent()

        return path.reverse()

    def line_of_sight2d(self, c1, c2):
        """
        Determines if c2 is visible from c1. That is c1s' line of sight to c2 is obstacle free.
        :param c1: The cell to check from.
        :param c2: The cell to check against.
        :return: True if there is line of sight. False otherwise.
        """
        if c1.get_z() != c2.get_z():
            raise ValueError

        xo, xt, yo, yt = c1.get_x(), c2.get_x(), c1.get_y(), c2.get_y()
        dx, dy = xt - xo, yt - yo
        sx, sy = 0, 0
        f = 0

        if dy < 0:
            dy = -dy
            sy = -1
        else:
            sy = 1

        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1

        if dx >= dy:
            while xo != xt:
                f = f + dy# Get the new point
                nx = xo + ((sx - 1) / 2)
                ny = yo + ((sy - 1) / 2)
                nz = c1.get_z()
                # Case 1
                if f >= dx:
                    if (not self.get_world().in_bounds(nx, ny, nz) or
                            self.get_world().get_state(nx, ny, nz) == worlds.State.BLOCKED):
                        return False
                    yo = yo + sy
                    f = f - dx
                # Case 2
                if (f != 0 and (not self.get_world().in_bounds(nx, ny, nz) or
                        self.get_world().get_state(nx, ny, nz) == worlds.State.BLOCKED)):
                    return False
                # Case 3
                if (dy == 0 and (not self.get_world().in_bounds(nx, yo, nz) or
                        self.get_world().get_state(nx, yo, nz) == worlds.State.BLOCKED) and
                        (not self.get_world().in_bounds(nx, yo-1, nz) or
                        self.get_world().get_state(nx, yo-1, nz) == worlds.State.BLOCKED)):
                    return False
                xo = xo + sx
        else:
            while yo != yt:
                f = f + dx
                # Get the new point
                nx = xo + ((sx - 1) / 2)
                ny = yo + ((sy - 1) / 2)
                nz = c1.get_z()
                # Case 1
                if f >= dy:
                    if (not self.get_world().in_bounds(nx, ny, nz) or
                            self.get_world().get_state(nx, ny, nz) == worlds.State.BLOCKED):
                        return False
                    xo = xo + sx
                    f = f - dy
                # Case 2
                if (f != 0 and (not self.get_world().in_bounds(nx, ny, nz) or
                        self.get_world().get_state(nx, ny, nz) == worlds.State.BLOCKED)):
                    return False
                # Case 3
                if (dy == 0 and (not self.get_world().in_bounds(xo, ny, nz) or
                        self.get_world().get_state(xo, ny, nz) == worlds.State.BLOCKED) and
                        (not self.get_world().in_bounds(xo-1, ny, nz) or
                        self.get_world().get_state(xo-1, ny, nz) == worlds.State.BLOCKED)):
                    return False
                yo = yo + sy
        return True


class SearchAgent(Agent):

    def __init__(self, world, type):
        if type != AgentType.NON_HEURISTIC and type != AgentType.HEURISTIC:
            raise ValueError
        Agent.__init__(self, world, type)
        self.open_list = None
        self.closed_list = None

    def get_open_list(self):
        return self.open_list

    def get_closed_list(self):
        return self.closed_list

    def get_neighbors(self, cell):
        x, y, z = cell.x, cell.y, cell.z
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                if self.world.in_bounds(x + dx, y + dy, z) and self.world.get_state(x, y, z) != worlds.State.BLOCKED:
                    if self.get_agent_type() == AgentType.NON_HEURISTIC:
                        neighbors.append(Cell3D(x + dx, y + dy, z, cell))
                    else:
                        neighbors.append(Cell3DWithCosts(x + dx, y + dy, z, cell))
        if self.world.get_transition_type(x, y, z) == worlds.TransitionType.TRANSITION_UPPER:
            if self.get_agent_type() == AgentType.NON_HEURISTIC:
                neighbors.append(Cell3D(x, y, z + 1, cell))
            else:
                neighbors.append(Cell3DWithCosts(x, y, z + 1, cell))
        elif self.world.get_transition_type(x, y, z) == worlds.TransitionType.TRANSITION_LOWER:
            if self.get_agent_type() == AgentType.NON_HEURISTIC:
                neighbors.append(Cell3D(x, y, z - 1, cell))
            else:
                neighbors.append(Cell3DWithCosts(x, y, z - 1, cell))
        return neighbors


class AStarSearchAgent(SearchAgent):

    def __init__(self, world, type):
        SearchAgent.__init__(self, world, type)
        self.open_list = PriorityQueue()
        self.closed_list = set()

    def astar(self):
        x, y, z = self.world.get_start()
        self.open_list.put_nowait(Cell3DWithCosts(x, y, z, None))

        while not self.open_list.empty():
            current = self.open_list.get_nowait()
            if self.world.is_goal_position(current.get_coordinates()):
                return self.reconstruct_path(current)
            if current not in self.closed_list:
                self.closed_list.add(current)
                for neighbor in self.get_neighbors(current):
                    neighbor.set_gscore = current.get_gscore() + 1
                    if self.get_world().is_goal_area(neighbor.get_z()):
                        neighbor.set_hscore(Heuristics.euclidean3_distance(neighbor.get_coordinates(),
                                                                          self.get_world().get_goal()))
                    else:
                        if self.get_world().get_goal()[2] > neighbor.get_z():
                            # Set levels upper transition as goal
                            neighbor.set_hscore(Heuristics.euclidean3_distance(neighbor.get_coordinates(),
                                                                              self.get_world().get_transitions_by_floor(
                                                                                  neighbor.get_z())[
                                                                                  worlds.TransitionType.TRANSITION_UPPER]))
                        elif self.get_world().get_goal()[2] < neighbor.get_z():
                            # Set levels lower transition as goal
                            neighbor.set_hscore(Heuristics.euclidean3_distance(neighbor.get_coordinates(),
                                                                              self.get_world().get_transitions_by_floor(
                                                                                  neighbor.get_z())[
                                                                                  worlds.TransitionType.TRANSITION_LOWER]))
                        neighbor.set_fscore = neighbor.get_gscore() + neighbor.get_fscore()
                    self.open_list.put_nowait(neighbor)

        return None

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


class RapidlyExploringRandomTreeAgent(Agent):

    def __init__(self, world, type, kmax, epsilon):
        Agent.__init__(self, world, type)
        self.max_nodes = kmax
        self.max_dist = epsilon

        # Generate the random node pool per level
        # TODO: Maybe optimize?
        self.available_nodes = {}
        for z in range(self.get_world().get_dimensions()[2]):
            nodes = []
            for y in range(self.get_world().get_dimensions()[1]):
                for x in range(self.get_world().get_dimensions()[0]):
                    if self.get_world().get_state(x, y, z) == worlds.State.OPEN:
                        nodes.append(Cell3D(x, y, z))
                self.available_nodes[z] = nodes

    def find_closest_cell_in_level(self, nodes, cell):
        """
        Determines the optimum parent for this node.
        :param nodes: The list of all nodes in the tree.
        :param cell: The cell to find the closest node to.
        :return: The cell in the tree that is closest to the passed in cell.
        """
        z = cell.get_z()
        mnode = None
        mdist = float("inf")
        for node in nodes:
            # only care if the two cells share levels
            if node.get_z() != z:
                continue
            value = Heuristics.euclidean_distance(cell.get_coordinates(), node.get_coordinates())
            if value < mdist:
                mnode = node
                mdist = value
        return mnode

    def rrt(self):
        # TODO: Fix rrt, it is currently looping 
        current = Cell3D(self.get_world().get_start()[0],
                         self.get_world().get_start()[1],
                         self.get_world().get_start()[2])
        nodes = []
        while len(nodes) < self.max_nodes:
            candidate = random.choice(self.available_nodes[current.get_z()])
            # Perform the distance check
            if Heuristics.euclidean_distance(current.get_coordinates(),
                                             candidate.get_coordinates()) > self.max_dist:
                continue
            # Perform the los check
            if self.line_of_sight2d(current, candidate):
                candidate.set_parent(current)
                # Perform the goal check
                if self.get_world().is_goal_position(current.get_coordinates()):
                    return self.reconstruct_path(current)
                # Perform the transition check
                if (self.get_world().get_transition_type(candidate.get_x(), candidate.get_y(), candidate.get_z()) ==
                    worlds.TransitionType.TRANSITION_UPPER):
                    cell = Cell3D(candidate.get_x(), candidate.get_y(), candidate.get_z() + 1, candidate)
                    nodes.append(cell)
                    if cell in self.available_nodes[candidate.get_z() + 1]:
                        self.available_nodes[candidate.get_z() + 1].remove(cell)
                    current = cell
                elif (self.get_world().get_transition_type(candidate.get_x(), candidate.get_y(), candidate.get_z()) ==
                    worlds.TransitionType.TRANSITION_LOWER):
                    cell = Cell3D(candidate.get_x(), candidate.get_y(), candidate.get_z() - 1, candidate)
                    nodes.append(cell)
                    if cell in self.available_nodes[candidate.get_z() - 1]:
                        self.available_nodes[candidate.get_z() - 1].remove(cell)
                    current = cell
                else:
                    current = candidate
                    nodes.append(current)
                    self.available_nodes[current.get_z()].remove(current)
        return None


start = (random.randint(0, X_LIM-1),random.randint(0, Y_LIM-1), random.randint(0, Z_LIM-1))
goal = (random.randint(0, X_LIM-1),random.randint(0, Y_LIM-1), random.randint(0, Z_LIM-1))
# bfs_agent = BreadthFirstSearchAgent(worlds.World3D(X_LIM, Y_LIM, Z_LIM, start, goal, NUM_OBS), AgentType.NON_HEURISTIC)
# dfs_agent = DepthFirstSearchAgent(worlds.World3D(X_LIM, Y_LIM, Z_LIM, start, goal, NUM_OBS), AgentType.NON_HEURISTIC)
# astar_agent = AStarSearchAgent(worlds.World3D(X_LIM, Y_LIM, Z_LIM, start, goal, NUM_OBS), AgentType.HEURISTIC)
#
# bfs_agent.bfs()
# dfs_agent.dfs()
# astar_agent.astar()
#
# print "BFS AGENT:"
# print bfs_agent.get_world()
# print "DFS AGENT"
# print dfs_agent.get_world()
# print "ASTAR AGENT"
# print astar_agent.get_world()

rrt_agent = RapidlyExploringRandomTreeAgent(worlds.World3D(X_LIM, Y_LIM, Z_LIM, start, goal, NUM_OBS),
                                            AgentType.HEURISTIC, 500, 2.0)
rrt_agent.rrt()

print rrt_agent
print rrt_agent.get_world()