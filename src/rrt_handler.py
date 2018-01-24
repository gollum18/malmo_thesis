#! /usr/bin/python

import MalmoPython
import math, time, sys, random, numpy
from util import PriorityQueue

import pycuda.autoinit
import pycuda.driver as driver
import pycuda.compiler as compiler
import pycuda.gpuarray as gpuarray

#
# Program Constants
#

_astar_maps = ["../out/AStar_Map1.csv",
               "../out/AStar_Map2.csv",
               "../out/AStar_Map3.csv",
               "../out/AStar_Map4.csv"]
_rrt_maps = ["../out/RRT_Map1.csv",
             "../out/RRT_Map2.csv",
             "../out/RRT_Map3.csv",
             "../out/RRT_Map4.csv"]
_rrt_gpu_maps = ["../out/RRTGPU_Map1.csv",
                 "../out/RRTGPU_Map2.csv",
                 "../out/RRTGPU_Map3.csv",
                 "../out/RRTGPU_Map4.csv"]
_mission_files = ['./missions/pp_maze_one.xml',
                  './missions/pp_maze_two.xml',
                  './missions/pp_maze_three.xml',
                  './missions/pp_maze_four.xml']
_mission_text_files = ['./missions/pp_maze_one.txt',
                      './missions/pp_maze_two.txt',
                      './missions/pp_maze_three.txt',
                      './missions/pp_maze_four.txt']
_mission_lower_bounds = [(-24, 55, -24), (-24, 55, -24), (-25, 53, -24), (-24, 55, -24)]
_mission_upper_bounds = [(26, 70, 25), (25, 70, 25), (25, 70, 25), (25, 70, 25)]
_mission_start = (0.5, 56, 24.5)
_mission_goal = [(0.5, 56, -23.5), (0.5, 58, -23.5), (0.5, 54, -23.5), (0.5, 56, 0.5)]

# Needed by the neighbor generation algorithm
_diff = [-1, 0, 1]

# Switches for debugging
_visualize = False
_gpu = True
_debug_paths = False

#
# CPU bound instructions
#

def cost(p1, p2):
    """
    Used by A* to determine the cost of going from p1 to p2.
    :param p1: The initial point.
    :param p2: The ending point.
    :return: The cost of going from p1 to p2.
    """
    return 1 + math.fabs(p2[1] - p1[1])


def degree_change(v1, v2):
    """
    Calculates the change ni angle between two vectors in space.
    :param v1: The first vector.
    :param v2: The second vector.
    :return: The angle of change between the two vectors in degrees.
    """
    return numpy.arccos(numpy.dot(v1, v2) /
                        (numpy.linalg.norm(v1)*numpy.linalg.norm(v2)))


def dist(p1, p2):
    """
    Determines the euclidean distance between p1 and p2.
    :param p1: The initial point.
    :param p2: The ending point.
    :return: The distance between p1 and p2.
    """
    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]
    dz = p2[2]-p1[2]
    return math.sqrt(dx*dx+dy*dy+dz*dz)


def heuristic(p1, p2):
    """
    Used by A* to determine a best guess estimate of the remaining distance between p1 and p2.
    :param p1: The initial point.
    :param p2: The ending point.
    :return: The estimated distance between p1 and p2.
    """
    dx = math.fabs(p2[0] - p1[0])
    dy = math.fabs(p2[1] - p1[1])
    dz = math.fabs(p2[2] - p1[2])
    # Weight the y dimension
    return dx + dy * 1.5 + dz


def reconstruct_path(n):
    """
    Constructs a path from a goal node.
    :param n: The goal node that was found by a planning algorithm.
    :return: A list containing the path.
    """
    path = []
    while n.get_parent():
        path.append(n.get_position())
        n = n.get_parent()
    return path

#
# GPU bound instructions
#
mod = compiler.SourceModule(
"""
    #include <float.h>
    __global__ void neighbors(int n, // The number of array elements
                             float3 result, // The closest point
                             float3 point, // The point to find neareast neighbor
                             float3 * neighbors // All of the neighbors
                             ) {
        // Get the starting index on the GPU
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        // Calculate and store the nearest neighbor
        float dist = (point.x-neighbors[i].x)*(point.x-neighbors[i].x)+
                     (point.y-neighbors[i].y)*(point.y-neighbors[i].y)+
                     (point.z-neighbors[i].z)*(point.z-neighbors[i].z);
        result = neighbors[i];
        # Advance to the second neighbor in this block
        i += blockDim.x * gridDim.x;
        // Stride through the array in GPU memory looking for the
        //  correct nearest neighbor
        for (; i < n; i += blockDim.x * gridDim.x) {
            float temp = (point.x-neighbors[i].x)*(point.x-neighbors[i].x)+
                         (point.y-neighbors[i].y)*(point.y-neighbors[i].y)+
                         (point.z-neighbors[i].z)*(point.z-neighbors[i].z);
            if (temp < dist) {
                dist = temp;
                result = neighbors[i];
            }
        }
    }
"""
)

class Agent(object):

    def __init__(self, start, goal, alpha=0.05, beta=3.25, epsilon=3.0, p_goal=.05, max_nodes=5000):
        """
        Creates a representation of a pathfinding agent.
        :param start: The starting node of the agent.
        :param goal: The goal node of the agent.
        :param alpha: The probability of sampling.
        :param beta: A modifier that affects sampling.
        :param epsilon: The maximum distance between nodes.
        :param max_nodes: The maximum amount of nodes an rrt agent can generate.
        """
        self.start = start
        self.goal = goal
        self.alpha = alpha
        self.beta = beta
        self.epsilon = epsilon
        self.goal_probability = p_goal
        self.max_nodes = max_nodes
        self.world = None
        self.traversed = set()

    def get_start(self):
        """
        Gets the starting state of the agent.
        :return: The starting state of the agent.
        """
        return self.start

    def get_goal(self):
        """
        Gets the goal state of the agent.
        :return: The starting state of the agent.
        """
        return self.goal

    def get_alpha(self):
        """
        The probability used when sampling in rrt.
        :return: Gets the probability used when sampling in rrt.
        """
        return self.alpha

    def get_beta(self):
        """
        Gets the modifier used when sampling in rrt.
        :return: The modifier used when sampling in rrt.
        """
        return self.beta

    def get_epsilon(self):
        """
        Gets the maximum amount of distance allowed between nodes in rrt.
        :return: The maximum distance allowed between each node in rrt.
        """
        return self.epsilon

    def get_max_nodes(self):
        """
        Gets the maximum amount of nodes rrt can generate.
        :return: The maximum amount of node rrt can generate.
        """
        return self.max_nodes

    def set_start(self, start):
        """
        Sets the starting state of the agent.
        :param start: The starting state of the agent.
        :return: N/A
        """
        self.start = start

    def set_goal(self, goal):
        """
        Sets the goal state of the agent.
        :param goal: The goal state of the agent.
        :return: N/A
        """
        self.goal = goal

    def set_alpha(self, alpha):
        """
        Sets the probability used when sampling.
        :param alpha: A probability between 0 and 1. Small values work best.
        :return: N/A
        """
        self.alpha = alpha

    def set_beta(self, beta):
        """
        Sets the modifier used when sampling in rrt.
        :param beta: The new sampling modifier.
        :return: N/A
        """
        self.beta = beta

    def set_epsilon(self, epsilon):
        """
        Sets the maximum distance between nodes in rrt.
        :param epsilon: The maximum distance between nodes in rrt.
        :return: N/A
        """
        self.epsilon = epsilon

    def set_max_nodes(self, max_nodes):
        """
        Sets the max amount of nodes that rrt can generate.
        :param max_nodes: The maximum amount of nodes.
        :return: N?A
        """
        self.max_nodes = max_nodes

    def astar(self):
        """
        Finds a path using the A* search algorithm.
        :return: A path if there is one otherwise an empty list.
        """
        open_list = PriorityQueue()
        closed_list = set()
        node = Node(self.start, None)
        node.set_gscore(0)
        open_list.push(node, 0)
        while not open_list.isEmpty():
            current = open_list.pop()
            if self.is_goal(current.get_position()):
                return reconstruct_path(current)
            if current.get_position() not in closed_list:
                for neighbor in self.generate_neighbors(current.get_position()):
                    if neighbor not in self.world.walkable:
                        continue
                    node = Node(neighbor, current)
                    node.set_gscore(current.get_gscore() + cost(current.get_position(), node.get_position()))
                    open_list.push(node, node.get_gscore() + heuristic(node.get_position(), self.goal))
                closed_list.add(current.get_position())
        return []

    def clear(self):
        """
        Prepares the agent for deletion.
        :return: N/A
        """
        self.traversed.clear()
        self.world.clear()
        del self.traversed
        del self.world

    def create_world(self, xdims, ydims, zdims, filename):
        """
        Creates a representation of the missions world.
        :param xdims: The dimensions of the world in the x axis (length).
        :param ydims: The dimensions of the world in the y axis (height).
        :param zdims: The dimensions of the world in the z axis (width).
        :param filename: The filename of the file that contains the layout of the mission.
        :return: N/A
        """
        if not xdims or not ydims or not zdims or not filename:
            raise ValueError
        reader = open(filename, 'r')
        if not reader:
            raise IOError
        # Create a world state
        self.world = World(xdims, ydims, zdims)
        # Read in obstacle information from file.
        ignoring = False
        for line in reader.readlines():
            line = line.strip("\n")
            if line.startswith("/*") and not ignoring:
                ignoring = True
                continue
            if line.endswith("*/") and ignoring:
                ignoring = False
                continue
            if not ignoring:
                if line.startswith("//") or not line:
                    continue
                parts = [int(part) for part in line.split()]
                self.world.add_obstacle((parts[0], parts[1]), (parts[2], parts[3]), (parts[4], parts[5]))
        reader.close()
        # Begin flood filling from the start to initialize walkable space.
        prospective = [self.start]
        evaluated = set()
        while prospective:
            current = prospective.pop(0)
            for neighbor in self.generate_neighbors(current):
                if neighbor in prospective or neighbor in evaluated:
                    continue
                prospective.append(neighbor)
            evaluated.add(current)
            self.world.add_walkable_space(current)

    def generate_neighbors(self, p):
        """
        Generates octilian neighbors from the given point.
        :param p: The point to generate neighbors for.
        :return: A list containing points from.
        """
        neighbors = []
        for dx in _diff:
            for dy in _diff:
                for dz in _diff:
                    n = (p[0]+dx, p[1]+dy, p[2]+dz)
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    if self.world.is_player_move_valid(n):
                        neighbors.append(n)
        return neighbors

    def is_goal(self, p):
        """
        Determines if the specified point is the goal node or not.
        :param p: The point to check.
        :return: True if the point is a goal, False otherwise.
        """
        return True if p == self.goal else False

    def line_of_sight(self, p1, p2):
        """
        Determines line of sight from p1 to p2.
        :param p1: The observing point.
        :param p2: The observed point.
        :return: True if there is line of sight from p1 to p2, False otherwise.
        """
        def points_on_line(q, r):
            """
            Recursively generates all the points on the line defined by q anr r.
            :param q: The first terminus on the line.
            :param r: The second terminus on the line.
            :return: A list containing all'' the points on the line from q to r.
            """
            if dist(q, r) < 1.0:
                return []
            else:
                m = (q[0]+r[0])/2, (q[1]+r[1])/2, (q[2]+r[2])/2
                return points_on_line(q, m) + [m] + points_on_line(m, r)

        for point in points_on_line(p1, p2):
            for obs in self.world.obstacles:
                if obs.point_inside(point):
                    return False
        return True

    def random(self):
        """
        Randomly samples a point from walkable space. Has a user defined percentage to sample the goal.
        :return: A randomly sampled point from available walkable space.
        """
        if random.random() < self.goal_probability:
            return self.goal
        return random.sample(self.world.walkable - self.traversed, 1)[0]

    def rrt(self):
        """
        Finds a path in the world using a rapidly exploring random tree.
        :return: A path if it is found, otherwise an empty list.
        """
        def neighbor(points, p):
            """
            Finds the nearest neighbor to a point using Tim Sort.
            :param points: The tree itself.
            :param p: The point being added to the tree.
            :return: The point nearest to p in the tree.
            """
            points.sort(key=lambda q: (p[0] - q.get_position()[0]) * (p[0] - q.get_position()[0]) +
                                      (p[1] - q.get_position()[1]) * (p[1] - q.get_position()[1]) +
                                      (p[2] - q.get_position()[2]) * (p[2] - q.get_position()[2]))
            return points[0]

        nodes = [Node(self.start)]

        for i in range(self.max_nodes):
            rand = self.random()
            nn = None
            # Runs Tim sort on the CPU to find nearest neighbor
            if not _gpu:
                nn = neighbor(nodes, rand)
            # Runs NN on the GPU
            else:
                # TODO: Need to convert nodes over to a list of tuples
                func = mod.get_function("neighbors")
                func(driver.In(len(nodes)), # The
                     driver.Out(nn), # The Nearest neighbor
                     driver.In(rand), # The point
                     driver.In(nodes), # The neighbors
                     block=(4, 4, 4))
            # From Steven M. Lavalles implementation
            #for p in nodes:
            #    if dist(p.get_position(), rand) < dist(nn.get_position(), rand):
            #        nn = p
            newnode = self.step_from_to(nn.get_position(), rand)
            nodes.append(Node(newnode, nn))
            self.traversed.add(newnode)
            if self.is_goal(nodes[-1].get_position()):
                return reconstruct_path(nodes[-1])
        return []

    def step_from_to(self, p1, p2):
        """
        Adapted from Steven M. Lavalles implementation of RRT and lifted into three dimensions.
        :param p1: The point being sampled from.
        :param p2: The point that was randomly sampled.
        :return: Either the randomly sampled point or a point along the line from p1 to p2.
        """
        if dist(p1, p2) < self.epsilon:
            return p2
        for i in range(20):
            r = random.random()
            x = (1 - r) * p1[0] + r * p2[0]
            y = math.floor((1 - r) * p1[1] + r * p2[1])
            z = (1 - r) * p1[2] + r * p2[2]
            for w in self.world.walkable:
                if (w[0] <= x <= w[0] + 1 and
                        w[1] == y and
                        w[2] <= z <= w[2] + 1):
                    return w
        return self.step_from_to(p1, self.random())


class Cube(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of a cuboid object.
        :param xdims: The dimensions of the cuboid in the x axis (length).
        :param ydims: The dimensions of the cuboid in the y axis (height).
        :param zdims: The dimensions of the cuboid in the z axis (width).
        """
        self.x_lower = min(xdims)
        self.x_upper = max(xdims)
        self.y_lower = min(ydims)
        self.y_upper = max(ydims)
        self.z_lower = min(zdims)
        self.z_upper = max(zdims)

    def __eq__(self, other):
        """
        Determines another obstacle is equal to this one.
        :param other: The other obstacle to check.
        :return: True if the two obstacles share the same dimensions, False otherwise.
        """
        return (self.x_lower == other.x_lower and self.y_lower == other.y_lower and self.z_lower == other.z_lower and
                self.x_upper == other.x_upper and self.y_upper == other.y_upper and self.z_upper == other.z_upper)

    def __ne__(self, other):
        """
        Determines another obstacle is not equal to this one.
        :param other: The other obstacle to check.
        :return: True if the two obstacles do not share the same dimensions, False otherwise.
        """
        return not self.__eq__(other)

    def __hash__(self):
        """
        Gets a hash representing this obstacle.
        :return: A hash representing this obstacle.
        """
        return hash((self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper))

    def __str__(self):
        """
        Gets an informal string representation of this obstacle.
        :return: An informal string representing the obstacle.
        """
        return "The Bounds of this Obstacle are -> \n\tx:({0}, {1}), ({2}, {3}), ({4}, {5})".format(
            self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper
        )

    def get_lower_bounds(self):
        """
        Gets a tuple (x, y, z) containing the lower bounds of the obstacle.
        :return: A tuple (x, y, z) containing the lower bounds of the obstacle.
        """
        return self.x_lower, self.y_lower, self.z_lower

    def get_upper_bounds(self):
        """
        Gets a tuple (x, y, z) containing the upper bounds of the obstacle.
        :return: A tuple (x, y, z) containing the upper bounds of the obstacle.
        """
        return self.x_upper, self.y_upper, self.z_upper

    def get_x_lower(self):
        """
        Gets the lower bound of the obstacle on the x dimension.
        :return: The lower bound of the obstacle on the x dimension.
        """
        return self.x_lower

    def get_x_upper(self):
        """
        Gets the upper bound of the obstacle on the x dimension.
        :return: The upper bound of the obstacle on the x dimension.
        """
        return self.x_upper

    def get_y_lower(self):
        """
        Gets the lower bound of the obstacle on the y dimension.
        :return: The lower bound of the obstacle on the y dimension.
        """
        return self.y_lower

    def get_y_upper(self):
        """
        Gets the upper bound of the obstacle on the y dimension.
        :return: The upper bound of the obstacle on the y dimension.
        """
        return self.y_upper

    def get_z_lower(self):
        """
        Gets the lower bound of the obstacle on the z dimension.
        :return: The lower bound of the obstacle on the z dimension.
        """
        return self.z_lower

    def get_z_upper(self):
        """
        Gets the upper bound of the obstacle on the z dimension.
        :return: The upper bound of the obstacle on the z dimension.
        """
        return self.z_upper

    def ontop(self, p):
        """
        Determines if a point is ontop of this obstacle.
        :param p: The point to check.
        :return: True if the point is ontop of the obstacle, False otherwise.
        """
        if (self.x_lower <= p[0] <= self.x_upper and
                self.y_upper == p[1] and
                self.z_lower <= p[2] <= self.z_upper):
            return True
        return False

    def player_inside(self, p, height=2):
        """
        Determines if the player is inside the obstacle.
        :param p: The point to check.
        :param height: The players height in units >= 1.
        :return: Determines if the player collides with this object.
        """
        if height < 1:
            height = 1
        points = set()
        for dy in range(1, height+1):
            points.add((p[0], p[1]+dy-.5, p[2]))
        for point in points:
            if self.point_inside(point):
                return True
        return False

    def point_inside(self, p):
        """
        Determines if the point is inside this object or not.
        :param p: The point to check.
        :return: True if the point is inside this object, False otherwise.
        """
        if (self.x_lower < p[0] < self.x_upper and
                self.y_lower < p[1] < self.y_upper and
                self.z_lower < p[2] < self.z_upper):
            return True
        return False


class Node(object):

    def __init__(self, pos, parent=None):
        """
        Creates an instance of a node object.
        :param pos: The position of this node.
        :param parent: The parent of this nod. Default is None.
        """
        self.position = pos
        self.parent = parent
        self.gscore = 0.0

    def __iter__(self):
        return iter(self.position)

    def get_position(self):
        """
        Gets the position of this node.
        :return: The position of this node.
        """
        return self.position

    def get_parent(self):
        """
        Gets the parent of this node.
        :return: The parent of this node.
        """
        return self.parent

    def set_parent(self, parent):
        """
        Sets the parent of this node.
        :param parent: The new parent of this node.
        :return: N/A
        """
        self.parent = parent

    def get_gscore(self):
        """
        Gets the gscore of this node.
        :return: The gscore of this node.
        """
        return self.gscore

    def set_gscore(self, gscore):
        """
        Sets the new gscore of this node.
        :param gscore: The new gscore of this node.
        :return: N/A
        """
        self.gscore = gscore


class World(Cube):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of a world object.
        :param xdims: The dimensions of the world in the x axis (length).
        :param ydims: The dimensions of the world in the y axis (height).
        :param zdims: The dimensions of the world in the z axis (width).
        """
        super(World, self).__init__(xdims, ydims, zdims)
        self.obstacles = set()
        self.walkable = set()

    def add_obstacle(self, xdims, ydims, zdims):
        """
        Adds an obstacle to the agents world representation.
        :param xdims: The dimensions of the obstacle in the x axis (length).
        :param ydims: The dimensions of the obstacle in the y axis (height).
        :param zdims: The dimensions of the obstacle in the z axis (width).
        :return: N/A
        """
        self.obstacles.add(Cube(xdims, ydims, zdims))

    def add_walkable_space(self, p):
        """
        Adds a point of walkable space to the agents world representation.
        :param p: The point ot add walkable space to.
        :return: N/A
        """
        self.walkable.add(p)

    def clear(self):
        """
        Clears the world object and prepares it for deletion.
        :return: N/A
        """
        self.obstacles.clear()
        self.walkable.clear()
        del self.obstacles
        del self.walkable

    def is_point_blocked(self, p):
        """
        Determines if the point is blocked or not.
        :param p: The point to check.
        :return: True if the point is blocked by an obstacle, False otherwise.
        """
        for obs in self.obstacles:
            if obs.point_inside(p):
                return True
        return False

    def is_player_blocked(self, p, height=2):
        """
        Determines if the specified point contains and region (determined by the players height) is blocked.
        :param p: The point to check.
        :param height: The height of the player in units >= 1.
        :return: True if the player cannot move to the specified point, False otherwise.
        """
        for obs in self.obstacles:
            if obs.player_inside(p, height):
                return True
        return False

    def is_on_obstacle(self, p):
        """
        Determines if the point is on top of an obstacle.
        :param p: The point to check.
        :return: True if the point is on an obstacle, False otherwise.
        """
        for obs in self.obstacles:
            if obs.ontop(p):
                return True
        return False

    def is_point_valid(self, p):
        """
        Determines if the specified point is inside the bounds of the map and is not an obstacle.
        :param p: The point to check.
        :return: True if the point is valid, False otherwise.
        """
        return self.point_inside(p) and self.is_on_obstacle(p) and not self.is_point_blocked(p)

    def is_player_move_valid(self, p, height=2):
        """
        Determines if a player can move to the specified location.
        :param p: The point to check.
        :param height: The height of the player in units >= 1.
        :return: True if the move is valid, False otherwise.
        """
        return self.player_inside(p) and self.is_on_obstacle(p) and not self.is_player_blocked(p, height)


def gather_data(alg=0, iterations=1000):
    """
    Gathers data on the pathfinding algorithms.
    :alg: The algorithm to use, 0 for A*. anything else for RRT.
    :map_number: The map number, an exception is raised if it is not between 0 and the total number of maps.
    :param iterations: The sample size.
    :return: N/A
    """
    agent = None
    out1 = open(_astar_maps[0], 'w') if alg == 0 else open(_rrt_maps[0] if not _gpu else _rrt_gpu_maps[0], 'w')
    out2 = open(_astar_maps[1], 'w') if alg == 0 else open(_rrt_maps[1] if not _gpu else _rrt_gpu_maps[1], 'w')
    out3 = open(_astar_maps[2], 'w') if alg == 0 else open(_rrt_maps[2] if not _gpu else _rrt_gpu_maps[2], 'w')
    out4 = open(_astar_maps[3], 'w') if alg == 0 else open(_rrt_maps[3] if not _gpu else _rrt_gpu_maps[3], 'w')
    header = "Run Time,Path Length,Heading Changes,Total Degrees (Degrees)\n"
    out1.write(header)
    out2.write(header)
    out3.write(header)
    out4.write(header)
    for i in range(iterations):
        for j in range(len(_mission_files)):
            # Attempt to start a mission:
            agent = Agent(_mission_start, _mission_goal[j])
            agent.create_world((_mission_lower_bounds[j][0], _mission_upper_bounds[j][0]),
                               (_mission_lower_bounds[j][1], _mission_upper_bounds[j][1]),
                               (_mission_lower_bounds[j][2], _mission_upper_bounds[j][2]),
                               _mission_text_files[j])
            path = None
            runtime = time.clock()
            if alg == 0:
                path = agent.astar()
            else:
                path = agent.rrt()
            if not path:
                if j == 0:
                    out1.write("{0},{1},{2},{3}\n"
                               .format(0, 0, 0, 0))
                elif j == 1:
                    out2.write("{0},{1},{2},{3}\n"
                               .format(0, 0, 0, 0))
                elif j == 2:
                    out3.write("{0},{1},{2},{3}\n"
                               .format(0, 0, 0, 0))
                elif j == 3:
                    out4.write("{0},{1},{2},{3}\n"
                               .format(0, 0, 0, 0))
                continue
            runtime = time.clock() - runtime
            pathlength = dist(agent.start, path[0])
            degrees = degree_change(agent.start, path[0])
            hchanges = 1 if degrees != 0 else 0
            for k in range(1, len(path)):
                pathlength += dist(path[k - 1], path[k])
                change = degree_change(path[k - 1], path[k])
                if change < 0:
                    change += 2 * math.pi
                degrees += change
                if change > 0:
                    hchanges += 1
            degrees = math.degrees(degrees)
            if j == 0:
                out1.write("{0},{1},{2},{3}\n"
                           .format(runtime, pathlength, hchanges, degrees))
            elif j == 1:
                out2.write("{0},{1},{2},{3}\n"
                           .format(runtime, pathlength, hchanges, degrees))
            elif j == 2:
                out3.write("{0},{1},{2},{3}\n"
                           .format(runtime, pathlength, hchanges, degrees))
            elif j == 3:
                out4.write("{0},{1},{2},{3}\n"
                           .format(runtime, pathlength, hchanges, degrees))
    out1.close()
    out2.close()
    out3.close()
    out4.close()


def visualize():
    # Create default Malmo objects:
    agent_host = MalmoPython.AgentHost()

    try:
        agent_host.parse(sys.argv)
    except RuntimeError as e:
        print('ERROR:', e)
        print(agent_host.getUsage())
        exit(1)
    if agent_host.receivedArgument("help"):
        print(agent_host.getUsage())
        exit(0)

    if agent_host.receivedArgument("test"):
        num_repeats = 1
    else:
        num_repeats = 10

    for i in range(len(_mission_files)):
        # Attempt to start a mission:
        my_mission = MalmoPython.MissionSpec(open(_mission_files[i], 'r').read(), False)
        planner = Agent(_mission_start, _mission_goal[i])
        planner.create_world((_mission_lower_bounds[i][0], _mission_upper_bounds[i][0]),
                             (_mission_lower_bounds[i][1], _mission_upper_bounds[i][1]),
                             (_mission_lower_bounds[i][2], _mission_upper_bounds[i][2]),
                             _mission_text_files[i])
        path = planner.rrt()

        if _debug_paths:
            w = dict()
            for point in planner.world.walkable:
                if point[1] in w.keys():
                    w[point[1]].add((point[0], point[2]))
                else:
                    w[point[1]] = set()
                    w[point[1]].add([point[0], point[2]])
            print(path)

        # Comment this out when actually testing with the Malmo platform
        #continue

        max_retries = 3
        for retry in range(max_retries):
            try:
                agent_host.startMission(my_mission, MalmoPython.MissionRecordSpec())
                break
            except RuntimeError as e:
                if retry == max_retries - 1:
                    print("Error starting mission:", e)
                    exit(1)
                else:
                    time.sleep(2)

        # Loop until mission starts:
        print("Waiting for the mission to start ",)
        world_state = agent_host.getWorldState()
        while not world_state.has_mission_begun:
            sys.stdout.write(".")
            time.sleep(.5)
            world_state = agent_host.getWorldState()
            for error in world_state.errors:
                print("Error:", error.text)

        print()
        print("Mission running ",)

        # Loop until mission ends:
        while world_state.is_mission_running:
            sys.stdout.write(".")
            time.sleep(1.0)
            world_state = agent_host.getWorldState()
            if path:
                position = path.pop()
                agent_host.sendCommand("tp {0} {1} {2}".format(*position))

        print()
        print("Mission ended")
        # Mission has ended.


if __name__ == '__main__':
    # If the switch for Malmo is enabled then run it, otherwise gather some data
    if _visualize:
        visualize()
    else:
        gather_data(1)
