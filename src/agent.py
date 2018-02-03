#! /usr/bin/python

import math
import time
import sys
import random
import numpy

from multiprocessing import pool
from multiprocessing.dummy import Pool as ThreadPool

import pycuda.autoinit
import pycuda.driver as driver
import pycuda.compiler as compiler
import pycuda.gpuarray as gpuarray
from pycuda.elementwise import ElementwiseKernel as Elementwise

import util
from structures import PriorityQueue
from nodes import ListNode, TreeNode
from world import World

import MalmoPython

#
# Program Constants
#

MAPS_ASTAR = ["../out/AStar_Map1.csv",
               "../out/AStar_Map2.csv",
               "../out/AStar_Map3.csv",
               "../out/AStar_Map4.csv"]
MAPS_RRT = ["../out/RRT_List_Map1.csv",
             "../out/RRT_list_Map2.csv",
             "../out/RRT_List_Map3.csv",
             "../out/RRT_List_Map4.csv"]
MAPS_RRT_GPU = ["../out/RRT_GPU_Map1.csv",
                 "../out/RRT_GPU_Map2.csv",
                 "../out/RRT_GPU_Map3.csv",
                 "../out/RRT_GPU_Map4.csv"]
MAPS_RRT_TREE = ["../out/RRT_Tree_Map1.csv",
                  "../out/RRT_Tree_Map2.csv",
                  "../out/RRT_Tree_Map3.csv",
                  "../out/RRT_Tree_Map4.csv"]
MISSION_FILES = ["./missions/pp_maze_one.xml",
                  "./missions/pp_maze_two.xml",
                  "./missions/pp_maze_three.xml",
                  "./missions/pp_maze_four.xml"]
MISSION_TEXT_FILES = ["./missions/pp_maze_one.txt",
                      "./missions/pp_maze_two.txt",
                      "./missions/pp_maze_three.txt",
                      "./missions/pp_maze_four.txt"]
MISSION_LOWER_BOUNDS = [(-24, 55, -24), (-24, 55, -24), (-25, 53, -24), (-24, 55, -24)]
MISSION_UPPER_BOUNDS = [(26, 70, 25), (25, 70, 25), (25, 70, 25), (25, 70, 25)]
MISSION_START = (0.5, 56, 24.5)
MISSION_GOALS = [(0.5, 56, -23.5), (0.5, 58, -23.5), (0.5, 54, -23.5), (0.5, 56, 0.5)]

# Needed by the neighbor generation algorithm
DIFF = [-1, 0, 1]

# Switches for debugging
BOOL_VISUALIZE = False
BOOL_GPU = False
BOOL_TREE = True
BOOL_DEBUG_PATHS = False

def get_file_name(index, alg):
    if alg == 0:
        return MAPS_ASTAR[index]
    else:
        if not BOOL_GPU:
            if not BOOL_TREE:
                return MAPS_RRT[index]
            else:
                return MAPS_RRT_TREE[index]
        else:
            return MAPS_RRT_GPU[index]

# Defines the distance operation performed on the nodes once they 
#   are transferred to the GPU
mod = Elementwise(
    arguments="float3 point, float3 *neighbors, float *distances",
    operation="distances[i] = pow(point.x-neighbors[i].x,2)+pow(point.y-neighbors[i].y,2)+pow(point.z-neighbors[i].z,2)",
    name="euclidean_distance",
    preamble="#include <math.h>"
)

class Agent(object):

    def __init__(self, start, goal, epsilon=3.0, p_goal=.05, max_nodes=5000):
        """
        Creates a representation of a pathfinding agent.
        start: The starting node of the agent.
        goal: The goal node of the agent.
        alpha: The probability of sampling.
        beta: A modifier that affects sampling.
        epsilon: The maximum distance between nodes.
        max_nodes: The maximum amount of nodes an rrt agent can generate.
        """
        self.start = start
        self.goal = goal
        self.epsilon = epsilon
        self.goal_probability = p_goal
        self.max_nodes = max_nodes
        self.world = None
        self.traversed = set()

    def get_start(self):
        """
        Gets the starting state of the agent.
        returns: The starting state of the agent.
        """
        return self.start

    def get_goal(self):
        """
        Gets the goal state of the agent.
        returns: The starting state of the agent.
        """
        return self.goal

    def get_epsilon(self):
        """
        Gets the maximum amount of distance allowed between nodes in rrt.
        returns: The maximum distance allowed between each node in rrt.
        """
        return self.epsilon

    def get_max_nodes(self):
        """
        Gets the maximum amount of nodes rrt can generate.
        returns: The maximum amount of node rrt can generate.
        """
        return self.max_nodes

    def set_start(self, start):
        """
        Sets the starting state of the agent.
        start: The starting state of the agent.
        returns: N/A
        """
        self.start = start

    def set_goal(self, goal):
        """
        Sets the goal state of the agent.
        goal: The goal state of the agent.
        returns: N/A
        """
        self.goal = goal

    def set_epsilon(self, epsilon):
        """
        Sets the maximum distance between nodes in rrt.
        epsilon: The maximum distance between nodes in rrt.
        returns: N/A
        """
        self.epsilon = epsilon

    def set_max_nodes(self, max_nodes):
        """
        Sets the max amount of nodes that rrt can generate.
        max_nodes: The maximum amount of nodes.
        returns: N?A
        """
        self.max_nodes = max_nodes

    def astar(self):
        """
        Finds a path using the A* search algorithm.
        returns: A path if there is one otherwise an empty list.
        """
        open_list = PriorityQueue()
        closed_list = set()
        node = ListNode(self.start, None)
        node.set_gscore(0)
        open_list.enqueue(0, node)
        while not open_list.empty():
            current = open_list.dequeue()
            if self.is_goal(current.get_position()):
                return util.reconstruct_path(current)
            if current.get_position() not in closed_list:
                for neighbor in self.generate_neighbors(current.get_position()):
                    if neighbor not in self.world.walkable:
                        continue
                    node = ListNode(neighbor, current)
                    node.set_gscore(current.get_gscore() + util.cost(current.get_position(), node.get_position()))
                    open_list.enqueue(node.get_gscore() + util.heuristic(node.get_position(), self.goal), node)
                closed_list.add(current.get_position())
        return []

    def clear(self):
        """
        Prepares the agent for deletion.
        returns: N/A
        """
        self.traversed.clear()
        self.world.clear()
        del self.traversed
        del self.world

    def create_world(self, xdims, ydims, zdims, filename):
        """
        Creates a representation of the missions world.
        xdims: The dimensions of the world in the x axis (length).
        ydims: The dimensions of the world in the y axis (height).
        zdims: The dimensions of the world in the z axis (width).
        filename: The filename of the file that contains the layout of the mission.
        returns: N/A
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
        p: The point to generate neighbors for.
        returns: A list containing points from.
        """
        neighbors = []
        for dx in DIFF:
            for dy in DIFF:
                for dz in DIFF:
                    n = (p[0]+dx, p[1]+dy, p[2]+dz)
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    if self.world.is_player_move_valid(n):
                        neighbors.append(n)
        return neighbors

    def is_goal(self, p):
        """
        Determines if the specified point is the goal node or not.
        p: The point to check.
        return: True if the point is a goal, False otherwise.
        """
        return p == self.goal

    def line_of_sight(self, p1, p2):
        """
        Determines line of sight from p1 to p2.
        p1: The observing point.
        p2: The observed point.
        return: True if there is line of sight from p1 to p2, False otherwise.
        """
        def points_on_line(q, r):
            """
            Recursively generates all the points on the line defined by q anr r.
            q: The first terminus on the line.
            r: The second terminus on the line.
            return: A list containing all'' the points on the line from q to r.
            """
            if util.dist(q, r) < 1.0:
                return []
            else:
                m = (q[0]+r[0])/2, (q[1]+r[1])/2, (q[2]+r[2])/2
                return points_on_line(q, m) + [m] + points_on_line(m, r)

        for point in points_on_line(p1, p2):
            for obs in self.world.obstacles:
                if obs.point_inside(point):
                    return False
        return True

    def parallel_rrt(self):
        """
        Finds a path utilizing a parallelized version of RRT, which splits the search space into sections, 
        and recombines them for a solution.
        return: A path if it is found, otherwise an empty list.
        """
        def rrt_region(region):
            """
            Runs rrt on the region of space.
            region: Nodes from the search space.
            """
            # Initialize nodes with a random starting location
            tree = TreeNode(region.pop(random.randint(0, len(region)-1)))
            while region:
                # Mini implementation of rrt
                rand = region.pop(random.randint(0, len(region)-1))
                nn = tree[0]
                nn = tree.nearest(rand)
                newnode = self.step_from_to(nn[1], rand)
                temp = TreeNode(newnode)
                nn[1].add_child(temp)
                temp.set_parent(nn[1])
            return tree

        # Create the search regions
        regions = [[], [], [], []]
        for p in self.world.walkable:
            # Region 1
            if (self.world.get_x_lower() < p[0] <= self.world.get_x_upper()//2 and
                        self.world.get_z_upper()//2 < p[2] < self.world.get_z_upper()):
                regions[0].append(p)
            # Region 2
            elif (self.world.get_x_upper()//2 < p[0] < self.world.get_x_upper() and
                        self.world.get_z_upper()//2 < p[2] < self.world.get_z_upper()):
                regions[1].append(p)
            # Region 3
            elif (self.world.get_x_lower() < p[0] <= self.world.get_x_upper()//2 and
                        self.world.get_z_lower() < p[2] <= self.world.get_z_upper()//2):
                regions[2].append(p)
            # Region 4
            elif (self.world.get_x_upper()//2 < p[0] < self.world.get_x_upper() and
                        self.world.get_z_lower() < p[2] <= self.world.get_z_upper()//2):
                regions[3].append(p)
        # Create the pool
        pool = ThreadPool(8)
        # Call the pool
        subtrees = pool.map(rrt_region, regions)
        # Close the pool and wait for work to finish
        pool.close()
        pool.join()
        start_node = None
        goal_node = None
        # Look for the start and goal
        for tree in subtrees:
            if start_node and goal_node:
                break
            if not start_node:
                start_node = tree.find(self.start)
            if not goal_node:
                goal_node = tree.find(self.goal)
        if not start_node or not goal_node:
            return None
        # TODO: Recombine the start and goal nodes to form a path
        # Brute force this guy, sorry!!!!!
        # FIX THIS SOMETIME PLEASE!!!!!!!!!!!!!!!!!!!!!!!!!!
        nearest_to_goal = start_node.nearest(goal_node.get_position())[1]
        goal_start = goal_node
        while goal_start.get_parent():
            goal_start = goal_start.get_parent()
        goal_start = goal_start.nearest(nearest_to_goal.get_position())[1]
        goal_start.set_parent(nearest_to_goal)
        # Return the path
        return util.reconstruct_path(goal_node)

    def random(self):
        """
        Randomly samples a point from walkable space. Has a user defined percentage to sample the goal.
        returns: A randomly sampled point from available walkable space.
        """
        if random.random() < self.goal_probability:
            return self.goal
        return random.sample(self.world.walkable - self.traversed, 1)[0]

    def rrt(self):
        """
        Finds a path in the world using a rapidly exploring random tree.
        returns: A path if it is found, otherwise an empty list.
        """
        def neighbor(points, p):
            """
            Finds the nearest neighbor to a point using Tim Sort.
            points: The tree itself.
            p: The point being added to the tree.
            return: The point nearest to p in the tree.
            """
            points.sort(key=lambda q: (p[0] - q.get_position()[0]) * (p[0] - q.get_position()[0]) +
                                      (p[1] - q.get_position()[1]) * (p[1] - q.get_position()[1]) +
                                      (p[2] - q.get_position()[2]) * (p[2] - q.get_position()[2]))
            return points[0]

        nodes = [ListNode(self.start)] if not BOOL_TREE else TreeNode(self.start)

        for i in range(self.max_nodes):
            rand = self.random()
            nn = None
            newnode = None
            # Runs Tim sort on the CPU to find nearest neighbor
            if not BOOL_GPU:
                if not BOOL_TREE:
                    nn = neighbor(nodes, rand)
                    newnode = self.step_from_to(nn.get_position(), rand)
                    nodes.append(ListNode(newnode, nn))
                    if self.is_goal(nodes[-1].get_position()):
                        return util.reconstruct_path(nodes[-1])
                else:
                    nn = nodes.nearest(rand)
                    newnode = self.step_from_to(nn[1], rand)
                    temp = TreeNode(newnode)
                    nn[1].add_child(temp)
                    temp.set_parent(nn[1])
                    self.traversed.add(newnode)
                    if self.is_goal(temp.get_position()):
                        return util.reconstruct_path(temp)
            # Runs NN on the GPU
            else:
                # TODO: Need to convert nodes over to a list of tuples
                # Note mod has the following signature
                #   float3 point, float3 * neighbors, float3 * results
                neighbors = gpuarray.to_gpu(numpy.asarray([tuple(n) for n in nodes]))
                results = gpuarray.zeros_like(neighbors, dtype=gpuarray.vec.float3)
                mod(gpuarray.vec.make_float3(*rand), neighbors, results)
                # TODO: Need to get the data passed into the gpu in a way
                #   it can work with
            self.traversed.add(newnode)
            # From Steven M. Lavalles implementation
            #for p in nodes:
            #    if dist(p.get_position(), rand) < dist(nn.get_position(), rand):
            #        nn = p

        return []

    def step_from_to(self, p1, p2):
        """
        Adapted from Steven M. Lavalles implementation of RRT and lifted into three dimensions.
        p1: The point being sampled from.
        p2: The point that was randomly sampled.
        returns: Either the randomly sampled point or a point along the line from p1 to p2.
        """
        if util.dist(p1, p2) < self.epsilon:
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

def gather_data(alg=0, iterations=1000):
    """
    Gathers data on the pathfinding algorithms.
    alg: The algorithm to use, 0 for A*. anything else for RRT.
    map_number: The map number, an exception is raised if it is not between 0 and the total number of maps.
    iterations: The sample size.
    return: N/A
    """
    agent = None
    out1 = open(get_file_name(0, alg), 'w')
    out2 = open(get_file_name(1, alg), 'w')
    out3 = open(get_file_name(2, alg), 'w')
    out4 = open(get_file_name(3, alg), 'w')
    header = "Run Time,Path Length,Heading Changes,Total Degrees (Degrees)\n"
    out1.write(header)
    out2.write(header)
    out3.write(header)
    out4.write(header)
    for i in range(iterations):
        for j in range(len(MISSION_FILES)):
            # Attempt to start a mission:
            agent = Agent(MISSION_START, MISSION_GOALS[j])
            agent.create_world((MISSION_LOWER_BOUNDS[j][0], MISSION_UPPER_BOUNDS[j][0]),
                               (MISSION_LOWER_BOUNDS[j][1], MISSION_UPPER_BOUNDS[j][1]),
                               (MISSION_LOWER_BOUNDS[j][2], MISSION_UPPER_BOUNDS[j][2]),
                               MISSION_TEXT_FILES[j])
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
            pathlength = util.dist(agent.start, path[0])
            degrees = util.degree_change(agent.start, path[0])
            hchanges = 1 if degrees != 0 else 0
            for k in range(1, len(path)):
                pathlength += util.dist(path[k - 1], path[k])
                change = util.degree_change(path[k - 1], path[k])
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

    for i in range(len(MISSION_FILES)):
        # Attempt to start a mission:
        my_mission = MalmoPython.MissionSpec(open(MISSION_FILES[i], 'r').read(), False)
        planner = Agent(MISSION_START, MISSION_GOALS[i])
        planner.create_world((MISSION_LOWER_BOUNDS[i][0], MISSION_UPPER_BOUNDS[i][0]),
                               (MISSION_LOWER_BOUNDS[i][1], MISSION_UPPER_BOUNDS[i][1]),
                               (MISSION_LOWER_BOUNDS[i][2], MISSION_UPPER_BOUNDS[i][2]),
                               MISSION_TEXT_FILES[i])
        path = planner.rrt()

        if BOOL_DEBUG_PATHS:
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
    #if BOOL_VISUALIZE:
    #    visualize()
    #else:
    #    gather_data(1)
    agent = None
    for i in range(1):
        for j in range(len(MISSION_FILES)):
            # Attempt to start a mission:
            agent = Agent(MISSION_START, MISSION_GOALS[j])
            agent.create_world((MISSION_LOWER_BOUNDS[j][0], MISSION_UPPER_BOUNDS[j][0]),
                               (MISSION_LOWER_BOUNDS[j][1], MISSION_UPPER_BOUNDS[j][1]),
                               (MISSION_LOWER_BOUNDS[j][2], MISSION_UPPER_BOUNDS[j][2]),
                               MISSION_TEXT_FILES[j])
            rt = time.clock()
            path = agent.parallel_rrt()
            print(path, time.clock()-rt)