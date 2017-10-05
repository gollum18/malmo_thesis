# Name: agent_rrt.py
# Since: 08/21/2017
# Modified: 10/05/2017
# Description: Defines an agent that traverses three dimensional space using a rapidly exploring random tree.

from __future__ import division
import MalmoPython
import math
import os
import sys
import time
import random

mission_files = ('./missions/pp_maze_one.xml',
                 './missions/pp_maze_two.xml',
                 './missions/pp_maze_three.xml',
                 './missions/pp_maze_four.xml')
mission_text_files = ('./missions/pp_maze_one_txt.txt',
                      './missions/pp_maze_two_txt.txt',
                      './missions/pp_maze_three_txt.txt',
                      './missions/pp_maze_four_txt.txt')


class Agent(object):

    def __init__(self, start, goal, alpha, beta, max_nodes, int_steps):
        """
        Creates an agent instance.
        :param start: The starting position of the agent.
        :param goal: The goal position of the agent.
        :param alpha: The probability of sampling the goal.
        :param beta: The probability of sampling using ellipsoid.
        :param max_nodes: The maxmimum amount of nodes generated by the agent.
        """
        self.start = start
        self.goal = goal
        self.alpha = alpha
        self.beta = beta
        self.max_nodes = max_nodes
        self.int_steps = int_steps
        self.world = None
        self.traversed = set()

    def get_start(self):
        """
        Gets the starting position of the agent.
        :return: A tuple (x, y, z) containing the starting position of the agent.
        """
        return self.start

    def get_goal(self):
        """
        Gets the goal position of the agent.
        :return: A tuple (x, y, z) containing the goal position of the agent.
        """
        return self.goal

    def get_alpha(self):
        """
        Gets the probability of sampling the goal.
        :return: A probability p | 0 <= p <= 1.
        """
        return self.alpha

    def get_beta(self):
        """
        Gets the probability of sampling using ellipsoid.
        :return: A probability p | 0 <= p <= 1.
        """
        return self.beta

    def get_max_nodes(self):
        """
        Gets the maximum amount of nodes the agent is allowed to generate.
        :return: An integer x | x > 0.
        """
        return self.max_nodes

    def get_int_steps(self):
        """
        Gets the amount of steps taken when interpolating.
        :return: An integer x | x > 0.
        """
        return self.int_steps

    def create_world(self, xdims, ydims, zdims, filename):
        """

        :param xdims: The x dimensions of the world.
        :param ydims: The y dimensions of the world.
        :param zdims: The z dimensions of the world.
        :param filename: The name of the file containing all obstacles and walkable space.
        :return: N/A
        """
        if not xdims or not ydims or not zdims or not filename:
            raise ValueError
        reader = open(filename, 'r')
        if not reader:
            raise IOError
        # TODO: Parse in the information about the obstacles and walkable space.
        raise NotImplementedError

    @staticmethod
    def distance(p1, p2):
        """
        Calculates the distance between two points in euclidean three space.
        :param p1: A tuple (x, y, z).
        :param p2: A tuple (x, y, z).
        :return: The euclidean distance between p1 and p2.
        """
        return math.sqrt(((p2[0] - p1[0]) * (p2[0] - p1[0])) +
                         ((p2[1] - p1[1]) * (p2[1] - p1[1])) +
                         ((p2[2] - p1[2]) * (p2[2] - p1[2])))

    def ellipsoid(self, p, xr=3.0, yr=3.0, zr=3.0):
        """
        Samples a point from the ellipsoidal search region around point p.
        :param p: A tuple (x, y, z).
        :param xr: A float x | x >= 1.0
        :param yr: A float x | x >= 1.0
        :param zr: A float x | x >= 1.0
        :return: A tuple (x, y, z).
        """
        def inside(x, y, z):
            """
            Determines if a set of coordinates fall within the ellipsoidal search region.
            :param x: An integer x | worlds minimum x dimension <= x < worlds maximum x dimension.
            :param y: An integer t | worlds minimum y dimension <= y < worlds maximum y dimension..
            :param z: An integer z | worlds minimum z dimension <= z < worlds maximum z dimension.
            :return: True if the coordinate falls within the ellipsoidal search region, False otherwise.
            """
            return (((x-p[0])/xr)*((x-p[0])/xr) +
                    ((y-p[1])/yr)*((y-p[1])/yr) +
                    ((z-p[2])/zr)*((z-p[2])/zr)) < 1
        space = []
        for p in self.world.get_walkable():
            if inside(*p):
                space.append(p)
        return self.line_to(p, self.nearest_neighbor(space, self.goal))

    def explore(self):
        """
        Finds a path using a rapidly exploring random tree.
        :return: A list containing the path, or [] if there is no path.
        """
        nodes = []
        for i in range(self.max_nodes):
            r = self.uniform()
            nn = nodes[0]
            for p in nodes:
                if self.distance(p.get_position(), r) < self.distance(nn.get_position, r):
                    nn = p
            nodes.append(Node(self.sample(nn.get_position()), nn))
            if self.is_goal(nodes[-1]):
                path = []
                current = nodes[-1]
                while current.get_parent():
                    path.append(current.get_position())
                    current = current.get_parent()
                return path
        return []

    @staticmethod
    def interpolate(p1, p2, t, r=False):
        """
        Interpolates along the line p1->p2.
        :param p1: A tuple (x, y, z).
        :param p2: A tuple (x, y, z).
        :param t: A float x | 0 <= x <= 1.
        :param r: A boolean indicating whether to floor the y coordinate.
        :return: A point (x, y, z). Note: y will be floored if r is True.
        """
        x = (1 - t) * (p1[0] + (t * p2[0]))
        y = (1 - t) * (p1[1] + (t * p2[1]))
        z = (1 - t) * (p1[2] + (t * p2[2]))
        return (x, y, z) if not r else (x, math.floor(y), z)

    def is_goal(self, p):
        """
        Determines if a point meets the goal conditions.
        :param p: A tuple (x, y, z).
        :return: True if a point meets the goal conditions, False otherwise.
        """
        return (self.goal[0] - .5 <= p[0] < self.goal[0] + .5 and
                self.goal[1] == p[1] and
                self.goal[2] - .5 <= p[0] < self.goal[2] + .5)

    def line_of_sight(self, p1, p2):
        """
        Determines line of sight from p1 to p2.
        :param p1: A tuple (x, y, z).
        :param p2: A tuple (x, y, z).
        :return: True if there is line of sight between p1 and p2, False otherwise.
        """
        t = 0
        while t < self.int_steps:
            p3 = self.interpolate(p1, p2, t / self.int_steps)
            if not self.world.is_valid(p3):
                return False
            t += 1
        return True

    def line_to(self, p1, p2):
        """
        Samples a point along the line p1->p2.
        :param p1: A tuple (x, y, z).
        :param p2: A tuple (x, y, z).
        :return: A tuple (x, y, z).
        """
        t = 0
        space = set()
        space.add(p1)
        while t < self.int_steps:
            p3 = self.interpolate(p1, p2, t / self.int_steps, True)
            if self.world.is_walkable(p3):
                space.add(p3)
            t += 1
        space.add(p2)
        return random.sample(space - self.traversed, 1)[0]

    def nearest_neighbor(self, nodes, p):
        """
        Finds the nearest neighbor amongst the collection of nodes to point p.
        :param nodes: An iterable containing tuples (x, y, z).
        :param p: A tuple (x, y, z).
        :return: The point closest to p amongst all the points in nodes.
        """
        v = self.distance(nodes[0], p)
        best = nodes[0]
        for node in nodes[1:-1]:
            temp = self.distance(node, p)
            if temp < v:
                v = temp
                best = node
        return best

    def sample(self, p1):
        """
        Samples a point from walkable space using a probabilistic algorithm.
        :param p1: A tuple (x, y, z).
        :return: A tuple (x, y, z).
        """
        p = random.random()
        if p > 1 - self.alpha:
            return self.line_to(p1, self.goal)
        elif p < (1 - self.alpha)/self.beta or not self.line_of_sight(p1, self.goal):
            return self.uniform()
        else:
            return self.ellipsoid(p1)

    def uniform(self):
        """
        Uniformly samples a point from walkable space.
        :return: A tuple (x, y, z).
        """
        return random.sample(self.world.get_walkable() - self.traversed, 1)[0]


class Node(object):

    def __init__(self, position, parent=None):
        """
        Creates an instance of a node object.
        :param position: The position of this node.
        :param parent: The parent of this node.
        """
        self.position = position
        self.parent = parent

    def get_parent(self):
        """
        Gets the parent node of this node.
        :return: A node object or None if it does not have a parent.
        """
        return self.parent

    def get_position(self):
        """
        Gets the position of this node.
        :return: A tuple (x, y, z).
        """
        return self.position

    def set_position(self, position):
        """
        Sets the position of this node.
        :param position: A tuple (x, y, z).
        :return: N/A
        """
        self.position = position


class Obstacle(object):

    _obstacle_id = 0

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of an obstacle object.
        :param xdims: The x dimensions of the object.
        :param ydims: The y dimensions of the object.
        :param zdims: The z dimensions of the object.
        """
        self.xdims = xdims
        self.ydims = ydims
        self.zdims = zdims
        self.obstacle_id = Obstacle._obstacle_id
        Obstacle._obstacle_id += 1

    def __eq__(self, other):
        """
        Determines if this obstacle is equal to another.
        :param other: The other obstacle to compare against.
        :return: True if the obstacles share the same obstacle ID, False otherwise.
        """
        return self.obstacle_id == other.obstacle_id

    def __ne__(self, other):
        """
        Determines if this obstacle is not equal to another.
        :param other: The other obstacle to compare against.
        :return: True if the obstacles do not share the same obstacle ID, False otherwise.
        """
        return not self.__eq__(other)

    def __hash__(self):
        """
        Creates a hash for this obstacle.
        :return: An integer.
        """
        return hash(self.obstacle_id)

    def get_xmin(self):
        """
        Gets the minimum x dimension of this obstacle.
        :return: An integer.
        """
        return min(self.xdims)

    def get_xmax(self):
        """
        Gets the maximum x dimension of this obstacle.
        :return: An integer.
        """
        return max(self.xdims)

    def get_ymin(self):
        """
        Gets the minimum y dimension of this obstacle.
        :return: An integer.
        """
        return min(self.ydims)

    def get_ymax(self):
        """
        Gets the maximum y dimension of this obstacle.
        :return: An integer.
        """
        return max(self.ydims)

    def get_zmin(self):
        """
        Gets the minimum z dimension of this obstacle.
        :return: An integer.
        """
        return min(self.zdims)

    def get_zmax(self):
        """
        Gets the maximum z dimenison of this obstacle.
        :return: An integer.
        """
        return max(self.zdims)

    def set_xdims(self, xdims):
        """
        Sets the x dimensions of the world.
        :param xdims: A tuple (min-x, max-x)
        :return: N/A
        """
        self.xdims = xdims

    def set_ydims(self, ydims):
        """
        Sets the y dimensions of the world.
        :param ydims: A tuple (min-y, max-y)
        :return: N/A
        """
        self.ydims = ydims

    def set_zdims(self, zdims):
        """
        Sets the z dimensions of the world.
        :param zdims: A tuple (min-z, max-z)
        :return: N/A
        """
        self.zdims = zdims

    def inside(self, p):
        """
        Determines if a point lies inside this obstacle.
        :param p: A tuple (x, y, z).
        :return: True if the point lies inside the obstacle, False otherwise.
        """
        return (self.get_xmin() <= p[0] < self.get_xmax() and
                self.get_ymin() <= p[1] < self.get_ymax() and
                self.get_zmin() <= p[2] < self.get_zmax())


class World(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates a world instance.
        :param xdims: The x dimensions of the world.
        :param ydims: The y dimensions of the world.
        :param zdims: The z dimensions of the world.
        """
        self.xdims = xdims
        self.ydims = ydims
        self.zdims = zdims
        self.obstacles = set()
        self.walkable = dict()

    def get_xmin(self):
        """
        Gets the minimum x dimension of the world.
        :return: An integer x | x < max(x dimensions).
        """
        return min(self.xdims)

    def get_xmax(self):
        """
        Gets the maximum y dimension of the world.
        :return: An integer y | y > min(y dimensions).
        """
        return max(self.xdims)

    def get_ymin(self):
        """
        Gets the minimum z dimension of the world.
        :return: An integer z | z < max(z dimensions).
        """
        return min(self.ydims)

    def get_ymax(self):
        """
        Gets the maximum x dimension of the world.
        :return: An integer x | x > min(x dimensions).
        """
        return max(self.ydims)

    def get_zmin(self):
        """
        Gets the minimum y dimension of the world.
        :return: An integer y | y < max(y dimensions).
        """
        return min(self.zdims)

    def get_zmax(self):
        """
        Gets the maximum z dimension of the world.
        :return: An integer z | z > min(z dimensions).
        """
        return max(self.zdims)

    def get_obstacles(self):
        """
        Gets a list containing all the obstacles.
        :return: A list containing all obstacles.
        """
        return self.obstacles

    def get_walkables(self):
        """
        Gets a set containing all walkable space.
        :return: A set containing all walkable space.
        """
        walkable = set()
        for y in self.walkable.keys():
            for x, z in self.walkable[y]:
                walkable.add((x, y, z))
        return walkable

    def empty(self):
        """
        Clears this worlds obstacles and walkable space.
        :return: N/A
        """
        self.obstacles.clear()
        self.walkable.clear()

    def in_bounds(self, p):
        """
        Determines if a point is inside the world.
        :param p: A tuple (x, y, z).
        :return: True if the point is within the world, False otherwise.
        """
        return (self.get_xmin() <= p[0] < self.get_xmax() and
                self.get_ymin() <= p[1] < self.get_ymax() and
                self.get_zmin() <= p[2] < self.get_zmax())

    def is_blocked(self, p):
        """
        Determines if a point is inside an obstacle.
        :param p: A tuple (x, y, z).
        :return: True if the point is within an obstacle in the world, False otherwise.
        """
        for obstacle in self.obstacles:
            if obstacle.inside(p):
                return True
        return False

    def is_valid(self, p):
        """
        Determines if a point is valid such that it is in bounds and outside an obstacle.
        :param p: A tuple (x, y, z).
        :return: True if the point is valid, False otherwise.
        """
        return self.in_bounds(p) and not self.is_blocked(p)

    def is_walkable(self, p):
        """
        Determines whether a point is walkable.
        :param p: A tuple (x, y, z).
        :return: True if the point is valid, False otherwise.
        """
        if p[1] in self.walkable.keys():
            return (p[0], p[2]) in self.walkable[p[1]]
        return False


def malmo():
    agent_host = MalmoPython.AgentHost()
    try:
        agent_host.parse(sys.argv)
    except RuntimeError as e:
        print 'ERROR', e
        print agent_host.getUsage()
        exit(1)
    if agent_host.receivedArgument("help"):
        print agent_host.getUsage()
        exit(0)

    for i in range(4):
        # Setup the mission parameters
        mission_file = mission_files[i]
        mission_txt_file = mission_text_files[i]

        with open(mission_file, 'r') as f:
            print "Loading mission from %s" % mission_file
            mission_xml = f.read()
            my_mission = MalmoPython.MissionSpec(mission_xml, True)
        my_mission_record = MalmoPython.MissionRecordSpec()

        # Attempt to start the mission
        max_retries = 3
        for retry in range(max_retries):
            try:
                agent_host.startMission(my_mission, my_mission_record)
            except RuntimeError as e:
                if retry == max_retries - 1:
                    print "Error starting mission:", e
                    exit(1)
                else:
                    time.sleep(2)

        # Loop until mission starts:
        print "Waiting for the mission to start ",
        world_state = agent_host.getWorldState()
        while not world_state.has_mission_begun:
            sys.stdout.write(".")
            time.sleep(0.5)
            world_state = agent_host.getWorldState()
            for error in world_state.errors:
                print "Error:", error.text

        print
        print "Mission running ",

        # Loop until mission ends:
        while world_state.is_mission_running:
            sys.stdout.write(".")
            time.sleep(0.5)
            world_state = agent_host.getWorldState()
            for error in world_state.errors:
                print "Error:", error.text

        print
        print "Mission ended"
        # Mission has ended.

    raise NotImplementedError

if __name__ == '__main__':
    malmo()