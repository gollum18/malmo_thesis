import MalmoPython
import math
import os
import time
import sys
import random
import json

#######################################
# UTILITY FUNCTIONS
#######################################

def distance(p1, p2):
    """
    Determines the 3D euclidean distance between two points.
    :param p1: The first point.
    :param p2: The second point.
    :return: The euclidean distance between the two points.
    """
    return math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1])+(p2[2]-p1[2])*(p2[2]-p1[2]))

#######################################
# CLASSES
#######################################

class RealTimeRapidlyExploringTreeNode:

    #######################################
    # CONSTRUCTOR
    #######################################

    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

    #######################################
    # ACCESSORS/MUTATORS
    #######################################

    def get_position(self):
        return self.position

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent

class RealTimeRapidlyExploringTreeAgent:

    #######################################
    # CONSTRUCTOR
    #######################################

    def __init__(self,
                 start=(0, 0, 0),
                 goal=(5, 0, 5),
                 xdims=(-10, 10),
                 ydims=(-5, 5),
                 zdims=(-10, 10),
                 p_line=.10,
                 p_goal=.05,
                 m_decider=3.25,
                 num_nodes=5000,
                 xradius=1,
                 yradius=1,
                 zradius=1,
                 g_tolerance=0.5,
                 max_distance=7.0,
                 obsx=1,
                 obsy=1,
                 obsz=1,
                 hazx=1,
                 hazy=1,
                 hazz=1,
                 movement_distance=1):
        self.start = start
        self.goal = goal
        self.xdims = xdims
        self.ydims = ydims
        self.zdims = zdims
        self.line_probability = p_line
        self.goal_probability = p_goal
        self.randomizer = m_decider
        self.nodes = num_nodes
        self.radii = xradius, yradius, zradius
        self.goal_tolerance = g_tolerance
        self.max_distance = max_distance
        self.obs_dims = obsx, obsy, obsz
        self.haz_dims = hazx, hazy, hazz
        self.movement_distance = movement_distance
        self.obstacles = {}
        self.hazards = {}

    #######################################
    # ACCESSORS/MUTATORS
    #######################################

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def get_xdims(self):
        return self.xdims

    def get_ydims(self):
        return self.ydims

    def get_zdims(self):
        return self.zdims

    def get_xmin(self):
        return min(self.xdims)

    def get_xmax(self):
        return max(self.xdims)

    def get_ymin(self):
        return min(self.ydims)

    def get_ymax(self):
        return max(self.ydims)

    def get_zmin(self):
        return min(self.zdims)

    def get_zmax(self):
        return max(self.zdims)

    def get_line_probability(self):
        return self.line_probability

    def get_goal_probability(self):
        return self.goal_probability

    def get_randomizer(self):
        return self.randomizer

    def get_max_nodes(self):
        return self.nodes

    def get_xradius(self):
        return self.radii[0]

    def get_yradius(self):
        return self.radii[1]

    def get_zradius(self):
        return self.radii[2]

    def get_goal_tolerance(self):
        return self.goal_tolerance

    def get_max_branch_distance(self):
        return self.max_distance

    def get_obstacle_dimensions(self):
        return self.obs_dims

    def get_hazard_dimensions(self):
        return self.haz_dims

    def get_movement_distance(self):
        return self.movement_distance

    def get_obstacles(self):
        return self.obstacles

    def get_hazards(self):
        return self.hazards

    #######################################
    # METHODS
    #######################################

    def add_hazard(self, p):
        """
        Adds a hazard to the search region.
        :param p: The bottom, lower-left point of the hazard.
        :return: N/A
        """
        if p[1] not in self.hazards.keys():
            self.hazards[p[1]] = list((p[0], p[2]))
        else:
            self.hazards[p[1]].append((p[0], p[2]))

    def add_obstacle(self, p):
        """
        Adds an obstacle to the search region.
        :param p: The bottom, lower-left point of the obstacle.
        :return: N/A
        """
        if p[1] not in self.obstacles.keys():
            self.obstacles[p[1]] = list((p[0], p[2]))
        else:
            self.obstacles[p[1]].append((p[0], p[2]))

    def ellipsoid(self):
        """
        Samples an ellipsoidal search region around the agent.
        :return: A point from the region immediately surrounding the agent.
        """
        theta = random.uniform(-math.pi / 2, math.pi / 2)
        fi = random.uniform(-math.pi, math.pi)
        return (self.radii[0] * math.cos(theta) * math.cos(fi),
                self.radii[1] * math.cos(theta) * math.sin(fi),
                self.radii[2] * math.sin(theta))

    def explore(self):
        """
        Finds a path utilizing a rapidly-exploring random tree.
        :return: The path if it is found, or an empty list if it is not.
        """

        def find_nearest_neighbor(node):
            """
            Finds the nearest neighbor across all nodes in the search tree to the specified node.
            :param node: The node to find nearest neighbor of.
            :return: The node in the search tree that is closest to the specified node.
            """
            best_node = nodes[0]
            best_distance = distance(nodes[0].get_position(), node)
            for j in range(1, len(nodes)):
                if nodes[j].get_position() == node.get_position():
                    continue
                v = distance(nodes[j].get_position(), node)
                if v < best_distance:
                    best_node = nodes[j]
                    best_distance = v
            return best_node

        nodes = [RealTimeRapidlyExploringTreeNode(self.get_start())]

        for i in range(self.get_max_nodes()):
            rand = self.uniform()
            nn = nodes[0]
            for p in nodes:
                if distance(p.get_position(), rand) < distance(nn.get_position(), rand):
                    nn = p
            nodes.append(RealTimeRapidlyExploringTreeNode(self.sample(nn.get_position(), rand), nn))
            if self.is_goal(nodes[-1].get_position()):
                path = []
                current = nodes[-1]
                while current.get_parent():
                    path.append(current.get_position())
                    current = current.get_parent()
                path.reverse()
                return path
        return []

    def in_bounds(self, p):
        """
        Determines if the point is in bounds of the search space.
        :param p: The point to check.
        :return: True if the point is in bounds, False otherwise.
        """
        if (self.get_xmin() <= p[0] <= self.get_xmax() and
                self.get_ymin() <= p[1] <= self.get_ymax() and
                self.get_zmin() <= p[2] <= self.get_zmax()):
            return True
        return False

    def is_blocked(self, p):
        """
        Determines if the point is a hazard or an obstacle.
        :param p: The point to check.
        :return: True if the point is an obstacle or hazard, False otherwise.
        """
        if self.is_blocked(p) or self.is_hazard(p):
            return True
        return False

    def is_goal(self, p):
        """
        Determines if a point is within the bounds of the goal or not.
        :param p: The point to check.
        :return: True if the point falls within the bounds of the goal, False otherwise.
        """
        x, y, z = p
        if (self.goal[0] - self.goal_tolerance <= x <= self.goal[0] + self.goal_tolerance and
                self.goal[1] - self.goal_tolerance <= y <= self.goal[1] + self.goal_tolerance and
                self.goal[2] - self.goal_tolerance <= z <= self.goal[2] + self.goal_tolerance):
            return True
        return False

    def is_hazard(self, p):
        """
        Determines if the point is a hazard.
        :param p: The point to check.
        :return: True if the point is a hazard, False otherwise.
        """
        for hazard in self.hazards[p[1]]:
            if (hazard[0] <= p[0] <= hazard[0] + self.haz_dims[0] and
                    hazard[1] <= p[1] <= hazard[1] + self.haz_dims[1]):
                return True
        return False

    def is_obstacle(self, p):
        """
        Determines if the point is an obstacle.
        :param p: The point to check.
        :return: True if the point is an obstacle, False otherwise.
        """
        for obstacle in self.obstacles[p[1]]:
            if (obstacle[0] <= p[0] <= obstacle[0] + self.obs_dims[0] and
                    obstacle[1] <= p[1] <= obstacle[1] + self.obs_dims[1]):
                return True
        return False

    def line_of_sight(self, p1, p2):
        """
        Determines line of sight between p1 and p2.
        :param p1: The looking point.
        :param p2: The observed point.
        :return: True if there is line of sight from p1 to p2.
        """
        x, y, z = p1
        # Incrementally march p1 to p2 by a 1 unit distance each time, checking for obstacles as we go.
        while (x, y, z) != p2:
            x, y, z = self.point_on_line(p1, p2)
            if self.is_blocked((x, y, z)):
                return False
        return True

    def line_to(self, p1, p2):
        """
        Gets a point on the line between p1 and p2.
        :param p1: The origin point of the line.
        :param p2: The end point of the line.
        :return: A point on the line between p1 and p2.
        """
        if distance(p1, p2) < self.max_distance:
            return p2
        else:
            return self.point_on_line(p1, p2,
                                      random.uniform(0.0, self.max_distance),
                                      random.uniform(0.0, self.max_distance),
                                      random.uniform(0.0, self.max_distance))

    def point_on_line(self, p1, p2, dx=1, dy=1, dz=1):
        """
        Determines a point on a line in 3D.
        :param p1: The origin point of the line.
        :param p2: The end point of the line.
        :param dx: Distance to move in terms of dx.
        :param dy: Distance to move in terms of dy.
        :param dz: Distance to move in terms of dz.
        :return: A point on the line between p1 and p2.
        """
        v = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
        l = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        uv = v[0] / l, v[1] / l, v[2] / l
        x = p1[0] + dx * uv[0]
        y = p1[1] + dy * uv[1]
        z = p1[2] + dz * uv[2]
        return x, y, z

    def random(self, xdim, ydim, zdim):
        """
        Gets a random point in the search region.
        :param xdim: The x-dimensions to search within.
        :param ydim: The y-dimensions to search within.
        :param zdim: The z-dimensions to search within.
        :return: A random point in the specified search space.
        """
        return (random.uniform(min(xdim), max(xdim)),
                random.uniform(min(ydim), max(ydim)),
                random.uniform(min(zdim), max(zdim)))

    def sample(self, p1, p2):
        """
        Samples the search region for a new point in the tree.
        :param p1: The parent point.
        :param p2: A randomly chosen point from the search space.
        :return: A sampled point from the search space.
        """
        p = random.random()
        if p >= 1-self.get_goal_probability():
            return self.line_to(p1, self.get_goal())
        elif p <= 1-self.get_line_probability():
            return self.line_to(p1, p2)
        elif p <= (1-self.get_line_probability()/self.get_randomizer()):
            return self.uniform()
        else:
            x, y, z = self.ellipsoid()
            return p1[0] + x, p1[1] + y, p1[2] + z

    def uniform(self):
        """
        Returns a uniformly sampled point in the search region.
        :return: A uniformly sampled point from the search region.
        """
        return self.random(self.get_xdims(), self.get_ydims(), self.get_zdims())

#######################################
# TESTING METHODS
#######################################

def rrt_test():
    agent = RealTimeRapidlyExploringTreeAgent()
    print agent.explore()

if __name__ == '__main__':
    rrt_test()