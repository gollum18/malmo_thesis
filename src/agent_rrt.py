from __future__ import division
import MalmoPython
import math
import random
import time

############################
##  UTILITY FUNCTIONS
############################


def distance(p1, p2):
    """
    Determines the distance between two three-dimensional points.
    :param p1: The first point.
    :param p2: The second point.
    :return: The distance between the two points.
    """
    return math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1])+(p2[2]-p1[2])*(p2[2]-p1[2]))


def random_point(lbx, ubx, lby, uby, lbz, ubz):
    """
    Randomly samples a point based on some bounds.
    :param lbx: The lower x bound.
    :param ubx: The upper x bound.
    :param lby: The lower y bound.
    :param uby: The upper y bound.
    :param lbz: THe lower z bound.
    :param ubz: The upper z bound.
    :return: A point that has been sampled from the region specified by the bounds.
    """
    return (random.uniform(lbx, ubx),
            random.uniform(lby, uby),
            random.uniform(lbz, ubz))


def reconstruct_path(node):
    """
    Reconstructs the path the agent should follow to get to the goal.
    :param node: The node that meets the goal state requirements.
    :return: The path the agent should follow.
    """
    path = [node.get_position()]

    while node.get_parent():
        node = node.get_parent()
        path.append(node.get_position())
    path.reverse()

    return path


class RRTNode:

    ############################
    ##  CONSTRUCTOR
    ############################

    def __init__(self, position, parent=None):
        """
        Creates a node for a rapidly exploring random tree.
        :param position:
        """
        self.position = position
        self.parent = parent

    ############################
    ##  ACCESSORS/MUTATORS
    ############################

    def get_parent(self):
        """
        Gets the parent of this node.
        :return: The parent node.
        """
        return self.parent

    def get_position(self):
        """
        Gets the position of this node.
        :return: The position of this node.
        """
        return self.position


class RRTAgent:

    ############################
    ##  CONSTRUCTOR
    ############################

    def __init__(self, start=(0, 0, 0), goal=(10, 5, 10),
                 xdim=(-10, 10), ydim=(-5, 5), zdim=(-10, 10),
                 alpha=0.10, beta=3.25, delta=0.05,
                 epsilon=7.0, kappa=5000, sigma=7.0,
                 tau=7.0, upsilon=7.0, charlie=0.5):
        """
        Creates an agent capable of generating rapidly exploring random trees over a sample space.
        This rapidly exploring random tree differs from other variants as it generates integer coordinates as opposed to
        floating point coordinates.
        :param start: The starting position of the agent.
        :param goal: The goal position of the agent.
        :param xdim: The x dimensions of the search space.
        :param ydim: The y dimensions of the search space.
        :param zdim: The z dimensions of the search space.
        :param alpha: The probability of sampling using the line-to algorithm. Default is 0.10.
        :param beta: The modifier used when sampling with the ellipsoid/uniform algorithms. Default is 3.25.
        :param delta: The probability of sampling towards the goal. Default is 0.05.
        :param epsilon: The max distance allowed between a parent node and child node. Default is 7.0.
        :param kappa: The maximum number of nodes the tree is allowed to generate per search. Default is 5,000.
        :param sigma: The radius of the sampling ellipsoid along the x plane.
        :param tau:: The radius of the sampling ellipsoid along the y plane.
        :param upsilon: The radius of the sampling ellipsoid along the z plane.
        :param charlie: The goal check tolerance. Default is 0.5.
        """
        self.root = RRTNode(start)
        self.start = start
        self.goal = goal
        self.xdim = xdim
        self.ydim = ydim
        self.zdim = zdim
        self.alpha = alpha
        self.beta = beta
        self.delta = delta
        self.epsilon = epsilon
        self.kappa = kappa
        self.charlie = charlie
        self.radii = sigma, tau, upsilon
        self.obstacles = {}

    ############################
    ##  ACCESSORS/MUTATORS
    ############################

    def get_root(self):
        """
        Returns the root node of the tree.
        :return: The root node of the tree.
        """
        return self.root

    def get_start(self):
        """
        Gets the starting position of the agent.
        :return: The starting position of the agent.
        """
        return self.start

    def get_goal(self):
        """
        Gets the goal position of the agent.
        :return: The goal position of the agent.
        """
        return self.goal

    def get_xmin(self):
        """
        Gets the minimum x dimension of the search space.
        :return: The minimum x dimension of the search space.
        """
        return min(self.xdim)

    def get_xmax(self):
        """
        Gets the maximum x dimension of the search space.
        :return: The maximum x dimension of the search space.
        """
        return max(self.xdim)

    def get_ymin(self):
        """
        Gets the minimum y dimension of the search space.
        :return: The minimum y dimension of the search space.
        """
        return min(self.ydim)

    def get_ymax(self):
        """
        Gets the maximum y dimension of the search space.
        :return: The maximum y dimension of the search space.
        """
        return max(self.ydim)

    def get_zmin(self):
        """
        Gets the minimum z dimension of the search space.
        :return: The minimum z dimension of the search space.
        """
        return min(self.zdim)

    def get_zmax(self):
        """
        Gets the maximum z dimension of the search space.
        :return: The maximum z dimension of the search space.
        """
        return max(self.zdim)

    def get_line_probability(self):
        """
        Gets the probability of utilizing the line-to sampling algorithm.
        :return: The probability of the line-to sampling algorithm.
        """
        return self.alpha

    def get_ellipsoid_uniform_modifier(self):
        """
        Gets the modifier used when sampling with either the ellipsoid or uniform sampling algorithms.
        :return: The modifier of sampling using ellipsoid of uniform algorithms.
        """
        return self.beta

    def get_goal_probability(self):
        """
        Gets the probability of sampling towards the goal.
        :return: Probability of sampling towards the goal.
        """
        return self.delta

    def get_goal_tolerance(self):
        """
        Gets the goal tolerance.
        :return: The goal tolerance.
        """
        return self.charlie

    def get_max_branch_distance(self):
        """
        Gest the maximum distance allowed between parent and child nodes.
        :return: The maximum distance allowed between parent and child nodes.
        """
        return self.epsilon

    def get_max_nodes(self):
        """
        Gets the maximum number of nodes allowed in a single search instance.
        :return: The maximum number of nodes allowed in a single search instance.
        """
        return self.kappa

    def get_x_radius(self):
        """
        Gets the x radius of the ellipsoidal search space surrounding the agent.
        :return: The x radius.
        """
        return self.radii[0]

    def get_y_radius(self):
        """
        Gets the y radius of the ellipsoidal search space surrounding the agent.
        :return: The y radius.
        """
        return self.radii[1]

    def get_z_radius(self):
        """
        Gest the z radius of the ellipsoidal search space surrounding the agent.
        :return: The z radius.
        """
        return self.radii[2]

    def get_obstacles(self, y):
        return self.obstacles[y]

    ############################
    ##  METHODS
    ############################

    def add_obstacle(self, p):
        """
        Adds an obstacle to the obstacles list.
        Obstacles are stored by height level. This way when we check for collision we only check by height.
        :param p: The lower left most point of the obstacle or anchor point.
        :return: True if we successfully added the obstacle. False otherwise
        """
        o = (p[0], p[2])
        if p[1] in self.obstacles.keys() and o not in self.obstacles[p[1]]:
            self.obstacles[p[1]] = o
            return True
        return False

    def ellipsoid(self):
        """
        Samples an ellipsoidal search space around the agent.
        :return: A sample coordinate that falls within the specified radius around the agent.
        """
        return random_point(-self.get_x_radius(), self.get_x_radius(),
                            -self.get_z_radius(), self.get_z_radius(),
                            -self.get_z_radius(), self.get_z_radius())

    def explore(self):
        """
        Runs the rapidly-exploring random tree algorithm.
        :return: The path if one is found. An empty list otherwise.
        """
        nodes = [self.get_root()]

        for i in range(self.get_max_nodes()):
            rand = agent.uniform()
            nn = nodes[0]
            for p in nodes:
                if distance(p.get_position(), rand) < distance(nn.get_position(), rand):
                    nn = p
            nodes.append(RRTNode(self.sample(nn.get_position(), rand), nn))
            if self.is_goal(nodes[-1]):
                return reconstruct_path(nodes[-1])
        return []

    def in_bounds(self, p):
        """
        Determines whether a point is in bounds or not.
        :param p: The point to check.
        :return: True if the point is in bounds. False otherwise.
        """
        x, y, z = p
        if (self.get_xmin() <= x <= self.get_xmax() and
                self.get_ymin() <= y <= self.get_ymax() and
                self.get_zmin() <= y <= self.get_zmax()):
            return True
        return False

    def is_blocked(self, p):
        """
        Determines if a point is invalid due to the region surrounding it containing an obstacle.
        :param p: The point to check.
        :return: True if the point is contained within an obstacle. False otherwise.
        """
        fy = math.floor(p[1])
        if fy in self.obstacles.keys():
            for o in self.obstacles[fy]:
                if (o[0] <= p[0] <= o[0] + 1 and
                        o[1] <= p[2] <= o[1] + 1):
                    return True
        return False

    def is_goal(self, n):
        """
        Determines if a specified node is the goal.
        :param n: The node to check for goal state.
        :return: True if the node is the goal. False otherwise.
        """
        x, y, z = n.get_position()
        t = self.get_goal_tolerance()
        if (self.get_goal()[0] - t <= x <= self.get_goal()[0] + t and
                self.get_goal()[1] - t <= y <= self.get_goal()[1] + t and
                self.get_goal()[2] - t <= z <= self.get_goal()[2] + t):
            return True
        return False

    def line_to(self, p1, p2):
        """
        Randomly samples a point on the line <p1, p2>.
        :param p1: The leading point of the line.
        :param p2: The trailing point of the line.
        :return: A point on the line <p1, p2>.
        """
        if distance(p1, p2) < self.get_max_branch_distance():
            return p2
        else:
            v = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
            l = math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
            v = (v[0]/l, v[1]/l, v[2]/l)
            x = p1[0] + random.uniform(0.0, self.get_max_branch_distance())*v[0]
            y = p1[1] + random.uniform(0.0, self.get_max_branch_distance())*v[1]
            z = p1[2] + random.uniform(0.0, self.get_max_branch_distance())*v[2]
            return x, y, z

    def sample(self, p1, p2):
        """
        Samples a point in the search space.
        :param p1: The point sampling from.
        :param p2: A randomly generated point.
        :return: A suitable sample point.
        """
        p = random.random()
        # Note: Reversing the comparison operator below creates a heavy bias towards the goal
        if p >= 1-self.get_goal_probability():
            return self.line_to(p1, self.get_goal())
        elif p <= 1-self.get_line_probability():
            return self.line_to(p1, p2)
        elif p <= (1-self.get_line_probability()/self.get_ellipsoid_uniform_modifier()):
            return self.uniform()
        else:
            return self.ellipsoid()

    def uniform(self):
        """
        Samples from the entire search space.
        :return: A sample coordinate that falls within the sample search space.
        """
        return random_point(self.get_xmin(), self.get_xmax(),
                            self.get_ymin(), self.get_ymax(),
                            self.get_zmin(), self.get_zmax())

if __name__ == '__main__':
    agent = RRTAgent()
    print agent.explore()
