from __future__ import division
import random, math, time


def dist(p1, p2):
    """
    Calculates the euclidean distance between two three dimensional points.
    :param p1: The first point.
    :param p2: The second point.
    :return: The euclidean distance between the two points.
    """
    dx, dy, dz = p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[1]
    return math.sqrt(dx**2+dy**2+dz**2)


def ellipsoid(sample, radii, pos):
    """
    Determines if a point lies inside a vector or not.
    :param sample: The sample point.
    :param radii: The radii of the ellipsoid.
    :param pos: The center of the ellipsis. i.e The agents position.
    :return: True if the sampled point falls within in the ellipsoid search space
    surrounding the agent, False otherwise.
    """
    return (((sample[0] - pos[0]) / radii[0])**2 +
            ((sample[1] - pos[1]) / radii[1])**2 +
            ((sample[2] - pos[2]) / radii[2])**2 < 1)


def line_of_sight(p1, p2):
    # TODO: Handle line of sight checking
    p = random.random()
    if p <= .5:
        return True
    return False


def line_to(p1, p2, epsilon):
    """
    Samples a point from p1 to a point that is near p2.
    :param p1: The first point.
    :param p2: The second point.
    :param epsilon: The maximum distance between points.
    :return: A sampled point that is either p2 if the distance is between p1 and p2 is less than epsilon or
    a randomly sampled point that is near p2 otherwise.
    """
    if dist(p1, p2) < epsilon:
        return p2
    theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    mx, my = epsilon * math.cos(theta), epsilon * math.sin(theta)
    return p1[0] + mx, p1[1] + my, p1[2] + (my/mx)


def octant(p):
    """
    Determines the octant of a point in 3D.
    :param p: The 3D point.
    :return: THe octant the point corresponds to.
    """
    if p[0] >= 0:
        if p[1] >= 0:
            if p[2] >= 0: # + + +
                return 1
            else: # + + -
                return 5
        else:
            if p[2] >= 0: # + - +
                return 4
            else: # + - -
                return 8
    else:
        if p[1] >= 0:
            if p[2] >= 0: # - + +
                return 2
            else: # - + -
                return 6
        else:
            if p[2] >= 0: # - - +
                return 3
            else: # - - -
                return 7


def rand(p1, p2, dim, alpha, beta, epsilon, radii):
    """
    Randomly samples the search space based on known criteria.
    :param p1: The agents position.
    :param p2: The sample point.
    :param dim: The dimensions of the search space.
    Should be a tuple in the form:
        (
            (xmin, xmax),
            (ymin, ymax),
            (zmin, zmax),
        )
    :param alpha: Used in determining the algorithm for sampling.
    :param beta: Used in determining the algorithm for sampling.
    :param epsilon: The max distance from the agents position to a sampled point.
    :param radii: The length of the x, y, and z vectors that create the ellipsoid.
    :return: A sample point for the agent to move to.
    """
    p = random.random()
    if p > 1-alpha:
        return line_to(p1, p2, epsilon)
    elif p <= ((1-alpha)/beta) or not line_of_sight(p1, p2):
        return uniform(dim)
    else:
        while not ellipsoid(p2, radii, p1):
            print p1, p2, radii
            p2 = sample_space(epsilon)
        return p2


def sample_space(epsilon):
    """
    Generates a 3D Point for path traversal.
    :param epsilon: Tha maximum distance to generate out to.
    :return: A sample point drawn from epsilon.
    """
    x = random.uniform(-epsilon, epsilon)
    y = random.uniform(-epsilon, epsilon)
    z = random.uniform(-epsilon, epsilon)
    return x, y, z


def uniform(dim):
    """
    Uniformly draws a sample from the search space.
    :param dim: The dimensions of the search space.
    Should be a tuple in the form:
        (
            (xmin, xmax),
            (ymin, ymax),
            (zmin, zmax),
        )
    :return: A random point that has been sampled uniformly in the search space.
    """
    x = random.uniform(dim[0][0], dim[0][1])
    y = random.uniform(dim[1][0], dim[1][1])
    z = random.uniform(dim[2][0], dim[2][1])
    return x, y, z


PLAYER = 3, 10, -2
EPSILON = 7.0
RADII = 7.0, 7.0, 7.0
DIM = ((-100, 100), (-10, 10), (-100, 100))
START = (0, 0, 0)
GOAL = (75, -5, 50)
NUMNODES = 5000


def is_goal(p):
    x, y, z = p
    if (math.fabs(GOAL[0] - x) <= 0.5 and
            math.fabs(GOAL[1] - y) <= 0.5 and
            math.fabs(GOAL[2] - z) <= 0.5):
        return True
    return False


class StopWatch:

    def __init__(self):
        self.current = time.time()

    def elapsed(self):
        return self.current - time.time()

    def reset(self):
        self.current = time.time()


class Node:

    def __init__(self, p):
        self.x, self.y, self.z = p
        self.coords = p
        self.parent = None


def main():
    timer = StopWatch()
    agent = Node(START)
    nodes = [agent]
    while not is_goal(agent.coords):
        while timer.elapsed() < 0.5:
            for i in range(NUMNODES):
                r = sample_space(EPSILON)
                nn = nodes[0]
                for p in nodes:
                    if dist(p.coords, r) < dist(nn.coords, r):
                        nn = p
                newnode = Node(rand(agent.coords, nn.coords, DIM, 0.1, 3.25, EPSILON, RADII))
                nodes.append(newnode)
                newnode.parent = nn
                # print i, "    ", nodes
            val = float("inf")
            for i in range(NUMNODES):
                temp = dist(nodes[i].coords, GOAL)
                if temp < val:
                    val = temp
                    agent = nodes[i].coords
        timer.reset()

    best = None
    val = float("inf")
    for i in range(NUMNODES):
        temp = dist(nodes[i].coords, GOAL)
        if temp < val:
            val = temp
            best = nodes[i]

    path = []
    while best.parent:
        path.append(best)
        best = best.parent
    print path.reverse()


if __name__ == '__main__':
    main()