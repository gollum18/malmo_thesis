import math
import numpy

def cost(p1, p2):
    """
    Used by A* to determine the cost of going from p1 to p2.
    p1: The initial point.
    p2: The ending point.
    returns: The cost of going from p1 to p2.
    """
    return 1 + math.fabs(p2[1] - p1[1])


def degree_change(v1, v2):
    """
    Calculates the change ni angle between two vectors in space.
    v1: The first vector.
    v2: The second vector.
    :returns: The angle of change between the two vectors in degrees.
    """
    return numpy.arccos(numpy.dot(v1, v2) /
                        (numpy.linalg.norm(v1)*numpy.linalg.norm(v2)))


def dist(p1, p2):
    """
    Determines the euclidean distance between p1 and p2.
    p1: The initial point.
    p2: The ending point.
    returns: The distance between p1 and p2.
    """
    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]
    dz = p2[2]-p1[2]
    return math.sqrt(dx*dx+dy*dy+dz*dz)


def heuristic(p1, p2):
    """
    Used by A* to determine a best guess estimate of the remaining distance between p1 and p2.
    p1: The initial point.
    p2: The ending point.
    returns: The estimated distance between p1 and p2.
    """
    dx = math.fabs(p2[0] - p1[0])
    dy = math.fabs(p2[1] - p1[1])
    dz = math.fabs(p2[2] - p1[2])
    # Weight the y dimension
    return dx + dy * 1.5 + dz


def reconstruct_path(node):
    """
    Constructs a path from a goal node.
    node: The goal node that was found by a planning algorithm.
    returns: A list containing the path.
    """
    path = []
    while node.get_parent():
        path.append(n.get_position())
        n = n.get_parent()
    return path