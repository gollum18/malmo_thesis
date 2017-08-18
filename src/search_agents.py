from octile_grid import OctileGrid
from octile_grid import OctileCell
from lists import Stack, Queue, PriorityQueue
import math, random


EPSILON = 7.0


def euclidean3_heuristic(p1, p2):
    """
    Determines the 3D Euclidean Distance from the first point to the second point. This is the same distance
    equation most students learn in Calculus III.
    :param p1: The starting point.
    :param p2: The ending point.
    :return: The 3D Euclidean Distance between the two points.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    return math.sqrt(dx*dx+dy*dy+dz*dz)


def octile3_heuristic(p1, p2):
    """
    Determines the 3D Octile Distance from the first point to the second point as specified in
    Alex Nashs' Ph.D. Thesis 'Any-Angle Path Planning' on pg. 112. This distance metric is more suited for
    a wider range of movement than euclidean distance.
    :param p1: The starting point.
    :param p2: The ending point.
    :return: The 3D Octile Distance between the two points.
    Note: Do not use this heuristic for maps with less than 3 floors or you stand a chance of
    it returning 0.
    """
    distances = sorted([math.fabs(p1[0] - p2[0]), # dx
                        math.fabs(p1[1] - p2[1]),  # dy
                        math.fabs(p1[2] - p2[2])]) # dz
    smallest = distances[0]
    middle = distances[1]
    largest = distances[2]
    return math.sqrt(3) * smallest + math.sqrt(2) * (middle - smallest) + (largest - middle)


def reconstruct_path(grid, end):
    """
    Recreates the path planned for the agent to reach the goal.
    :param grid: The octile grid that was searched.
    :param end: The goal cell that was reached by the search algorithm.
    :return: A list containing all of the cells the agent must visit in-order to reach the goal state.
    """
    path = []
    current = end.get_parent()
    while current.get_parent():
        grid.set_state(current.get_x(), current.get_y(), current.get_z(), OctileGrid.STATE_PATH)
        current = current.get_parent()
    print grid
    return path.reverse()


def step_from_to3(p1, p2):
    if octile3_heuristic(p1, p2) < EPSILON:
        return p2
    else:
        theta = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        c = EPSILON*math.cos(theta)
        s = EPSILON*math.sin(theta)
        return int(math.fabs(p1[0] + c)), int(math.fabs(p1[1] + s)), int(math.fabs(p1[2] + (s)/(c)))


def _astar(grid, start, goal):
    """
    Runs the A* search algorithm on a 3D grid world.
    :param grid: The grid world to search.
    :param start: The starting position to search outward from.
    :param goal: The goal position to search for.
    :return: The best path as determined by A*.
    """
    open_list = PriorityQueue()
    open_list.enqueue(0, OctileCell(start[0], start[1], start[2]))
    closed_list = set()

    while open_list:
        current = open_list.dequeue()
        if current.get_position() == goal:
            return reconstruct_path(grid, current)
        if current not in closed_list:
            closed_list.add(current)
            for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                cell = OctileCell(neighbor[0], neighbor[1], neighbor[2], current)
                cell.set_gscore(current.get_gscore() + 1)
                cell.set_hscore(octile3_heuristic(neighbor, goal))
                cell.set_fscore(cell.get_gscore() + cell.get_hscore())
                open_list.enqueue(cell.get_fscore(), cell)
    return None

def astar(grid):
    """
    Runs the A* search algorithm on a 3D grid world.
    :param grid: The grid world to search.
    :return: The best path as determined by A*.
    """
    return _astar(grid, grid.get_start(), grid.get_goal())

def bfs(grid):
    """
    Runs the breadth-first search algorithm on a 3D grid world.
    :param grid: The grid world to search.
    :return: The best path as determined by breadth-first search.
    """
    open_list = Queue()
    open_list.enqueue(OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
    closed_list = set()

    while open_list:
        current = open_list.dequeue()
        if grid.is_goal_cell(current):
            return reconstruct_path(grid, current)
        if current not in closed_list:
            closed_list.add(current)
            for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                open_list.enqueue(OctileCell(neighbor[0], neighbor[1], neighbor[2], current))
    return None

def dfs(grid):
    """
    Runs the depth-first search algorithm on a 3D grid world.
    :param grid: The grid world to search.
    :return: The best path as determined by breadth-first search.
    """
    open_list = Stack()
    open_list.push(OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
    closed_list = set()

    while open_list:
        current = open_list.pop()
        if grid.is_goal_cell(current):
            return reconstruct_path(grid, current)
        if current not in closed_list:
            closed_list.add(current)
            for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                open_list.push(OctileCell(neighbor[0], neighbor[1], neighbor[2], current))
    return None

def dijkstra(grid):
    """
    Runs Dijkstras search algorithm on a 3D grid world. Identical to A* with a heuristic of 0.
    :param grid: The grid world to search.
    :return: The best path as determined by Dijkstras search.
    """
    open_list = PriorityQueue()
    open_list.enqueue(0, OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
    closed_list = set()

    while open_list:
        current = open_list.dequeue()
        if grid.is_goal_cell(current):
            return reconstruct_path(grid, current)
        if current not in closed_list:
            closed_list.add(current)
            for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                cell = OctileCell(neighbor[0], neighbor[1], neighbor[2], current)
                cell.set_gscore(current.get_gscore() + 1)
                cell.set_fscore(cell.get_gscore() + cell.get_hscore())
                open_list.enqueue(0, cell)
    return None

def kruskal(grid):
    open_list = PriorityQueue()
    open_list.enqueue(0, OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
    closed_list = set()

    while open_list:
        current = open_list.dequeue()
        if grid.is_goal_cell(current):
            return reconstruct_path(grid, current)
        if current not in closed_list:
            closed_list.add(current)
            for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                cell = OctileCell(neighbor[0], neighbor[1], neighbor[2], current)
                cell.set_fscore(octile3_heuristic(cell.get_position(), grid.get_goal()))
                open_list.enqueue(cell.get_fscore(), cell)
    return None

class ModernSearchAgents:

    @staticmethod
    def rrt(grid, num_nodes):
        """
        Determines the optimal path through a grid world using a 3D version of Steven M. LaValles' Rapidly
        Exploring Random Tree algorithm.
        :param grid: The grid world to search.
        :return: The best as determined by RRT.
        """

        ### Simplified Implementation
        # This implementation randomly chooses a neighbor from a valid neighbors list for the current cell
        # Should explore as quickly as the original algorithm while guaranteeing a path is returned, if there is one.
        # This is accomplished without the expensive math calls of the original algorithm.
        nodes = [OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2])]

        for i in range(num_nodes):
            for node in nodes:
                if grid.is_goal_cell(node):
                    return reconstruct_path(grid, node)
                pos = random.choice(grid.generate_neighbors3(node.get_x(), node.get_y(), node.get_z()))
                neighbor = OctileCell(pos[0], pos[1], pos[2], node)
                nodes.append(neighbor)
        return None