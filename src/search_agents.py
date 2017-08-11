from octile_grid import OctileGrid
from octile_grid import OctileCell
from collections import deque
from Queue import PriorityQueue
import math


def octile3_heuristic(p1, p2):
    """
    Determines the 3D Octile Distance from the first point to the second point as specified in
    Alex Nashs' Ph.D. Thesis 'Any-Angle Path Planning' on pg. 112.
    :param p1: The starting point.
    :param p2: The ending point.
    :return: The 3D Octile Distance between the two points.
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


class ClassicalSearchAgents:

    @staticmethod
    def astar(grid):
        """
        Runs the A* search algorithm on a 3D grid world.
        :param grid: The grid world to search.
        :return: The best path as determined by A*.
        """
        open_list = PriorityQueue()
        open_list.put_nowait(OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
        closed_list = set()

        while open_list:
            current = open_list.get_nowait()
            if grid.is_goal_cell(current):
                return reconstruct_path(grid, current)
            if current not in closed_list:
                closed_list.add(current)
                for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                    cell = OctileCell(neighbor[0], neighbor[1], neighbor[2], current)
                    cell.set_gscore(current.get_gscore() + 1)
                    cell.set_hscore(octile3_heuristic(neighbor, grid.get_goal()))
                    cell.set_fscore(cell.get_gscore() + cell.get_hscore())
                    open_list.put_nowait(cell)
        return None

    @staticmethod
    def bfs(grid):
        """
        Runs the breadth-first search algorithm on a 3D grid world.
        :param grid: The grid world to search.
        :return: The best path as determined by breadth-first search.
        """
        open_list = deque()
        open_list.append(OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
        closed_list = set()

        while open_list:
            current = open_list.popleft()
            if grid.is_goal_cell(current):
                return reconstruct_path(grid, current)
            if current not in closed_list:
                closed_list.add(current)
                for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                    open_list.append(OctileCell(neighbor[0], neighbor[1], neighbor[2], current))
        return None

    @staticmethod
    def dfs(grid):
        """
        Runs the depth-first search algorithm on a 3D grid world.
        :param grid: The grid world to search.
        :return: The best path as determined by breadth-first search.
        """
        open_list = deque()
        open_list.appendleft(OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
        closed_list = set()

        while open_list:
            current = open_list.popleft()
            if grid.is_goal_cell(current):
                return reconstruct_path(grid, current)
            if current not in closed_list:
                closed_list.add(current)
                for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                    open_list.appendleft(OctileCell(neighbor[0], neighbor[1], neighbor[2], current))
        return None


class ModernSearchAgents:

    def qlearning(self, grid):
        """
        Iteratively finds the best path to traverse a grid world using the reinforcement learning technique
        'Q-Learning'.
        :param grid: The grid world to search.
        :return: The best path as determined by q-learning.
        """
        raise NotImplementedError

    def rrt(self, grid):
        """
        Determines the optimal path through a grid world using a 3D version of Steven M. LaValles' Rapidly
        Exploring Random Tree algorithm.
        :param grid: The grid world to search.
        :return: The best as determined by RRT.
        """
        raise NotImplementedError