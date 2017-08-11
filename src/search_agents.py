from octile_grid import OctileGrid
from octile_grid import OctileCell
from collections import deque
from Queue import PriorityQueue
import math

def octile3_heuristic(p1, p2):
    distances = sorted([math.fabs(p1[0] - p2[0]), # dx
                        math.fabs(p1[1] - p2[1]),  # dy
                        math.fabs(p1[2] - p2[2])]) # dz
    smallest = distances[0]
    middle = distances[1]
    largest = distances[2]
    return math.sqrt(3) * smallest + math.sqrt(2) * (middle - smallest) + (largest - middle)

def reconstruct_path(grid, end):
    path = []
    current = end.get_parent()
    while current.get_parent():
        grid.set_state(current.get_x(), current.get_y(), current.get_z(), OctileGrid.STATE_PATH)
        current = current.get_parent()
    print grid
    return path

class ClassicalSearchAgents:

    @staticmethod
    def astar(grid):
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
        open_list = []
        open_list.append(OctileCell(grid.get_start()[0], grid.get_start()[1], grid.get_start()[2]))
        closed_list = set()

        while open_list:
            current = open_list.pop()
            if grid.is_goal_cell(current):
                return reconstruct_path(grid, current)
            if current not in closed_list:
                closed_list.add(current)
                for neighbor in grid.generate_neighbors3(current.get_x(), current.get_y(), current.get_z()):
                    open_list.append(OctileCell(neighbor[0], neighbor[1], neighbor[2], current))
        return None

    @staticmethod
    def dfs(grid):
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

ClassicalSearchAgents.astar(OctileGrid(10, 10, 3, percent_obs=0.25))
ClassicalSearchAgents.bfs(OctileGrid(10, 10, 3, percent_obs=0.25))
ClassicalSearchAgents.dfs(OctileGrid(10, 10, 3, percent_obs=0.25))