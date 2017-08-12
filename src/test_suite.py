from octile_grid import *
from dynamic_grid import DynamicGrid
from search_agents import *
import time

def eval_enemy_pos(pos):
    # A rather simple evaluation function
    if pos == grid.get_agent_position():
        return 1000
    return 1.0 / octile3_heuristic(pos, grid.get_agent_position())

def eval_agent_pos(pos):
    # A rather simple evaluation function
    for agent in grid.get_enemy_positions():
        if pos == agent:
            return -1000
    return 1.0/octile3_heuristic(pos, grid.get_goal())

grid = DynamicGrid(10, 10, 3)

while grid.is_agent_alive() and grid.get_agent_position() != grid.get_goal():
    # Move all the enemies
    i = 1
    for pos in grid.get_enemy_positions():
        best = None
        bestv = -float("inf")
        for npos in grid.generate_neighbors3(pos[0], pos[1], pos[2]):
            temp = eval_enemy_pos(npos)
            if temp > bestv:
                best = npos
                bestv = temp
            grid.move_agent(i, best)
        i += 1
    print grid
    time.sleep(1.5)

    if not grid.is_agent_alive():
        break

    # Get the agents turn
    best = None
    bestv = -float("inf")
    pos = grid.get_agent_position()
    for npos in grid.generate_neighbors3(pos[0], pos[1], pos[2]):
        temp = eval_agent_pos(npos)
        if temp > bestv:
            best = npos
            bestv = temp
    grid.move_agent(0, best)

print grid
