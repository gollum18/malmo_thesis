from octile_grid import *
import time


class StopWatch:

    def __init__(self):
        self.current_time = time.time()

    def elapsed(self):
        return time.time()-self.current_time

    def reset(self):
        self.current_time = time.time()


class DynamicGrid(OctileGrid):

    def __init__(self, x_dim, y_dim, z_dim, start=None, goal=None, percent_obs=0.20, percent_enemies=.01):
        """
        Creates a dynamic grid world.
        :param x_dim: The maximum width(x) of the search grid.
        :param y_dim: The maximum length(y) of the search grid.
        :param z_dim: The maximum height(z) of the search grid.
        :param start: The starting position for the search agent. If one is not defined then one will be randomly
        generated for you.
        :param goal: The goal position for the search agent. If one is not defined then one will be randomly generated
        for you.
        :param percent_obs: The percent of the search grid that is unreachable by the agent. For best results try a range
        from 10% to 50% (.10 to .50). Exceeding 50% greatly increases the probability that the agent will not be able to
        find the goal.
        :param percent_enemies: The percent of the search grid that is occupied by enemies.
        """
        if percent_obs + percent_enemies >= 1.0:
            raise ValueError
        OctileGrid.__init__(self, x_dim, y_dim, z_dim, start, goal, percent_obs)
        self.agents = [self.start]
        # Generate and place some enemies
        for i in range(int(x_dim*y_dim*z_dim*percent_enemies)):
            pos = self.rand3()
            while self.get_state(pos[0], pos[1], pos[2]) != self.STATE_EMPTY or pos in self.agents:
                pos = self.rand3()
            self.agents.append(pos)

    def __str__(self):
        s = "Printing dynamic grid with metrics (x-dimension={0}, y-dimension={1}, z-dimension={2})\n".format(
            self.dimensions[0], self.dimensions[1], self.dimensions[2]
        )
        is_enemy = False
        for z in range(self.dimensions[2]):
            s += "Level {0}:\n".format(z)
            for y in range(self.dimensions[1]):
                s += "|"
                for x in range(self.dimensions[0]):
                    if (x, y, z) in self.agents:
                        if self.agents.index((x, y, z)) == 0:
                            for enemy in self.get_enemy_positions():
                                if (x, y, z) == enemy:
                                    is_enemy = True
                                    s += "E|"
                                    continue
                            if not is_enemy:
                                s += "A|"
                        else:
                            s += "E|"
                    else:
                        s += "{0}|".format(self.state_string(self.get_state(x, y, z)))
                s += "\n"
            s += "\n"
        return s

    def get_agent_position(self):
        """
        Gets the position of the agent.
        :return: The agents position.
        """
        return self.agents[0]

    def get_enemy_positions(self):
        """
        Gets a list containing all of the enemy positions.
        :return: A list containing all enemy positions.
        """
        return self.agents[1:-1]

    def get_num_agents(self):
        """
        Gets the number of agents in the grid world.
        :return: Number of agents in the grid world.
        """
        return len(self.agents)

    def generate_neighbors3(self, x, y, z):
        """
        Generates neighbors for a location in the grid world.
        :param x: The x coordinate.
        :param y: The y coordinate.
        :param z: The z coordinate.
        :return: A list of valid neighbors for the location.
        """
        if not self.is_valid_cell(x, y, z):
            raise ValueError

        if (x, y, z) not in self.agents:
            raise ValueError

        neighbors = []
        enemies = self.get_enemy_positions()

        # Generate valid neighbors for this cell
        for direction in self.Directions:
            neighbor = self.move(x, y, z, direction)
            if neighbor and neighbor not in enemies:
                neighbors.append(neighbor)

        return neighbors

    def is_agent_alive(self):
        """
        Determines if the agent is still alive.
        :return: True if the agent does not share a cell with an enemy, False otherwise.
        """
        for agent in self.get_enemy_positions():
            if self.agents[0] == agent:
                return False
        return True

    def move_agent(self, index, new_pos):
        """
        Moves an agent in the grid world
        :param index: The index of the agent. Index 0 is ALWAYS the primary agent while each agent afterwards is an
        opposing agent.
        :param new_pos: The new position to move the agent to.
        :return: Nothing.
        """
        nx, ny, nz = new_pos
        if not self.in_bounds(nx, ny, nz):
            raise ValueError
        self.agents[index] = new_pos