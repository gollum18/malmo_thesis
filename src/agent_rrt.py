import MalmoPython
import constants
import math
import random
import sys
import time

def dist(p1, p2):
    """
    Calculates the distance between p1 and p2.
    :param p1: A tuple (x, y, z).
    :param p2: A tuple (x, y, z).
    :return: The distance between two points.
    """
    dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)

class Node(object):

    def __init__(self, position, parent=None):
        """
        Creates an instance of a Node in a search tree.
        :param position: The position of the node.
        :param parent: The parent of this node. Default = None.
        :return: A new Node object.
        """
        self.position = position
        self.parent = parent

    def set_position(self, position):
        """
        Sets the position of the Node.
        :param position: The new position for the Node.
        :return: N/A
        """
        self.position = position

    def set_parent(self, parent):
        """
        Sets the parent of the Node.
        :param parent: The new parent for the Node.
        :return: N/A
        """
        self.parent = parent

    def get_position(self):
        """
        Returns the position of the Node.
        :return: A tuple (x, y, z).
        """
        return self.position

    def get_parent(self):
        """
        Returns the parent of the Node.
        :return: A reference to the parent Node of this Node.
        """
        return self.parent

class Agent(object):

    def __init__(self, start=(0, 0, 0), goal=(10, 10, 10), 
                 xdims=(0, 10), ydims=(0, 10), zdims=(0, 10), 
                 nodes=5000, p_goal=.5, p_line=.05):
        """
        Creates a new instance of a search Agent.
        :param start: The starting position of the Agent.
        :param goal: The goal position of the Agent.
        :param xdims: The x dimensions of the search space.
        :param ydims: The y dimensions of the search space.
        :param zdims: The z dimensions of the search space.
        :param nodes: The maximum number of node the Agent can generate.
        :param p_goal: The probability of sampling the goal.
        :param p_line: Thr probability of sampling using line-to.
        :return: A new Agent instance.
        """
        self.start = start
        self.goal = goal
        self.xdims = xdims
        self.ydims = ydims
        self.zdims = zdims
        self.nodes = nodes
        self.p_goal = p_goal
        self.p_line = p_line
        self.obstacles = set()
        self.walkable = {}

    def set_start(self, start):
        """
        Sets the starting position of the agent.
        :param start: The new starting position of the agent.
        :return: N/A
        """
        self.start = start

    def set_goal(self, goal):
        """
        Sets the goal position of the agent.
        :param goal: The new goal position of the agent.
        :return: N/A
        """
        self.goal = goal

    def set_xdims(self, xdims):
        """
        Sets the x dimensions of the agent.
        :param xdims: The x dimensions of the agent.
        :return: N/A
        """
        self.xdims = xdims

    def set_ydims(self, ydims):
        """
        Sets the y dimensions of the agent.
        :param ydims: The y dimensions of the agent.
        :return: N/A
        """
        self.ydims = ydims

    def set_zdims(self, zdims):
        """
        Sets the z dimensions of the agent.
        :param zdims: The z dimensions of the agent.
        :return: N/A
        """
        self.zdims = zdims

    def set_goal_probability(self, p):
        """
        Sets the probability of sampling the goal.
        :param p: A probability such that 0 <= p <= 1
        """
        if 0 <= p <= 1:
            self.p_goal = p

    def set_line_probability(self, p):
        """
        Sets the proabibility of sampling using line-to.
        :param p: A probability such that 0 <= p <= 1.
        :return: N/A
        """
        if 0 <= p <= 1:
            self.p_line = p

    def get_start(self):
        """
        Returns the starting position of the agent.
        :return: A tuple (x, y, z).
        """
        return self.start

    def get_goal(self):
        """
        Returns the goal position of the agent.
        :return: A tuple (x, y, z).
        """
        return self.goal

    def get_xmin(self):
        """
        Returns the smallest value available in the x-direction.
        :return: An integer representing the min x value.
        """
        return min(self.xdims)

    def get_ymin(self):
        """
        Returns the smallest value available in the y-direction.
        :return: An integer representing the max y value.
        """
        return min(self.ydims)

    def get_zmin(self):
        """
        Returns the smallest value available in the z-direction.
        :return: An integer representing the min z value.
        """
        return min(self.zdims)

    def get_xmax(self):
        """
        Returns the largest value available in the x-direction.
        :return: An integer representing the max x value.
        """
        return max(self.xdims)

    def get_ymax(self):
        """
        Returns the largest value available in the y-direction.
        :return: An integer representing the max y value.
        """
        return max(self.ydims)

    def get_zmax(self):
        """
        Returns the largest value available in the z-direction.
        :return: An integer representing the max z value.
        """
        return max(self.zdims)

    def get_max_nodes(self):
        """
        Returns the max number of nodes the agent is allowed to generate.
        :return: An integer representing the maximum number of nodes.
        """
        return self.nodes

    def get_goal_probability(self):
        """
        Returns the probability of sampling towards the goal.
        :return: A probability p such that 0 <= p <= 1.
        """
        return self.p_goal

    def get_line_probability(self):
        """
        Returns the probability of sampling using the line-to algorithm.
        :return: A probability p such that 0 <= p <= 1.
        """
        return self.p_line

    def add_obstacle(self, obs):
        """
        Adds an obstacle region to the search space.
        :param obs: A tuple (x1, y1, z1, x2, y2, z2).
        :return: N/A
        """
        self.obstacles.add(obs)

    def add_walkable(self, walk):
        """
        Adds a walkable point to the search space.
        :param walk: The walkable point to add.
        :return: N/A
        """
        if walk[1] in self.walkable.keys():
            self.walkable[walk[1]].add((walk[0], walk[2]))
        else:
            self.walkable[walk[1]] = set()
            self.walkable[walk[1]].add((walk[0], walk[2]))

    def ellipsoid(self, p, xr, yr, zr):
        """
        Samples a point using the ellipsoid sampling algorithm.
        :param p: The point to sample outwards from.
        :param xr: The x radius of the ellipsoid.
        :param yr: The y radius of the ellipsoid.
        :param zr: The z radius of the ellipsoid.
        :return: A tuple (x, y, z).
        """
        def inside(self, x, y, z):
            return ((x-p[0]/xr)*(x-p[0]/xr) +
                    (y-p[1]/yr)*(y-p[1]/yr) +
                    (z-p[2]/zr)*(z-p[2]/zr)) < 1
        space = set()
        y = p[1]
        if y in self.walkable.keys():
            for q in self.walkable[y]:
                if (self.in_bounds((q[0], y, q[1])) and
                    inside(q[0], y, q[1]) and 
                    self.line_of_sight(p, (q[0], y, q[1]))):
                    space.append((q[0], y, q[1]))
        if y < max(self.ydims) and y+1 in self.walkable.keys():
            for q in self.walkable[y+1]:
                if (self.in_bounds((q[0], y+1, q[1])) and
                    inside(q[0], y+1, q[1]) and 
                    self.line_of_sight(p, (q[0], y+1, q[1]))):
                    space.append((q[0], y+1, q[1]))
        if y > min(self.ydims) and y-1 in self.walkable.keys():
            for q in self.walkable[y-1]:
                if (self.in_bounds((q[0], y-1, q[1])) and
                    inside(q[0], y-1, q[1]) and 
                    self.line_of_sight(p, (q[0], y-1, q[1]))):
                    space.append((q[0], y-1, q[1]))
        return random.sample(space, 1)[0]

    def explore(self):
        """
        Runs RT-RRT* algorithm to find a path from start to goal.
        :return: A list containing the path if there is one. Otherwise an 
            empty list.
        """
        nodes = [Node(self.get_start())]

        for i in range(self.get_max_nodes()):
            rand = self.random_point()
            nn = nodes[0]
            for p in nodes:
                if dist(p.get_position(), rand) < dist(nn.get_position(), rand):
                    nn = p
            node = Node(self.sample(nn.get_position(), rand), nn)
            self.remove_walkable(node.get_position())
            nodes.append(node)
            if self.is_goal(nodes[-1].get_position()):
                path = []
                current = nodes[-1]
                while current.get_parent():
                    path.append(current.get_position())
                    current = current.get_parent()
                return path
        return []

    def generate_walkable_space(self):
        """
        Determines all walkable space in a search space.
        :return: N/A
        """
        for obs in self.obstacles:
            for x in range(obs[0], obs[1] + 1):
                for z in range(obs[4], obs[5] + 1):
                    if (not self.is_obstacle((x, obs[3] + 1, z)) and 
                        not self.is_obstacle((x, obs[3] + 2, z)) and
                        self.in_bounds((x+.5, obs[3], z+.5))):
                        # TODO: Perform in bounds checking
                        # for some reason here, bounds checking eliminates
                        # valid points.
                        self.add_walkable((x+.5, obs[3], z+.5))

    def in_bounds(self, p):
        """
        Determines if the specified point is in bounds.
        :param p: The point to check.
        :return: True if the point is in bounds, False otherwise.
        """
        return (self.get_xmin() <= p[0] < self.get_xmax() and 
                self.get_ymin() <= p[1] < self.get_ymax() and
                self.get_zmin() <= p[2] < self.get_zmax())

    def is_obstacle(self, p):
        """
        Determines if the specified point falls within an obstacle region.
        :param p: The point to check.
        :return: True if the point lies inside an obstacle, False otherwise.
        """
        for obs in self.obstacles:
            if (obs[0] <= p[0] < obs[1] and 
                obs[2] <= p[1] < obs[3] and
                obs[4] <= p[2] < obs[5]):
                return True
        return False

    def is_goal(self, p):
        """
        Determines if the specified point is the goal.
        :param p: The point to check.
        :return: True if the specified point is the goal, False otherwise.
        """
        # Simply call is same point, it essentially does what we want
        return self.is_same_point(p, self.goal)

    def is_walkable(self, p):
        """
        Determines if a point is walkable or not.
        :param p: The point to check.
        :return: True if the point is within walkable space, False otherwise.
        """
        # Round down the y to use as a key
        y = int(math.floor(p[1]))
        if y in self.walkable.keys():
            # Loop through each walkable space with the current y
            for walk in self.walkable[y]:
                # If there is a point here, then we can walk, return True
                if (walk[0] <= p[0] < walk[0] + 1 and
                    walk[1] <= p[2] < walk[1] + 1):
                    return True
        # If we get here then there is no walkable space at p, return False
        return False

    def is_same_point(self, p1, p2):
        """
        Determines if p1 and p2 are the same point.
        :param p1: The first point.
        :param p2: The second point.
        :return: True if the points are the same, False otherwise.
        """
        # This algorithm accounts for floating point error with 
        #   line_to and line_of_sight
        return (p2[0] - .05 <= p1[0] < p2[0] + .05 and
                p2[1] - .05 <= p1[1] < p2[1] + .05 and 
                p2[2] - .05 <= p1[2] < p2[2] + .05)

    def line_of_sight(self, p1, p2):
        """
        Determines if there is line of sight from p1 to p2.
        :param p1: The point to check line of sight from.
        :param p2: The point to check line of sight to.
        :return: True if there is line of sight from p1 to p2, False otherwise.
        """
        # This is a very similar algorithm to line_to
        dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
        s = max(dx, dy, dz)
        if s == 0:
            s = 1
        # The only difference is that if we come across an obstacle we return False
        while not self.is_same_point(p1, p2):
            if self.is_blocked(p1):
                return False
            p1 = p1[0] + dx/s, p1[1] + dy/s, p1[2] + dz/s
        # Otherwise if we successfully step to the destination then return True 
        return True

    def line_to(self, p1, p2):
        """
        Samples a random point along the line p1, p2.
        :param p1: The first terminus of the line.
        :param p2: The second terminus of the line.
        :return: A random point on the line p1, p2.
        """
        # Calculate the greatest distance to be our normalizer for division
        dx, dy, dz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
        s = max(dx, dy, dz)
        if s == 0:
            s = 1
        # Perform the initial step
        p1 = p1[0] + dx/s, p1[1] + dy/s, p1[2] + dz/s
        space = set()
        # Loop until we hit an obstacle or reach our destination
        while not self.is_obstacle(p1) and not self.is_same_point(p1, p2):
            if self.is_walkable(p1):
                space.add(self.nearest_walkable(p1))
            p1 = p1[0] + dx/s, p1[1] + dy/s, p1[2] + dz/s
        # Check if the last point is our destination, if so add it
        if self.is_same_point(p1, p2):
            space.add(p2)
        # Fallback to ensure we return something in case we didnt get to any 
        #   points
        if not space:
            return self.uniform(p1)
        # Otherwise return a random point on the line from p1 to p2
        return random.sample(space, 1)[0]
        
    def load_mission_parameters(self, filename):
        """
        Loads in geographical info about a mission from file.
        :param fliename: The file that contains the information about the current mission.
        :raise IOError: If the file specified does not exist.
        """
        # Attempt to open the file
        input_file = open(filename, 'r')
        # Raise an error if it does not exist
        if not input_file:
            raise IOError("IOError: File {0} was not found".format(filename))
        # For each obstacle in the file
        for line in input_file.readlines():
            # Skip comments
            if "//" in line:
                continue
            # Add the obstacle to the agents world state
            parts = line.split()
            self.add_obstacle((int(parts[0]), int(parts[1]), int(parts[2]),
                               int(parts[3]), int(parts[4]), int(parts[5])))
        # Close the file to avoid corruption
        input_file.close()

    def nearest_walkable(self, p):
        """
        Finds the nearest neighbor point to the passed in point.
        :param p: The point to check.
        :return: THe closest point to p.
        """
        y = int(math.floor(p[1]))
        if y in self.walkable.keys():
            d = float("inf")
            nearest = None
            for obs in self.walkable[y]:
                val = dist(p, (obs[0], y, obs[1]))
                if val < d:
                    d = val
                    nearest = (obs[0], y, obs[1])
            return nearest
        return None

    def random_point(self):
        """
        Finds a random point in the entire search space.
        :return: A random point from the search space.
        """
        y = random.choice(list(self.walkable.keys()))
        x, z = random.sample(self.walkable[y], 1)[0]
        return x, y, z

    def remove_obstacle(self, obs):
        """
        Removes an obstacle from the search space.
        :param obs: The obstacle to remove from the search space.
        :return: N/A
        """
        if obs in self.obstacles:
            self.obstacles.remove(obs)

    def remove_walkable(self, p):
        """
        Removes a walkable point from the search space.
        :param p: The walkable point to remove from the search space.
        :return: N/A
        """
        if p[1] in self.walkable.keys() and (p[0], p[1]) in self.walkable[p[1]]:
            self.walkable[p[1]].remove((p[0], p[2]))

    def sample(self, p1, p2):
        """
        Samples a point from walkable space.
        :param p1: The point being sampled from.
        :param p2: A random point.
        :return: A point sampled from walkable space.
        """
        p = random.random()
        if p >= 1-self.get_goal_probability():
            return self.line_to(p1, self.get_goal())
        elif p <= 1-self.get_line_probability():
            return self.line_to(p1, p2)
        elif p <= (1-self.get_line_probability()/self.get_randomizer()):
            return self.uniform(p1[1])
        else:
            return self.ellipsoid(p1)

    def uniform(self, p):
        """
        Uniformly samples a point from reachable space.
        :param p: The current position to sample outwards from.
        :return: A uniformly sampled point.
        """
        # Create a set of walkable space that is the union of walkable space 
        #   on the current floor, next floor up, and next floor down and 
        #   sample a point from it
        # Normally this algorithm would sample all walkable space, but
        #   we restrict it by adjacent floors to make it more realistic
        y = p[1]
        space = set()
        if y in self.walkable.keys():
            space = space.union(self.walkable[y])
        if y + 1 in self.walkable.keys():
            space = space.union(self.walkable[y + 1])
            y = y + 1
        if y - 1 in self.walkable.keys():
            space = space.union(self.walkable[y - 1])
            y = y - 1
        x, z = random.sample(space, 1)[0]
        return x, y, z

for i in range(len(constants.mission_xml)):
    # Create an agent and generate a path
    agent = Agent()
    agent.set_start(constants.start)
    agent.set_goal(constants.goal[i])
    agent.set_xdims((constants.lower[i][0], constants.upper[i][0]))
    agent.set_ydims((constants.lower[i][1], constants.upper[i][1]))
    agent.set_zdims((constants.lower[i][2], constants.upper[i][2]))
    agent.load_mission_parameters(constants.mission_txt[i])
    agent.generate_walkable_space()
    path = agent.explore()

    if not path:
        print "No path was found!"
        continue

    # Setup a malmo client
    agent_host = MalmoPython.AgentHost()

    try:
        agent_host.parse(sys.argv)
    except RuntimeError as e:
        print 'ERROR:', e
        print agent_host.getUsage()
        exit(1)
    if agent_host.receivedArgument("help"):
        print agent_host.getUsage()
        exit(0)

    my_mission = None
    # Load in the mission
    with open(constants.mission_xml[i], 'r') as f:
        print "Loading mission from {0}".format(constants.mission_xml[i])
        my_mission = MalmoPython.MissionSpec(f.read(), True)
    my_mission_record = MalmoPython.MissionRecordSpec()

    # Attempt to start a mission:
    max_retries = 3
    for retry in range(max_retries):
        try:
            agent_host.startMission(my_mission, my_mission_record)
            break
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
        time.sleep(1)
        world_state = agent_host.getWorldState()
        for error in world_state.errors:
            print "Error:", error.text

    print
    print "Mission running ",

    # Needed to prevent skipped commands
    time.sleep(3)

    while world_state.is_mission_running:
        # Needed to transition between missions
        world_state = agent_host.getWorldState()

        # Guide the agent along the path
        sys.stdout.write(".")
        if path:
            pos = path.pop()
            agent_host.sendCommand("tp {0} {1} {2}".format(pos[0], pos[1], pos[2]))
        time.sleep(.5)

    print "Mission ended ",
    print