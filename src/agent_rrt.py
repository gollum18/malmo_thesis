import MalmoPython
import constants
import math
import random
import time
import sys

# TODO: Come up with a way to merge adjacent objects/hazards/walkable space into rectangular chunks
# This will reduce overhead in checking for object/hazard collision

#######################################
# FILE SPECIFIC PARAMETERS
#######################################
_debug = True

#######################################
# UTILITY FUNCTIONS
#######################################


def add_to_dictionary(dictionary, p):
    if p[1] not in dictionary.keys():
        dictionary[p[1]] = [(p[0], p[2])]
    else:
        dictionary[p[1]].append((p[0], p[2]))


def distance(p1, p2):
    """
    Determines the 3D euclidean distance between two points.
    :param p1: The first point.
    :param p2: The second point.
    :return: The euclidean distance between the two points.
    """
    return math.sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1])+(p2[2]-p1[2])*(p2[2]-p1[2]))


def point_on_line(p1, p2, dx=1, dy=1, dz=1):
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
    return math.floor(x)+.5, math.floor(y), math.floor(z)+.5


def read_in_txt(txt):
    """
    Reads in information on obstacles, hazards, and air from the missions descriptor text file.
    :param txt: The name of the mission descriptor file.
    :return: A dictionary containing all the obstacle, hazard, and air information about a mission.
    """
    objects = {constants.obstacle: set(), constants.hazard: set()}
    input_file = open(txt, 'r')
    input_amt = 0
    btype = None
    for line in input_file.readlines():
        if constants.obstacle in line:
            input_amt = int(line.split()[1])
            btype = constants.obstacle
            continue
        elif constants.hazard in line:
            input_amt = int(line.split()[1])
            btype = constants.hazard
            continue
        elif constants.air in line:
            input_amt = int(line.split()[1])
            btype = constants.air
            continue
        if input_amt > 0:
            parts = line.split()
            xl, xu, yl = int(parts[0]), int(parts[1]), int(parts[2])
            yu, zl, zu = int(parts[3]), int(parts[4]), int(parts[5])
            for dx in range(xu-xl+1):
                for dy in range(yu-yl+1):
                    for dz in range(zu-zl+1):
                        point = xl+dx, yl+dy, zl+dz
                        if btype == constants.obstacle:
                            objects[constants.obstacle].add(point)
                        elif btype == constants.hazard:
                            objects[constants.hazard].add(point)
                        elif btype == constants.air:
                            if point in objects[constants.obstacle]:
                                objects[constants.obstacle].remove(point)
                            if point in objects[constants.hazard]:
                                objects[constants.hazard].remove(point)
            input_amt -= 1
    input_file.close()
    return objects

#######################################
# CLASSES
#######################################


class Timer(object):

    def __init__(self):
        self.time = time.time()

    def elapsed(self):
        return time.time() - self.time

    def reset(self):
        self.time = time.time()


class RTRRT_Node(object):

    #######################################
    # CONSTRUCTOR
    #######################################

    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

    #######################################
    # ACCESSORS/MUTATORS
    #######################################

    def __eq__(self, other):
        return self.position == other.position

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.position)

    def get_position(self):
        return self.position

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent


class RTRRT_Agent(object):

    #######################################
    # CONSTRUCTOR
    #######################################

    def __init__(self,
                 start=(0, 0, 0),
                 goal=(10, 2, 10),
                 xdims=(-20, 20),
                 ydims=(-10, 10),
                 zdims=(-20, 20),
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
                 movement_distance=1,
                 move_time=2.5):
        self.start = start
        self.goal = goal
        self.position = self.start
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
        self.walkable = {}
        self.move_time = move_time
        self.move_timer = Timer()

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

    def get_walkable(self):
        return self.walkable

    #######################################
    # METHODS
    #######################################

    def add_hazard(self, p):
        """
        Adds a hazard to the search region.
        :param p: The bottom, lower-left point of the hazard.
        :return: N/A
        """
        add_to_dictionary(self.hazards, p)

    def add_obstacle(self, p):
        """
        Adds an obstacle to the search region.
        :param p: The bottom, lower-left point of the obstacle.
        :return: N/A
        """
        add_to_dictionary(self.obstacles, p)

    def add_walkable(self, p):
        add_to_dictionary(self.walkable, p)

    def ellipsoid(self, p):
        """
        Samples an ellipsoidal search region around the agent.
        :return: A point from the region immediately surrounding the agent.
        """

        # Create a sample space where each point is +- 1 within the current location and within the ellipsoid
        space = self.reachable(p, constants.sample_ellipsoidal)

        if space:
            # Get a random sample from it
            sample = random.choice(space)

            # Remove the sample from the players space and return the sample
            self.walkable[sample[1]].remove((sample[0], sample[2]))
            return sample
        else:
            return self.uniform()

    def explore(self):
        """
        Finds a path utilizing a rapidly-exploring random tree.
        :return: The path if it is found, or an empty list if it is not.
        """
        nodes = [RTRRT_Node(self.get_start())]

        for i in range(self.get_max_nodes()):
            rand = self.uniform()
            nn = nodes[0]
            for p in nodes:
                if distance(p.get_position(), rand) < distance(nn.get_position(), rand):
                    nn = p
            nodes.append(RTRRT_Node(self.sample(nn.get_position(), rand), nn))
            if self.is_goal(nodes[-1].get_position()):
                path = []
                current = nodes[-1]
                while current.get_parent():
                    path.append(current.get_position())
                    current = current.get_parent()
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
        if self.is_obstacle(p) or self.is_hazard(p):
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
        if p[1] in self.hazards.keys():
            for hazard in self.hazards[p[1]]:
                if (hazard[0] <= p[0] < hazard[0] + self.haz_dims[0] and
                        hazard[1] <= p[2] < hazard[1] + self.haz_dims[2]):
                    return True
        return False

    def is_obstacle(self, p):
        """
        Determines if the point is an obstacle.
        :param p: The point to check.
        :return: True if the point is an obstacle, False otherwise.
        """
        if p[1] in self.obstacles.keys():
            for obstacle in self.obstacles[p[1]]:
                if (obstacle[0] <= p[0] < obstacle[0] + self.obs_dims[0] and
                        obstacle[1] <= p[2] < obstacle[1] + self.obs_dims[2]):
                    return True
        return False

    def is_walkable(self, p):
        if math.floor(p[1]) in self.walkable.keys():
            for walkable in self.walkable[math.floor(p[1])]:
                if (walkable[0]-.5 <= p[0] < walkable[0]+.5 + self.obs_dims[0] and
                        walkable[1]-.5 <= p[2] < walkable[1]+.5 + self.obs_dims[2]):
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
            x, y, z = point_on_line(p1, p2)
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
            self.walkable[p2[1]].remove((p2[0], p2[2]))
            return p2
        else:
            p = point_on_line(p1, p2)
            p = p[0], math.floor(p[1]), p[2]
            while not self.is_walkable(p):
                p = point_on_line(p1, p2)
                p = p[0], math.floor(p[1]), p[2]
            return p

    def random_point(self):
        space = []

        for level in self.walkable.keys():
            for p in self.walkable[level]:
                space.append((p[0], level, p[1]))

        return random.choice(space)

    def reachable(self, p, sampling_method):

        def inside(x, y, z):
            return ((x-p[0]/self.radii[0])*(x-p[0]/self.radii[0]) +
                    (y-p[1]/self.radii[1])*(y-p[1]/self.radii[1]) +
                    (z-p[2]/self.radii[2])*(z-p[2]/self.radii[2])) < 1

        space = []

        y = p[1]
        if y in self.walkable.keys():
            for q in self.walkable[y]:
                if sampling_method == constants.sample_ellipsoidal and inside(q[0], y, q[1]):
                    space.append((q[0], y, q[1]))
        if y < max(self.ydims) and y+1 in self.walkable.keys():
            for q in self.walkable[y+1]:
                if sampling_method == constants.sample_ellipsoidal and inside(q[0], y+1, q[1]):
                    space.append((q[0], y+1, q[1]))
        if y > min(self.ydims) and y-1 in self.walkable.keys():
            for q in self.walkable[y-1]:
                if sampling_method == constants.sample_ellipsoidal and inside(q[0], y-1, q[1]):
                    space.append((q[0], y-1, q[1]))

        return space

    def sample(self, p1, p2):
        """
        Samples the search region for a new point in the tree.
        :param p1: The parent point.
        :param p2: A randomly chosen point from the search space.
        :return: A sampled point from the search space.
        """
        # TODO: This should sample from walkable space to be meaningful
        p = random.random()
        if p >= 1-self.get_goal_probability():
            return self.line_to(p1, self.get_goal())
        elif p <= 1-self.get_line_probability():
            return self.line_to(p1, p2)
        elif p <= (1-self.get_line_probability()/self.get_randomizer()):
            return self.uniform()
        else:
            return self.ellipsoid(p1)

    def uniform(self):
        """
        Returns a uniformly sampled point in the search region.
        :return: A uniformly sampled point from the search region.
        """
        p = self.random_point()
        self.walkable[p[1]].remove((p[0], p[2]))
        return p

#######################################
# TESTING METHODS
#######################################


def descriptor_test():
    paths = []
    for i in range(len(constants.mission_txt)):
        agent = RTRRT_Agent(start=constants.start,
                            goal=constants.goal[i],
                            xdims=(constants.lower_dimensions[i][0], constants.upper_dimensions[i][0]),
                            ydims=(constants.lower_dimensions[i][1], constants.upper_dimensions[i][1]),
                            zdims=(constants.lower_dimensions[i][2], constants.upper_dimensions[i][2]))
        for key, value in read_in_txt(constants.mission_txt[i]).iteritems():
            if key == constants.obstacle:
                for obstacle in value:
                    agent.add_obstacle(obstacle)
            elif key == constants.hazard:
                for hazard in value:
                    agent.add_obstacle(hazard)
        paths.append(agent.explore())
    for path in paths:
        print path


def malmo_test():
    for i in range(len(constants.mission_xml)):
        # Create an agent and generate a path
        agent = RTRRT_Agent(start=constants.start,
                            goal=constants.goal[i],
                            xdims=(constants.lower_dimensions[i][0], constants.upper_dimensions[i][0]),
                            ydims=(constants.lower_dimensions[i][1], constants.upper_dimensions[i][1]),
                            zdims=(constants.lower_dimensions[i][2], constants.upper_dimensions[i][2]))

        # Read in the obstacles/hazards from file and add them to the agents world model
        for key, value in read_in_txt(constants.mission_txt[i]).iteritems():
            if key == constants.obstacle:
                for obstacle in value:
                    agent.add_obstacle(obstacle)
            elif key == constants.hazard:
                for hazard in value:
                    agent.add_obstacle(hazard)

        # Add in walkable space
        for level, obstacles in agent.get_obstacles().iteritems():
            for obstacle in obstacles:
                x, z = obstacle
                p = x+.5, level + 1, z+.5
                if agent.in_bounds(p) and not agent.is_blocked(p) and not agent.is_blocked((x, level + 2, z)):
                    agent.add_walkable(p)

        # Generate a path
        path = agent.explore()

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
        time.sleep(5)

        while world_state.is_mission_running:
            # Guide the agent along the path
            world_state = agent_host.getWorldState()
            if path:
                loc = path.pop()
                agent_host.sendCommand("tp {0} {1} {2}".format(loc[0], loc[1], loc[2]))
            time.sleep(.3)


def rrt_test():
    agent = RTRRT_Agent()
    for i in range(random.randint(1, 20)):
        agent.add_hazard(agent.uniform())
    for i in range(random.randint(1, 20)):
        agent.add_obstacle(agent.uniform())
    print agent.explore()

if __name__ == '__main__':
    malmo_test()
