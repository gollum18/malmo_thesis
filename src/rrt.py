# Basic imports
from __future__ import division
import sys, time, random, constants, math
import MalmoPython
from numba import jit

@jit
def dist(p1, p2):
    """
    Calculates euclidean distance in three space.
    :param p1: The first point.
    :param p2: The second point.
    :return:
    """
    if len(p1) != 3 or len(p2) != 3:
        raise ValueError("ValueError: Cannot calculate distance on a non three dimensional point!")
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)

@jit
def interpolate(p1, p2, t):
    """
    Determines points on a line using linear interpolation.
    :param p1: The starting point.
    :param p2: The ending point.
    :param t: The current interpolation step (0 to 1).
    :return: An interpolated point.
    """
    x = (1 - t) * p1[0] + t * p2[0]
    y = (1 - t) * p1[1] + t * p2[1]
    z = (1 - t) * p1[2] + t * p2[2]
    return x, y, z

class Agent(object):

    def __init__(self, start, goal, nodes=5000, p_goal=.5, p_line=.05, step=50):
        """
        Creates an agent object.
        :param start: The starting position of the agent in the world.
        :param goal: The goal position of the agent in the world.
        :param nodes: THe maximum number of nodes rrt can generate.
        :param p_goal: The probability of sampling the goal.
        :param p_line: The probability of sampling using line to.
        :param step: The number of steps taken by the interpolator.
        """
        self.start = start
        self.goal = goal
        self.max_nodes = nodes
        self.goal_probability = p_goal
        self.line_probability = p_line
        self.interpolations = step
        self.sampled = set()
        self.world = None

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def get_max_nodes(self):
        return self.max_nodes

    def get_goal_probability(self):
        return self.goal_probability

    def get_line_probability(self):
        return self.line_probability

    def get_interpolation_amt(self):
        return self.interpolations

    def get_sampled_space(self):
        return self.sampled

    def get_world(self):
        return self.world

    def create_world(self, xdims, ydims, zdims, filename):
        # Realistically, dont do this!!!!!
        # I of course am referring to the check here.
        if not xdims or not ydims or not zdims or not filename:
            raise ValueError("ValueError: You passed in null world parameters! Each parameter must be " +
                             "defined in order to create a world!\n")
        # Attempt to open the file
        reader = open(filename, 'r')
        if not reader:
            raise ValueError("ValueError: The geometry file you passed in does not exist! Please " +
                             "ensure it points to a valid file on disk!\n")
        # Create a world instance
        self.world = World(xdims, ydims, zdims)
        # Read in the world geometry from disk
        ln = 0
        for line in reader.readlines():
            if "//" in line or not line:
                continue
            try:
                geom = [int(g) for g in line.split()]
                # Raise an error if the geometry is not exactly six points
                if len(geom) != 6:
                    raise ValueError
                self.world.add_obstacle(Obstacle((geom[0], geom[1]), (geom[2], geom[3]), (geom[4], geom[5])))
            except ValueError:
                sys.stderr.write("WARNING: Bad geometry encountered at line {0}!".format(ln))
                sys.stderr.write("--> The geometry present in this line was ignored!\n")
            ln += 1
        reader.close()
        self.world.generate_walkable_positions()

    def ellipsoid(self, p, xr=1.0, yr=1.25, zr=1.0):
        """
        Samples using the ellipsoidal sampling algorithm.
        :param p: The agents position.
        :param xr: The x radius of the ellipsoid.
        :param yr: The y radius of the ellipsoid.
        :param zr: The z radius of the ellipsoid.
        :return: A walkable point within the agents ellipsoid search area.
        """
        @jit
        def inside(x, y, z):
            """
            Determines if a point is inside the ellipse or not.
            :param x:
            :param y:
            :param z:
            :return:
            """
            return ((x-p[0]/xr)*(x-p[0]/xr) +
                    (y-p[1]/yr)*(y-p[1]/yr) +
                    (z-p[2]/zr)*(z-p[2]/zr)) < 1

        sample_space = set()
        for i in [-1, 0, 1]:
            for p in self.world.get_walkable_space_by_level(p[1] + i):
                if inside(*p):
                    sample_space.add(p)
        return random.sample(sample_space, 1)[0]


    def is_goal(self, position):
        """

        :param position:
        :return:
        """
        return (self.goal[0] <= position[0] < self.goal[0] + 1 and
                self.goal[1] == position[1] and
                self.goal[2] <= position[2] < self.goal[2] + 1)

    def line_of_sight(self, p1, p2):
        """
        Determines line of sight between two points.
        :param p1: The first point.
        :param p2: The second point.
        :return: True if there is line of sight between the points, False otherwise.
        """
        t = 0
        while t < self.interpolations:
            if not self.world.is_valid(interpolate(p1, p2, t)):
                return False
            t += 1
        print "Line of sight between: ", p1, p2
        return True

    def line_to(self, p1, p2):
        """

        :param p1:
        :param p2:
        :return:
        """
        def point(p):
            """
            Creates a rounded point that is exact to a walkable point in the world.
            :param p: The point to round.
            :return: A point (x, y, z).
            """
            return math.floor(p[0]) + .5, math.floor(p[1]), math.floor(p[2]) + .5

        t = 0
        sample_space = set()
        while t < self.interpolations:
            p3 = interpolate(p1, p2, t/self.interpolations)
            if self.world.is_blocked(p3):
                break
            p = point(p3)
            if self.world.is_walkable(p) and p not in self.sampled:
                sample_space.add(p)
            t += 1
        if self.line_of_sight(p1, p2) and self.world.is_walkable(p2):
            sample_space.add(p2)
        if not sample_space:
            print p1, p2

        return random.sample(sample_space, 1)[0]

    def random_point(self):
        """
        Gets a random point in the world that has not already been sampled.
        :return: A point (x, y, z).
        """
        return random.sample(self.world.get_walkable_space() - self.sampled, 1)[0]

    def rapidly_exploring_random_tree(self):
        """
        Finds a path using a rapidly exploring random tree.
        :return:
        """
        if not self.world:
            raise ValueError("ValueError: Cannot explore a world that does not exist! Call " +
                             "agent.create_world() first!")
        nodes = [Node(self.get_start())]
        for i in range(self.get_max_nodes()):
            rand = self.random_point()
            nn = nodes[0]
            for p in nodes:
                if dist(p.get_position(), rand) < dist(nn.get_position(), rand):
                    nn = p
            node = Node(self.sample(nn.get_position(), rand), nn)
            nodes.append(node)
            self.sampled.add(node.get_position())
            if self.is_goal(nodes[-1].get_position()):
                path = []
                current = nodes[-1]
                while current.get_parent():
                    path.append(current.get_position())
                    current = current.get_parent()
                return path
        return []

    def sample(self, p1, p2):
        """

        :param p1:
        :param p2:
        :return:
        """
        p = random.random()
        if p >= 1 - self.get_goal_probability():
            return self.line_to(p1, self.get_goal())
        elif p <= 1 - self.get_line_probability():
            return self.line_to(p1, p2)
        elif p <= (1 - self.get_line_probability() / 3.25):
            return self.uniform(p1[1])
        else:
            return self.ellipsoid(p1)

    def uniform(self, p):
        """
        Samples the search space immediately below and above the specified point as well as on the same level.
        :param p:
        :return:
        """
        sample_space = set()
        for i in [-1, 0, 1]:
            sample_space.union(self.world.get_walkable_space_by_level(p[1] + i))
        return random.sample(sample_space, 1)[0]

class Node(object):

    def __init__(self, position, parent=None, fscore=0.0, gscore=0.0, hscore=0.0):
        """
        Creates an instance of a node object.
        :param position:
        :param parent:
        """
        self.position = position
        self.parent = parent
        self.fscore = fscore
        self.gscore = gscore
        self.hscore = hscore

    def __eq__(self, other):
        """
        Determines if this node is equal to the specified node.
        :param other:
        :return:
        """
        return self.position == other.position

    def __ne__(self, other):
        """
        Determines if this node is not equal to the specified node.
        :param other:
        :return:
        """
        return not self.__eq__(other)

    def __str__(self):
        """
        Gets a string representation of this node.
        :return:
        """
        return "{0}".format(self.position)

    def get_parent(self):
        """
        Gets the parent of this node.
        :return:
        """
        return self.parent

    def get_position(self):
        """
        Gets a tuple (x, y, z) representing the coordinates of this node.
        :return:
        """
        return self.position

    def get_x(self):
        """
        Gets the x coordinate of this node.
        :return:
        """
        return self.position()[0]

    def get_y(self):
        """
        Gets the y coordinate of this node.
        :return:
        """
        return self.position()[1]

    def get_z(self):
        """
        Gets the z coordinate of this node.
        :return:
        """
        return self.position()[2]

    def get_fscore(self):
        """
        Gets the fscore of this node.
        :return:
        """
        return self.fscore

    def get_gscore(self):
        """
        Gets the gscore of this node.
        :return:
        """
        return self.gscore

    def get_hscore(self):
        """
        Gets the hscore of this node.
        :return:
        """
        return self.hscore

    def set_parent(self, parent):
        """
        Sets the parent of this node.
        :param parent:
        :return:
        """
        self.parent = parent

    def set_fscore(self, fscore):
        """
        Sets the fscore of this node.
        :param fscore:
        :return:
        """
        self.fscore = fscore

    def set_gscore(self, gscore):
        """
        Sets the gscore of this node.
        :param gscore:
        :return:
        """
        self.gscore = gscore

    def set_hscore(self, hscore):
        """
        Sets the hscore of this node.
        :param hscore:
        :return:
        """
        self.hscore = hscore

class Obstacle(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an obstacle region.
        :param xdims: The x dimensions of this obstacle region.
        :param ydims: The y dimensions of this obstacle region.
        :param zdims: The z dimensions of this obstacle region.
        """
        self.xlower = min(xdims)
        self.xupper = max(xdims)
        self.ylower = min(ydims)
        self.yupper = max(ydims)
        self.zlower = min(zdims)
        self.zupper = max(zdims)

    def __str__(self):
        return "{0} {1} {2} {3} {4} {5}".format(
            self.xlower, self.xupper, self.ylower, self.yupper, self.zlower, self.zupper)

    def inside(self, position):
        """
        Determines if a point is inside this obstacle region.
        :param position: The point to check.
        :return: True if the point is inside, False otherwise.
        """
        return (self.xlower <= position[0] < self.xupper and
                self.ylower <= position[1] < self.yupper and
                self.zlower <= position[2] < self.zupper)

    def get_xmin(self):
        """
        Gets the minimum x dimension of this obstacle region.
        :return: The min y dimension.
        """
        return self.xlower

    def get_xmax(self):
        """
        Gets the maximum x dimension of this obstacle region.
        :return: The max x dimension.
        """
        return self.xupper

    def get_ymin(self):
        """
        Gets the minimum y dimension of this obstacle region.
        :return: The min y dimension.
        """
        return self.ylower

    def get_ymax(self):
        """
        Gets the maximum y dimension of this obstacle region.
        :return: The max y dimension.
        """
        return self.yupper

    def get_zmin(self):
        """
        Gets the minimum z dimension of this obstacle region.
        :return: The min z dimension.
        """
        return self.zlower

    def get_zmax(self):
        """
        Gets the maximum z dimension of this obstacle region.
        :return: The max z dimension.
        """
        return self.zupper

class World(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates a world object.
        :param xdims: The x dimensions of the world.
        :param ydims: The y dimensions of the world.
        :param zdims: The z dimensions of the world.
        """
        self.xlower = min(xdims)
        self.xupper = max(xdims)
        self.ylower = min(ydims)
        self.yupper = max(ydims)
        self.zlower = min(zdims)
        self.zupper = max(zdims)
        self.obstacles = set()
        self.walkable = set()

    def get_xmin(self):
        """
        Get the minimum x dimension of the world.
        :return: The minimum x dimension of the world.
        """
        return self.xlower

    def get_xmax(self):
        """
        Get the maximum x dimension of the world.
        :return: The maximum x dimension of the world.
        """
        return self.xupper

    def get_ymin(self):
        """
        Get the minimum y dimension of the world.
        :return: The minimum y dimension of the world.
        """
        return self.ylower

    def get_ymax(self):
        """
        Get the maximum y dimension of the world.
        :return: The maximum y dimension of the world.
        """
        return self.yupper

    def get_zmin(self):
        """
        Get the minimum z dimension of the world.
        :return: The minimum z dimension of the world.
        """
        return self.zlower

    def get_zmax(self):
        """
        Get the maximum z dimension of the world.
        :return: The max z dimension of the world.
        """
        return self.zupper

    def get_obstacles(self):
        """
        Get a set containing all obstacles in the world.
        :return: A set containing all the obstacles.
        """
        return set.union(self.obstacles)

    def add_obstacle(self, obstacle):
        """
        Adds an obstacle object to the world.
        :param obstacle: An obstacle object.
        :return:
        """
        self.obstacles.add(obstacle)

    def generate_walkable_positions(self):
        """
        Generates walkable space inside the game world.
        :return: N/A
        """
        for obs in self.obstacles:
            for x in range(obs.get_xmin(), obs.get_xmax()):
                for z in range(obs.get_zmin(), obs.get_zmax()):
                    x += .5
                    z += .5
                    p = x, obs.get_ymax() + 1, z
                    if (self.is_valid(p) and
                            not self.is_blocked((x, obs.get_ymax() + 2, z))):
                        self.walkable.add(p)

    def get_walkable_space(self):
        """
        Gets all walkable space for the specified y.
        :param y: The y dimension of the game world.
        :return: All walkable space in the game world defined at y.
        """
        return self.walkable

    def get_walkable_space_by_level(self, y):
        """
        Gets walkable space by level.
        :param y: The level tha player is on.
        :return: A set object containing all the walkable space in this y level.
        """
        w = set()
        for p in filter(lambda pos: pos[1] == y, self.walkable):
            w.add(p)
        return w

    def in_bounds(self, position):
        """
        Determines if a point is inside the bounds of the game world.
        :param position: The position to check.
        :return: True if the point is inside the game world, False otherwise.
        """
        return (self.xlower <= position[0] < self.xupper and
                self.ylower <= position[1] < self.yupper and
                self.zlower <= position[2] < self.zupper)

    def is_blocked(self, position):
        """
        Determines if a specified point falls within the bounds of an obstacle region.
        :param position: The point to check.
        :return: True if the point falls within an obstacle region, False otherwise.
        """
        for obstacle in self.obstacles:
            if obstacle.inside(position):
                return True
        return False

    def is_valid(self, position):
        """
        Determines if a point is valid. A point must fall within the game world and not occupy an obstacle region.
        :param position: The point to check.
        :return: True if the point is valid, False otherwise.
        """
        return self.in_bounds(position) and not self.is_blocked(position)

    def is_walkable(self, p):
        """
        Determines if a point is walkable or not.
        :param p: The point to check.
        :return: True if the point is walkable, False otherwise.
        """
        for w in self.walkable:
            if p == w:
                return True
        return False

def malmo_test():
    #for i in range(len(constants.mission_xml)):
    for i in [0]:
        # Create an agent and generate a path
        agent = Agent(constants.start, constants.goal[i])
        agent.create_world((constants.lower[i][0], constants.upper[i][0]),
                           (constants.lower[i][1], constants.upper[i][1]),
                           (constants.lower[i][2], constants.upper[i][2]),
                           constants.mission_txt[i])
        path = agent.rapidly_exploring_random_tree()

        if not path:
            print("No path was found!")
            continue

        # Setup a malmo client
        agent_host = MalmoPython.AgentHost()

        try:
            agent_host.parse(sys.argv)
        except RuntimeError as e:
            print('ERROR:', e)
            print(agent_host.getUsage())
            exit(1)
        if agent_host.receivedArgument("help"):
            print(agent_host.getUsage())
            exit(0)

        my_mission = None
        # Load in the mission
        with open(constants.mission_xml[i], 'r') as f:
            print("Loading mission from {0}".format(constants.mission_xml[i]))
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
                    print("Error starting mission:", e)
                    exit(1)
                else:
                    time.sleep(2)

        # Loop until mission starts:
        print("Waiting for the mission to start ")
        world_state = agent_host.getWorldState()
        while not world_state.has_mission_begun:
            sys.stdout.write(".")
            time.sleep(1)
            world_state = agent_host.getWorldState()
            for error in world_state.errors:
                print("Error:", error.text)

        print()
        print("Mission running ")

        print path

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

        print("Mission ended ")
        print()


if __name__ == '__main__':
    malmo_test()