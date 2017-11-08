import MalmoPython
import math, time, sys, random
from util import PriorityQueue

#
# Program Constants
#

_mission_files = ('./missions/pp_maze_one.xml',
                 './missions/pp_maze_two.xml',
                 './missions/pp_maze_three.xml',
                 './missions/pp_maze_four.xml')
_mission_text_files = ('./missions/pp_maze_one.txt',
                      './missions/pp_maze_two.txt',
                      './missions/pp_maze_three.txt',
                      './missions/pp_maze_four.txt')
_mission_lower_bounds = ((-24, 55, -24), (-24, 55, -24), (-25, 53, -24), (-24, 55, -24))
_mission_upper_bounds = ((26, 70, 25), (25, 70, 25), (25, 70, 25), (25, 70, 25))
_mission_start = (0.5, 56, 24.5)
_mission_goal = ((0.5, 56, -23.5), (0.5, 58, -23.5), (0.5, 54, -23.5), (0.5, 56, 0.5))

# Needed by the neighbor generation algorithm
_diff = [-1, 0, 1]


def cost(p1, p2):
    """
    Used by A* to determine the cost of going from p1 to p2.
    :param p1: The initial point.
    :param p2: The ending point.
    :return: The cost of going from p1 to p2.
    """
    return 1 + math.fabs(p2[1] - p1[1])


def dist(p1, p2):
    """
    Determines the euclidean distance between p1 and p2.
    :param p1: The initial point.
    :param p2: The ending point.
    :return: The distance between p1 and p2.
    """
    dx = p2[0]-p1[0]
    dy = p2[1]-p1[1]
    dz = p2[2]-p1[2]
    return math.sqrt(dx*dx+dy*dy+dz*dz)


def heuristic(p1, p2):
    """
    Used by A* to determine a best guess estimate of the remaining distance between p1 and p2.
    :param p1: The initial point.
    :param p2: The ending point.
    :return: The estimated distance between p1 and p2.
    """
    dx = math.fabs(p2[0] - p1[0])
    dy = math.fabs(p2[1] - p1[1])
    dz = math.fabs(p2[2] - p1[2])
    return dx * 1.0 + dy * 1.5 + dz * 1.0


class Agent(object):

    def __init__(self, start, goal, alpha=0.05, beta=3.25, epsilon=7.0, max_nodes=5000):
        """
        Creates a representation of a pathfinding agent.
        :param start: The starting node of the agent.
        :param goal: The goal node of the agent.
        :param alpha: The probability of sampling.
        :param beta: A modifier that affects sampling.
        :param epsilon: The maximum distance between nodes.
        :param max_nodes: The maximum amount of nodes an rrt agent can generate.
        """
        self.start = start
        self.goal = goal
        self.alpha = alpha
        self.beta = beta
        self.epsilon = epsilon
        self.max_nodes = max_nodes
        self.world = None
        self.traversed = set()

    def get_start(self):
        """
        Gets the starting state of the agent.
        :return: The starting state of the agent.
        """
        return self.start

    def get_goal(self):
        """
        Gets the goal state of the agent.
        :return: The starting state of the agent.
        """
        return self.goal

    def get_alpha(self):
        """
        The probability used when sampling in rrt.
        :return: Gets the probability used when sampling in rrt.
        """
        return self.alpha

    def get_beta(self):
        """
        Gets the modifier used when sampling in rrt.
        :return: The modifier used when sampling in rrt.
        """
        return self.beta

    def get_epsilon(self):
        """
        Gets the maximum amount of distance allowed between nodes in rrt.
        :return: The maximum distance allowed between each node in rrt.
        """
        return self.epsilon

    def get_max_nodes(self):
        """
        Gets the maximum amount of nodes rrt can generate.
        :return: The maximum amount of node rrt can generate.
        """
        return self.max_nodes

    def set_start(self, start):
        """
        Sets the starting state of the agent.
        :param start: The starting state of the agent.
        :return: N/A
        """
        self.start = start

    def set_goal(self, goal):
        """
        Sets the goal state of the agent.
        :param goal: The goal state of the agent.
        :return: N/A
        """
        self.goal = goal

    def set_alpha(self, alpha):
        """
        Sets the probability used when sampling.
        :param alpha: A probability between 0 and 1. Small values work best.
        :return: N/A
        """
        self.alpha = alpha

    def set_beta(self, beta):
        """
        Sets the modifier used when sampling in rrt.
        :param beta: The new sampling modifier.
        :return: N/A
        """
        self.beta = beta

    def set_epsilon(self, epsilon):
        """
        Sets the maximum distance between nodes in rrt.
        :param epsilon: The maximum distance between nodes in rrt.
        :return: N/A
        """
        self.epsilon = epsilon

    def set_max_nodes(self, max_nodes):
        """
        Sets the max amount of nodes that rrt can generate.
        :param max_nodes: The maximum amount of nodes.
        :return: N?A
        """
        self.max_nodes = max_nodes

    def astar(self):
        """
        Finds a path using the A* search algorithm.
        :return: A path if there is one otherwise an empty list.
        """
        open_list = PriorityQueue()
        closed_list = set()
        node = Node(self.start, None)
        node.set_gscore(0)
        open_list.push(node, 0)
        while not open_list.isEmpty():
            current = open_list.pop()
            if self.is_goal(current.get_position()):
                return self.reconstruct_path(current)
            if current.get_position() not in closed_list:
                for neighbor in self.generate_neighbors(current.get_position()):
                    if neighbor not in self.world.walkable:
                        continue
                    node = Node(neighbor, current)
                    node.set_gscore(current.get_gscore() + cost(current.get_position(), node.get_position()))
                    open_list.push(node, node.get_gscore() + heuristic(current.get_position(), node.get_position()))
                closed_list.add(current.get_position())
        return []

    def create_world(self, xdims, ydims, zdims, filename):
        """
        Creates a representation of the missions world.
        :param xdims: The dimensions of the world in the x axis (length).
        :param ydims: The dimensions of the world in the y axis (height).
        :param zdims: The dimensions of the world in the z axis (width).
        :param filename: The filename of the file that contains the layout of the mission.
        :return: N/A
        """
        if not xdims or not ydims or not zdims or not filename:
            raise ValueError
        reader = open(filename, 'r')
        if not reader:
            raise IOError
        # Create a world state
        self.world = World(xdims, ydims, zdims)
        # Read in obstacle information from file.
        ignoring = False
        for line in reader.readlines():
            line = line.strip("\n")
            if line.startswith("/*") and not ignoring:
                ignoring = True
                continue
            if line.endswith("*/") and ignoring:
                ignoring = False
                continue
            if not ignoring:
                if line.startswith("//") or not line:
                    continue
                parts = [int(part) for part in line.split()]
                self.world.add_obstacle((parts[0], parts[1]), (parts[2], parts[3]), (parts[4], parts[5]))
        reader.close()
        # Begin flood filling from the start to initialize walkable space.
        prospective = [self.start]
        evaluated = set()
        while prospective:
            current = prospective.pop(0)
            for neighbor in self.generate_neighbors(current):
                if neighbor in prospective or neighbor in evaluated:
                    continue
                prospective.append(neighbor)
            evaluated.add(current)
            self.world.add_walkable_space(current)
        print()

    def ellipsoid(self, p, xr=7.0, yr=3.0, zr=7.0):
        """
        Randomly samples a point from the ellipsoid region around the agent.
        :param p: The center of the ellipsoid.
        :param xr: The length of the x radius.
        :param yr: The length of the y radius.
        :param zr: The length of the z radius.
        :return: A randomly sampled point from the ellipsoid around the agent.
        """
        theta = random.uniform(0, 2*math.pi)
        phi = random.uniform(-math.pi/2, math.pi/2)
        return (math.floor(xr*math.cos(theta)*math.cos(phi))+.5,
                math.floor(yr*math.sin(phi)),
                math.floor(zr*math.sin(theta)*math.cos(phi))+.5)

    def generate_neighbors(self, p):
        """
        Generates octilian neighbors from the given point.
        :param p: The point to generate neighbors for.
        :return: A list containing points from.
        """
        neighbors = []
        for dx in _diff:
            for dy in _diff:
                for dz in _diff:
                    n = (p[0]+dx, p[1]+dy, p[2]+dz)
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    if self.world.is_player_move_valid(n):
                        neighbors.append(n)
        return neighbors

    def is_goal(self, p):
        """
        Determines if the specified point is the goal node or not.
        :param p: The point to check.
        :return: True if the point is a goal, False otherwise.
        """
        return True if p == self.goal else False

    def line_of_sight(self, p1, p2):
        """
        Determines line of sight from p1 to p2.
        :param p1: The observing point.
        :param p2: The observed point.
        :return: True if there is line of sight from p1 to p2, False otherwise.
        """
        def points_on_line(q, r):
            """
            Recursively generates all the points on the line defined by q anr r.
            :param q: The first terminus on the line.
            :param r: The second terminus on the line.
            :return: A list containing all'' the points on the line from q to r.
            """
            if dist(q, r) < 1.0:
                return []
            else:
                m = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2
                return points_on_line(p1, m) + [m] + points_on_line(m, p2)

        for point in points_on_line(p1, p2):
            for obs in self.world.obstacles:
                if obs.point_inside(point):
                    return False
        return True

    def line_to(self, p1, p2):
        """
        Randomly samples a point along the line defined by p1 and p2.
        :param p1: The first terminus of the line.
        :param p2: The second terminus of the line.
        :return: A randomly sampled point defined by p1 and p2.
        """
        if dist(p1, p2) < self.epsilon:
            return p2
        else:
            theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
            p = (math.floor(p1[0] + self.epsilon*math.cos(theta))+.5,
                    math.floor(p1[1] + self.epsilon*math.sin(theta)),
                    math.floor(p1[2] + self.epsilon*math.sin(theta)*math.cos(theta))+.5)
            if p in self.world.walkable - self.traversed:
                return p
            else:
                return self.nearest_neighbor(p)

    def nearest_neighbor(self, p):
        """
        Finds the nearest point from walkable space.
        :param p: The point to find the nearest neighbor of.
        :return: The nearest neighbor in walkable space from this point.
        """
        mindist = float("inf")
        neighbor = None
        for n in self.world.walkable - self.traversed:
            temp = dist(p, n)
            if temp < mindist:
                mindist = temp
                neighbor = n
        return neighbor

    def random(self):
        """
        Randomly samples a point from walkable space.
        :return: A randomly sampled point from available walkable space.
        """
        return random.sample(self.world.walkable - self.traversed, 1)[0]

    def reconstruct_path(self, n):
        """
        Constructs a path from a goal node.
        :param n: The goal node that was found by a planning algorithm.
        :return: A list containing the path.
        """
        path = []
        while n.get_parent():
            path.append(n.get_position())
            n = n.get_parent()
        return path

    def rrt(self):
        """
        Finds a path in the world using a rapidly exploring random tree.
        :return: A path if it is found, otherwise an empty list.
        """
        nodes = [Node(self.start)]

        for i in range(self.max_nodes):
            rand = self.random()
            nn = nodes[0]
            for p in nodes:
                if dist(p.get_position(), rand) < dist(nn.get_position(), rand):
                    nn = p
            newnode = self.sample(nn.get_position(), rand)
            nodes.append(Node(newnode, nn))
            self.traversed.add(newnode)
            if self.is_goal(nodes[-1].get_position()):
                return self.reconstruct_path(nodes[-1])

        return []

    def sample(self, p1, p2):
        """
        Samples a point from walkable space.
        :param p1: A point evaluating from.
        :param p2: A randomly drawn walkable point.
        :return: A point from walkable space.
        """
        p = random.random()
        if p > 1 - self.alpha:
            return self.line_to(p1, p2)
        elif p <= (1 - self.alpha) / self.beta or not self.line_of_sight(p1, p2):
            return self.random()
        else:
            return self.ellipsoid(p1)


class Cube(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of a cuboid object.
        :param xdims: The dimensions of the cuboid in the x axis (length).
        :param ydims: The dimensions of the cuboid in the y axis (height).
        :param zdims: The dimensions of the cuboid in the z axis (width).
        """
        self.x_lower = xdims[0]
        self.x_upper = xdims[1]
        self.y_lower = ydims[0]
        self.y_upper = ydims[1]
        self.z_lower = zdims[0]
        self.z_upper = zdims[1]

    def __eq__(self, other):
        """
        Determines another obstacle is equal to this one.
        :param other: The other obstacle to check.
        :return: True if the two obstacles share the same dimensions, False otherwise.
        """
        return (self.x_lower == other.x_lower and self.y_lower == other.y_lower and self.z_lower == other.z_lower and
                self.x_upper == other.x_upper and self.y_upper == other.y_upper and self.z_upper == other.z_upper)

    def __ne__(self, other):
        """
        Determines another obstacle is not equal to this one.
        :param other: The other obstacle to check.
        :return: True if the two obstacles do not share the same dimensions, False otherwise.
        """
        return not self.__eq__(other)

    def __hash__(self):
        """
        Gets a hash representing this obstacle.
        :return: A hash representing this obstacle.
        """
        return hash((self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper))

    def __str__(self):
        """
        Gets an informal string representation of this obstacle.
        :return: An informal string representing the obstacle.
        """
        return "The Bounds of this Obstacle are -> \n\tx:({0}, {1}), ({2}, {3}), ({4}, {5})".format(
            self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper
        )

    def get_lower_bounds(self):
        """
        Gets a tuple (x, y, z) containing the lower bounds of the obstacle.
        :return: A tuple (x, y, z) containing the lower bounds of the obstacle.
        """
        return self.x_lower, self.y_lower, self.z_lower

    def get_upper_bounds(self):
        """
        Gets a tuple (x, y, z) containing the upper bounds of the obstacle.
        :return: A tuple (x, y, z) containing the upper bounds of the obstacle.
        """
        return self.x_upper, self.y_upper, self.z_upper

    def get_x_lower(self):
        """
        Gets the lower bound of the obstacle on the x dimension.
        :return: The lower bound of the obstacle on the x dimension.
        """
        return self.x_lower

    def get_x_upper(self):
        """
        Gets the upper bound of the obstacle on the x dimension.
        :return: The upper bound of the obstacle on the x dimension.
        """
        return self.x_upper

    def get_y_lower(self):
        """
        Gets the lower bound of the obstacle on the y dimension.
        :return: The lower bound of the obstacle on the y dimension.
        """
        return self.y_lower

    def get_y_upper(self):
        """
        Gets the upper bound of the obstacle on the y dimension.
        :return: The upper bound of the obstacle on the y dimension.
        """
        return self.y_upper

    def get_z_lower(self):
        """
        Gets the lower bound of the obstacle on the z dimension.
        :return: The lower bound of the obstacle on the z dimension.
        """
        return self.z_lower

    def get_z_upper(self):
        """
        Gets the upper bound of the obstacle on the z dimension.
        :return: The upper bound of the obstacle on the z dimension.
        """
        return self.z_upper

    def ontop(self, p):
        """
        Determines if a point is ontop of this obstacle.
        :param p: The point to check.
        :return: True if the point is ontop of the obstacle, False otherwise.
        """
        if (self.x_lower <= p[0] <= self.x_upper and
                self.y_upper == p[1] and
                self.z_lower <= p[2] <= self.z_upper):
            return True
        return False

    def player_inside(self, p, height=2):
        """
        Determines if the player is inside the obstacle.
        :param p: The point to check.
        :param height: The players height in units >= 1.
        :return: Determines if the player collides with this object.
        """
        if height < 1:
            height = 1
        points = set()
        for dy in range(1, height+1):
            points.add((p[0], p[1]+dy-.5, p[2]))
        for point in points:
            if self.point_inside(point):
                return True
        return False

    def point_inside(self, p):
        """
        Determines if the point is inside this object or not.
        :param p: The point to check.
        :return: True if the point is inside this object, False otherwise.
        """
        if (self.x_lower < p[0] < self.x_upper and
                self.y_lower < p[1] < self.y_upper and
                self.z_lower < p[2] < self.z_upper):
            return True
        return False


class Node(object):

    def __init__(self, pos, parent=None):
        """
        Creates an instance of a node object.
        :param pos: The position of this node.
        :param parent: The parent of this nod. Default is None.
        """
        self.position = pos
        self.parent = parent
        self.gscore = 0.0

    def get_position(self):
        """
        Gets the position of this node.
        :return: The position of this node.
        """
        return self.position

    def get_parent(self):
        """
        Gets the parent of this node.
        :return: The parent of this node.
        """
        return self.parent

    def set_parent(self, parent):
        """
        Sets the parent of this node.
        :param parent: The new parent of this node.
        :return: N/A
        """
        self.parent = parent

    def get_gscore(self):
        """
        Gets the gscore of this node.
        :return: The gscore of this node.
        """
        return self.gscore

    def set_gscore(self, gscore):
        """
        Sets the new gscore of this node.
        :param gscore: The new gscore of this node.
        :return: N/A
        """
        self.gscore = gscore


class World(Cube):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of a world object.
        :param xdims: The dimensions of the world in the x axis (length).
        :param ydims: The dimensions of the world in the y axis (height).
        :param zdims: The dimensions of the world in the z axis (width).
        """
        super(World, self).__init__(xdims, ydims, zdims)
        self.obstacles = set()
        self.walkable = set()

    def add_obstacle(self, xdims, ydims, zdims):
        """
        Adds an obstacle to the agents world representation.
        :param xdims: The dimensions of the obstacle in the x axis (length).
        :param ydims: The dimensions of the obstacle in the y axis (height).
        :param zdims: The dimensions of the obstacle in the z axis (width).
        :return: N/A
        """
        self.obstacles.add(Cube(xdims, ydims, zdims))

    def add_walkable_space(self, p):
        """
        Adds a point of walkable space to the agents world representation.
        :param p: The point ot add walkable space to.
        :return: N?A
        """
        self.walkable.add(p)

    def is_point_blocked(self, p):
        """
        Determines if the point is blocked or not.
        :param p: The point to check.
        :return: True if the point is blocked by an obstacle, False otherwise.
        """
        for obs in self.obstacles:
            if obs.point_inside(p):
                return True
        return False

    def is_player_blocked(self, p, height=2):
        """
        Determines if the specified point contains and region (determined by the players height) is blocked.
        :param p: The point to check.
        :param height: The height of the player in units >= 1.
        :return: True if the player cannot move to the specified point, False otherwise.
        """
        for obs in self.obstacles:
            if obs.player_inside(p, height):
                return True
        return False

    def is_on_obstacle(self, p):
        """
        Determines if the point is on top of an obstacle.
        :param p: The point to check.
        :return: True if the point is on an obstacle, False otherwise.
        """
        for obs in self.obstacles:
            if obs.ontop(p):
                return True
        return False

    def is_point_valid(self, p):
        """
        Determines if the specified point is inside the bounds of the map and is not an obstacle.
        :param p: The point to check.
        :return: True if the point is valid, False otherwise.
        """
        return self.point_inside(p) and self.is_on_obstacle(p) and not self.is_point_blocked(p)

    def is_player_move_valid(self, p, height=2):
        """
        Determines if a player can move to the specified location.
        :param p: The point to check.
        :param height: The height of the player in units >= 1.
        :return: True if the move is valid, False otherwise.
        """
        return self.player_inside(p) and self.is_on_obstacle(p) and not self.is_player_blocked(p, height)

def malmo():
    # Create default Malmo objects:
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

    if agent_host.receivedArgument("test"):
        num_repeats = 1
    else:
        num_repeats = 10

    for i in range(len(_mission_files)):
        # Attempt to start a mission:
        my_mission = MalmoPython.MissionSpec(open(_mission_files[i], 'r').read(), False)
        planner = Agent(_mission_start, _mission_goal[i])
        planner.create_world((_mission_lower_bounds[i][0], _mission_upper_bounds[i][0]),
                             (_mission_lower_bounds[i][1], _mission_upper_bounds[i][1]),
                             (_mission_lower_bounds[i][2], _mission_upper_bounds[i][2]),
                             _mission_text_files[i])
        path = planner.astar()
        print(path)

        max_retries = 3
        for retry in range(max_retries):
            try:
                agent_host.startMission(my_mission, MalmoPython.MissionRecordSpec())
                break
            except RuntimeError as e:
                if retry == max_retries - 1:
                    print("Error starting mission:", e)
                    exit(1)
                else:
                    time.sleep(2)

        # Loop until mission starts:
        print("Waiting for the mission to start ",)
        world_state = agent_host.getWorldState()
        while not world_state.has_mission_begun:
            sys.stdout.write(".")
            time.sleep(.5)
            world_state = agent_host.getWorldState()
            for error in world_state.errors:
                print("Error:", error.text)

        print()
        print("Mission running ",)

        # Loop until mission ends:
        while world_state.is_mission_running:
            sys.stdout.write(".")
            time.sleep(1.0)
            world_state = agent_host.getWorldState()
            if path:
                position = path.pop()
                agent_host.sendCommand("tp {0} {1} {2}".format(*position))

        print()
        print("Mission ended")
        # Mission has ended.

if __name__ == '__main__':
    malmo()