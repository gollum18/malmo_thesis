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
        self.start = start
        self.goal = goal
        self.alpha = alpha
        self.beta = beta
        self.epsilon = epsilon
        self.max_nodes = max_nodes
        self.world = None
        self.traversed = set()

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def get_alpha(self):
        return self.alpha

    def get_beta(self):
        return self.beta

    def get_epsilon(self):
        return self.epsilon

    def get_max_nodes(self):
        return self.max_nodes

    def set_start(self, start):
        self.start = start

    def set_goal(self, goal):
        self.goal = goal

    def set_alpha(self, alpha):
        self.alpha = alpha

    def set_beta(self, beta):
        self.beta = beta

    def set_epsilon(self, epsilon):
        self.epsilon = epsilon

    def set_max_nodes(self, max_nodes):
        self.max_nodes = max_nodes

    def astar(self):
        open_list = PriorityQueue()
        closed_list = set()
        node = Node(self.start, None)
        node.set_gscore(0)
        open_list.push(node, 0)
        while open_list:
            current = open_list.pop()
            if self.is_goal(current.get_position()):
                return self.reconstruct_path(current)
            if current.get_position() not in closed_list:
                for neighbor in self.generate_neighbors(current.get_position()):
                    if neighbor not in self.world.walkable:
                        continue
                    node = Node(neighbor, current)
                    node.set_gscore(current.get_gscore() + cost(current.get_position(), node.get_position()))
                    open_list.push(node, node.get_gscore()+heuristic(current.get_position(), node.get_position()))
                closed_list.add(current.get_position())
        return []

    def create_world(self, xdims, ydims, zdims, filename):
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

    def generate_neighbors(self, p):
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
        return True if p == self.goal else False

    def line_to(self, p1, p2):
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
        mindist = float("inf")
        neighbor = None
        for n in self.world.walkable - self.traversed:
            temp = dist(p, n)
            if temp < mindist:
                mindist = temp
                neighbor = n
        return neighbor

    def random(self):
        return random.sample(self.world.walkable - self.traversed, 1)[0]

    def reconstruct_path(self, n):
        path = []
        while n.get_parent():
            path.append(n.get_position())
            n = n.get_parent()
        return path

    def rrt(self):
        nodes = [Node(self.start)]

        for i in range(self.max_nodes):
            rand = self.random()
            nn = nodes[0]
            for p in nodes:
                if dist(p.get_position(), rand) < dist(nn.get_position(), rand):
                    nn = p
            newnode = self.line_to(nn.get_position(), rand)
            nodes.append(Node(newnode, nn))
            self.traversed.add(newnode)
            if self.is_goal(nodes[-1].get_position()):
                return self.reconstruct_path(nodes[-1])

        return []


class Cube(object):

    def __init__(self, xdims, ydims, zdims):
        self.x_lower = xdims[0]
        self.x_upper = xdims[1]
        self.y_lower = ydims[0]
        self.y_upper = ydims[1]
        self.z_lower = zdims[0]
        self.z_upper = zdims[1]

    def __eq__(self, other):
        return (self.x_lower == other.x_lower and self.y_lower == other.y_lower and self.z_lower == other.z_lower and
                self.x_upper == other.x_upper and self.y_upper == other.y_upper and self.z_upper == other.z_upper)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper))

    def __str__(self):
        return "The Bounds of this Obstacle are -> \n\tx:({0}, {1}), ({2}, {3}), ({4}, {5})".format(
            self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper
        )

    def get_lower_bounds(self):
        return self.x_lower, self.y_lower, self.z_lower

    def get_upper_bounds(self):
        return self.x_upper, self.y_upper, self.z_upper

    def get_x_lower(self):
        return self.x_lower

    def get_x_upper(self):
        return self.x_upper

    def get_y_lower(self):
        return self.y_lower

    def get_y_upper(self):
        return self.y_upper

    def get_z_lower(self):
        return self.z_lower

    def get_z_upper(self):
        return self.z_upper

    def ontop(self, p):
        return True if p[1] == self.y_upper else False

    def player_inside(self, p, height=2):
        if height < 1:
            height = 2
        points = set()
        for dy in range(1, height+1):
            points.add((p[0], p[1]+dy-.5, p[2]))
        for point in points:
            if self.point_inside(point):
                return True
        return False

    def point_inside(self, p):
        if (self.x_lower < p[0] < self.x_upper and
                self.y_lower < p[1] < self.y_upper and
                self.z_lower < p[2] < self.z_upper):
            return True
        return False


class Node(object):

    def __init__(self, pos, parent=None):
        self.position = pos
        self.parent = parent
        self.gscore = 0.0

    def get_position(self):
        return self.position

    def get_parent(self):
        return self.parent

    def set_parent(self, parent):
        self.parent = parent

    def get_gscore(self):
        return self.gscore

    def set_gscore(self, gscore):
        self.gscore = gscore


class World(Cube):

    def __init__(self, xdims, ydims, zdims):
        super(World, self).__init__(xdims, ydims, zdims)
        self.obstacles = set()
        self.walkable = set()

    def add_obstacle(self, xdims, ydims, zdims):
        self.obstacles.add(Cube(xdims, ydims, zdims))

    def add_walkable_space(self, p):
        self.walkable.add(p)

    def is_point_blocked(self, p):
        for obs in self.obstacles:
            if obs.point_inside(p):
                return True
        return False

    def is_player_blocked(self, p, height=2):
        for obs in self.obstacles:
            if obs.player_inside(p, height):
                return True
        return False

    def is_on_obstacle(self, p):
        for obs in self.obstacles:
            if obs.ontop(p):
                return True
        return False

    def is_point_valid(self, p):
        return self.point_inside(p) and self.is_on_obstacle(p) and not self.is_point_blocked(p)

    def is_player_move_valid(self, p, height=2):
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

        #msg = world_state.observations[-1].text
        #obs = json.loads(msg)
        #print obs.get(u'XPos'), obs.get(u'YPos'), obs.get(u'ZPos')

if __name__ == '__main__':
    malmo()