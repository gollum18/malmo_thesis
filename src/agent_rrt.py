from __future__ import division
from decimal import Decimal, ROUND_HALF_UP
from collections import deque
import MalmoPython
import os
import sys
import json
import math
import time
import random

AIR = u'air'
DIAMOND = u'diamond'
START = (0.5, 56, 24.5)
GOALS = ([0.5, 56, -24.5], [0.5, 58, -24.5], [0.5, 55, -24.5], [0.5, 61, 0.5], [0.5, 56, -28.5])
HAZARDS = [u'lava', u'water']
MAX_DROP_HEIGHT = 20

# Represents the range of blocks that are 2 steps from agent
FAR = [0, 1, 2, 3, 4, 5, 9, 10, 14, 15, 19, 20, 21, 22, 23, 24]

# Represents the range of blocks that are 1 step from the agent
NEAR = [6, 7, 8, 11, 13, 16, 17, 18]

# Represents the agents tile
AGENT = 12

# Represents the change in x and z respectively from going to another cell discretely
# Cell 12 is always the players position
PDIFF = {
    0: [-2, -2], 1: [-1, -2], 2: [0, -2], 3: [1, -2], 4: [2, -2],
    5: [-2, -1], 6: [-1, -1], 7: [0, -1], 8: [1, -1], 9: [2, -1],
    10: [-2, 0], 11: [-1, 0], 12: [0, 0], 13: [1, 0], 14: [2, 0],
    15: [-2, 1], 16: [-1, 1], 17: [0, 1], 18: [1, 1], 19: [2, 1],
    20: [-2, 2], 21: [-1, 2], 22: [0, 2], 23: [1, 2], 24: [2, 2]
}

# Represents the world dimensions of the various maps
MDIMS = (
    ((-25, 25), (55, 70), (-25, 25)),
    ((-25, 25), (55, 70), (-25, 25)),
    ((-25, 25), (55, 70), (-25, 25)),
    ((-25, 25), (55, 70), (-25, 25)),
    ((-3, 3), (55, 75), (-30, 25))
)

missions = [
    # Walk to goal mission
    './missions/pp_maze_one.xml',
    # Climb to goal mission
    './missions/pp_maze_two.xml',
    # Drop to goal mission
    './missions/pp_maze_three.xml',
    # Climb the big central pillar
    './missions/pp_maze_four.xml',
    # Time for an obstacle course, combining everything into one
    './missions/pp_maze_five.xml'
]


class Timer:

    def __init__(self):
        self.current_time = time.time()

    def elapsed(self):
        return time.time() - self.current_time

    def reset(self):
        self.current_time = time.time()


class RRTAgent:

    def __init__(self, start=(0, 0, 0), goal=(15, 0, 15),
                 xdims=(-25, 25), ydims=(-10, 10), zdims=(-25, 25),
                 alpha=0.1, beta=1.25, epsilon=3.0, zulu=.1,
                 xrad=2.0, yrad=2.0, zrad=2.0,
                 move_time=0.5, path_time=0.5):
        if alpha < 0 or alpha >= 1: raise ValueError
        if beta <= 0: raise ValueError
        if epsilon <= 0: raise ValueError
        if xrad <= 0: raise ValueError
        if yrad <= 0: raise ValueError
        if zrad <= 0: raise ValueError
        self.world = {}
        self.stage = 0
        self.start = start
        self.position = start
        self.goal = goal
        self.dimensions = (xdims, ydims, zdims)
        self.alpha = alpha
        self.beta = beta
        self.epsilon = epsilon
        self.zulu = zulu
        self.radii = xrad, yrad, zrad
        self.move_time = move_time
        self.path_time = path_time
        self.pathing_timer = Timer()
        self.move_timer = Timer()

        # Construct an empty world state
        for z in range(self.get_zmin(), self.get_zmax()):
            for y in range(self.get_ymin(), self.get_ymax()):
                for x in range(self.get_xmin(), self.get_xmax()):
                    self.world[(x, y, z)] = AIR

        # Set the start and goal blocks
        self.world[(self.start[0], self.start[1], self.start[2])] = u'gold_block'
        self.world[(self.goal[0], self.goal[1], self.goal[2])] = u'diamond_block'

    def get_world_state_at_position(self, p):
        if self.in_bounds(p):
            return self.world[p]

    def set_world_state_at_position(self, p, s):
        if self.in_bounds(p) and self.get_world_state_at_position(p) != s:
            self.world[p] = s

    def get_xmin(self):
        return min(self.dimensions[0])

    def get_xmax(self):
        return max(self.dimensions[0])

    def get_ymin(self):
        return min(self.dimensions[1])

    def get_ymax(self):
        return max(self.dimensions[1])

    def get_zmin(self):
        return min(self.dimensions[2])

    def get_zmax(self):
        return max(self.dimensions[2])

    def get_start(self):
        return self.start

    def fpoint(self, p):
        return math.floor(p[0]), math.floor(p[1]), math.floor(p[2])

    @staticmethod
    def dist(p1, p2):
        return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]))

    def is_blocked(self, p):
        if self.get_world_state_at_position((math.floor(p[0]), math.floor(p[1]), math.floor(p[2]))):
            return True
        return False

    def is_drop(self, p):
        """
        Determines whether a point is a drop.
        :param p: The point to check.
        :return: True if the point will lead to a drop, false if not.
        """
        fp = self.fpoint(p)
        return True if self.get_world_state_at_position((fp[0], fp[1]-1, fp[2])) == AIR else False

    def drop_distance(self, p):
        """
        Determines the drop height for a specific point. Only needed if a point is a drop.
        :param p:
        :return:
        """
        fp = self.fpoint(p)
        i = 1
        while self.get_world_state_at_position((fp[0], fp[1]+i, fp[2])) == AIR and i <= MAX_DROP_HEIGHT:
            i += 1
        if i > MAX_DROP_HEIGHT:
            return -1
        if self.get_world_state_at_position((fp[0], fp[1]+i, fp[2])) in HAZARDS:
            return -1
        return i-1

    def is_climb(self, p):
        fp = self.fpoint(p)
        if self.is_blocked(fp):
            if self.get_world_state_at_position((fp[0], fp[1]+1, fp[2])) == AIR:
                if self.get_world_state_at_position((fp[0], fp[1]+2, fp[2])) == AIR:
                    return True
        return False

    def is_walkable(self, p):
        fp = self.fpoint(p)
        s = self.get_world_state_at_position((fp[0], fp[1]-1, fp[2]))
        if s != AIR and s not in HAZARDS:
            return True
        return False

    def is_valid_cell(self, p):
        if not self.in_bounds(p):
            return False
        if self.is_blocked(p):
            return False
        return True

    def los2d(self, p1, p2):
        """
        Checks for line of sight between blocks on the players level.
        Line of sight is always calculated from the players current position.
        :param p: The point to check line of sight for.
        :return: True if there is line of sight (i.e. the path is not blocked), False otherwise.
        """
        xc, xg = p1[0], p2[0]
        zc, zg = p1[2], p2[2]
        dx, dz = xg - xc, zg - zc
        sx, sz = 0, 0
        f = 0

        if dz < 0:
            dz = -dz
            sz = -1
        else:
            sz = 1

        if dx < 0:
            dx = -dx
            sx = -1
        else:
            sx = 1

        if dx >= dz:
            while xc != xg:
                f = f + dz
                # Case One
                if f >= dx:
                    if not self.is_valid_cell((xc+((sx-1)/2), p1[1], zc + (sz-1)/2)):
                        return False
                    zc = zc + sz
                    f = f - dx
                # Case Two
                if f != 0 and not self.is_valid_cell((xc+((sx-1)/2), p1[1], zc + (sz-1)/2)):
                    return False
                # Case Three
                if (dz == 0 and
                    not self.is_valid_cell((xc+((sx-1)/2), p1[1], zc)) and
                        not self.is_valid_cell((xc+((sx-1)/2), p1[1], zc-1))):
                        return False
                xc = xc + sx
        else:
            while zc != zg:
                f = f + dx
                # Case One
                if f >= dz:
                    if not self.is_valid_cell((xc+((sx-1)/2), p1[1], zc + (sz-1)/2)):
                        return False
                    xc = xc + sx
                    f = f - dz
                # Case Two
                if f != 0 and not self.is_valid_cell((xc+((sx-1)/2), p1[1], zc + (sz-1)/2)):
                    return False
                # Case Three
                if (dz == 0 and
                    not self.is_valid_cell((xc, p1[1], zc + (sz-1)/2)) and
                        not self.is_valid_cell((xc-1, p1[1], zc + (sz-1)/2))):
                    return False
                zc = zc + sz
        return True

    def los3d(self, p1, p2):
        """
        Determines line of sight between two points in 3D.
        :param p1: The point to check line of sight from.
        :param p2: The point to check line of sight to.
        :return: True if there is unobscured line of sight between the two points, False otherwise.
        """
        dx, dy, dz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
        s = max(dx, dy, dz)
        dx = Decimal(dx/s)
        dy = Decimal(dy/s)
        dz = Decimal(dz/s)
        cx, cy, cz = Decimal(p1[0]), Decimal(p1[1]), Decimal(p1[2])
        gx, gy, gz = p2
        while int(cx) != gx and int(cy) != gy and int(cz) != gz:
            cx += dx
            cy += dy
            cz += dz
            if not self.is_valid_cell((bankers_round(cx), bankers_round(cy), bankers_round(cz))):
                return False
        return True

    def rrt(self):
        nodes = [Node(self.position)]

        for i in range(50000):
            rand = self.uniform()
            nn = nodes[0]
            for p in nodes:
                if RRTAgent.dist(p.pos, rand) < RRTAgent.dist(nn.pos, rand):
                    nn = p
            nodes.append(Node(self.sample(nn.pos, rand), nn))
            if self.is_goal(nodes[-1].pos):
                print "We found a path :)"
                return self.reconstruct_path(nodes[-1])

        print "There was no path found :("

    def reconstruct_path(self, node):
        path = []

        while node.parent:
            path.append(node.pos)
            node = node.parent

        path.reverse()
        return path

    def is_goal(self, p):
        x, y, z = p
        if x >= self.goal[0]-.5 and x <= self.goal[0]+.5:
            if y >= self.goal[1]-.5 and y <= self.goal[1]+.5:
                if z >= self.goal[2]-.5 and z <= self.goal[2]+.5:
                    return True
        return False

    def in_bounds(self, p):
        x, y, z = p
        if x < self.get_xmin() or x > self.get_xmax():
            return False
        if y < self.get_ymin() or y > self.get_ymax():
            return False
        if z < self.get_zmin() or z > self.get_zmax():
            return False
        return True

    def ellipsoid(self):
        return (random.uniform(-self.radii[0], self.radii[0]),
                random.uniform(-self.radii[1], self.radii[1]),
                random.uniform(-self.radii[2], self.radii[2]))

    def neighbors(self, sub, floor, level, roof, super):
        n = []
        x, y, z = math.floor(self.position[0]), math.floor(self.position[1]), math.floor(self.position[2])

        for i in range(len(level)):
            # This section builds the representation of the world as we discover it
            if sub[i] != AIR:
                self.set_world_state_at_position((x+PDIFF[i][0], y-2, z+PDIFF[i][1]), sub[i])
            if floor[i] != AIR:
                self.set_world_state_at_position((x+PDIFF[i][0], y-1, z+PDIFF[i][1]), floor[i])
            if level[i] != AIR:
                self.set_world_state_at_position((x+PDIFF[i][0], y, z+PDIFF[i][1]), level[i])
            if roof[i] != AIR:
                self.set_world_state_at_position((x+PDIFF[i][0], y+1, z+PDIFF[i][1]), roof[i])
            if super[i] != AIR:
                self.set_world_state_at_position((x+PDIFF[i][0], y+2, z+PDIFF[i][1]), super[i])

        # This section generates valid neighbors from the current position
        # Check the far blocks first
        for i in FAR:

            above = self.get_world_state_at_position((x+PDIFF[i][0], y+1, z+PDIFF[i][1]))
            level = self.get_world_state_at_position((x+PDIFF[i][0], y, z+PDIFF[i][1]))
            below = self.get_world_state_at_position((x+PDIFF[i][0], y-1, z+PDIFF[i][1]))

            if (self.los2d(self.position, (x+PDIFF[i][0], y, z+PDIFF[i][1])) and
                    above != AIR and below not in HAZARDS and level == AIR and above == AIR and below != DIAMOND):
                n.append((x+PDIFF[i][0], y, z+PDIFF[i][1]))
        for i in NEAR:
            above = self.get_world_state_at_position((x+PDIFF[i][0], y+1, z+PDIFF[i][1]))
            level = self.get_world_state_at_position((x+PDIFF[i][0], y, z+PDIFF[i][1]))
            below = self.get_world_state_at_position((x+PDIFF[i][0], y-1, z+PDIFF[i][1]))
            above2 = self.get_world_state_at_position((x+PDIFF[i][0], y+2, z+PDIFF[i][1]))
            below2 = self.get_world_state_at_position((x+PDIFF[i][0], y-2, z+PDIFF[i][1]))
            # Walking case
            if (self.is_valid_cell((x+PDIFF[i][0], y, z+PDIFF[i][1])) and
                    below != DIAMOND):
                n.append((x+PDIFF[i][0], y, z+PDIFF[i][1]))
            # Jumping case
            elif level != AIR and level != DIAMOND and level not in HAZARDS:
                if above == AIR and above2 == AIR:
                    n.append((x+PDIFF[i][0], y+1, z+PDIFF[i][1]))
            # Dropping case
            elif below == AIR:
                if below2 == AIR and below2 != DIAMOND and below2 not in HAZARDS:
                    n.append((x+PDIFF[i][0], y-1, z+PDIFF[i][1]))

        return n

    def line_to(self, p1, p2):
        if self.dist(p1, p2) < self.epsilon:
            return p2
        else:
            theta = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
            mx, my = math.cos(theta), math.sin(theta)
            return p1[0] + mx * self.epsilon, p1[1] + my * self.epsilon, p2[1] + (my/mx) * self.epsilon

    def sample(self, p1, p2):
        p = random.random()
        # Sample towards the goal
        if 1-self.zulu <= p:
            point = self.line_to(p1, self.goal)
            while not (self.is_valid_cell(p) and self.los2d(p1, p2) if p2[1]-p1[1] == 0 else self.los3d(p1, p2)):
                point = self.line_to(p1, self.goal)
            return point
        elif p <= 1-self.alpha:
            point = self.line_to(p1, p2)
            while not (self.is_valid_cell(p) and self.los2d(p1, p2) if p2[1] - p1[1] == 0 else self.los3d(p1, p2)):
                point = self.line_to(p1, p2)
            return point
        elif p <= (1-self.alpha)/self.beta or not self.los2d(p1, p2) if p2[1]-p1[1] == 0 else self.los3d(p1, p2):
            point = self.uniform()
            while not (self.is_valid_cell(point) and self.los2d(p1, p2) if p2[1]-p1[1] == 0 else self.los3d(p1, p2)):
                point = self.uniform()
            return point
        else:
            point = self.ellipsoid()
            while not (self.is_valid_cell(p) and self.los2d(p1, p2) if p2[1] - p1[1] == 0 else self.los3d(p1, p2)):
                point = self.ellipsoid()
            return point

    def uniform(self):
        return (random.uniform(self.get_xmin(), self.get_xmax()),
                random.uniform(self.get_ymin(), self.get_ymax()),
                random.uniform(self.get_zmin(), self.get_zmax()))


class Node:

    def __init__(self, pos, parent=None):
        self.pos = pos
        self.parent = parent


def bankers_round(d):
    """
    Will return a decimal alue that is properly rounded up or down depending on bankers rounding.
    :param d: A decimal to round.
    :return: A rounded decimal.
    """
    return Decimal(d).quantize(0, ROUND_HALF_UP)


def dist(p1, p2):
    dx = math.fabs(p1[0]-p2[0])
    dy = math.fabs(p1[1]-p2[1])
    dz = math.fabs(p1[2]-p2[2])
    return math.sqrt(dx*dx+dy*dy+dz*dz)

# More interesting generator string: "3;7,44*49,73,35:1,159:4,95:13,35:13,159:11,95:10,159:14,159:6,35:6,95:6;12;"


def get_world_xdims(i):
    return MDIMS[i][0]


def get_world_ydims(i):
    return MDIMS[i][1]


def get_world_zdims(i):
    return MDIMS[i][2]


def testrrt():
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)  # flush print output immediately
    for i in range(len(missions)):
        # Get the agent host
        host = MalmoPython.AgentHost()
        host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)

        # Parse any command line arguments
        try:
            host.parse(sys.argv)
        except RuntimeError as e:
            print 'ERROR:', e
            print host.getUsage()
        if host.receivedArgument("help"):
            print host.getUsage()
            exit(0)

        mission = None
        # Load in the mission
        with open(missions[i], 'r') as f:
            print "Loading mission from {0}".format(missions[i])
            mission = MalmoPython.MissionSpec(f.read(), True)
        record = MalmoPython.MissionRecordSpec()

        # Attempt to start a mission
        retries = 3
        for retry in range(retries):
            try:
                host.startMission(mission, record)
            except RuntimeError as e:
                if retry == retries - 1:
                    print "Error starting missions:", e
                    exit(1)
                else:
                    time.sleep(2.5)

        # Loop until mission starts
        print "Waiting for the mission to start",
        world = host.getWorldState()
        while not world.has_mission_begun:
            sys.stdout.write(".")
            time.sleep(0.1)
            world = host.getWorldState()
            for error in world.errors:
                print "Error:", error.text

        print
        print "Mission running",

        # Create an rrt holding the agent
        rrt = RRTAgent(start=START, goal=GOALS[i], xdims=get_world_xdims(i), ydims=get_world_ydims(i),
                       zdims=get_world_zdims(i), epsilon=1.0, xrad=1.0, yrad=1.0, zrad=1.0)

        # Loop until missions ends
        openlist = deque()
        closedlist = set()
        path = None
        while world.is_mission_running:
            sys.stdout.write(".")
            time.sleep(.1)
            world = host.getWorldState()
            for error in world.errors:
                print "Error:", error.text
            # TODO: Complete the first stage, discovery
            # We basically want to do a breadth first search here without ending the mission
            # The objective is to just discover the world since we do not have it coded in
            if rrt.stage == 0 and world.number_of_observations_since_last_state > 0:
                msg = world.observations[-1].text
                ob = json.loads(msg)
                rrt.position = (ob.get(u'XPos'), ob.get(u'YPos'), ob.get(u'ZPos'))
                if rrt.position not in closedlist:
                    closedlist.add(rrt.position)
                    for neighbor in rrt.neighbors(ob.get(u'SubFloor'), ob.get(u'Floor'), ob.get(u'Level'),
                                                    ob.get(u'Roof'), ob.get(u'SuperRoof')):
                        openlist.append(neighbor)
                if not openlist:
                    # Reset the agents position to the start
                    host.sendCommand("tp {0} {1} {2}".format(rrt.get_start()[0], rrt.get_start()[1], rrt.get_start()[2]))
                    time.sleep(.3)
                    rrt.stage = 1
                    continue
                popped = openlist.popleft()
                host.sendCommand("tp {0} {1} {2}".format(popped[0], popped[1], popped[2]))
                time.sleep(.3)
            # TODO: Complete the second stage, pathing
            # At this stage, we simulate rrt on the emulated environment
            # We get a path back from the agent and pass it to the MalmoAgent in stage 3
            elif rrt.stage == 1:
                path = rrt.rrt()
                if path:
                    stage = 2
                    continue
                else:
                    break
            elif rrt.stage == 2:
                pos = path.pop()
                host.sendCommand("tp {0} {1} {2}".format(pos[0], pos[1], pos[2]))
                time.sleep(.3)

def test3d():

    agent = RRTAgent()
    nodes = [Node(agent.position)]
    found = False

    for i in range(50000):
        rand = agent.uniform()
        nn = nodes[0]
        for p in nodes:
            if RRTAgent.dist(p.pos, rand) < RRTAgent.dist(nn.pos, rand):
                nn = p
        nodes.append(Node(agent.sample(nn.pos, rand), nn))
        if agent.is_goal(nodes[-1].pos):
            found = True
            path = agent.reconstruct_path(nodes[-1])
            print "Path is: "
            for j in range(len(path)):
                print "{0}.) {1}".format(j+1, path[j])
            break

    if not found:
        print "There was no path found :("

if __name__ == '__main__':
    testrrt()