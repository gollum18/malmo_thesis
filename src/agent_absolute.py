from __future__ import division
from math import fabs, sqrt
import MalmoPython
import os
import sys
import time
import json

JUMP = "JUMP"
WALK = "WALK"
DROP = "DROP"
AIR = u'air'
START = (0.5, 56, 24.5)
GOAL = ([0.5, 56, -24.5], [0.5, 58, -24.5], [0.5, 55, -24.5], [0.5, 61, 0.5], [0.5, 56, -28.5])
HAZARDS = [u'lava', u'water']
POS = {
    0: [-1, -1], 1: [0, -1], 2: [1, -1],
    3: [-1, 0], 4: [0, 0], 5: [1, 0],
    6: [-1, 1], 7: [0, 1], 8: [1, 1]
}

def is_goal(cell, index):
    goal = GOAL[index]
    if cell.x == goal[0] and cell.y == goal[1] and cell.z == goal[2]:
        return True
    return False

def dist(p1, p2):
    dx = fabs(p1[0]-p2[0])
    dy = fabs(p1[1]-p2[1])
    dz = fabs(p1[2]-p2[2])
    return sqrt(dx*dx+dy*dy+dz*dz)

def neighbors(pos, sub, floor, level, roof, super):
    n = []

    for i in range(len(level)):
        # Ignore the block we are standing on
        if i == 4:
            continue
        dx, dz = POS[i]
        # Check for a block at level
        if level[i] != AIR and level[i] not in HAZARDS:
            if roof[i] == AIR and super[i] == AIR:
                # Then this is a block we can jump to
                n.append(((pos[0] + POS[i][0], pos[1] + 1, pos[2] + POS[i][1]), JUMP))
        # Check for a block at floor
        if floor[i] != AIR and floor[i] not in HAZARDS:
            # This is a block we can walk to
            if roof[i] == AIR:
                n.append(((pos[0] + POS[i][0], pos[1], pos[2] + POS[i][1]), WALK))
        if floor[i] == AIR:
            # This is a block we can drop to
            if sub[i] != AIR and sub[i] not in HAZARDS:
                n.append(((pos[0] + POS[i][0], pos[1] - 1, pos[2] + POS[i][1]), DROP))

    return n

sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)  # flush print output immediately

# More interesting generator string: "3;7,44*49,73,35:1,159:4,95:13,35:13,159:11,95:10,159:14,159:6,35:6,95:6;12;"

missions = [
    # Walk to goal mission
    './missions/pp_maze_one.xml',
    # Climb to goal mission
    './missions/pp_maze_two.xml',
    # Drop to goal mission
    './missions/pp_maze_three.xml',
    # Climb the big central pillar
    './missions/pp_maze_four.xml'
]

# Create default Malmo objects:

for i in range(len(missions)):
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
    with open(missions[i], 'r') as f:
        print "Loading mission from {0}".format(missions[i])
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
                print "Error starting mission:",e
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
            print "Error:",error.text

    print
    print "Mission running ",

    # Loop until mission ends:
    closed = set()
    gs = 0
    while world_state.is_mission_running:
        sys.stdout.write(".")
        world_state = agent_host.getWorldState()
        for error in world_state.errors:
            print "Error:",error.text
        if world_state.number_of_observations_since_last_state > 0:
            msg = world_state.observations[-1].text
            ob = json.loads(msg)
            pos = (ob.get(u'XPos'), ob.get(u'YPos'), ob.get(u'ZPos'))
            action = None
            best = None
            val = float("inf")
            for neighbor in neighbors(pos=pos, sub=ob.get(u'SubFloor'), floor=ob.get(u'Floor'),
                                      level=ob.get(u'Level'), roof=ob.get(u'Roof'), super=ob.get(u'SuperRoof')):
                if neighbor[0] not in closed:
                    temp = dist(neighbor[0], GOAL[i])
                    if temp + gs < val:
                        best = neighbor[0]
                        action = neighbor[1]
                        val = temp + gs
            gs += 1
            if best:
                closed.add(best)
                agent_host.sendCommand("tp {0} {1} {2}".format(best[0], best[1], best[2]))
                time.sleep(0.3)
    print
    print "Mission ended"
    # Mission has ended.
