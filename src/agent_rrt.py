# ------------------------------------------------------------------------------------------------
# Copyright (c) 2016 Microsoft Corporation
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all copies or
# substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ------------------------------------------------------------------------------------------------

# Tutorial sample #2: Run simple mission using raw XML

from __future__ import division
from math import floor, sqrt, tan, atan2
import MalmoPython
import os
import sys
import time
import json

GOAL = [0, 56, -24]
HAZARDS = [u'lava', u'water']
POS = {
    0: [-1, -1], 1: [0, -1], 2: [1, -1],
    3: [-1, 0], 4: [0, 0], 5: [1, 0],
    6: [-1, 1], 7: [0, 1], 8: [1, 1]
}

sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)  # flush print output immediately

# More interesting generator string: "3;7,44*49,73,35:1,159:4,95:13,35:13,159:11,95:10,159:14,159:6,35:6,95:6;12;"

missionXML='''<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
            <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
            
              <About>
                <Summary>Maze One</Summary>
              </About>
              
              <ServerSection>
                  <ServerInitialConditions>
                      <Time>
                          <StartTime>12000</StartTime>
                          <AllowPassageOfTime>false</AllowPassageOfTime>
                      </Time>
                      <AllowSpawning>false</AllowSpawning>
                  </ServerInitialConditions>
                <ServerHandlers>
                  <FlatWorldGenerator generatorString="3;7,44*49,73,35:1,159:4,95:13,35:13,159:11,95:10,159:14,159:6,35:6,95:6;12;"/>
                    <DrawingDecorator>
                        <DrawCuboid x1="-25" y1="55" z1="-25" x2="25" y2="70" z2="-25" type="obsidian"/>
                        <DrawCuboid x1="-25" y1="55" z1="25" x2="25" y2="70" z2="25" type="obsidian"/>
                        <DrawCuboid x1="-25" y1="55" z1="-25" x2="-25" y2="70" z2="25" type="obsidian"/>
                        <DrawCuboid x1="25" y1="55" z1="-25" x2="25" y2="70" z2="25" type="obsidian"/>
                        <DrawBlock x="0" y="55" z="-24" type="diamond_block"/>
                        <DrawBlock x="0" y="55" z="24" type="gold_block"/>
                        <DrawCuboid x1="-5" y1="55" z1="-5" x2="5" y2="60" z2="5" type="obsidian"/>
                    </DrawingDecorator>
                  <ServerQuitFromTimeUp timeLimitMs="30000"/>
                  <ServerQuitWhenAnyAgentFinishes/>
                </ServerHandlers>
              </ServerSection>
              
              <AgentSection mode="Survival">
                <Name>Puzzle Solver</Name>
                <AgentStart>
                    <Placement x="0.5" y="56" z="24.5" yaw="180"/>
                </AgentStart>
                <AgentHandlers>
                  <ObservationFromGrid>
                    <Grid name="Floor">
                        <min x="-1" y="-1" z="-1"/>
                        <max x="1" y="-1" z="1"/>
                    </Grid>
                    <Grid name="Level">
                        <min x="-1" y="0" z="-1"/>
                        <max x="1" y="0" z="1"/>
                    </Grid>
                    <Grid name="Roof">
                        <min x="-1" y="1" z="-1"/>
                        <max x="1" y="1" z="1"/>
                    </Grid>
                  </ObservationFromGrid>
                  <ObservationFromFullStats/>
                  <AbsoluteMovementCommands/>
                  <AgentQuitFromTouchingBlockType>
                    <Block type="diamond_block"/>
                  </AgentQuitFromTouchingBlockType>
                </AgentHandlers>
              </AgentSection>
            </Mission>'''

# Create default Malmo objects:

def round_even(number):
    return round(number *2) / 2

def dist(p1, p2):
    dx, dy, dz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
    return sqrt(dx*dx+dy*dy+dz*dz)

def is_goal(point):
    x, y, z = point
    xg, yg, zg = GOAL
    if x >= xg and x <= xg+1:
        if y >= yg and y <= yg+1:
            if z >= zg and z <= zg+1:
                return True
    return False

def generate_neighbors(point, floor, level, roof):
    neighbors = []

    for i in range(len(floor)):
        # Ignore the block were currently at
        if i == 4:
            continue

        if is_goal((point[0]+POS[i][0], point[1], point[2]+POS[i][1])):
            return [((point[0]+POS[i][0], point[1], point[2]+POS[i][1]), "WALK_ON")]

        if floor[i] != u'air' and floor[i] not in HAZARDS:
            # Regular walking case
            if level[i] == u'air' and roof[i] == u'air':
                neighbors.append(((point[0]+POS[i][0], point[1], point[2]+POS[i][1]), "WALK_ON"))
            # Jump to elevation case
            elif level[i] != u'air' and level[i] not in HAZARDS:
                if roof[i] == u'air':
                    neighbors.append(((point[0] + POS[i][0], point[1], point[2] + POS[i][1]), "JUMP_ON"))
            # Crouching case
            elif level[i] == u'air' and roof[i] != u'air':
                neighbors.append(((point[0] + POS[i][0], point[1], point[2] + POS[i][1]), "CROUCH_ON"))

    return neighbors

def determine_camera_angles(camera, point):
    dx, dy = point[1]-camera[1], point[0]-camera[0]
    azimuth = atan2(dy, dx)
    polar = tan(sqrt(dx*dx+dy*dy), point(2))
    return azimuth, polar

agent_host = MalmoPython.AgentHost()

try:
    agent_host.parse( sys.argv )
except RuntimeError as e:
    print 'ERROR:',e
    print agent_host.getUsage()
    exit(1)
if agent_host.receivedArgument("help"):
    print agent_host.getUsage()
    exit(0)

my_mission = MalmoPython.MissionSpec(missionXML, True)
my_mission_record = MalmoPython.MissionRecordSpec()

# Attempt to start a mission:
max_retries = 3
for retry in range(max_retries):
    try:
        agent_host.startMission( my_mission, my_mission_record )
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
    time.sleep(0.1)
    world_state = agent_host.getWorldState()
    for error in world_state.errors:
        print "Error:",error.text

print
print "Mission running ",

# Loop until mission ends:
closed = set()
while world_state.is_mission_running:
    sys.stdout.write(".")
    world_state = agent_host.getWorldState()
    for error in world_state.errors:
        print "Error:",error.text
    if world_state.number_of_observations_since_last_state > 0:
        msg = world_state.observations[-1].text
        ob = json.loads(msg)
        best = None
        action = None
        value = float("inf")
        cell = ob.get(u'XPos'), ob.get(u'YPos'),  ob.get(u'ZPos')
        for neighbor in generate_neighbors(cell, ob.get(u'Floor'), ob.get(u'Level'), ob.get(u'Roof')):
            temp = dist(neighbor[0], GOAL)
            if temp < value:
                if neighbor[0] not in closed:
                    value = temp
                    best = neighbor[0]
                    action = neighbor[1]
        if best:
            closed.add(best)
            agent_host.sendCommand("tp {0} {1} {2}".format(best[0], best[1], best[2]))
        time.sleep(0.3)
print
print "Mission ended"
# Mission has ended.
