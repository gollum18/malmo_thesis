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
from math import fabs, floor, sqrt, tan, atan2
from Queue import PriorityQueue
import MalmoPython
import os
import sys
import time
import json

AIR = u'air'
START = (0.5, 56, 24.5)
GOAL = ([0.5, 56, -24.5], [0.5, 58, -24.5], [0.5, 55, -24.5], [0.5, 61, 0.5])
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
                n.append((pos[0] + POS[i][0], pos[1] + 1, pos[2] + POS[i][1]))
        # Check for a block at floor
        if floor[i] != AIR and floor[i] not in HAZARDS:
            # This is a block we can walk to
            if roof[i] == AIR:
                n.append((pos[0] + POS[i][0], pos[1], pos[2] + POS[i][1]))
        if floor[i] == AIR:
            # This is a block we can drop to
            if sub[i] != AIR and sub[i] not in HAZARDS:
                n.append((pos[0] + POS[i][0], pos[1] - 1, pos[2] + POS[i][1]))

    return n

sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)  # flush print output immediately

# More interesting generator string: "3;7,44*49,73,35:1,159:4,95:13,35:13,159:11,95:10,159:14,159:6,35:6,95:6;12;"

missions=[
            # Walk to goal mission
            '''<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
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
                        <DrawCuboid x1="-1" y1="55" z1="24" x2="-1" y2="57" z2="18" type="obsidian"/>
                        <DrawCuboid x1="1" y1="55" z1="24" x2="1" y2="57" z2="18" type="obsidian"/>
                        <DrawCuboid x1="0" y1="58" z1="24" x2="0" y2="58" z2="18" type="obsidian"/>
                    </DrawingDecorator>
                  <ServerQuitFromTimeUp timeLimitMs="45000"/>
                  <ServerQuitWhenAnyAgentFinishes/>
                </ServerHandlers>
              </ServerSection>
        
              <AgentSection mode="Survival">
                <Name>Walking Agent</Name>
                <AgentStart>
                    <Placement x="0.5" y="56" z="24.5" yaw="180"/>
                </AgentStart>
                <AgentHandlers>
                  <ObservationFromFullStats/>
                  <ObservationFromGrid>
                    <Grid name="SubFloor">
                      <min x="-1" y="-2" z="-1"/>
                      <max x="1" y="-2" z="1"/>
                    </Grid>
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
                    <Grid name="SuperRoof">
                      <min x="-1" y="2" z="-1"/>
                      <max x="1" y="2" z="1"/>
                    </Grid>
                  </ObservationFromGrid>
                  <AbsoluteMovementCommands/>
                  <AgentQuitFromTouchingBlockType>
                    <Block type="diamond_block"/>
                  </AgentQuitFromTouchingBlockType>
                </AgentHandlers>
              </AgentSection>
            </Mission>''',

            # Climb to goal mission
            '''<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
                <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
    
                  <About>
                    <Summary>Maze Two</Summary>
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
                            <DrawBlock x="0" y="56" z="-23" type="obsidian"/>
                            <DrawBlock x="-1" y="56" z="-24" type="obsidian"/>
                            <DrawBlock x="0" y="56" z="-24" type="obsidian"/>
                            <DrawBlock x="1" y="56" z="-24" type="obsidian"/>
                            <DrawBlock x="0" y="57" z="-24" type="diamond_block"/>
                            <DrawBlock x="0" y="55" z="24" type="gold_block"/>
                            <DrawCuboid x1="-5" y1="55" z1="-5" x2="5" y2="60" z2="5" type="obsidian"/>
                            <DrawCuboid x1="-1" y1="55" z1="24" x2="-1" y2="57" z2="18" type="obsidian"/>
                            <DrawCuboid x1="1" y1="55" z1="24" x2="1" y2="57" z2="18" type="obsidian"/>
                            <DrawCuboid x1="0" y1="58" z1="24" x2="0" y2="58" z2="18" type="obsidian"/>
                        </DrawingDecorator>
                      <ServerQuitFromTimeUp timeLimitMs="45000"/>
                      <ServerQuitWhenAnyAgentFinishes/>
                    </ServerHandlers>
                  </ServerSection>
    
                  <AgentSection mode="Survival">
                    <Name>Climbing Agent</Name>
                    <AgentStart>
                        <Placement x="0.5" y="56" z="24.5" yaw="180"/>
                    </AgentStart>
                    <AgentHandlers>
                      <ObservationFromFullStats/>
                      <ObservationFromGrid>
                        <Grid name="SubFloor">
                            <min x="-1" y="-2" z="-1"/>
                            <max x="1" y="-2" z="1"/>
                        </Grid>
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
                        <Grid name="SuperRoof">
                            <min x="-1" y="2" z="-1"/>
                            <max x="1" y="2" z="1"/>
                        </Grid>
                      </ObservationFromGrid>
                      <AbsoluteMovementCommands/>
                      <AgentQuitFromTouchingBlockType>
                        <Block type="diamond_block"/>
                      </AgentQuitFromTouchingBlockType>
                    </AgentHandlers>
                  </AgentSection>
                </Mission>''',

            # Drop to goal mission
            '''<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
            <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">

              <About>
                <Summary>Maze Three</Summary>
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
                        <DrawBlock x="1" y="55" z="-24" type="air"/>
                        <DrawBlock x="-1" y="55" z="-24" type="air"/>
                        <DrawBlock x="0" y="55" z="-23" type="air"/>
                        <DrawBlock x="0" y="55" z="-24" type="air"/>
                        <DrawBlock x="0" y="54" z="-24" type="diamond_block"/>
                        <DrawBlock x="0" y="55" z="24" type="gold_block"/>
                        <DrawCuboid x1="-5" y1="55" z1="-5" x2="5" y2="60" z2="5" type="obsidian"/>
                        <DrawCuboid x1="-1" y1="55" z1="24" x2="-1" y2="57" z2="18" type="obsidian"/>
                        <DrawCuboid x1="1" y1="55" z1="24" x2="1" y2="57" z2="18" type="obsidian"/>
                        <DrawCuboid x1="0" y1="58" z1="24" x2="0" y2="58" z2="18" type="obsidian"/>
                    </DrawingDecorator>
                  <ServerQuitFromTimeUp timeLimitMs="45000"/>
                  <ServerQuitWhenAnyAgentFinishes/>
                </ServerHandlers>
              </ServerSection>

              <AgentSection mode="Survival">
                <Name>Dropping Agent</Name>
                <AgentStart>
                    <Placement x="0.5" y="56" z="24.5" yaw="180"/>
                </AgentStart>
                <AgentHandlers>
                  <ObservationFromFullStats/>
                  <ObservationFromGrid>
                    <Grid name="SubFloor">
                      <min x="-1" y="-2" z="-1"/>
                      <max x="1" y="-2" z="1"/>
                    </Grid>
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
                    <Grid name="SuperRoof">
                      <min x="-1" y="2" z="-1"/>
                      <max x="1" y="2" z="1"/>
                    </Grid>
                  </ObservationFromGrid>
                  <AbsoluteMovementCommands/>
                  <AgentQuitFromTouchingBlockType>
                    <Block type="diamond_block"/>
                  </AgentQuitFromTouchingBlockType>
                </AgentHandlers>
              </AgentSection>
            </Mission>''',

            # Climb the big central pillar
            '''<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
            <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">

              <About>
                <Summary>Maze Four</Summary>
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
                        <DrawCuboid x1="-5" y1="55" z1="-5" x2="5" y2="60" z2="5" type="obsidian"/>
                        <DrawBlock x="0" y="60" z="0" type="diamond_block"/>
                        <DrawBlock x="0" y="55" z="24" type="gold_block"/>
                        <DrawCuboid x1="-1" y1="55" z1="24" x2="-1" y2="57" z2="18" type="obsidian"/>
                        <DrawCuboid x1="1" y1="55" z1="24" x2="1" y2="57" z2="18" type="obsidian"/>
                        <DrawCuboid x1="0" y1="58" z1="24" x2="0" y2="58" z2="18" type="obsidian"/>
                        
                        <DrawCuboid x1="-8" y1="58" z1="7" x2="8" y2="58" z2="7" type="obsidian"/>
                        <DrawCuboid x1="-8" y1="58" z1="6" x2="-8" y2="58" z2="0" type="obsidian"/>
                        <DrawCuboid x1="8" y1="58" z1="6" x2="8" y2="58" z2="0" type="obsidian"/>
                        <DrawCuboid x1="-7" y1="59" z1="0" x2="-6" y2="59" z2="0" type="obsidian"/>
                        <DrawCuboid x1="7" y1="59" z1="0" x2="6" y2="59" z2="0" type="obsidian"/>
                        
                        <DrawBlock x="0" y="57" z="8" type="obsidian"/>
                        <DrawBlock x="0" y="56" z="9" type="obsidian"/>
                    </DrawingDecorator>
                  <ServerQuitFromTimeUp timeLimitMs="45000"/>
                  <ServerQuitWhenAnyAgentFinishes/>
                </ServerHandlers>
              </ServerSection>

              <AgentSection mode="Survival">
                <Name>Hill Climbing Agent</Name>
                <AgentStart>
                    <Placement x="0.5" y="56" z="24.5" yaw="180"/>
                </AgentStart>
                <AgentHandlers>
                  <ObservationFromFullStats/>
                  <ObservationFromGrid>
                    <Grid name="SubFloor">
                      <min x="-1" y="-2" z="-1"/>
                      <max x="1" y="-2" z="1"/>
                    </Grid>
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
                    <Grid name="SuperRoof">
                      <min x="-1" y="2" z="-1"/>
                      <max x="1" y="2" z="1"/>
                    </Grid>
                  </ObservationFromGrid>
                  <AbsoluteMovementCommands/>
                  <AgentQuitFromTouchingBlockType>
                    <Block type="diamond_block"/>
                  </AgentQuitFromTouchingBlockType>
                </AgentHandlers>
              </AgentSection>
            </Mission>'''
          ]

# Create default Malmo objects:

class Cell:

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.f = 0
        self.g = 0
        self.h = 0

    def __lt__(self, other):
        return True if self.f < other.f else False

    def __le__(self, other):
        return True if self.f <= other.f else False

    def __gt__(self, other):
        return True if self.f > other.f else False

    def __ge__(self, other):
        return True if self.f >= other.f else False

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __str__(self):
        return "{0}, {1}, {2}, {3}, {4}, {5}".format(self.x, self.y, self.z, self.f, self.g, self.h)

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

    my_mission = MalmoPython.MissionSpec(missions[i], True)
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
            best = None
            val = float("inf")
            for neighbor in neighbors(pos, ob.get(u'SubFloor'), ob.get(u'Floor'),
                                      ob.get(u'Level'), ob.get(u'Roof'), ob.get(u'SuperRoof')):
                if neighbor not in closed:
                    temp = dist(neighbor, GOAL[i])
                    if temp + gs < val:
                        best = neighbor
                        val = temp + gs
            gs += 1
            if best:
                closed.add(best)
                agent_host.sendCommand("tp {0} {1} {2}".format(best[0], best[1], best[2]))
                time.sleep(0.3)
    print
    print "Mission ended"
    # Mission has ended.
