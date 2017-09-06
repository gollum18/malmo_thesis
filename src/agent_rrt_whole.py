from agent_rrt_decimal import *

class RealTimeRapidlyExploringTreeAgentWhole(RealTimeRapidlyExploringTreeAgentDecimal):

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
                 movement_distance=1):
        RealTimeRapidlyExploringTreeAgentDecimal.__init__(self, start, goal, xdims, ydims, zdims, p_line, p_goal, m_decider,
                                                   num_nodes, xradius, yradius, zradius, g_tolerance, max_distance,
                                                   obsx, obsy, obsz, hazx, hazy, hazz, movement_distance)

    def explore(self):
        """
                Finds a path utilizing a rapidly-exploring random tree.
                :return: The path if it is found, or an empty list if it is not.
                """

        def find_nearest_neighbor(node):
            """
            Finds the nearest neighbor across all nodes in the search tree to the specified node.
            :param node: The node to find nearest neighbor of.
            :return: The node in the search tree that is closest to the specified node.
            """
            best_node = nodes[0]
            best_distance = distance(nodes[0].get_position(), node)
            for j in range(1, len(nodes)):
                if nodes[j].get_position() == node.get_position():
                    continue
                v = distance(nodes[j].get_position(), node)
                if v < best_distance:
                    best_node = nodes[j]
                    best_distance = v
            return best_node

        def generate_neighbors(node):
            neighbors = []
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    for dz in range(-1, 2):
                        if dx == 0 and dy == 0 and dz == 0:
                            continue
                        p = node.get_position()[0] + dx, node.get_position()[1] + dy, node.get_position()[2] + dz
                        if self.in_bounds(p) and not self.is_blocked(p):
                            q = node.get_position()[0] + dx, node.get_position()[1] + dy - 1, node.get_position()[2] + dz
                            if self.in_bounds(q) and self.is_obstacle(q):
                                neighbors.append(RealTimeRapidlyExploringTreeNode(p, node))
            return neighbors

if __name__ == '__main__':
    paths = []
    for i in range(len(constants.mission_txt)):
        agent = RealTimeRapidlyExploringTreeAgentWhole(start=constants.start,
                                                         goal=constants.goal[i],
                                                         xdims=(constants.lower_dimensions[i][0],
                                                                constants.upper_dimensions[i][0]),
                                                         ydims=(constants.lower_dimensions[i][1],
                                                                constants.upper_dimensions[i][1]),
                                                         zdims=(constants.lower_dimensions[i][2],
                                                                constants.upper_dimensions[i][2]))
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
