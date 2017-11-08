class Obstacle(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of an obstacle object.
        :param xdims: The x dimensions of the object.
        :param ydims: The y dimensions of the object.
        :param zdims: The z dimensions of the object.
        """
        self.xdims = xdims
        self.ydims = ydims
        self.zdims = zdims

    def __eq__(self, other):
        """
        Determines if this obstacle is equal to another.
        :param other: The other obstacle to compare against.
        :return: True if the obstacles share the same obstacle ID, False otherwise.
        """
        return (self.xdims == other.xdims and
                self.ydims == other.ydims and
                self.zdims == other.zdims)

    def __ne__(self, other):
        """
        Determines if this obstacle is not equal to another.
        :param other: The other obstacle to compare against.
        :return: True if the obstacles do not share the same obstacle ID, False otherwise.
        """
        return not self.__eq__(other)

    def __hash__(self):
        """
        Creates a hash for this obstacle.
        :return: An integer.
        """
        return hash((self.xdims, self.ydims, self.zdims))

    def get_xmin(self):
        """
        Gets the minimum x dimension of this obstacle.
        :return: An integer.
        """
        return min(self.xdims)

    def get_xmax(self):
        """
        Gets the maximum x dimension of this obstacle.
        :return: An integer.
        """
        return max(self.xdims)

    def get_ymin(self):
        """
        Gets the minimum y dimension of this obstacle.
        :return: An integer.
        """
        return min(self.ydims)

    def get_ymax(self):
        """
        Gets the maximum y dimension of this obstacle.
        :return: An integer.
        """
        return max(self.ydims)

    def get_zmin(self):
        """
        Gets the minimum z dimension of this obstacle.
        :return: An integer.
        """
        return min(self.zdims)

    def get_zmax(self):
        """
        Gets the maximum z dimenison of this obstacle.
        :return: An integer.
        """
        return max(self.zdims)

    def set_xdims(self, xdims):
        """
        Sets the x dimensions of the world.
        :param xdims: A tuple (min-x, max-x)
        :return: N/A
        """
        self.xdims = xdims

    def set_ydims(self, ydims):
        """
        Sets the y dimensions of the world.
        :param ydims: A tuple (min-y, max-y)
        :return: N/A
        """
        self.ydims = ydims

    def set_zdims(self, zdims):
        """
        Sets the z dimensions of the world.
        :param zdims: A tuple (min-z, max-z)
        :return: N/A
        """
        self.zdims = zdims

    def inside(self, p):
        """
        Determines if a point lies inside this obstacle.
        :param p: A tuple (x, y, z).
        :return: True if the point lies inside the obstacle, False otherwise.
        """
        return (self.get_xmin() <= p[0] < self.get_xmax() and
                self.get_ymin() <= p[1] < self.get_ymax() and
                self.get_zmin() <= p[2] < self.get_zmax())

    def ontop(self, p):
        """
        Determines if a point is on top of the obstacle or not.
        :param p: A tuple (x, y, z).
        :return: True if the point is on top of the obstacle, False otherwise.
        """
        return (self.get_xmin() <= p[0] < self.get_xmax() and
                p[1] == self.get_ymax() and
                self.get_zmin() <= p[2] < self.get_zmax())

class World(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates a world instance.
        :param xdims: The x dimensions of the world.
        :param ydims: The y dimensions of the world.
        :param zdims: The z dimensions of the world.
        """
        self.xdims = xdims
        self.ydims = ydims
        self.zdims = zdims
        self.obstacles = set()
        self.walkable = dict()
        self.sampling_space = set()

    def get_xmin(self):
        """
        Gets the minimum x dimension of the world.
        :return: An integer x | x < max(x dimensions).
        """
        return min(self.xdims)

    def get_xmax(self):
        """
        Gets the maximum y dimension of the world.
        :return: An integer y | y > min(y dimensions).
        """
        return max(self.xdims)

    def get_ymin(self):
        """
        Gets the minimum z dimension of the world.
        :return: An integer z | z < max(z dimensions).
        """
        return min(self.ydims)

    def get_ymax(self):
        """
        Gets the maximum x dimension of the world.
        :return: An integer x | x > min(x dimensions).
        """
        return max(self.ydims)

    def get_zmin(self):
        """
        Gets the minimum y dimension of the world.
        :return: An integer y | y < max(y dimensions).
        """
        return min(self.zdims)

    def get_zmax(self):
        """
        Gets the maximum z dimension of the world.
        :return: An integer z | z > min(z dimensions).
        """
        return max(self.zdims)

    def get_obstacles(self):
        """
        Gets a list containing all the obstacles.
        :return: A list containing all obstacles.
        """
        return self.obstacles

    def get_sampling_space(self):
        return self.sampling_space

    def add_obstacle(self, xdims, ydims, zdims):
        """
        Adds an obstacle to the world.
        :param xdims: A tuple (min-x, max-x).
        :param ydims: A tuple (min-y, max-y).
        :param zdims: A tuple (min-z, max-z).
        :return: N/A
        """
        self.obstacles.add(Obstacle(xdims, ydims, zdims))

    def add_walkable(self, p):
        """
        Adds a walkable point to the world.
        :param p: A tuple (x, y, z).
        :return: N/A
        """
        if p[1] in self.walkable.keys():
            self.walkable[p[1]].add((p[0], p[2]))
        else:
            self.walkable[p[1]] = set()
            self.walkable[p[1]].add((p[0], p[2]))

    def empty(self):
        """
        Clears this worlds obstacles and walkable space.
        :return: N/A
        """
        self.obstacles.clear()
        self.walkable.clear()

    def in_bounds(self, p):
        """
        Determines if a point is inside the world.
        :param p: A tuple (x, y, z).
        :return: True if the point is within the world, False otherwise.
        """
        return (self.get_xmin() <= p[0] < self.get_xmax() and
                self.get_ymin() <= p[1] < self.get_ymax() and
                self.get_zmin() <= p[2] < self.get_zmax())

    def is_blocked(self, p):
        """
        Determines if a point is inside an obstacle.
        :param p: A tuple (x, y, z).
        :return: True if the point is within an obstacle in the world, False otherwise.
        """
        for obstacle in self.obstacles:
            if obstacle.inside(p):
                return True
        return False

    def is_valid(self, p):
        """
        Determines if a point is valid such that it is in bounds and outside an obstacle.
        :param p: A tuple (x, y, z).
        :return: True if the point is valid, False otherwise.
        """
        return self.in_bounds(p) and not self.is_blocked(p)

    def is_traversable(self, p):
        """
        Determines whether a point can be traversed or not. This is distinct from is_walkable as this checks whether
            a point falls on top of an obstacle or not.
        :param p: A tuple (x, y, z).
        :return: True if the point is traversable, False otherwise.
        """
        for obs in self.obstacles:
            if obs.inside(p):
                return False
        for obs in self.obstacles:
            if obs.ontop(p):
                return True

    def is_walkable(self, p):
        """
        Determines if a point is walkable or not.
        :param p: A tuple (x, y, z).
        :return: True if the point is walkable, False otherwise.
        """
        if p[1] in self.walkable.keys():
            for q in self.walkable[p[1]]:
                if (q[0] < p[0] < q[0] + 1 and
                        q[1] < p[2] < q[1] + 1):
                    return True
        return False