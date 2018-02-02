class Cube(object):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of a cuboid object.
        xdims: The dimensions of the cuboid in the x axis (length).
        ydims: The dimensions of the cuboid in the y axis (height).
        zdims: The dimensions of the cuboid in the z axis (width).
        """
        self.x_lower = min(xdims)
        self.x_upper = max(xdims)
        self.y_lower = min(ydims)
        self.y_upper = max(ydims)
        self.z_lower = min(zdims)
        self.z_upper = max(zdims)

    def __eq__(self, other):
        """
        Determines another obstacle is equal to this one.
        other: The other obstacle to check.
        returns: True if the two obstacles share the same dimensions, False otherwise.
        """
        return (self.x_lower == other.x_lower and self.y_lower == other.y_lower and self.z_lower == other.z_lower and
                self.x_upper == other.x_upper and self.y_upper == other.y_upper and self.z_upper == other.z_upper)

    def __ne__(self, other):
        """
        Determines another obstacle is not equal to this one.
        other: The other obstacle to check.
        returns: True if the two obstacles do not share the same dimensions, False otherwise.
        """
        return not self.__eq__(other)

    def __hash__(self):
        """
        Gets a hash representing this obstacle.
        returns: A hash representing this obstacle.
        """
        return hash((self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper))

    def __str__(self):
        """
        Gets an informal string representation of this obstacle.
        returns: An informal string representing the obstacle.
        """
        return "The Bounds of this Obstacle are -> \n\tx:({0}, {1}), ({2}, {3}), ({4}, {5})".format(
            self.x_lower, self.x_upper, self.y_lower, self.y_upper, self.z_lower, self.z_upper
        )

    def get_lower_bounds(self):
        """
        Gets a tuple (x, y, z) containing the lower bounds of the obstacle.
        returns: A tuple (x, y, z) containing the lower bounds of the obstacle.
        """
        return self.x_lower, self.y_lower, self.z_lower

    def get_upper_bounds(self):
        """
        Gets a tuple (x, y, z) containing the upper bounds of the obstacle.
        returns: A tuple (x, y, z) containing the upper bounds of the obstacle.
        """
        return self.x_upper, self.y_upper, self.z_upper

    def get_x_lower(self):
        """
        Gets the lower bound of the obstacle on the x dimension.
        returns: The lower bound of the obstacle on the x dimension.
        """
        return self.x_lower

    def get_x_upper(self):
        """
        Gets the upper bound of the obstacle on the x dimension.
        returns: The upper bound of the obstacle on the x dimension.
        """
        return self.x_upper

    def get_y_lower(self):
        """
        Gets the lower bound of the obstacle on the y dimension.
        returns: The lower bound of the obstacle on the y dimension.
        """
        return self.y_lower

    def get_y_upper(self):
        """
        Gets the upper bound of the obstacle on the y dimension.
        returns: The upper bound of the obstacle on the y dimension.
        """
        return self.y_upper

    def get_z_lower(self):
        """
        Gets the lower bound of the obstacle on the z dimension.
        returns: The lower bound of the obstacle on the z dimension.
        """
        return self.z_lower

    def get_z_upper(self):
        """
        Gets the upper bound of the obstacle on the z dimension.
        returns: The upper bound of the obstacle on the z dimension.
        """
        return self.z_upper

    def ontop(self, p):
        """
        Determines if a point is ontop of this obstacle.
        p: The point to check.
        returns: True if the point is ontop of the obstacle, False otherwise.
        """
        if (self.x_lower <= p[0] <= self.x_upper and
                self.y_upper == p[1] and
                self.z_lower <= p[2] <= self.z_upper):
            return True
        return False

    def player_inside(self, p, height=2):
        """
        Determines if the player is inside the obstacle.
        p: The point to check.
        height: The players height in units >= 1.
        returns: Determines if the player collides with this object.
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
        p: The point to check.
        returns: True if the point is inside this object, False otherwise.
        """
        if (self.x_lower < p[0] < self.x_upper and
                self.y_lower < p[1] < self.y_upper and
                self.z_lower < p[2] < self.z_upper):
            return True
        return False

class World(Cube):

    def __init__(self, xdims, ydims, zdims):
        """
        Creates an instance of a world object.
        xdims: The dimensions of the world in the x axis (length).
        ydims: The dimensions of the world in the y axis (height).
        zdims: The dimensions of the world in the z axis (width).
        """
        super(World, self).__init__(xdims, ydims, zdims)
        self.obstacles = set()
        self.walkable = set()

    def add_obstacle(self, xdims, ydims, zdims):
        """
        Adds an obstacle to the agents world representation.
        xdims: The dimensions of the obstacle in the x axis (length).
        ydims: The dimensions of the obstacle in the y axis (height).
        zdims: The dimensions of the obstacle in the z axis (width).
        returns: N/A
        """
        self.obstacles.add(Cube(xdims, ydims, zdims))

    def add_walkable_space(self, p):
        """
        Adds a point of walkable space to the agents world representation.
        p: The point ot add walkable space to.
        returns: N/A
        """
        self.walkable.add(p)

    def clear(self):
        """
        Clears the world object and prepares it for deletion.
        returns: N/A
        """
        self.obstacles.clear()
        self.walkable.clear()
        del self.obstacles
        del self.walkable

    def is_point_blocked(self, p):
        """
        Determines if the point is blocked or not.
        p: The point to check.
        returns: True if the point is blocked by an obstacle, False otherwise.
        """
        for obs in self.obstacles:
            if obs.point_inside(p):
                return True
        return False

    def is_player_blocked(self, p, height=2):
        """
        Determines if the specified point contains and region (determined by the players height) is blocked.
        p: The point to check.
        height: The height of the player in units >= 1.
        returns: True if the player cannot move to the specified point, False otherwise.
        """
        for obs in self.obstacles:
            if obs.player_inside(p, height):
                return True
        return False

    def is_on_obstacle(self, p):
        """
        Determines if the point is on top of an obstacle.
        p: The point to check.
        returns: True if the point is on an obstacle, False otherwise.
        """
        for obs in self.obstacles:
            if obs.ontop(p):
                return True
        return False

    def is_point_valid(self, p):
        """
        Determines if the specified point is inside the bounds of the map and is not an obstacle.
        p: The point to check.
        returns: True if the point is valid, False otherwise.
        """
        return self.point_inside(p) and self.is_on_obstacle(p) and not self.is_point_blocked(p)

    def is_player_move_valid(self, p, height=2):
        """
        Determines if a player can move to the specified location.
        p: The point to check.
        height: The height of the player in units >= 1.
        returns: True if the move is valid, False otherwise.
        """
        return self.player_inside(p) and self.is_on_obstacle(p) and not self.is_player_blocked(p, height)