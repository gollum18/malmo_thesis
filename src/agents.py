import worlds


class Cell3D:

    def __init__(self, x, y, z, parent):
        """
        Creates a cell. A cell is used by path planning algorithms to find paths.
        :param x: The x coordinate.
        :param y: The y coordinate.
        :param z: The z coordinate.
        :param parent: The parent. Used for generating the final path.
        """
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __str__(self):
        return "Cell3D: ({0}, {1}, {2})".format(self.x, self.y, self.z)

    def get_coordinates(self):
        """
        Gets a tuple containaing coordinates in the form (x, y, z)
        :return: A tuple containing the coordinates of this cell.
        """
        return self.x, self.y, self.z

    def get_x(self):
        """
        Gets the x coordinate of this cell.
        :return: The x coordinate.
        """
        return self.x

    def get_y(self):
        """
        Gets the y coordinate of this cell.
        :return: The y coordinate.
        """
        return self.y

    def get_z(self):
        """
        Gets the z coordinate of this cell.
        :return: The z coordinate.
        """
        return self.z

    def get_parent(self):
        """
        Gets the parent of this cell.
        :return: The parent.
        """
        return self.parent

    def set_parent(self, parent):
        """
        Sets the parent for this cell.
        :param parent: The new parent.
        :return: Does not return.
        """
        self.parent = parent


class Cell3DWithCosts(Cell3D):

    def __init__(self, x, y, z, parent):
        """
        Creates a 3d cell acceptable for use with an algorithm that uses costs/heuristics like A*.
        :param x:
        :param y:
        :param z:
        :param parent:
        """
        Cell3D.__init__(self, x, y, z, parent)
        self.f_score = 0.0
        self.g_score = 0.0
        self.h_score = 0.0

    def __str__(self):
        return "Cell3D: ({0}, {1}, {2}\nF: {3}, G: {4}, H: {5})".format(self.x, self.y, self.z,
                                        self.f_score, self.g_score, self.h_score)

    def get_fscore(self):
        return self.f_score

    def get_gscore(self):
        return self.g_score

    def get_hscore(self):
        return self.h_score

    def set_fscore(self, score):
        self.f_score = score

    def set_gscore(self, score):
        self.g_score = score

    def set_hscore(self, score):
        self.h_score = score