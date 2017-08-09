import worlds


class Cell:

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