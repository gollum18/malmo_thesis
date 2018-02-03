import math

class ListNode(object):

    def __init__(self, pos, parent=None):
        """
        Creates an instance of a node object.
        :param pos: The position of this node.
        :param parent: The parent of this nod. Default is None.
        """
        self.position = pos
        self.parent = parent
        self.gscore = 0.0

    def __iter__(self):
        yield self.position[0]
        yield self.position[1]
        yield self.position[2]

    def __str__(self):
        return "({0}, {1}, {2})".format(self.position[0], self.position[1], self.position[2])

    def as_float3(self):
        """
        Returns a float3 object containing the position of this node.
        :return: A float3 containing the position of this node.
        """
        return gpuarray.vec.make_float3(*self.position)

    def get_position(self):
        """
        Gets the position of this node.
        :return: The position of this node.
        """
        return self.position

    def get_parent(self):
        """
        Gets the parent of this node.
        :return: The parent of this node.
        """
        return self.parent

    def set_parent(self, parent):
        """
        Sets the parent of this node.
        :param parent: The new parent of this node.
        :return: N/A
        """
        self.parent = parent

    def get_gscore(self):
        """
        Gets the gscore of this node.
        :return: The gscore of this node.
        """
        return self.gscore

    def set_gscore(self, gscore):
        """
        Sets the new gscore of this node.
        :param gscore: The new gscore of this node.
        :return: N/A
        """
        self.gscore = gscore

class TreeNode(object):

    def __init__(self, position, parent=None):
        """
        Creates an instance of a TreeNode object.
        Node for the Tree version of RRT.
        :param position: The position of thie TreeNode.
        :param parent: The parent of this TreeNode.
        """
        self.position = position
        self.parent = parent
        self.children = []

    def __iter__(self):
        yield self.position[0]
        yield self.position[1]
        yield self.position[2]

    def __getitem__(self, key):
        if 0 <= key < len(self.position):
            return self.position[key]
        else:
            raise IndexError

    def __str__(self):
        return "({0}, {1}, {2})".format(self.position[0], self.position[1], self.position[2])

    def add_child(self, node):
        """
        Adds a child to this TreeNode.
        :param node: The node to add to this TreeNodes children list.
        """
        self.children.append(node)

    def get_children(self):
        return self.children

    def get_parent(self):
        return self.parent

    def get_position(self):
        return self.position

    def set_parent(self, parent):
        self.parent = parent

    def distance(self, position):
        return math.sqrt((self.position[0]-position[0])**2
                          + (self.position[1]-position[1])**2
                          + (self.position[2]-position[2])**2)

    def find(self, position):
        """
        Finds a node in the tree with the given position.
        position: The position of the node to find.
        returns: The node in the tree that contains the given position.
        """
        # Check for the terminal nodes
        if self.position == position:
            return self
        elif not self.children:
            return None
        # If we did not find the position were looking for and there are children
        #   then call find on each child
        node = None
        for child in self.children:
            node = child.find(position)
            if node != None:
                break
        return node 

    def nearest(self, position):
        """
        Finds the nearest node to the given position.
        position: The position to find the nearest node of.
        returns: A tuple containing the nearest nodes distance, and the node itself.
        """
        # Check for a terminal node
        if not self.children:
            return (self.distance(position), self)
        # Iterate through all children, finding the closest position
        best_dist = self.distance(position)
        best_node = self
        for node in self.children:
            temp = node.nearest(position)
            if temp[0] < best_dist:
                best_dist = temp[0]
                best_node = temp[1]
        return (best_dist, best_node)
