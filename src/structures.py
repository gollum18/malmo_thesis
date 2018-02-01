"""
    Name: structures.py
    Date Created: 1/31/2018
    Last Modified: 1/31/2018
    Purpose: Stores custom data structures utilized in the thesis.
"""

class PriorityQueueNode(object):

    def __init__(self, priority, data):
        """
        Creates a node object to be stored in a priority queue.
        priority: The priority assigned to this node.
        data: The data stored in this node.
        """
        self.priority = priority
        self.data = data

    def __eq__(self, other):
        if isinstance(other, PriorityQueueNode):
            return self.priority == other.priority
        raise TypeError

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, PriorityQueueNode):
            return self.priority < other.priority
        raise TypeError

    def __le__(self, other):
        if isinstance(other, PriorityQueueNode):
            return self.priority <= other.priority
        raise TypeError

    def __gt__(self, other):
        if isinstance(other, PriorityQueueNode):
            return self.priority > other.priority
        raise TypeError

    def __ge__(self, other):
        if isinstance(other, PriorityQueueNode):
            return self.priority >= other.priority
        raise TypeError

    def __str__(self):
        return "Priority: {0}, \n\tData: {1}".format(self.priority, self.data)

    def get_priority(self):
        """
        Gets the priority of this node.
        returns: The priority of this node, a sortable data type.
        """
        return self.priority

    def get_data(self):
        """
        Gets the data stored in this node.
        returns: The data stored in this node.
        """
        return self.data

    def set_priority(self, priority):
        """
        Updates the priority of this node.
        priority: The new priority for this node.
        """
        self.priority = priority

class PriorityQueue(object):

    def __init__(self, order="ASC"):
        """
        Create an empty prority queue. This priority queue allows for indexing of elements in addition to 
        queue insertion and deletion routines.
        order: The order items are sorted in the priority queue, either ASC or DES.
        """
        self.queue = []
        order.capitalize()
        if order != "ASC" or order != "DES":
            self.order = "ASC"
        else: 
            self.order = order

    def __iter__(self):
        """
        Get an iterator over the items in the priority queue.
        returns: An iterator object.
        """
        return self.queue.__iter__()

    def __len__(self):
        return self.queue.__len__()

    def __str__(self):
        s = ""
        for iteration in range(len(self.queue)):
            s += "{0}) {1}\n".format(iteration, self.queue[iteration])
        return s

    def __getitem__(self, key):
        return self.queue.__getitem__(key)

    def __delitem__(self, key):
        self.queue.__delitem__(key)

    def __setitem__(self, key, priority):
        if 0 <= key <= len(self.queue):
            if isinstance(priority, self.queue[key].get_priority()):
                self.queue[key].set_priority(priority)
                self.queue.sort()
            else:
                raise TypeError("New priority data type does not match the data type for the current priority of the item!")
        else:
            raise IndexError("Index exceeded boudns of the priority queue!")

    def __contains__(self, data):
        for item in self.queue:
            if item.get_data() == data:
                return True
        return False

    def dequeue(self):
        """
        Get the item at the front of the priority queue.
        returns: An item from the queue.
        """
        if self.queue:
            return self.queue.pop(0).get_data()
        else:
            raise IndexError("No items remaining in the priority queue!")

    def empty(self):
        """
        Determines if the priority queue is empty or not.
        returns: True of the queue is empty, False otherwise.
        """
        return self.queue == []

    def enqueue(self, priority, data):
        """
        Places an item in the priority queue with the given priority.
        priority: The items priority in the queue.
        data: Whatever is to be stored in the queue.
        """
        self.queue.append(PriorityQueueNode(priority, data))
        self.sort()

    def sort(self):
        """
        Sort the priority queue.
        """
        self.queue.sort(reverse=False if self.order=="ASC" else True)

def main(*args, **kwargs):
    import random

    queue = PriorityQueue()

    for iteration in range(random.randint(5, 25)):
        queue.enqueue(random.randint(0, 100), random.randint(-100, 100))

if __name__ == '__main__':
    main()
