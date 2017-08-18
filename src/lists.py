class Node:

    def __init__(self, value):
        """
        Creates a list node.
        :param value: The value of the node.
        """
        self.next = None
        self.value = value

    def __str__(self):
        """
        Gets a string representation of this node.
        :return: A string representing this node.
        """
        if self.get_next():
            return str(self.get_value()) + ", " + self.get_next().__str__()
        else:
            return str(self.get_value()) + "]"

    def contains(self, value):
        """
        Recursive function used to determine if a list contains a value.
        :param value: The value to search for.
        :return: True if the node has the specified value. False if the end of the list is reached and the
        value was not found.
        """
        if self.get_value() == value:
            return True
        if self.get_next():
            return self.get_next().contains(value )
        return False

    def count(self):
        """
        Recursive function used to determine the size of a list.
        :return: The size of the list.
        """
        if self.next:
            return self.next.count() + 1
        return 1

    def get(self, index):
        """
        Recursive function used to get a value at a specific position in the list.
        :param index: The index of the item in the list.
        :return: The value at the index in the list.
        :raises: KeyError if the index is out of range.
        """
        if index == 0:
            return self.get_value()
        if self.get_next():
            return self.get_next().get(index-1)
        raise KeyError

    def set(self, index, value):
        """
        Sets the value at the specified index to the new value.
        :param index: The index to update.
        :param value: The new value to update to.
        :return: Nothing.
        :raises: KeyError if the index is out of range.
        """
        if index == 0:
            self.set_value(value)
        if self.get_next():
            self.get_next().set(index-1, value)
        raise KeyError

    def get_next(self):
        """
        Returns the next node after this one.
        :return: The next node.
        """
        return self.next

    def set_next(self, node):
        """
        Updates the pointer to the next node.
        :param node: The node to make next.
        :return: Nothing
        """
        self.next = node

    def get_value(self):
        """
        Returns the value of this node.
        :return: The value of this node.
        """
        return self.value

    def set_value(self, value):
        """
        Updates the value of this node.
        :param value: The value to update the node to.
        :return: Nothing
        """
        self.value = value

    def swap_values(self, node):
        """
        Swaps the value of this node with the node specified.
        :param node: The node to swap values with.
        :return: Nothing
        """
        if node:
            temp = self.get_value()
            self.set_value(node.get_value())
            node.set_value(temp)


class PriorityNode(Node):

    def __init__(self, priority, value):
        """
        Creates a specialized node for use in a priority queue.
        :param priority: The priority assigned to this node.
        :param value: The value that corresponds to this node.
        """
        Node.__init__(self, value)
        self.previous = None
        self.priority = priority

    def __str__(self):
        """
        Gets a string representation of this node.
        :return: A string representing this node.
        """
        if self.get_next():
            return "{0}:{1}, {2}".format(self.get_priority(), self.get_value(), self.get_next().__str__())
        else:
            return "{0}:{1}]".format(self.get_priority(), self.get_value())

    def get_previous(self):
        """
        Returns the previous node in the queue.
        :return: The previous node.
        """
        return self.previous

    def set_previous(self, node):
        """
        Updates the pointer to the previous node.
        :param node: Updates the previous node in the queue.
        :return: Nothing
        """
        self.previous = node

    def get_priority(self):
        """
        Returns the priority assigned to this node.
        :return: The priority of this node.
        """
        return self.priority

    def set_priority(self, priority):
        """
        Updates the priority of this node.
        :param priority: The new priority for this node.
        :return: Nothing
        """
        self.priority = priority


class LinkedList:

    def __init__(self):
        """
        Creats an empty linked list.
        """
        self.head = None

    def __str__(self):
        """
        A string representation of the items in the list.
        :return: A string representing the list.
        """
        if self.head:
            return "[" + self.head.__str__()
        else:
            return "Empty"

    def add(self, value):
        """
        Adds a value to the front of the list.
        :param value: The value to add.
        :return: nothing
        """
        node = Node(value)
        node.set_next(self.head)
        self.head = node

    def contains(self, value):
        """
        Determines if the list contains a specified value.
        :param value: The value to search for.
        :return: True if an instance of this value is found. False if not.
        """
        if self.head:
            return self.head.contains(value)

    def count(self):
        """
        Returns the amount of items in the list.
        :return: The amount of items in the list.
        """
        if self.head:
            return self.head.count()
        return 0

    def get(self, index):
        """
        Gets the value at the specified position in the list.
        :param index: The position of the value in the list.
        :return: The value if the specified index is in range of the length of the list. None otherwise.
        """
        if self.head:
            return self.head.get_next()
        return None

    def is_empty(self):
        """
        Determines if the list is empty or not.
        :return: True if the list contains items. False otherwise.
        """
        return True if not self.head else False

    def remove(self, value):
        """
        Removes the first instance of the specified value from the list.
        :param value: The value to remove from the list.
        :return: True if the value was successfully removed. False otherwise.
        """
        if self.head:
            # Case 1: First item is what is being removed
            if self.head.get_value() == value:
                self.head = self.head.get_next()
                return True
            # Case 2: Item is somewhere in the list, search for it
            previous = self.head
            while previous.get_next():
                current = previous.get_next()
                if current.get_value() == value:
                    previous.set_next(current.get_next())
                    return True
                previous = current
        return False

    def set(self, index, value):
        """
        Sets the value at the specified index to the new value.
        :param index: The index to update.
        :param value: The value to update to.
        :return: Nothing
        """
        if self.head:
            self.head.set(index, value)


class Set(LinkedList):

    def __init__(self):
        LinkedList.__init__(self)

    def add(self, value):
        if not self.contains(value):
            LinkedList.add(self, value)

    def union(self, set):
        """
        Returns a new set that is the mathematical union of the current set instance and the indicated set.
        :param set: The set to union with.
        :return: The mathematical union of the two sets.
        """
        ns = Set()
        for i in range(self.count()):
            ns.add(self.get(i))
        for i in range(set.count()):
            val = set.get(i)
            if not ns.contains(val):
                ns.add(val)

    def intersection(self, set):
        """
        Returns a new set that is the mathematical intersection of the current set instance and the indicated set.
        :param set: The set to intersect with.
        :return: The mathematical intersection of the two sets.
        """
        ns = Set()
        cself = self.count()
        cset = set.count()
        if cself >= cset:
            for i in range(cset):
                val = set.get(i)
                if self.contains(i):
                    ns.add(val)
        else:
            for i in range(cself):
                val = self.get(i)
                if set.contains(i):
                    ns.add(val)
        return ns

    def difference(self, set):
        """
        Returns a new set that is the mathematical difference of the current set instance and the indicated set.
        That is A-B where A is the caller and B is the passed in set.
        :param set: The set to difference with.
        :return: The mathematical difference of the two sets.
        """
        ns = Set()
        for i in range(self.count()):
            val = self.get(i)
            if not set.contains(val):
                ns.add(val)
        return ns

    def subset(self, set):
        """
        Tests whether the indicated set is a subset of this set instance or not.
        :param set: The set to check as a subset.
        :return: True if the indeicated set is a subset of this set instance, False if not.
        """
        for i in range(set.count()):
            if not self.contains(set.get(i)):
                return False
        return True


class Queue(LinkedList):

    def __init__(self):
        """
        Creates an empty queue.
        """
        LinkedList.__init__(self)

    def add(self, value):
        """
        Not implemented for a queue.
        :param value: N/A
        :return: Nothing
        """
        pass

    def dequeue(self):
        """
        Removes a value from the front of the queue and returns it.
        :return: Nothing
        """
        if self.head:
            value = self.head.get_value()
            self.head = self.head.get_next()
            return value
        return None

    def enqueue(self, value):
        """
        Adds an item to the back of the queue.
        :param value: The value to add to the queue.
        :return: Nothing
        """
        if self.head:
            current = self.head
            while current.get_next():
                current = current.get_next()
            node = Node(value)
            node.set_previous(current)
            current.set_next(node)
        else:
            node = Node(value)
            node.set_next(self.head)
            self.head = node

    def get(self, index):
        """
        Not implemented for a queue.
        :param index: N/A
        :return: Nothing
        """
        pass

    def remove(self, value):
        """
        Not implemented for a queue.
        :param value: N/A
        :return: Nothing
        """
        pass

    def set(self, index, value):
        """
        Not implemented for a queue.
        :param index: N/A
        :param value: N/A
        :return: Nothing
        """
        pass


class PriorityQueue(Queue):

    def __init__(self):
        """
        Creates a minimum priority queue.
        """
        Queue.__init__(self)

    def dequeue(self):
        """
        Removes a value from the front of the queue and returns it.
        :returns Nothing
        """
        if self.head:
            value = self.head.get_value()
            self.head = self.head.get_next()
            self.head.set_previous(None)
            return value
        return None

    def enqueue(self, priority, value):
        """
        Adds an item to the priority queue.
        :param priority: The priority of the item.
        :param value: The value of the item.
        :return: Nothing
        """
        if self.head:
            current = self.head
            while current.get_next():
                current = current.get_next()
            node = PriorityNode(priority, value)
            node.set_previous(current)
            current.set_next(node)
            self.sift_up(node)
        else:
            node = PriorityNode(priority, value)
            self.head = node

    def sift_up(self, node):
        if node.get_previous():
            next = node.get_previous()
            while node.get_previous():
                if next.get_priority() < node.get_priority():
                    break
                else:
                    tp = node.get_priority()
                    tv = node.get_value()
                    node.set_priority(next.get_priority())
                    node.set_value(next.get_value())
                    next.set_priority(tp)
                    next.set_value(tv)
                    node = next
                    next = node.get_previous()

class Stack(LinkedList):

    def __init__(self):
        """
        Creates an empty stack.
        """
        LinkedList.__init__(self)

    def add(self, value):
        """
        Not implemented for a stack.
        :param value: N/A
        :return: Nothing
        """
        pass

    def get(self, index):
        """
        Not implemented for a stack.
        :param index: N/A
        :return: Nothing
        """
        pass

    def peek(self):
        """
        Returns the top value on the stack.
        :return: The top value on the stack.
        """
        return self.head.get_value() if self.head else None
    
    def pop(self):
        """
        Pops the top value from the stack and returns it.
        :return: The top value on the stack.
        """
        if self.head:
            value = self.head.get_value()
            self.head = self.head.get_next()
            return value
        return None

    def push(self, value):
        """
        Will push a value to the top of the stack.
        :param value: Pushes a value onto the top of the stack.
        :return: Nothing
        """
        node = Node(value)
        node.set_next(self.head)
        self.head = node

    def remove(self, value):
        """
        Not implemented for a stack.
        :param value: N/A
        :return: Nothing
        """
        pass

    def set(self, index, value):
        """
        Not implemented for a stack.
        :param index: N/A
        :param value: N/A
        :return: Nothing
        """
        pass

def main():
    import random, time

    pq = PriorityQueue()
    for i in range(50):
        pq.enqueue(round(random.random()*10, 4), i)
    print(pq)

if __name__ == '__main__':
    main()