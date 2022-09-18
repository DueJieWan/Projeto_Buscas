import numpy as np
from math import inf, sqrt


class CostMap(object):
    """
    Represents a cost map where higher values indicates terrain which are harder to tranverse.
    """

    def __init__(self):
        """
        Creates the cost map from teacher's example.
        """

        self.width = 24
        self.height = 21
        
        self.grid = np.array([  [ 1, 1, 1, 1,-1, 1, 1, 1,-1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1],
                                [ 1, 1, 1, 1,-1, 1, 1, 1,-1,-1, 1, 1, 1,-1, 1, 1, 1,-1, 1, 1, 1],
                                [ 1, 1, 1, 1, 1, 1, 1,-1, 1,-1,-1, 1, 1, 1, 1,-1, 1,-1, 1, 1, 1],
                                [ 1, 1,-1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                [ 1,-1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1],
                                [ 1, 1, 1,-1, 1, 1,-1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1],
                                [ 1, 1, 1, 1, 1, 1, 1, 1, 1,-1,-1,-1, 1, 1, 1,-1, 1, 1, 1, 1, 1],
                                [ 1, 1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1],
                                [ 1, 1, 1,-1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1],
                                [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1,-1, 1, 1,-1, 1,-1,-1, 1, 1],
                                [-1,-1, 1, 1, 1, 1,-1, 1,-1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1,-1],
                                [ 1, 1, 1, 1, 1, 1,-1,-1, 1,-1, 1,-1, 1, 1,-1, 1, 1, 1, 1, 1, 1],
                                [ 1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1, 1,-1, 1, 1],
                                [ 1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1],
                                [-1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1,-1],
                                [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1,-1, 1, 1, 1, 1, 1],
                                [ 1, 1,-1, 1, 1, 1, 1, 1, 1,-1, 1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1],
                                [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1,-1, 1,-1, 1, 1, 1],
                                [ 1,-1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1, 1],
                                [ 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1, 1, 1, 1,-1, 1, 1,-1],
                                [ 1, 1, 1, 1, 1, 1,-1, 1, 1, 1, 1,-1,-1, 1, 1, 1,-1, 1,-1, 1, 1],
                                [ 1, 1, 1, 1,-1, 1, 1, 1, 1,-1, 1, 1, 1, 1, 1,-1, 1,-1, 1, 1, 1],
                                [-1,-1, 1, 1, 1,-1, 1, 1, 1, 1, 1,-1, 1,-1, 1, 1, 1, 1, 1, 1, 1]])
        
        self.grid = self.grid.transpose()
        

        
    def get_cell_cost(self, i :int, j :int) -> float:
        """
        Obtains the cost of a cell in the cost map.

        i = y coordinate (row)
        j = x coordinate (column)
        return type: float.
        """

        return self.grid[i, j]


    def get_edge_cost(self, start, end) -> float:
        """
        Obtains the cost of an edge.

        start type: float.
        end type: float.
        return type: float.
        """

        return self.get_cell_cost(start[0], start[1]) + self.get_cell_cost(end[0], end[1]) / 2.0

    def is_occupied(self, i, j) -> bool:
        """
        Checks if a cell is occupied.

        i = y coordinate (row)
        j = x coordinate (column)
        return type: bool.
        """

        return self.grid[i][j] < 0.0

    def is_index_valid(self, i, j) -> bool:
        """
        Checks if the position (i, j) is valid (within the map boundaries).

        i = y coordinate (row)
        j = x coordinate (column)
        return type: bool.
        """

        return 0 <= i < self.height and 0 <= j < self.width


class NodeGrid(object):
    """
    Represents a grid of graph nodes used by the planning algorithm.
    """

    def __init__(self, cost_map):
        """
        Creates a grid of graph nodes.
        
        :param cost_map: cost map used for planning.
        :type cost_map: CostMap.
        """

        self.cost_map = cost_map
        self.width = cost_map.width
        self.height = cost_map.height
        self.grid = np.empty((self.height, self.width), dtype=Node)
        for i in range(np.size(self.grid, 0)):
            for j in range(np.size(self.grid, 1)):
                self.grid[i, j] = Node(i, j)

    def reset(self):
        """
        Resets all nodes of the grid.
        """

        for row in self.grid:
            for node in row:
                node.reset()

    def get_node(self, i:int, j:int):
        """
        Obtains the node at row i and column j.

        i: row 
        j: column
        return: node at row i and column j.
        """

        return self.grid[i, j]

    def get_successors(self, i:int, j:int):
        """
        Obtains a list of the 4-connected successors of the node at (i, j).

        i: row
        j: column
        return: list of Node.
        """

        successors = []
        for di in range(-1, 2):
            if di != 0:
                if self.cost_map.is_index_valid(i + di, j):
                    if not self.cost_map.is_occupied(i + di, j):
                        successors.append((i + di, j))

                if self.cost_map.is_index_valid(i , j + di):
                    if not self.cost_map.is_occupied(i, j + di):
                        successors.append((i, j + di))

        return successors

class Node(object):
    """
    Represents a node of a graph used for planning paths.
    """

    def __init__(self, i=0, j=0):
        """
        Creates a node of a graph used for planning paths.
        i: row
        j: column
        """

        self.i = i
        self.j = j
        self.f = inf
        self.g = inf
        self.closed = False
        self.parent = None

    def get_position(self):
        """
        Obtains the position of the node as a tuple.

        :return: (i, j) 
        :return type: 2-dimensional tuple of int.
        """

        return self.i, self.j
    
    def set_position(self, i:int, j:int):
        """
        Sets the position of this node.

        i: row
        j: column
        """

        self.f = inf
        self.g = inf
        self.closed = False
        self.parent = None

    def reset(self):
        """
        Resets the node to prepare it for a new path planning.
        """

        self.f = inf
        self.g = inf
        self.closed = False
        self.parent = None

    def distance_to(self, i:int , j:int):
        """
        Computes the distance from this node to the position (i, j).

        i: row
        j: column
        return type: float
        """

        return sqrt((self.i - i) ** 2 + (self.j - j) ** 2)

    def __lt__(self, another_node):
        if self.i < another_node.i:
            return True
        if self.j < another_node.j:
            return True
        return False
