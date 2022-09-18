from grid import CostMap, Node, NodeGrid
from math import isinf
import heapq

class PathPlanner(object):
    """
    Represents a path planner, which I'm going to use A* to plan a path.
    """

    def __init__(self, cost_map : CostMap):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        """

        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)
    
    @staticmethod
    def construct_path(goal_node: Node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :return: the path as a sequence of (x, y) positions: [(x1,y1), (x2,y2), ... , (xn, yn)].
        :return type: list of typles
        """

        node = goal_node
        # We are going from the goal node to the start node following the parents.

        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1] # Reverse the reverse list to return proper order.

    def a_star(self, start_position = (0, 0), goal_position = (20, 23)):
        """
        Plans a path using A*
        """

        self.node_grid.reset()
        self.pq = []
        self.path = []
        self.cost = 0
        self.nowPosition = start_position

        heapq.heapify(self.pq)
        heapq.heappush(self.pq, (self.cost, (start_position[0], start_position[1])))
        self.node_grid.get_node(start_position[0], start_position[1]).g = 0
        self.node_grid.get_node(start_position[0], start_position[1]).f = 0
        while len(self.pq):
            (self.cost, self.nowPosition) = heapq.heappop(self.pq)
            if self.node_grid.get_node(self.nowPosition[0], self.nowPosition[1]).closed:
                continue
            self.node_grid.get_node(self.nowPosition[0], self.nowPosition[1]).closed = True

            if self.nowPosition == goal_position:
                break

            for successor in self.node_grid.get_successors(self.nowPosition[0], self.nowPosition[1]):
                self.successorNode = self.node_grid.get_node(successor[0], successor[1])

                if not self.successorNode.closed and self.successorNode.f > self.node_grid.get_node(self.nowPosition[0], self.nowPosition[1]).g + self.successorNode.distance_to(goal_position[0], goal_position[1]) + self.successorNode.distance_to(self.nowPosition[0], self.nowPosition[1])*(self.cost_map.get_cell_cost(self.nowPosition[0], self.nowPosition[1]) + self.cost_map.get_cell_cost(successor[0], successor[1]))/2:
                    self.successorNode.g = self.node_grid.get_node(self.nowPosition[0], self.nowPosition[1]).g + self.successorNode.distance_to(self.nowPosition[0], self.nowPosition[1])*(self.cost_map.get_cell_cost(self.nowPosition[0], self.nowPosition[1]) + self.cost_map.get_cell_cost(successor[0], successor[1]))/2
                    self.successorNode.f = self.successorNode.g + self.successorNode.distance_to(goal_position[0], goal_position[1])
                    self.successorNode.parent = self.node_grid.get_node(self.nowPosition[0], self.nowPosition[1])
                    heapq.heappush(self.pq, (self.successorNode.f, successor))
        
        self.path = PathPlanner.construct_path(self.node_grid.get_node(goal_position[0], goal_position[1]))

        return self.path, self.cost



