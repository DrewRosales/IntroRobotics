import signal
import sys

import numpy as np
from typing import List, Tuple

from heapq import heappush, heappop
import cv2


class Edge: 
    """
    This class provides a basic data structure for representing
    a directional edge in a graph. Travel is possible between
    the starting node to the ending node at the given cost
    but travel in the opposite direction is not allowed.
    """
    def __init__(self,starting_node, ending_node, cost):
        self.start = starting_node
        self.end = ending_node 
        self.cost = cost 

    def __repr__(self):
        return 'Node'+self.__str__()
    def __str__(self):
        return f'({self.start.name},{self.end.name},{self.cost})'

    def __eq__(self,obj):
        if  isinstance(obj, Edge):
            return self.start == obj.start and obj.end == obj.end and self.cost == self.cost 
        return False

class Node:
    """
    This class provides a basic data structure for representing
    a node in A* Graph
    """
    def __init__(self, name, h):
        #The name of the node (can be anything, just for human readable output)
        self.name = name
        #The current best cost-to-come for the node
        self.g = np.inf 
        #The current best estimate of the node's total cost
        self.f = np.inf 
        #The heuristic estimate of the cost-to-go for the node
        self.h = h 
        #The list of edges which connect the node 
        self.edges = []
        #The previous node in path to the goal
        self.previous = None

    def add_neighbor(self, node, cost):
        new_edge = Edge(self, node, cost)
        self.edges.append(new_edge)

    def add_neighbor_bidirectional(self, node, cost):
        self.add_neighbor(node, cost)
        node.add_neighbor(self, cost)


    def __str__(self):
        return f'({self.name},{self.f},{self.g},{self.h})'

    def __eq__(self,obj):
        if  isinstance(obj, Node):
            return self.name == obj.name and self.f == self.f  and obj.g == obj.g and self.h == self.h 
        return False

    def __ge__(self, other):
        return self.f >= other.f

    def __lt__(self, other):
        return self.f < other.f

def create_path_grid(cost_to_come, current):
    path = [current]
    while current in cost_to_come:
        current = cost_to_come[current]
        path.append(current)
    path.reverse()
    return path

def heuristic(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def inflate_grid(grid: np.ndarray, robot_radius: float, resolution: float) -> np.ndarray:

    # Convert to binary grid (0 for free, 1 for occupied)
    binary_grid = (grid >= 50).astype(np.uint8)
    
    # Calculate kernel size based on robot radius
    kernel_size = int(np.ceil(robot_radius / resolution))
    kernel = np.ones((kernel_size * 2 + 1, kernel_size * 2 + 1), np.uint8)
    
    # Dilate obstacles
    inflated_grid = cv2.dilate(binary_grid, kernel, iterations=1)
    
    # Convert back to occupancy grid format
    inflated_grid = inflated_grid * 100
    
    return inflated_grid

def a_star_grid(map: np.ndarray, start: tuple, goal: tuple) -> List[tuple]:
    height, width = map.shape  # Note: occupancy grid is row-major
    
    def is_valid(x, y):
        return (0 <= x < width and 0 <= y < height and map[y, x] < 50)
    
    open_set = []
    heappush(open_set, (0, start))
    cost_to_come = {}
    g = {start: 0}
    f = {start: heuristic(start, goal)}

    while open_set:
        _, current = heappop(open_set)
        
        if current == goal:
            return create_path_grid(cost_to_come, current)

        # Check 8-connected grid
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), 
                      (-1,-1), (1,1), (-1,1), (1,-1)]:
            next_x = current[0] + dx
            next_y = current[1] + dy
            neighbor = (next_x, next_y)
            
            if not is_valid(next_x, next_y):
                continue

            # Use correct cost for diagonal vs straight moves
            cost = np.sqrt(2) if abs(dx) + abs(dy) == 2 else 1
            tentative_g = g[current] + cost
            
            if neighbor not in g or tentative_g < g[neighbor]:
                cost_to_come[neighbor] = current
                g[neighbor] = tentative_g
                f[neighbor] = tentative_g + heuristic(neighbor, goal)
                heappush(open_set, (f[neighbor], neighbor))
    
    return []

def a_star_graph(start: Node, goal: Node) -> List[Node]:
    """
    This function will compute the optimal path between a starting node and an ending node.
    The result should be a list of the Edges that represent the optimal path to the goal. 
    For this function the cost and heuristic functions are defined when the node is originally created.

    
    If no path exists then this function should return an empty list.

    Worth 50 pts
    
    Input
      :param start: The starting node of the search
      :param goal: The ending node of the search

    Output
      :return: path: a list of Node objects representing the optimal path to the goal 
    """

    open_list = []
    heappush(open_list, (0, start))
    start.g = 0
    start.f = start.h

    while open_list:
        _, current = heappop(open_list)

        if current == goal:
            return create_graph_path(goal)

        for edge in current.edges:
            neighbor = edge.end
            tentative_g = current.g + edge.cost

            if tentative_g < neighbor.g:
                neighbor.previous = current
                neighbor.g = tentative_g
                neighbor.f = neighbor.g + neighbor.h
                heappush(open_list, (neighbor.f, neighbor))

    return []

def graph_demo():
    nodes = []

    nodes.append(Node('A', 10)) # A
    nodes.append(Node('B', 5))  # B
    nodes.append(Node('C', 6))  # C
    nodes.append(Node('D', 2))  # D
    nodes.append(Node('E', 3))  # E
    nodes.append(Node('F', 0))  # F

    nodes[0].add_neighbor(nodes[1],3)
    nodes[0].add_neighbor(nodes[2],4)
    nodes[1].add_neighbor(nodes[3],2)
    nodes[1].add_neighbor(nodes[4],2)
    nodes[2].add_neighbor(nodes[4],4)
    nodes[3].add_neighbor(nodes[5],5)
    nodes[3].add_neighbor(nodes[4],4)
    nodes[4].add_neighbor(nodes[5],4)

    path = a_star_graph(nodes[0],nodes[-1])

    print(path)

    for e_i in path:
        print(e_i)

def grid_demo():

    map = np.array([[0,0,1,0,0,0,0,0,0],
                    [0,0,1,0,0,0,0,0,0],
                    [0,0,1,0,0,1,1,1,0],
                    [0,0,1,0,0,1,0,0,0],
                    [0,0,1,0,0,1,0,1,1],
                    [0,0,1,0,0,1,0,0,0],
                    [0,0,0,0,0,1,0,0,0],
                    [0,0,0,0,0,1,0,0,0],
                    [0,0,0,0,0,1,0,0,0],
        ])

    start = (0,0)
    goal =  (8,8)
    path = a_star_grid(map, start, goal)
    for c_i in path:
        print(c_i)

def main():
    graph_demo()
    grid_demo()

if __name__ == '__main__':
    main()
