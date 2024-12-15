import signal
import sys
import numpy as np
from typing import List, Tuple
from heapq import heappush, heappop
import cv2

class Edge: 
    def __init__(self,starting_node, ending_node, cost):
        self.start = starting_node
        self.end = ending_node 
        self.cost = cost 

    def __repr__(self):
        return 'Node'+self.__str__()
    def __str__(self):
        return f'({self.start.name},{self.end.name},{self.cost})'

    def __eq__(self,obj):
        if isinstance(obj, Edge):
            return self.start == obj.start and obj.end == obj.end and self.cost == self.cost 
        return False

class Node:
    def __init__(self, name, h):
        self.name = name
        self.g = np.inf 
        self.f = np.inf 
        self.h = h 
        self.edges = []
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
        if isinstance(obj, Node):
            return self.name == obj.name and self.f == self.f and obj.g == obj.g and self.h == self.h 
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

def directional_heuristic(current: tuple, next_pos: tuple, goal: tuple, start: tuple) -> float:
    # Calculate basic Euclidean distance
    base_h = np.sqrt((next_pos[0] - goal[0]) ** 2 + (next_pos[1] - goal[1]) ** 2)
    
    if current == start:
        return base_h
        
    # Calculate desired direction (from start to goal)
    goal_dx = goal[0] - start[0]
    goal_dy = goal[1] - start[1]
    goal_mag = np.sqrt(goal_dx**2 + goal_dy**2)
    if goal_mag == 0:
        return base_h
    goal_dir = (goal_dx/goal_mag, goal_dy/goal_mag)
    
    # Calculate actual movement direction
    move_dx = next_pos[0] - current[0]
    move_dy = next_pos[1] - current[1]
    move_mag = np.sqrt(move_dx**2 + move_dy**2)
    if move_mag == 0:
        return base_h
    move_dir = (move_dx/move_mag, move_dy/move_mag)
    
    alignment = move_dir[0]*goal_dir[0] + move_dir[1]*goal_dir[1]

    FORWARD_WEIGHT = 0.75
    direction_factor = 2 - (alignment + 1) * FORWARD_WEIGHT
    
    return base_h * direction_factor

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
    f = {start: directional_heuristic(start, start, goal, start)}

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
                f[neighbor] = tentative_g + directional_heuristic(current, neighbor, goal, start)
                heappush(open_set, (f[neighbor], neighbor))
    
    return []

def a_star_graph(start: Node, goal: Node) -> List[Node]:
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
