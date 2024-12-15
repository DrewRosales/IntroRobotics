import numpy as np
from matplotlib import pyplot
from matplotlib.patches import Rectangle

class MapType:
    UNKNOWN = -1
    UNOCCUPIED = 0
    OCCUPIED = 1

class QuadMapNode:
    def __init__(self, parent, origin: np.ndarray, size):
        if parent is None:
            self.parent = None
            assert size is not None
            self.size = size
        else:
            self.parent = parent
            self.size = parent.size / 2.0 

        self.origin = origin
        self.children = []
        self.state = MapType.UNKNOWN

    def split(self):
        if self.children:
            return
        
        child_size = self.size / 2
        # Order: top-right, bottom-right, top-left, bottom-left
        child_centers = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in child_centers:
            child_origin = self.origin + np.array([dx, dy]) * child_size / 2
            child = QuadMapNode(parent=self, origin=child_origin, size=child_size)
            child.state = self.state
            self.children.append(child)

    def combine(self):
        """Combines children if they all have the same state and propagates up"""
        if not self.children:
            return
        
        child_state = self.children[0].state
        for child in self.children:
            if child.state != child_state:
                return
                
        self.state = child_state
        self.children = []
        
        if self.parent:
            self.parent.combine()

    def recursive_get_grid(self, ngrid: np.ndarray):
        """Recursively fills grid with correct quadrant ordering"""
        l = len(ngrid)
        l_half = int(l / 2)
        
        if not self.children:
            ngrid[:] = self.state
        else:
            # Order matches child creation order
            self.children[0].recursive_get_grid(ngrid[l_half:, l_half:])  # top-right
            self.children[1].recursive_get_grid(ngrid[:l_half, l_half:])  # bottom-right
            self.children[2].recursive_get_grid(ngrid[l_half:, :l_half])  # top-left
            self.children[3].recursive_get_grid(ngrid[:l_half, :l_half])  # bottom-left

    def plot_node(self, ax):
        if self.children:
            for child in self.children:
                child.plot_node(ax)
        else:
            facecolor = 'grey'
            if self.state == MapType.UNKNOWN:
                facecolor = 'grey'
            elif self.state == MapType.UNOCCUPIED:
                facecolor = 'white'
            elif self.state == MapType.OCCUPIED:
                facecolor = 'black'

            ax.add_patch(Rectangle(
                (self.origin[0] - self.size / 2, self.origin[1] - self.size / 2),
                self.size, self.size,
                edgecolor='black',
                facecolor=facecolor,
                fill=True,
                lw=5
            ))

class QuadMap:
    def __init__(self, max_depth=3, size=4.0, origin=np.array([0.0, 0.0])):
        self.max_depth = max_depth
        self.size = size
        self.origin = origin
        self.root = QuadMapNode(parent=None, origin=origin, size=size)

    def point_update(self, point: np.ndarray, state: int):
        """Updates the value of the given point in the QuadMap"""
        # Check bounds
        half_size = self.size / 2
        if (abs(point[0] - self.origin[0]) > half_size or 
            abs(point[1] - self.origin[1]) > half_size):
            return

        node = self.root
        depth = 0
        
        while depth <= self.max_depth:
            if depth == self.max_depth:
                if node.state != state:
                    node.state = state
                    if node.parent:
                        node.parent.combine()
                break
                
            if not node.children:
                node.split()

            child_size = node.size / 2
            for child in node.children:
                if (abs(point[0] - child.origin[0]) <= child_size / 2 and 
                    abs(point[1] - child.origin[1]) <= child_size / 2):
                    node = child
                    break

            depth += 1

    def ray_update(self, origin: np.ndarray, endpoint: np.ndarray):
        """Updates the map based on a rangefinding beam"""
        # Check if points are within bounds
        def is_in_bounds(point):
            half_size = self.size / 2
            return (abs(point[0] - self.origin[0]) <= half_size and 
                   abs(point[1] - self.origin[1]) <= half_size)
        
        if not (is_in_bounds(origin) and is_in_bounds(endpoint)):
            return
        
        # Calculate ray properties
        vec = endpoint - origin
        vec_len = np.linalg.norm(vec)
        if vec_len < 1e-6:
            return
            
        # Use resolution-based step size
        step_size = self.size / (2 ** (self.max_depth + 2))
        num_steps = int(vec_len / step_size)
        num_steps = max(num_steps, 10)  # Ensure minimum number of steps
        
        # Update free space along ray
        normalized_vec = vec / vec_len
        for i in range(num_steps):
            point = origin + (i * step_size) * normalized_vec
            if is_in_bounds(point):
                self.point_update(point, MapType.UNOCCUPIED)
        
        # Mark endpoint as occupied
        self.point_update(endpoint, MapType.OCCUPIED)

    def get_state(self, point: np.ndarray):
        """Returns the MapType state for a given point"""
        node = self.root

        while True:
            if not node.children:
                return node.state
        
            child_size = node.size / 2
            for child in node.children:
                if (abs(point[0] - child.origin[0]) <= child_size / 2 and 
                    abs(point[1] - child.origin[1]) <= child_size / 2):
                    node = child
                    break
            else:
                return MapType.UNKNOWN

    def to_occupancygrid(self):
        """Converts QuadMap to flattened occupancy grid for ROS"""
        grid_size = 2 ** self.max_depth
        grid = np.full((grid_size, grid_size), MapType.UNKNOWN)
        self.root.recursive_get_grid(grid)
        
        # Flip vertically to match ROS convention
        grid = np.flip(grid, axis=0)
        
        return grid.flatten().tolist()

    def plot(self):
        """Visualization helper"""
        fig = pyplot.figure(1, figsize=[5, 5], dpi=90)
        ax = fig.add_subplot(111)
        self.root.plot_node(ax)
        pyplot.xlim([0, 10])
        pyplot.ylim([0, 10])
        pyplot.draw()
        pyplot.pause(0.0001)

def main():
    map = QuadMap(max_depth=5, size=10.0, origin=np.array([5.0, 5.0]))
    
    x = np.array([5, 6.5])
    ray = np.array([2, 3])
    for i in range(30):
        map.ray_update(x, x + ray)
        map.plot()
        x += np.array([0.025, -0.025])
        ray = np.array([[np.cos(0.1), -np.sin(0.1)],
                       [np.sin(0.1), np.cos(0.1)]]) @ ray

if __name__ == '__main__':
    main()
