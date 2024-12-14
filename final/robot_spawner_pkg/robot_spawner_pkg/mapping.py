from .quadtree import *

class Mapping(Node):
    def __init__(self):
        super().__init__('mapping')

        #TODO: find the map size based on the map spawner

        max_size = 4
        max_depth = 5

        #TODO: also fix the origin based on the map spawn
        self.quad_map = QuadMap(max_depth=max_depth,
                                size=map_size,
                                origin=np.array([0.0,0.0]))



