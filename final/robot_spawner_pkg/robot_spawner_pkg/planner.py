import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from typing import Tuple, List

from . import a_star_planner

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Initialize subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/en613/occupancy_grid',
            self.map_callback,
            10
        )
        
        # Initialize publishers
        self.path_pub = self.create_publisher(
            Path,
            '/en613/path',
            10
        )
        
        # Store map data
        self.map_data = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        
        self.get_logger().info('Path Planner Node Started')
    
    def world_to_grid(self, world_point: Tuple[float, float]) -> Tuple[int, int]:
        """Convert world coordinates to grid cell indices"""
        grid_x = int((world_point[0] - self.origin.x) / self.resolution)
        grid_y = int((world_point[1] - self.origin.y) / self.resolution)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_point: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid cell indices to world coordinates"""
        world_x = grid_point[0] * self.resolution + self.origin.x
        world_y = grid_point[1] * self.resolution + self.origin.y
        return (world_x, world_y)
    
    def plan_path(self, start_world: Tuple[float, float], 
                 goal_world: Tuple[float, float]) -> Path:
        """Plan path using A* from start to goal in world coordinates"""
        if self.map_data is None:
            self.get_logger().warn('No map data available')
            return None
            
        # Convert world coordinates to grid coordinates
        start_grid = self.world_to_grid(start_world)
        goal_grid = self.world_to_grid(goal_world)
        
        self.get_logger().info(f'Planning path from {start_grid} to {goal_grid}')
        
        # Check if start and goal are valid
        if not (0 <= start_grid[0] < self.width and 0 <= start_grid[1] < self.height):
            self.get_logger().error('Start position out of bounds')
            return None
        if not (0 <= goal_grid[0] < self.width and 0 <= goal_grid[1] < self.height):
            self.get_logger().error('Goal position out of bounds')
            return None
        
        # Reshape map data into 2D array
        occupancy_grid = np.array(self.map_data).reshape((self.height, self.width))
        
        # Run A* search
        grid_path = a_star_planner.a_star_search(start_grid, goal_grid, occupancy_grid)
        
        if not grid_path:
            self.get_logger().warn('No path found')
            return None
            
        # Convert path to world coordinates and create ROS message
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for grid_point in grid_path:
            world_point = self.grid_to_world(grid_point)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_point[0]
            pose.pose.position.y = world_point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        return path_msg
    
    def map_callback(self, msg: OccupancyGrid):
        """Handle incoming occupancy grid updates"""
        self.map_data = msg.data
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
        self.width = msg.info.width
        self.height = msg.info.height
        
        self.get_logger().info(
            f'Received map update: {self.width}x{self.height} cells, '
            f'resolution: {self.resolution}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
