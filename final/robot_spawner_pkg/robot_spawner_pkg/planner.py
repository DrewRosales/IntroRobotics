import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Transform
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
from typing import Tuple, List
from . import a_star_planner

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Transform setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/en613/occupancy_grid',
            self.map_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseArray,
            '/en613/goals',
            self.goal_callback,
            10
        )
        
        # Publishers
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
        
        # Goal management
        self.current_goals = []
        self.current_goal_index = 0
        
        self.get_logger().info('Path Planner Node Started')
    
    def get_robot_position(self) -> tuple:
        """Get current robot position from transform"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'chassis',
                rclpy.time.Time()
            )
            return (transform.transform.translation.x, 
                   transform.transform.translation.y)
        except TransformException as ex:
            self.get_logger().warning(f'Could not get robot position: {ex}')
            return None
    
    def world_to_grid(self, world_point: tuple) -> tuple:
        """Convert world coordinates to grid cell indices"""
        # Origin is at (-6, -6) for a 12x12 map
        # Need to shift coordinates to positive space first
        grid_x = int((world_point[0] - self.origin.x) / self.resolution)
        grid_y = int((world_point[1] - self.origin.y) / self.resolution)
    
        self.get_logger().debug(
            f"World to grid conversion:"
            f"\n  World: {world_point}"
            f"\n  Origin: ({self.origin.x}, {self.origin.y})"
            f"\n  Resolution: {self.resolution}"
            f"\n  Calculation: x=({world_point[0]} - ({self.origin.x}))/{self.resolution}"
            f"\n  Grid: ({grid_x}, {grid_y})"
        )
    
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_point: tuple) -> tuple:
        """Convert grid cell indices to world coordinates"""
        world_x = grid_point[0] * self.resolution + self.origin.x
        world_y = grid_point[1] * self.resolution + self.origin.y
        return (world_x, world_y)
    
    def plan_path(self, start_world: tuple, goal_world: tuple) -> Path:
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
        grid_path = a_star_planner.a_star_grid(occupancy_grid, start_grid, goal_grid)
        
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
    
    def goal_callback(self, msg: PoseArray):
        """Handle new goal messages"""
        if not msg.poses or self.map_data is None:
            return
            
        # Store new goals
        self.current_goals = [(pose.position.x, pose.position.y) 
                            for pose in msg.poses]
        self.current_goal_index = 0
        
        self.get_logger().info(f'Received {len(self.current_goals)} goals')
        self.plan_to_next_goal()
    
    def plan_to_next_goal(self):
        self.get_logger().info("PLAN TO NEXT GOAL")
        """Plan path to the next goal in sequence"""
        if self.current_goal_index >= len(self.current_goals):
            self.get_logger().info('All goals reached')
            return
        
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
        
        current_goal = self.current_goals[self.current_goal_index]
        self.get_logger().info(
            f'Planning to goal {self.current_goal_index + 1}/{len(self.current_goals)}: '
            f'{current_goal}'
        )
        
        path = self.plan_path(robot_pos, current_goal)
        if path:
            self.path_pub.publish(path)
            self.get_logger().info('Published path to current goal')
            self.current_goal_index += 1
            
            # Plan to next goal if available
            if self.current_goal_index < len(self.current_goals):
                self.plan_to_next_goal()
        else:
            self.get_logger().warn('Failed to plan path to current goal')
    
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
