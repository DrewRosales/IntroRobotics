import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Transform
from tf2_ros import TransformException, Buffer, TransformListener
import numpy as np
from typing import Tuple, List
import cv2
from . import a_star_planner

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
        self.received_goals = []  # All goals from topic
        self.planning_goals = []  # Goals for planning
        self.goal_index = 0     # Current goal index
        self.current_goal = None  # Active goal being pursued
        
        # Robot parameters
        self.ROBOT_RADIUS = 0.3  # meters
        self.GOAL_TOLERANCE = 0.5  # meters
        
        # Debug flags
        self.debug = True
        
        # Progress checking timer
        self.progress_timer = self.create_timer(0.1, self.check_progress)
        
        self.get_logger().info('Path Planner Node Started')
    
    def debug_log_state(self, context: str):
        """Log current state for debugging"""
        if self.debug:
            self.get_logger().info(
                f"[{context}] State:"
                f"\n  Current Goal: {self.current_goal}"
                f"\n  Goal Index: {self.goal_index}/{len(self.planning_goals)}"
                f"\n  Total Goals: {len(self.received_goals)}"
            )

    def inflate_grid(self, grid: np.ndarray) -> np.ndarray:
        """Inflate obstacles in the grid by robot radius"""
        # Convert to binary grid (0 for free, 1 for occupied)
        binary_grid = (grid >= 50).astype(np.uint8)
        
        # Calculate kernel size based on robot radius
        kernel_size = int(np.ceil(self.ROBOT_RADIUS / self.resolution))
        kernel = np.ones((kernel_size * 2 + 1, kernel_size * 2 + 1), np.uint8)
        
        # Dilate obstacles
        inflated_grid = cv2.dilate(binary_grid, kernel, iterations=1)
        
        # Convert back to occupancy grid format
        inflated_grid = inflated_grid * 100
        
        return inflated_grid
    
    def check_progress(self):
        """Periodically check distance to current goal"""
        if not self.current_goal:
            return
            
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
            
        distance = np.sqrt((robot_pos[0] - self.current_goal[0])**2 + 
                          (robot_pos[1] - self.current_goal[1])**2)
        
        self.get_logger().debug(f"Distance to goal: {distance:.2f}m")
        
        if distance <= self.GOAL_TOLERANCE:
            self.get_logger().info(
                f'Reached goal {self.goal_index + 1}/{len(self.planning_goals)}!'
                f' Distance: {distance:.2f}m'
            )
            
            # Increment goal index and clear current goal
            self.goal_index += 1
            self.current_goal = None
            
            self.debug_log_state("Goal Reached")
    
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
        grid_x = int((world_point[0] - self.origin.x) / self.resolution)
        grid_y = int((world_point[1] - self.origin.y) / self.resolution)
        
        self.get_logger().debug(
            f"World to grid conversion:"
            f"\n  World: {world_point}"
            f"\n  Origin: ({self.origin.x}, {self.origin.y})"
            f"\n  Resolution: {self.resolution}"
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
        
        # Inflate grid for robot size
        inflated_grid = self.inflate_grid(occupancy_grid)
        
        # Run A* search
        grid_path = a_star_planner.a_star_grid(inflated_grid, start_grid, goal_grid)
        
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
        
        self.get_logger().info(f'Created path message with {len(path_msg.poses)} poses')
        return path_msg
    
    def goal_callback(self, msg: PoseArray):
        """Handle new goals from topic"""
        if not msg.poses:
            return
            
        # Add goal if it's new
        for pose in msg.poses:
            new_goal = (pose.position.x, pose.position.y)
            if new_goal not in self.received_goals:
                self.received_goals.append(new_goal)
                self.planning_goals.append(new_goal)
        
        self.get_logger().info(f'Total goals: {len(self.planning_goals)}')
        self.debug_log_state("Goal Callback")
    
    def plan_to_next_goal(self):
        """Plan path to the current goal based on index"""
        self.get_logger().info("PLAN TO NEXT GOAL")
        
        if self.goal_index >= len(self.planning_goals):
            self.get_logger().info("All goals completed")
            return
        
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            self.get_logger().warn("Cannot plan: No robot position")
            return
        
        # Get goal at current index if we don't have one
        if not self.current_goal:
            self.current_goal = self.planning_goals[self.goal_index]
            self.get_logger().info(f"Setting new current goal: {self.current_goal}")
        
        # Always plan if we have a current goal
        if self.current_goal:
            self.get_logger().info(
                f'Planning to goal {self.goal_index + 1}/{len(self.planning_goals)}: '
                f'{self.current_goal}'
            )
            
            self.get_logger().info(f'Planning path from {robot_pos} to {self.current_goal}')
            path = self.plan_path(robot_pos, self.current_goal)
            
            if path:
                self.path_pub.publish(path)
                self.get_logger().info('Published updated path to current goal')
            else:
                self.get_logger().warn('Failed to plan path to current goal')
        else:
            self.get_logger().info("No current goal to plan for")
        
        self.debug_log_state("After Planning")
    
    def map_callback(self, msg: OccupancyGrid):
        """Handle incoming occupancy grid updates and trigger planning"""
        self.map_data = msg.data
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin.position
        self.width = msg.info.width
        self.height = msg.info.height
        
        self.get_logger().info(
            f'Received map update: {self.width}x{self.height} cells, '
            f'resolution: {self.resolution}m'
        )
        
        # Always try to plan if we have goals and haven't completed them all
        if self.planning_goals and self.goal_index < len(self.planning_goals):
            self.plan_to_next_goal()

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
