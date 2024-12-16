import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer, LookupException
import numpy as np
import tf2_ros
import math
from collections import deque

class DiffDrivePid(Node):
    def __init__(self):
        super().__init__('diffdrive_pid')
        
        # Path following parameters
        self.path_memory = deque(maxlen=50)
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.25
        self.path_update_min_distance = 0.5
        self.executing_path = False
        self.last_plan_time = self.get_clock().now()
        self.plan_timeout = 2.0
        
        # PID state variables
        self.x_goal = np.array([0.0, 0.0])
        self.has_goal = False
        self.x_prev = np.array([0.0, 0.0])
        self.x_error = np.array([0.0, 0.0])
        self.T_chassis_odom = None
        
        # Control parameters
        self.dt = 0.1
        self.l = 0.15
        
        # Adjusted PID gains
        self.kp = 0.6
        self.kd = 0.2
        self.ka = 0.6  # Angular gain for turn compensation
        
        # Modified velocity limits
        self.max_linear_vel = 0.8
        self.max_angular_vel = 1.5
        self.min_linear_vel = 0.1  # Minimum linear velocity during turns
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.path_sub = self.create_subscription(
            Path,
            '/en613/path',
            self.path_callback,
            10
        )
        
        self.timer = self.create_timer(self.dt, self.run)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/en613/cmd_vel',
            10
        )
        
        self.get_logger().info('DiffDrive PID Node Started')

    def update_path_memory(self, poses):
        """Update path memory with new poses"""
        # Clear existing memory if the new path is significantly different
        if self.path_memory:
            last_point = np.array([
                self.path_memory[-1].pose.position.x,
                self.path_memory[-1].pose.position.y
            ])
            new_point = np.array([
                poses[0].pose.position.x,
                poses[0].pose.position.y
            ])
            if np.linalg.norm(last_point - new_point) > self.path_update_min_distance:
                self.path_memory.clear()

        # Add new poses to memory
        for pose in poses:
            self.path_memory.append(pose)

    def path_callback(self, msg: Path):
        """Handle new path messages"""
        if not msg.poses:
            return
            
        self.last_plan_time = self.get_clock().now()
        self.update_path_memory(msg.poses)
        self.current_waypoint_idx = 0
        self.update_goal()
        self.executing_path = True
        self.get_logger().info(f'Updated path memory with {len(msg.poses)} points')

    def update_goal(self):
        """Update goal to next point in path memory"""
        if not self.path_memory or self.current_waypoint_idx >= len(self.path_memory):
            self.has_goal = False
            self.executing_path = False
            return
            
        current_goal = self.path_memory[self.current_waypoint_idx].pose
        self.x_goal = np.array([
            current_goal.position.x,
            current_goal.position.y,
        ])
        self.has_goal = True
        
        self.get_logger().info(
            f'Updated goal position: ({self.x_goal[0]:.2f}, {self.x_goal[1]:.2f}) '
            f'[{self.current_waypoint_idx + 1}/{len(self.path_memory)}]'
        )

    def is_plan_stale(self):
        """Check if current plan is too old"""
        time_since_plan = (self.get_clock().now() - self.last_plan_time).nanoseconds / 1e9
        return time_since_plan > self.plan_timeout

    def check_waypoint_progress(self, x_current):
        """Check if we've reached the current waypoint"""
        if not self.has_goal:
            return
            
        # Calculate distance to current waypoint
        distance = np.linalg.norm(x_current - self.x_goal)
        
        if distance < self.waypoint_threshold:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx + 1}/{len(self.path_memory)}'
            )
            self.current_waypoint_idx += 1
            
            # If we have more points in memory, move to next one
            if self.current_waypoint_idx < len(self.path_memory):
                self.update_goal()
            else:
                # Only clear goals if we don't have a stale plan
                if not self.is_plan_stale():
                    self.has_goal = False
                    self.executing_path = False
                else:
                    # Keep pursuing last point if plan is stale
                    self.current_waypoint_idx = len(self.path_memory) - 1
                    self.update_goal()

    def limit_velocity(self, linear: float, angular: float) -> tuple:
        """Limit velocities based on robot capabilities"""
        linear = np.clip(linear, -self.max_linear_vel, self.max_linear_vel)
        angular = np.clip(angular, -self.max_angular_vel, self.max_angular_vel)
        return linear, angular

    def get_robot_position(self):
        """Get current robot position from transform"""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                "odom",
                "chassis",
                now
            )
            return transform
        except tf2_ros.LookupException as ex:
            self.get_logger().warning(f'Could not get robot position: {ex}')
            return None

    def send_zero_velocity(self):
        """Helper function to stop the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)

    def calculate_desired_heading(self, current_pos):
        """Calculate desired heading to goal"""
        goal_direction = self.x_goal - current_pos
        return np.arctan2(goal_direction[1], goal_direction[0])

    def run(self):
        """Main control loop with improved turn handling"""
        if not self.has_goal and not self.path_memory:
            self.send_zero_velocity()
            return
            
        try:
            now = rclpy.time.Time()
            self.T_chassis_odom = self.tf_buffer.lookup_transform("odom", "chassis", now)
        except tf2_ros.LookupException:
            self.get_logger().info("Transform isn't available, waiting...")
            self.send_zero_velocity()
            return

        # Get current position and orientation
        x_pose_current = self.T_chassis_odom.transform.translation.x
        y_pose_current = self.T_chassis_odom.transform.translation.y
        q = self.T_chassis_odom.transform.rotation
        
        # Calculate heading
        theta_current = np.arctan2(
            2*(q.w * q.z + q.x * q.y), 
            1-2*(q.y * q.y + q.z * q.z)
        )
        theta_current = (theta_current + 2 * np.pi) % (2 * np.pi)
        
        # Current state
        x_current = np.array([x_pose_current, y_pose_current])
        v_current = (x_current - self.x_prev) / self.dt
        
        # Check progress
        self.check_waypoint_progress(x_current)
        
        # Calculate desired heading
        desired_heading = self.calculate_desired_heading(x_current)
        heading_error = (desired_heading - theta_current + np.pi) % (2 * np.pi) - np.pi
        
        # Trailer hitch point
        x_p = np.array([
            x_current[0] + self.l*np.cos(theta_current), 
            x_current[1] + self.l*np.sin(theta_current)
        ])
        
        # Position error
        self.x_error = x_p - self.x_goal
        
        # Basic PID control
        R = np.array([
            [np.cos(theta_current), np.sin(theta_current)],
            [-np.sin(theta_current), np.cos(theta_current)]
        ])
        inner = -self.kp*self.x_error - self.kd*v_current
        u = R @ inner
        
        # Calculate velocities with turn compensation
        linear_vel = u[0]
        angular_vel = u[1]/self.l + self.ka * heading_error
        
        # Adjust linear velocity based on turn angle
        heading_factor = abs(heading_error) / np.pi
        linear_vel *= (1 - 0.7 * heading_factor)  # Reduce speed during turns
        linear_vel = max(linear_vel, self.min_linear_vel)  # Maintain minimum speed
        
        # Apply limits
        linear_vel, angular_vel = self.limit_velocity(linear_vel, angular_vel)
        
        # Update previous position
        self.x_prev = x_current
        
        # Publish commands
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePid()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
