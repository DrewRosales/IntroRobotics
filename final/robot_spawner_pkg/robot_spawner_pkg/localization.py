import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformListener, Buffer, TransformException
import numpy as np

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        
        # Initialize tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.scan_points = []
        self.new_scan = None
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/en613/scan',
            self.laser_callback,
            10
        )
        
        self.ray_publisher = self.create_publisher(
            PointStamped,
            '/en613/post_scan',
            10
        )
        
        # Enhanced debugging counters
        self.points_published = 0
        self.points_skipped = 0
        self.points_out_of_range = 0
        self.right_side_points = 0
        self.left_side_points = 0
        self.debug_timer = self.create_timer(5.0, self.debug_callback)
        
        self.get_logger().info("Localization Node Started")
        
    def debug_callback(self):
        """Enhanced debug information printed every 5 seconds"""
        self.get_logger().info(
            f'Debug Statistics:\n'
            f'- Total points published: {self.points_published}\n'
            f'- Points skipped (step): {self.points_skipped}\n'
            f'- Points out of range: {self.points_out_of_range}\n'
            f'- Left side points (-π to 0): {self.left_side_points}\n'
            f'- Right side points (0 to π): {self.right_side_points}'
        )
    
    def timer_callback(self):
        if self.new_scan is None:
            return
            
        msg = self.new_scan
        angle = msg.angle_min
        
        # Log angle range information
        if self.points_published == 0:  # Only log once
            self.get_logger().info(
                f'Scan Parameters:\n'
                f'- Angle min: {msg.angle_min:.2f} rad ({np.degrees(msg.angle_min):.2f}°)\n'
                f'- Angle max: {msg.angle_max:.2f} rad ({np.degrees(msg.angle_max):.2f}°)\n'
                f'- Angle increment: {msg.angle_increment:.4f} rad\n'
                f'- Range min: {msg.range_min:.2f}\n'
                f'- Range max: {msg.range_max:.2f}'
            )
        
        step = 2  # Adjust this value to control point density
        
        for i, ray in enumerate(msg.ranges):
            if i % step != 0:
                self.points_skipped += 1
                angle += msg.angle_increment
                continue
                
            if msg.range_min <= ray <= msg.range_max:
                x = ray * np.cos(angle)
                y = ray * np.sin(angle)
                
                # Track points by side
                if angle >= 0:
                    self.right_side_points += 1
                else:
                    self.left_side_points += 1
                
                point_msg = PointStamped()
                point_msg.header = msg.header
                point_msg.header.frame_id = "laser_link"
                point_msg.point.x = x
                point_msg.point.y = y
                point_msg.point.z = 0.0
                
                try:
                    if self.tf_buffer.can_transform('chassis', 'laser_link', rclpy.time.Time()):
                        self.ray_publisher.publish(point_msg)
                        self.points_published += 1
                        
                        # Detailed point logging every 100 points
                        if self.points_published % 100 == 0:
                            self.get_logger().debug(
                                f'Point details:\n'
                                f'- Position: ({x:.2f}, {y:.2f})\n'
                                f'- Angle: {angle:.2f} rad ({np.degrees(angle):.2f}°)\n'
                                f'- Range: {ray:.2f}'
                            )
                    else:
                        self.get_logger().warning('Static transform (chassis->laser_link) not available')
                except TransformException as e:
                    self.get_logger().warning(f'Transform error: {str(e)}')
            else:
                self.points_out_of_range += 1
                
            angle += msg.angle_increment
    
    def laser_callback(self, msg):
        self.get_logger().debug(
            f'Received laser scan with {len(msg.ranges)} points'
        )
        self.new_scan = msg

def main(args=None):
    rclpy.init(args=args)
    
    node = LocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
