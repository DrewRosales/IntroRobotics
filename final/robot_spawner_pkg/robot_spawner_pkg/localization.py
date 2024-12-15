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
        
        # Reduce timer frequency to not overwhelm the system
        self.timer = self.create_timer(0.1, self.timer_callback)
        
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
        
        # Add debug counter
        self.points_published = 0
        self.debug_timer = self.create_timer(5.0, self.debug_callback)
        
        self.get_logger().info("Localization Node Started")
        
    def debug_callback(self):
        """Print debug information every 5 seconds"""
        self.get_logger().debug(f'Total points published: {self.points_published}')
    
    def timer_callback(self):
        if self.new_scan is None:
            return
            
        msg = self.new_scan
        angle = msg.angle_min
        
        # Process fewer points to avoid overwhelming the system
        step = 10  # Adjust this value to control point density
        
        for i, ray in enumerate(msg.ranges):
            if i % step != 0:  # Skip points based on step size
                angle += msg.angle_increment
                continue
                
            if msg.range_min <= ray <= msg.range_max:
                # Calculate point in laser frame
                x = ray * np.cos(angle)
                y = ray * np.sin(angle)
                
                # Create point message
                point_msg = PointStamped()
                point_msg.header = msg.header
                point_msg.header.frame_id = "laser_link"  # Set correct frame
                point_msg.point.x = x
                point_msg.point.y = y
                point_msg.point.z = 0.0
                
                try:
                    # First verify we have the static transform from chassis to laser
                    if self.tf_buffer.can_transform('chassis', 'laser_link', rclpy.time.Time()):
                        # If we have both transforms, publish the point
                        self.ray_publisher.publish(point_msg)
                        self.points_published += 1
                        self.get_logger().debug(
                            f'Published point: ({x:.2f}, {y:.2f}) in laser frame'
                        )
                    else:
                        self.get_logger().warning('Static transform (chassis->laser_link) not available')
                except TransformException as e:
                    self.get_logger().warning(f'Transform error: {str(e)}')
                
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
