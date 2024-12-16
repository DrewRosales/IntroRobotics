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
        
        self.get_logger().info("Localization Node Started")
        
    def timer_callback(self):
        if self.new_scan is None:
            return
            
        msg = self.new_scan
        angle = msg.angle_min
        
        for i, ray in enumerate(msg.ranges):
            if msg.range_min <= ray <= msg.range_max:
                x = ray * np.cos(angle)
                y = ray * np.sin(angle)
                
                point_msg = PointStamped()
                point_msg.header = msg.header
                point_msg.header.frame_id = "laser_link"
                point_msg.point.x = x
                point_msg.point.y = y
                point_msg.point.z = 0.0
                
                try:
                    if self.tf_buffer.can_transform('chassis', 'laser_link', rclpy.time.Time()):
                        self.ray_publisher.publish(point_msg)
                    else:
                        self.get_logger().warning('Static transform (chassis->laser_link) not available')
                except TransformException as e:
                    self.get_logger().warning(f'Transform error: {str(e)}')
                    
            angle += msg.angle_increment
    
    def laser_callback(self, msg):
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
