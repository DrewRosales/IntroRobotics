import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer, TransformException, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import numpy as np
from robot_spawner_pkg.quadtree import QuadMap, MapType

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')

        # Initialize transform buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize QuadMap
        # Size of 12.0 ensures coverage of all possible maze configurations
        # Origin at (0,0) matches Gazebo spawn point
        self.quad_map = QuadMap(
            max_depth=6,
            size=12.0,
            origin=np.array([0.0, 0.0])
        )
        
        # Subscribe to point cloud data from localization
        self.point_subscription = self.create_subscription(
            PointStamped,
            '/en613/post_scan',
            self.point_callback,
            10
        )
        
        # Publisher for occupancy grid visualization
        self.occupancy_publisher = self.create_publisher(
            OccupancyGrid,
            '/en613/occupancy_grid',
            10
        )
        
        # Timer for publishing occupancy grid
        self.timer = self.create_timer(0.01, self.publish_occupancy_grid)
        
        self.get_logger().info(
            f'Mapping Node Started with:'
            f'\n  Map Size: {self.quad_map.size}m x {self.quad_map.size}m'
            f'\n  Resolution: {self.quad_map.size / (2 ** 6)}m'
            f'\n  Coverage: ({-self.quad_map.size/2}, {-self.quad_map.size/2}) to '
            f'({self.quad_map.size/2}, {self.quad_map.size/2})'
        )

    def point_callback(self, msg):
        """Handle incoming point data from the laser scan"""
        try:
            self.get_logger().info("1. Starting point callback")
            
            try:
                # Get latest transform time first
                latest_tf_time = self.tf_buffer.get_latest_common_time('odom', 'laser_link')
                self.get_logger().info(f"2. Latest common transform time: {latest_tf_time.nanoseconds}")
    
                # Get transforms using this common time
                chassis_to_odom = self.tf_buffer.lookup_transform(
                    'odom',
                    'chassis',
                    latest_tf_time,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.get_logger().info("3. Successfully got chassis->odom transform")
                
                robot_pos = np.array([
                    chassis_to_odom.transform.translation.x,
                    chassis_to_odom.transform.translation.y
                ])
                self.get_logger().info(f"4. Robot position: {robot_pos}")
    
                # Create a new PointStamped with the common transform time
                point_stamped = PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id
                point_stamped.header.stamp = latest_tf_time.to_msg()
                point_stamped.point = msg.point
    
                self.get_logger().info("7. Attempting to transform point to odom")
                point_in_odom = self.tf_buffer.transform(
                    point_stamped,
                    'odom',
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                self.get_logger().info("8. Successfully transformed point")
    
                point = np.array([
                    point_in_odom.point.x,
                    point_in_odom.point.y
                ])
                self.get_logger().info(f"9. Point position: {point}")
    
                # Debug information about the ray
                dist = np.linalg.norm(point - robot_pos)
                self.get_logger().info(f"10. Ray length: {dist}m")
                
                if dist > 10.0:  # Add reasonable distance check
                    self.get_logger().warn(f"Suspicious ray length: {dist}m, skipping update")
                    return
                
                # Update map with ray tracing
                self.quad_map.ray_update(
                    origin=robot_pos,
                    endpoint=point
                )
                self.get_logger().info("11. Updated quadmap")
                
                # Print map stats after update
                grid_data = self.quad_map.to_occupancygrid()
                occupied_count = sum(1 for x in grid_data if x == MapType.OCCUPIED)
                unoccupied_count = sum(1 for x in grid_data if x == MapType.UNOCCUPIED)
                unknown_count = sum(1 for x in grid_data if x != MapType.OCCUPIED and x != MapType.UNOCCUPIED)
                self.get_logger().info(
                    f"12. Map stats - Occupied: {occupied_count}, "
                    f"Unoccupied: {unoccupied_count}, Unknown: {unknown_count}"
                )
                
            except Exception as ex:
                self.get_logger().error(f"Transform or update failed: {str(ex)}")
                return
                
        except Exception as ex:
            self.get_logger().error(f'Error in point_callback: {str(ex)}')
            self.get_logger().error(f'Exception type: {type(ex)}')
    def publish_occupancy_grid(self):
        """Convert QuadMap to OccupancyGrid message and publish"""
        try:
            msg = OccupancyGrid()
            
            # Set header
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            
            # Set metadata
            msg.info.resolution = self.quad_map.size / (2 ** self.quad_map.max_depth)
            msg.info.width = 2 ** self.quad_map.max_depth
            msg.info.height = 2 ** self.quad_map.max_depth
            
            # Set origin (bottom-left corner of the grid)
            msg.info.origin.position.x = -self.quad_map.size/2
            msg.info.origin.position.y = -self.quad_map.size/2
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.w = 1.0
            
            # Convert QuadMap data to occupancy grid format
            grid_data = self.quad_map.to_occupancygrid()
            msg.data = [100 if x == MapType.OCCUPIED else 
                       (0 if x == MapType.UNOCCUPIED else -1) 
                       for x in grid_data]
            
            # Publish the message
            self.occupancy_publisher.publish(msg)
            
        except Exception as ex:
            self.get_logger().error(f'Error publishing occupancy grid: {str(ex)}')

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
