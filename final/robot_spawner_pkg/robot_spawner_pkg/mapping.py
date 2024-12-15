import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer, TransformException, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import numpy as np
from robot_spawner_pkg.quadtree import QuadMap, MapType
from rclpy.duration import Duration

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        
        self.tf_buffer = Buffer(Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
                self.get_logger().info(f"Map size: {self.quad_map.size}")
                self.get_logger().info(f"Map origin: {self.quad_map.origin}")
                self.get_logger().info(f"Robot world pos: {robot_pos}")
                self.get_logger().info(f"Point world pos: {point}")
                
                map_half_size = self.quad_map.size / 2
                if (abs(robot_pos[0]) > map_half_size or 
                    abs(robot_pos[1]) > map_half_size or
                    abs(point[0]) > map_half_size or
                    abs(point[1]) > map_half_size):
                    self.get_logger().warn("Point or robot position outside map bounds!")
                    return
                
                # Calculate grid coordinates
                resolution = self.quad_map.size / (2 ** self.quad_map.max_depth)
                robot_grid_x = int((robot_pos[0] - (self.quad_map.origin[0] - self.quad_map.size/2)) / resolution)
                robot_grid_y = int((robot_pos[1] - (self.quad_map.origin[1] - self.quad_map.size/2)) / resolution)
                point_grid_x = int((point[0] - (self.quad_map.origin[0] - self.quad_map.size/2)) / resolution)
                point_grid_y = int((point[1] - (self.quad_map.origin[1] - self.quad_map.size/2)) / resolution)
                
                self.get_logger().info(f"Robot grid pos: ({robot_grid_x}, {robot_grid_y})")
                self.get_logger().info(f"Point grid pos: ({point_grid_x}, {point_grid_y})")

                # Calculate grid coordinates with more precise debug info
                resolution = self.quad_map.size / (2 ** self.quad_map.max_depth)
                
                # Robot grid coordinates
                robot_x = (robot_pos[0] + self.quad_map.size/2) / resolution
                robot_y = (robot_pos[1] + self.quad_map.size/2) / resolution
                
                # Point grid coordinates
                point_x = (point[0] + self.quad_map.size/2) / resolution
                point_y = (point[1] + self.quad_map.size/2) / resolution
    
                self.get_logger().info(
                    f"Grid calculation:"
                    f"\n  Resolution: {resolution}m/cell"
                    f"\n  Robot raw coords: ({robot_x:.2f}, {robot_y:.2f})"
                    f"\n  Point raw coords: ({point_x:.2f}, {point_y:.2f})"
                    f"\n  Robot grid pos: ({int(robot_x)}, {int(robot_y)})"
                    f"\n  Point grid pos: ({int(point_x)}, {int(point_y)})"
                )
    
    
                # Debug information about the ray
                dist = np.linalg.norm(point - robot_pos)
                self.get_logger().info(f"10. Ray length: {dist}m")
                
                if dist > 10.0:  # Add reasonable distance check
                    self.get_logger().warn(f"Suspicious ray length: {dist}m, skipping update")
                    return
                
                self.get_logger().info(
                    f"Ray trace:"
                    f"\n  From: world ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})"
                    f"\n  To: world ({point[0]:.2f}, {point[1]:.2f})"
                )
    
                # Calculate grid coordinates with explicit steps
                resolution = self.quad_map.size / (2 ** self.quad_map.max_depth)
                
                # Step 1: Translate to positive coordinates
                robot_translated = robot_pos + np.array([self.quad_map.size/2, self.quad_map.size/2])
                point_translated = point + np.array([self.quad_map.size/2, self.quad_map.size/2])
                
                # Step 2: Convert to grid coordinates
                robot_grid = robot_translated / resolution
                point_grid = point_translated / resolution
                
                self.get_logger().info(
                    f"Coordinate transformation steps:"
                    f"\n  1. Translated robot: ({robot_translated[0]:.2f}, {robot_translated[1]:.2f})"
                    f"\n  2. Translated point: ({point_translated[0]:.2f}, {point_translated[1]:.2f})"
                    f"\n  3. Grid robot: ({robot_grid[0]:.2f}, {robot_grid[1]:.2f})"
                    f"\n  4. Grid point: ({point_grid[0]:.2f}, {point_grid[1]:.2f})"
                )
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
        try:
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            
            msg.info.resolution = self.quad_map.size / (2 ** self.quad_map.max_depth)
            msg.info.width = 2 ** self.quad_map.max_depth
            msg.info.height = 2 ** self.quad_map.max_depth
            
            msg.info.origin.position.x = -self.quad_map.size/2
            msg.info.origin.position.y = -self.quad_map.size/2
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.w = 1.0
            
            # Convert QuadMap data to occupancy grid format
            grid_data = self.quad_map.to_occupancygrid()
            msg.data = [100 if x == MapType.OCCUPIED else 
                       (0 if x == MapType.UNOCCUPIED else -1) 
                       for x in grid_data]
            
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
