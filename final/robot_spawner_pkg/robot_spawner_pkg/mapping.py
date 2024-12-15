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
            
            try:
                # Get latest transform time first
                latest_tf_time = self.tf_buffer.get_latest_common_time('odom', 'laser_link')
    
                # Get transforms using this common time
                chassis_to_odom = self.tf_buffer.lookup_transform(
                    'odom',
                    'chassis',
                    latest_tf_time,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                robot_pos = np.array([
                    chassis_to_odom.transform.translation.x,
                    chassis_to_odom.transform.translation.y
                ])
    
                # Create a new PointStamped with the common transform time
                point_stamped = PointStamped()
                point_stamped.header.frame_id = msg.header.frame_id
                point_stamped.header.stamp = latest_tf_time.to_msg()
                point_stamped.point = msg.point
    
                point_in_odom = self.tf_buffer.transform(
                    point_stamped,
                    'odom',
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                point = np.array([
                    point_in_odom.point.x,
                    point_in_odom.point.y
                ])

                
                map_half_size = self.quad_map.size / 2
                if (abs(robot_pos[0]) > map_half_size or 
                    abs(robot_pos[1]) > map_half_size or
                    abs(point[0]) > map_half_size or
                    abs(point[1]) > map_half_size):
                    self.get_logger().warn("Point or robot position outside map bounds!")
                    return
                
                resolution = self.quad_map.size / (2 ** self.quad_map.max_depth)
                robot_grid_x = int((robot_pos[0] - (self.quad_map.origin[0] - self.quad_map.size/2)) / resolution)
                robot_grid_y = int((robot_pos[1] - (self.quad_map.origin[1] - self.quad_map.size/2)) / resolution)
                point_grid_x = int((point[0] - (self.quad_map.origin[0] - self.quad_map.size/2)) / resolution)
                point_grid_y = int((point[1] - (self.quad_map.origin[1] - self.quad_map.size/2)) / resolution)
                

                resolution = self.quad_map.size / (2 ** self.quad_map.max_depth)
                
                robot_x = (robot_pos[0] + self.quad_map.size/2) / resolution
                robot_y = (robot_pos[1] + self.quad_map.size/2) / resolution
                
                point_x = (point[0] + self.quad_map.size/2) / resolution
                point_y = (point[1] + self.quad_map.size/2) / resolution
    
                
                
                self.quad_map.ray_update(
                    origin=robot_pos,
                    endpoint=point
                )
                
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
