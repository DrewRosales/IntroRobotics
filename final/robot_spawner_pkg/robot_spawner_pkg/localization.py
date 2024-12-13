import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import numpy as np

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')

        self.scan_points = []

        self.new_scan = None

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/en613/scan',
            self.laser_callback,
            10
        )

        self.ray_publisher = self.create_publisher(
            PointStamped,
            'scan_points',
            10
        )

        self.get_logger().info("Localization Node Started")

    def timer_callback(self):

        self.get_logger().info("Localization Timer callback")
        if self.new_scan is None:
            return

        msg = self.new_scan
        angle = msg.angle_min

        for ray in msg.ranges:
            if msg.range_min <=  ray <= msg.range_max:
                x = ray * np.cos(angle)
                y = ray * np.sin(angle)

                point_msg = PointStamped()
                point_msg.header = msg.header
                point_msg.point.x = x
                point_msg.point.y = y

                self.ray_publisher.publish(point_msg)

            angle += msg.angle_increment

    def laser_callback(self, msg):
        self.new_scan = msg
        self.get_logger().info("Localization Received new scan!!")


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




