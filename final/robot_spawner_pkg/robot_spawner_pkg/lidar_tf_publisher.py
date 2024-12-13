import os
import sys
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TFPublisher(Node):

    def __init__(self):
        super().__init__('lidar_tf_publisher')
        self._tf_publisher = StaticTransformBroadcaster(self)

        self.timer = self.create_timer(0.01, self.lidar_transform)

    def lidar_transform(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'chassis'
        static_transformStamped.child_frame_id = 'laser_link'
        static_transformStamped.transform.translation.x = 0.15
        static_transformStamped.transform.translation.y = 0.0 
        static_transformStamped.transform.translation.z = 0.35
        static_transformStamped.transform.rotation.x = 0.0
        static_transformStamped.transform.rotation.y = 0.0
        static_transformStamped.transform.rotation.z = 0.0
        static_transformStamped.transform.rotation.w = 1.0

        self._tf_publisher.sendTransform(static_transformStamped)

def main():

    # Start node
    rclpy.init()


    #Start publishing the goal position
    tf_publisher = TFPublisher()

    rclpy.spin(tf_publisher)

    tf_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()