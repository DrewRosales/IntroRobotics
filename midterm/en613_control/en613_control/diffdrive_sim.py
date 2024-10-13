from .differential_drive import DifferentialDrive

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, TransformStamped
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
import numpy as np

class DiffDriveSim(Node):
    def __init__(self):
        super().__init__('diff_drive_sim')

        self.x = np.array([0, 0, 0])
        self.q = np.array([0, 0])

        self.dt = 1.0/30.0

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.omega = 0.0


        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        qos_profile = QoSProfile(depth=10)
        self.joint_publisher  = self.create_publisher(JointState, '/joint_states', qos_profile)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.reset_pose_service = self.create_service(
            Trigger,
            '/robot_pose_reset',
            self.reset_pose_callback
        )
        self.wheel_radius = 0.4
        self.length = 0.825

        self.diff_drive = DifferentialDrive(self.wheel_radius, self.length)

        self.timer = self.create_timer(1.0 / 30.0, self.run)

    def cmd_vel_callback(self, msg):
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.omega = msg.angular.z
    
    def update_joint_position(self):

        v_desired = np.array([self.vel_x, self.vel_y, self.omega])

        u = self.diff_drive.inverse(v_desired, self.x)
        v = self.diff_drive.forward(u, self.x)

        # update x-position, y-position, and heading
        self.x = self.x + v*self.dt
        # update Left Wheel angle and Right Wheel angle
        self.q = self.q + u*self.dt/self.wheel_radius

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["chassis_to_left_wheel", "chassis_to_right_wheel"]
        joint_state_msg.position = self.q
        self.joint_publisher.publish(joint_state_msg)

    def tf_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "chassis"
        t.transform.translation.x = self.x[0]
        t.transform.translation.y = self.x[1]
        t.transform.translation.z = 0.4
        t.transform.rotation.z = np.sin(self.x[2]/2)
        t.transform.rotation.w = np.cos(self.x[2]/2)
        self.tf_broadcaster.sendTransform(t)
    
    def reset_pose_callback(self, request, response):
        self.x = np.array([0, 0 ,0])
        self.q = np.array([0, 0])
        response.success = True
        response.message = ""
        return response

    def run(self):
        self.update_joint_position()
        self.tf_callback()
        self.publish_joint_state()
    
def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
