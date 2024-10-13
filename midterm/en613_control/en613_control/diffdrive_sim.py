from differential_drive import DifferentialDrive

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class DiffDriveSim(Node):
    def __init__(self):
        super().__init__('diff_drive_sim')

        self.x = np.array([0, 0, 0])
        self.q = np.array([0, 0, 0])

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

        self.joint_publisher  = self.create_publisher(JointState, '/joint_states', 10)

        self.wheel_radius = 0.05
        self.length = 0.3

        self.diff_drive = DifferentialDrive(self.wheel_radius, self.length)

    def cmd_vel_callback(self, msg):
        self.vel_x = msg.linear.x
        self.vel_y = msg.linear.y
        self.omega = msg.angular.z
    
    def update(self):

        v_desired = np.array([self.vel_x, self.vel_y, self.omega])

        u = self.diff_drive.inverse(v_desired, self.x)
        v = self.diff_drive.forward(u, self.x)

        # update x-position, y-position, and heading
        self.x = self.x + v*self.dt
        # update Left Wheel angle and Right Wheel angle
        self.q = self.q + u*self.dt

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = []
        self.joint_publisher.publish()


        

        

        
