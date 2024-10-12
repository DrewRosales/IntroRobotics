from .assignment4 import Mecanum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np

class MecanumSim(Node):
    def __init__(self):
        super().__init__('mecanum_sim')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription

        self.fl_publisher  = self.create_publisher(Float32, '/fl_wheel_vel_cmd', 10)
        self.fr_publisher  = self.create_publisher(Float32, '/fr_wheel_vel_cmd', 10)
        self.bl_publisher  = self.create_publisher(Float32, '/bl_wheel_vel_cmd', 10)
        self.br_publisher  = self.create_publisher(Float32, '/br_wheel_vel_cmd', 10)

        self.length = 0.3
        self.width =  0.15
        self.wheel_radius = 0.05
        self.roller_radius = 0.01
        self.roller_angle = 45/360*np.pi

        self.mecanum = Mecanum(self.length, self.width, self.wheel_radius, self.roller_radius, self.roller_angle)

    def cmd_vel_callback(self, msg):
        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        # Compute the desired velocity vector [Vx, Vy, Vtheta]
        v = np.array([x, y, z])

        # Compute the wheel velocities using inverse kinematics
        psi = self.mecanum.inverse([0, 0, 0], v)

        self.fl_publisher.publish(Float32(data=psi[0]))
        self.fr_publisher.publish(Float32(data=psi[1]))
        self.bl_publisher.publish(Float32(data=psi[2]))
        self.br_publisher.publish(Float32(data=psi[3]))



def main(args=None):
    rclpy.init(args=args)

    mecanum_sim = MecanumSim()

    rclpy.spin(mecanum_sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mecanum_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
