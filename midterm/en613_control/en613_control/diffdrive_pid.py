import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer, LookupException
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros
import math

class DiffDrivePid(Node):
    def __init__(self):
        super().__init__('diffdrive_pid')
        self.x_goal = None
        self.x_prev = np.array([0.0, 0.0])
        self.x_error = np.array([0.0, 0.0])

        self.T_chassis_odom = None

        self.dt = 1.0/30.0

        # trailer hitch length
        self.l = 0.3

        self.kp = 0.8
        self.kd = 0.075

        #Subscribe to /tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Subscribe to goal_pose
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.timer = self.create_timer(self.dt, self.run)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)


    def goal_pose_callback(self, msg):

        self.x_goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
        ])

    def run(self):
        if self.x_goal is None:
            return

        try:
            # 1. Update robot's position by subscribing to the chassis->odom transform
            now = rclpy.time.Time()
            self.T_chassis_odom = self.tf_buffer.lookup_transform("odom", "chassis", now)

        except tf2_ros.LookupException:
            self.get_logger().info("Transform isn\'t available, waiting...")
            return

        x_pose_current = self.T_chassis_odom.transform.translation.x
        y_pose_current = self.T_chassis_odom.transform.translation.y
        q = self.T_chassis_odom.transform.rotation

        self.get_logger().info(f"{q}")

        theta_current  = np.arctan2(2*(q.w * q.z + q.x *q.y), 1-2*(q.y * q.y + q.z * q.z))
        theta_current = (theta_current + 2 * np.pi) % (2 * np.pi)

        x_current = np.array([x_pose_current, y_pose_current])
        v_current = x_current - self.x_prev


        x_p = np.array([x_current[0] + self.l*np.cos(theta_current), 
                            x_current[1] + self.l*np.sin(theta_current)])

        self.x_error = x_p - self.x_goal

        R = np.array([[np.cos(theta_current), np.sin(theta_current)],
                      [-np.sin(theta_current), np.cos(theta_current)]])

        inner = -self.kp*self.x_error - self.kd*v_current
        u = R @ (inner)

        # 2. Update robot's velocity by comparing against previous position
        self.x_prev = x_current

        # 3. Compute the new velocity commands and publish them to /cmd_vel
        msg = Twist()
        msg.linear.x = u[0]
        msg.angular.z = u[1]/self.l
        self.cmd_vel_publisher.publish(msg)

    
def main(args=None):
    rclpy.init(args=args)
    node = DiffDrivePid()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
