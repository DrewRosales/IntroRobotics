from .differential_drive import DifferentialDrive
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
import numpy as np

class DiffDriveSim(Node):
   def __init__(self):
       super().__init__('diffdrive_sim')
       self.x = np.array([0.0, 0.0, 0.0])  # x, y, theta
       self.q = np.array([0.0, 0.0])  # left wheel, right wheel angles
       self.dt = 1.0/30.0
       self.vel_x = 0.0
       self.vel_y = 0.0
       self.omega = 0.0
       
       self.subscription = self.create_subscription(
           Twist,
           '/en613/cmd_vel',
           self.cmd_vel_callback,
           10
       )
       
       qos_profile = QoSProfile(depth=10)
       #self.joint_publisher = self.create_publisher(JointState, '/joint_states', qos_profile)
       self.tf_broadcaster = TransformBroadcaster(self)
       
       #self.reset_pose_service = self.create_service(
       #    Trigger,
       #    '/robot_pose_reset',
       #    self.reset_pose_callback
       #)
       
       # Parameters matching the SDF
       self.wheel_radius = 0.1
       self.length = 0.26
       
       self.diff_drive = DifferentialDrive(self.wheel_radius, self.length)
       self.timer = self.create_timer(self.dt, self.run)

   def cmd_vel_callback(self, msg):
       self.vel_x = msg.linear.x
       self.vel_y = msg.linear.y
       self.omega = msg.angular.z
   
   def update_joint_position(self):
       v_desired = np.array([self.vel_x, self.omega])
       u = self.diff_drive.inverse(v_desired)
       v = self.diff_drive.forward(u)
       
       self.x[0] = self.x[0] + v[0] * np.cos(self.x[2]) * self.dt
       self.x[1] = self.x[1] + v[0] * np.sin(self.x[2]) * self.dt
       self.x[2] = self.x[2] + v[1] * self.dt
       self.x[2] = (self.x[2] + 2 * np.pi) % (2 * np.pi)
       
       self.q = self.q + u * self.dt / self.wheel_radius


   def publish_joint_state(self):
       joint_state_msg = JointState()
       joint_state_msg.header.stamp = self.get_clock().now().to_msg()
       joint_state_msg.name = ["left_wheel_hinge", "right_wheel_hinge"]

        # Add value checking and cleaning
       cleaned_positions = []
       for val in self.q:
           # Convert to float and ensure it's within reasonable bounds
           float_val = float(val)
           # Normalize large rotation values
           float_val = float_val % (2 * np.pi)
           cleaned_positions.append(float_val)

       joint_state_msg.position = cleaned_positions
       joint_state_msg.velocity = []
       joint_state_msg.effort = []
       self.joint_publisher.publish(joint_state_msg)

   def tf_callback(self):
       t = TransformStamped()
       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = "/odom"
       t.child_frame_id = "/chassis"
       t.transform.translation.x = float(self.x[0])
       t.transform.translation.y = float(self.x[1])
       t.transform.translation.z = 0.1

   # Correct quaternion calculation for z-axis rotation
       yaw = float(self.x[2])
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = float(np.sin(yaw/2))
       t.transform.rotation.w = float(np.cos(yaw/2))

       self.tf_broadcaster.sendTransform(t)

   def reset_pose_callback(self, request, response):
       self.x = np.array([0.0, 0.0, 0.0])
       self.q = np.array([0.0, 0.0])
       response.success = True
       response.message = ""
       return response

   def run(self):
       self.update_joint_position()
       self.tf_callback()
       #self.publish_joint_state()

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
