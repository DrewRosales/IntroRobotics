#! /usr/bin/env python
from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

from .assignment3 import *

x_0 = np.array([-1.5,0.8,0.9])
x_1 = np.array([-1.5,-0.8,0.9])
x_2 = np.array([-1.5,-0.8,0.3])
x_3 = np.array([-1.0,-0.8,0.3])
x_4 = np.array([-1.0,0.8,0.3])
x_5 = np.array([-1.5,0.8,0.3])

class StatePublisher(Node):

  def __init__(self):
      rclpy.init()
      super().__init__('state_publisher')

      qos_profile = QoSProfile(depth=10)
      self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
      self.pose_pub = self.create_publisher(Pose, 'end_effector_pos2', qos_profile)

      self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
      self.nodeName = self.get_name()
      self.get_logger().info("{0} started".format(self.nodeName))

      degree = pi / 180.0
      loop_rate = self.create_rate(30)

      # robot state
      tilt = 0.
      tinc = degree
      self.Q = np.array([-np.pi/8,-np.pi/4,-np.pi/6])
      ks = np.array([[0,0,1],[0,1,0],[0,1,0]])
      ts = np.array([[0,0,0],[0,0.1,0.1],[0,0.1,0.9]])
      p_N = [0,0,0.975]
      self.kc = KinematicChain(ks,ts)
      self.desired_poses = np.array([x_0,x_1,x_2,x_3,x_4,x_5])#,[-0.5,1.5,0.5],[-1.5,-0.75,0.75],[0.5,-0.75,1.75]])
      self.pose_idx = 0
      hinc = 0.005

      # message declarations
      joint_state = JointState()
      pose = Pose()
      k = 0
      vel_steps = 0.02*np.array([[-1.0,0.0,0.0],[1.0,0.0,0.0],[0,0,1.0],[0,1.0,0.0],[0,0,-1.0],[0,-1.0,0.0]])
      vel_idx = 0
      try:
          while rclpy.ok():
              rclpy.spin_once(self)

              # update joint_state
              now = self.get_clock().now()
              joint_state.header.stamp = now.to_msg()
              joint_state.name = ['single_rrbot_yaw', 'single_rrbot_joint1', 'single_rrbot_joint2']
              joint_state.position = [self.Q[0], self.Q[1], self.Q[2]]

              p = self.kc.pose(self.Q,-1,p_N)
              pose.position.x = p[0]
              pose.position.y = p[1]
              pose.position.z = p[2]

              if np.linalg.norm(p-self.desired_poses[self.pose_idx,:]) < 0.05:
                  print('Desired pose met!')
                  self.pose_idx = (self.pose_idx + 1) % self.desired_poses.shape[0]


              delta_pos = self.desired_poses[self.pose_idx,:]
              self.Q = self.kc.pseudo_inverse(self.Q,p_N,delta_pos,max_steps=1)

              # send the joint state and transform
              self.joint_pub.publish(joint_state)
              self.pose_pub.publish(pose)

              # This will adjust as needed per iteration
              loop_rate.sleep()

      except KeyboardInterrupt:
          pass

def main():
  node = StatePublisher()

if __name__ == '__main__':
  main()