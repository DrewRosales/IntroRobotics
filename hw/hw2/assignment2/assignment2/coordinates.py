import math
import tf2_ros
import rclpy
from time import sleep
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import numpy as np
from .assignment2 import transform_pose

class MinimalSubscriber(Node):

	def __init__(self):
		super().__init__('minimal_subscriber')
		self.subscription = self.create_subscription(
			Pose,
			'dummy_objects',
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning

		self.publisher_ = self.create_publisher(Pose, 'dummy_objects_world', 10)

		self._tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self._tf_buffer,self)

		self._to_frame = 'world'
		self._from_frame = 'single_rrbot_camera_link'


	def listener_callback(self, msg: Pose):

		try:
			when = rclpy.time.Time()
			trans = self._tf_buffer.lookup_transform(self._to_frame, self._from_frame,
													 when, timeout=Duration(seconds=5.0))
		except tf2_ros.LookupException:
			self.get_logger().info('Transform isn\'t available, waiting...')
			sleep(1)
			return

		X0 = np.array([msg.position.x,msg.position.y,msg.position.z])
		Q0 = np.array([msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z])

		T = np.array([trans.transform.translation.x,
					  trans.transform.translation.y,
					  trans.transform.translation.z
					])
		R = np.array([
					  trans.transform.rotation.w,
					  trans.transform.rotation.x,
					  trans.transform.rotation.y,
					  trans.transform.rotation.z,
					])

		X1, Q1 = transform_pose(X0,Q0,T,R)

		self.publish_pose(X1,Q1)


	def publish_pose(self,X1: np.ndarray,Q1: np.ndarray):

		msg = Pose()
		msg.position.x = X1[0]
		msg.position.y = X1[1]
		msg.position.z = X1[2]
		msg.orientation.w = Q1[0]
		msg.orientation.x = Q1[1]
		msg.orientation.y = Q1[2]
		msg.orientation.z = Q1[3]

		self.publisher_.publish(msg)


		

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
