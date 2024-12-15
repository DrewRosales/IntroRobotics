import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from numpy import genfromtxt


class GoalPublisher(Node):

    def __init__(self,goal_list):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseArray, '/en613/goals', 10)
        timer_period = 0.5  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Goal list node created")
        self.goal_list = goal_list 

    def timer_callback(self):
        poses = []
        for goal in self.goal_list:
            msg = Pose()
            msg.position.x = goal[0]
            msg.position.y = goal[1]
            msg.position.z = goal[2]
            poses.append(msg)
        pose_array = PoseArray()
        pose_array.poses = poses
        self.publisher_.publish(pose_array)

def spawn_entity(node, client, request):

    node.get_logger().info("Sending service request to `/spawn_entity`")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())


def spawn_map(node, client, map_folder):
    map_file_path = os.path.join(
        get_package_share_directory("robot_spawner_pkg"), "models",
        map_folder, "model.sdf")
    node.get_logger().info(f"map_sdf={map_file_path}")
    request = SpawnEntity.Request()
    request.name = "Maze"
    request.xml = open(map_file_path, 'r').read()
    request.initial_pose.position.x = float(0.0)
    request.initial_pose.position.y = float(0.0)
    request.initial_pose.position.z = float(0.0)
    node.get_logger().info("spawning map")

    spawn_entity(node, client, request)

def spawn_robot(node, client,
                start_position,
                robot_namespace = 'en613',
                robot_name='BasicRobot',
                robot_folder="basic_robot"):

    sdf_file_path = os.path.join(
        get_package_share_directory("robot_spawner_pkg"), "models",
        robot_folder, "model.sdf")
    node.get_logger().info(f"robot_sdf={sdf_file_path}")

    request = SpawnEntity.Request()
    request.name = robot_name
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = robot_namespace
    request.initial_pose.position.x = float(start_position[0])
    request.initial_pose.position.y = float(start_position[1])
    request.initial_pose.position.z = float(start_position[2])

    request.initial_pose.orientation.x = 0.0
    request.initial_pose.orientation.y = 0.0
    request.initial_pose.orientation.z = 0.0
    request.initial_pose.orientation.w = 1.0

    node.get_logger().info("spawning robot")

    spawn_entity(node, client, request)

def get_goals(map_folder):
    goal_file_path = os.path.join(
        get_package_share_directory("robot_spawner_pkg"), "models",
        map_folder, "pose.csv")
    goal_data = genfromtxt(goal_file_path, delimiter=',')
    start_position = goal_data[0,:]
    goal_list = goal_data[1:,:]
    return start_position, goal_list


def spawn_goals(node, client, goal_list, robot_namespace = 'en613'):
    marker_sdf_file_path = os.path.join(
        get_package_share_directory("robot_spawner_pkg"), "models",
        "globe", "model.sdf")
    node.get_logger().info(f"goal_marker_sdf={marker_sdf_file_path}")
    node.get_logger().info("Start pose={goal}")
    # Set data for request
    for i, goal in enumerate(goal_list):
        request = SpawnEntity.Request()
        request.name = f'Goal{i}'
        request.xml = open(marker_sdf_file_path, 'r').read()
        request.robot_namespace = robot_namespace
        request.initial_pose.position.x = float(goal[0])
        request.initial_pose.position.y = float(goal[1])
        request.initial_pose.position.z = float(goal[2])


        node.get_logger().info(f"spawning goal{i}")
        spawn_entity(node, client, request)



start_positions = [[1.0,-1,0.1],[4.5,-3.25,0.1],[-1.5,0.5,0.1],[3,0.8,0.1]]
goal_positions = [[-1.5,0.5,0.1],[-1.5,-1,0.1],[3.0,-2.5,0.1],[-1.5,-4.25,0.1]]
def main():
    """ Main for spwaning turtlebot node """
    # Get input arguments from user
    argv = sys.argv[1:]

    robot_name = argv[0]
    robot_namespace = argv[0]
    map_folder = argv[2]

    # Start node
    rclpy.init()

    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")
    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    start, goal_list = get_goals(map_folder)
    node.get_logger().info(f'Robot starting pose = {start}')
    node.get_logger().info(f'Goals  = {goal_list}')

    spawn_map(node, client, map_folder)
    spawn_robot(node, client, start)
    spawn_goals(node, client, goal_list)

    node.get_logger().info("Done! Shutting down spawner node.")
    node.destroy_node()


    #Start publishing the goal position
    goal_publisher = GoalPublisher(goal_list)

    rclpy.spin(goal_publisher)

    goal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
