from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    urdf_file = "basic_robot.urdf"
    urdf_path = os.path.join(
        get_package_share_directory("en613_control"),
        'urdf',
        urdf_file
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}]
        ),

        ExecuteProcess(
            cmd=["rviz2", "-d", os.path.join(
                get_package_share_directory("en613_control"), "rviz", "basic_robot.rviz"
            )],
            output="screen"
        )
    ])
