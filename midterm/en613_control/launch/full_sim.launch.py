from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = "basic_robot.urdf"
    urdf_path = os.path.join(
        get_package_share_directory("en613_control"),
        'urdf',
        urdf_file
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("en613_control"), "rviz", "config.rviz"
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    if not os.path.exists(rviz_config_file):
        raise FileNotFoundError(f"RViz configuration file not found: {rviz_config_file}")

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

        Node(
            package="en613_control",
            executable="diffdrive_sim",
            name="diffdrive_sim",
            output="screen",
        ),

        Node(
            package="en613_control",
            executable="diffdrive_pid",
            name="diffdrive_pid",
            output="screen",
        ),

        ExecuteProcess(
            cmd=["rviz2", "-d", rviz_config_file],
            output="screen",
            cwd=os.path.dirname(rviz_config_file)
        )
    ])
