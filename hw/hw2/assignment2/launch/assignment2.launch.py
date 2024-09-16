import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('assignment2').find('assignment2')
    urdf_file = os.path.join(pkg_share, 'launch', 'single_rrbot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[rsp_params]),
        Node(package='assignment2', executable='dummy_object_detector',
        	 output='screen'),
        Node(package='dummy_sensors', executable='dummy_joint_states',
         output='screen'),
        Node(package='assignment2', executable='coordinates',
        	 output='screen')
    ])