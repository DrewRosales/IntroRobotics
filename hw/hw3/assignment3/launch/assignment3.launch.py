import os

import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('assignment3').find('assignment3')
    urdf_file = os.path.join(pkg_share, 'launch', 'single_rrbot.urdf')
    rviz_file = os.path.join(pkg_share, 'launch', 'dummy_model.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    rviz_params = {'-d': robot_desc}

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[rsp_params]),
        Node(package='assignment3', executable='joint_driver',
         output='screen'),
        launch.actions.ExecuteProcess(cmd=['rviz2','-d',rviz_file])
    ])