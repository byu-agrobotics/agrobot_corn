import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agrobot_nav',
            executable='tof_sensor',
            name='tof_sensor',
            output='screen'
        ),
        Node(
            package='agrobot_nav',
            executable='mini_nav',
            name='mini_nav_state_machine',
            output='screen'
        )
    ])
