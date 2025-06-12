# Created by Nelson Durrant, Feb 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_fsm_dir = get_package_share_directory("agrobot_fsm")
    robot_fsm_launch_dir = os.path.join(robot_fsm_dir, "launch")
    robot_nav_dir = get_package_share_directory("agrobot_navigation")
    robot_nav_launch_dir = os.path.join(robot_nav_dir, "launch")
    robot_cv_dir = get_package_share_directory("agrobot_perception")
    robot_cv_launch_dir = os.path.join(robot_cv_dir, "launch")

    fsm_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_fsm_launch_dir, "agrobot_fsm.launch.py")
        ),
    )

    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_nav_launch_dir, "agrobot_navigation.launch.py")
        ),
    )

    cv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_cv_launch_dir, "agrobot_perception.launch.py")
        ),
    )

    ld = LaunchDescription()
    ld.add_action(fsm_cmd)
    ld.add_action(nav_cmd)
    ld.add_action(cv_cmd)

    return ld
