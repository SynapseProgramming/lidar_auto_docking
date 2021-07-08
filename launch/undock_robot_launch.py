import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    current_dir = get_package_share_directory("lidar_auto_docking")
    ld = LaunchDescription()

    config = os.path.join(current_dir, "config", "autodock_params.yaml")

    run_autodock_client = Node(
        package="lidar_auto_docking",
        executable="undock_robot.py",
        name="undock_bot",
        emulate_tty=True,
        output="screen",
    )

    run_autodock = Node(
        package="lidar_auto_docking",
        executable="auto_dock",
        parameters=[config],
        output="screen",
        remappings=[("/cmd_vel", "/autodock/cmd_vel")],
    )
    ld.add_action(run_autodock)
    ld.add_action(run_autodock_client)
    return ld
