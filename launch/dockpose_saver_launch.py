import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # goal file to load poses from
    dock_pose_file = "init_dock.json"

    load_file_path = os.path.join(
        get_package_share_directory("lidar_auto_docking"),
        "initial_dock_pose",
        dock_pose_file,
    )
    run_dock_coordinates = Node(
        package="lidar_auto_docking",
        executable="dock_coordinates",
        name="dock_coordinates",
        parameters=[{"reset_goal_button": 1}],
    )
    run_dock_saver = Node(
        package="lidar_auto_docking",
        executable="dock_saver.py",
        name="dock_saver",
        emulate_tty=True,
        parameters=[{"load_file_path": load_file_path}],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(run_dock_saver)
    ld.add_action(run_dock_coordinates)

    return ld
