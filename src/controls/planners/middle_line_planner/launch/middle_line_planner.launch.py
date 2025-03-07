import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("middle_line_planner"),
        "config",
        "middle_line_planner_config.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="middle_line_planner",
                executable="middle_line_planner",
                name="middle_line_planner",
                output="screen",
                parameters=[config],
            )
        ]
    )
