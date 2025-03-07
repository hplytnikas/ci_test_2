from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("depth_estimation"), "config", "default.yaml"
    )

    depth_estimation_node = Node(
        package="depth_estimation",
        executable="depth_estimation",
        name="depth_estimation",
        parameters=[config],
    )

    return LaunchDescription([depth_estimation_node])
