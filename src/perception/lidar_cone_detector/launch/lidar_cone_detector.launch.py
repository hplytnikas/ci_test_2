from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_cone_detector',
            executable='lidar_cone_detector_node',
            name='lidar_cone_detector',
            output='screen',
            remappings=[
                # Example: if you need to remap the input LiDAR topic
                # ('lidar_points', '/my_lidar_topic'), 
            ]
        )
    ])
