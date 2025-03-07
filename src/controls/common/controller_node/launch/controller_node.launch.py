# AMZ Driverless Project
#
# Copyright (c) 2024 - 2025  Authors:
# - Nazim Yasar <nyasar@ethz.ch>
# - Nazim Ozan Yasar <nyasar@ethz.ch>                                       
# - Mattia Mangili <mangilim@ethz.ch>                                       
# - Kevin Gabriel <kegabriel@ethz.ch>                                       
# - Vincent Rasse <vrasse@ethz.ch>                                          
# - Audrey Kubler <akubler@ethz.ch>                                         
# - Sinan Laloui <slaloui@ethz.ch>                                          
# - Alexander Terrail <aterrail@ethz.ch>      
#
# All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    package = get_package_share_directory("controller_node")
    default_config = os.path.join(package, "config", "controller_node_config.yaml")
    node = Node(
        package="controller_node",
        executable="controller_node",
        output="screen",
        parameters=[default_config],
    )
    return LaunchDescription([node])