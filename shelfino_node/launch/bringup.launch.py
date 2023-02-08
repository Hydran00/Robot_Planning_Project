#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map = LaunchConfiguration('map', default='empty')
    nav = LaunchConfiguration('nav', default='true')
    remote = LaunchConfiguration('remote', default='false')
    headless = LaunchConfiguration('headless', default='false')
    robot_id = LaunchConfiguration('robot_id', default='404')
    robot_name = PythonExpression(["'", 'shelfino', robot_id, "'"])
    
    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')

    nav2_launch_file_dir = os.path.join(get_package_share_directory('shelfino_navigation'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(name='map', default_value='empty', choices=['empty', 'povo', 'hexagon'],
                        description='World used in the gazebo simulation'),
        DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),
        DeclareLaunchArgument(name='nav', default_value='true', choices=['true', 'false'],
                        description='Flag to start the navigation stack'),
        DeclareLaunchArgument(name='remote', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between navigation stack running on robot or locally'),
        DeclareLaunchArgument(name='headless', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between navigation stack running on robot or locally'),
        DeclareLaunchArgument(name='robot_id', default_value='G',
                        description='ID of the robot'),

        Node(
            package='shelfino_node',
            namespace=robot_id,
            executable='shelfino_node'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': robot_id}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_file_dir, 'shelfino_nav.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time,
                            'map': map,
                            'remote':remote,
                            'headless':headless,
                            'robot_id':robot_id,
                            'robot_name':robot_name}.items(),
            condition=IfCondition(nav)
        ),

        Node(
            package='get_positions',
            executable='get_positions',
            namespace=robot_id
        ),
    ])
