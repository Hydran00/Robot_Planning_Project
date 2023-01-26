# Copyright 2019 Open Source Robotics Foundation, Inc.
# Author: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map = LaunchConfiguration('map', default='empty')
    remote = LaunchConfiguration('remote', default='false')
    headless = LaunchConfiguration('headless', default='false')
    robot_id = LaunchConfiguration('robot_id', default='G')
    robot_name = PythonExpression(["'", 'shelfino', robot_id, "'"])
    
    map = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_navigation'),'map', ''), map, '.yaml', "'"])
    params = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_navigation'),'config', 'shelfino'), robot_id, '.yaml', "'"])
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_navigation'), 'rviz', 'shelfino'), robot_id, '.rviz', "'"])
    
    remappings = [('/goal_pose', 'goal_pose'),
                  ('/clicked_point', 'clicked_point'),
                  ('/initialpose', 'initialpose')]

    return LaunchDescription([
        DeclareLaunchArgument(name='map', default_value='empty', choices=['empty', 'povo', 'hexagon'],
                        description='World used in the gazebo simulation'),
        DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),
        DeclareLaunchArgument(name='remote', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between navigation stack running on robot or locally'),
        DeclareLaunchArgument(name='headless', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between navigation stack running on robot or locally'),
        DeclareLaunchArgument(name='robot_id', default_value='G',
                        description='ID of the robot'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map,
                'use_sim_time': use_sim_time,
                'namespace': robot_name,
                'use_namespace': 'True',
                'use_composition': 'False',
                'autostart': 'False',
                'use_respawn': 'True',
                'params_file': params}.items(),
            condition=UnlessCondition(remote),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_name,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(headless),
            remappings=remappings,
            output='screen'),
    ])

    