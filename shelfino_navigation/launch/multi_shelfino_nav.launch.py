# Copyright 2019 Open Source Robotics Foundation, Inc.
# Author: Placido Falqueto

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map = LaunchConfiguration('map', default='empty')
    remote = LaunchConfiguration('remote', default='false')
    headless = LaunchConfiguration('headless', default='false')
    
    pkg_shelfino_nav = get_package_share_directory('shelfino_navigation')
    map_path = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_navigation'),'map', ''), map, '.yaml', "'"])

    def get_namespaces(context, *args, **kwargs):
        ps = subprocess.Popen(("ros2", "node", "list"), 
                                    stdout=subprocess.PIPE)
        output = subprocess.check_output(('grep', 'robot_state_publisher'), 
                                        stdin=ps.stdout, 
                                        text=True)
        namespaces = output.replace("/robot_state_publisher", "")
        namespaces = namespaces.replace("/", "").splitlines()
        print(namespaces)

        nav_instances_cmds = []
        for robot in namespaces:
            robot_id = robot.replace("shelfino", "")
            nav_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_shelfino_nav, 'launch', 'shelfino_nav.launch.py')),
                launch_arguments={'use_sim_time':use_sim_time,
                                'map': map,
                                'remote':remote,
                                'headless':headless,
                                'robot_id':robot_id,
                                'robot_name':robot,
                                'multi':'True'}.items(),
            )
            nav_instances_cmds.append(nav_launch)
        return nav_instances_cmds

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

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'use_sim_time': use_sim_time},
                        {'topic_name': "/map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_path}],
        ),

        OpaqueFunction(function=get_namespaces),
    ])

    