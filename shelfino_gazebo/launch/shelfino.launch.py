#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    robot_id = LaunchConfiguration('robot_id', default='G')
    robot_name = PythonExpression(["'", 'shelfino', robot_id, "'"])
    world = LaunchConfiguration('world', default='empty')
    world_path = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_gazebo'),'worlds', ''), world, '.world', "'"])

    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_models_path = os.path.join(get_package_share_directory('shelfino_description'), 'models')
    xacro_model = os.path.join(gazebo_models_path, 'shelfino', 'model.sdf.xacro')
    
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    model = PythonExpression(["'", os.path.join(gazebo_models_path,'shelfino','shelfino'), robot_id, '.sdf', "'"])

    rviz_config = os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfinoG.rviz')
    rviz_config = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_gazebo'), 'rviz', 'shelfino'), robot_id, '.rviz', "'"])

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                        description='Flag to enable gazebo visualization'),
        DeclareLaunchArgument(name='rviz', default_value='true', choices=['true', 'false'],
                        description='Flag to enable rviz visualization'),
        DeclareLaunchArgument(name='robot_id', default_value='G',
                        description='ID of the robot'),
        DeclareLaunchArgument(name='world', default_value='empty', choices=['empty', 'povo', 'hexagon'],
                        description='World used in the gazebo simulation'),

        ExecuteProcess(
            cmd=[[
                'xacro ',
                xacro_model,
                ' robot_id:=',
                robot_id,
                ' > ',
                model
            ]],
            shell=True
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            condition=IfCondition(gui)
        ),
    
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', model,
                       '-entity', robot_name,
                       '-robot_namespace', robot_name]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'robot_id': robot_id}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),

        Node(
            package='get_positions',
            executable='get_positions',
            namespace=robot_name,
        ),
    ])
