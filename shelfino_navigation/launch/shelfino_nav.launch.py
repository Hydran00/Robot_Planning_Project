# Copyright 2019 Open Source Robotics Foundation, Inc.
# Author: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map = LaunchConfiguration('map', default='empty')
    remote = LaunchConfiguration('remote', default='false')
    headless = LaunchConfiguration('headless', default='false')
    robot_id = LaunchConfiguration('robot_id', default='G')
    robot_name = PythonExpression(["'", 'shelfino', robot_id, "'"])
    
    map_path = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_navigation'),'map', ''), map, '.yaml', "'"])
    params = os.path.join(get_package_share_directory('shelfino_navigation'),'config', 'shelfino.yaml')
    rviz_config = PythonExpression(["'", os.path.join(get_package_share_directory('shelfino_navigation'), 'rviz', 'shelfino'), robot_id, '_nav.rviz', "'"])

    map_node = PythonExpression(["'/", robot_name, '/map_server', "'"])
    amcl_node = PythonExpression(["'/", robot_name, '/amcl', "'"])
    bt_navigator_node = PythonExpression(["'/", robot_name, '/bt_navigator', "'"])
    controller_node = PythonExpression(["'/", robot_name, '/controller_server', "'"])
    planner_node = PythonExpression(["'/", robot_name, '/planner_server', "'"])
    waypoint_follower_node = PythonExpression(["'/", robot_name, '/waypoint_follower', "'"])
    behavior_node = PythonExpression(["'/", robot_name, '/behavior_server', "'"])
    smoother_node = PythonExpression(["'/", robot_name, '/smoother_server', "'"])
    velocity_smoother_node = PythonExpression(["'/", robot_name, '/velocity_smoother', "'"])

    lifecycle_nodes_loc = [[map_node],
                          [amcl_node]]

    lifecycle_nodes_nav = [[bt_navigator_node], 
                          [controller_node], 
                          [planner_node], 
                          [waypoint_follower_node], 
                          [behavior_node], 
                          [smoother_node], 
                          [velocity_smoother_node]]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = { 'use_sim_time': use_sim_time,
                            'base_frame_id': PythonExpression(["'", robot_name, '/base_link', "'"]),
                            'odom_frame_id': PythonExpression(["'", robot_name, '/odom', "'"]),
                            'robot_base_frame': PythonExpression(["'", robot_name, '/base_link', "'"]),
                            'global_frame': PythonExpression(["'", robot_name, '/odom', "'"]),
                            'topic': PythonExpression(["'/", robot_name, '/scan', "'"]),
                          }

    configured_params = RewrittenYaml(
        source_file=params,
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True)

    def evaluate_rviz(context, *args, **kwargs):
        rn = 'shelfino' + LaunchConfiguration('robot_id').perform(context)
        rviz_path = os.path.join(get_package_share_directory('shelfino_navigation'), 'rviz', 'shelfino_nav.rviz')
        cr_path = os.path.join(get_package_share_directory('shelfino_navigation'), 'rviz', 'shelfino') + LaunchConfiguration('robot_id').perform(context) + '_nav.rviz'
        
        f = open(rviz_path,'r')
        filedata = f.read()
        f.close()

        newdata = filedata.replace("shelfinoX",rn)

        f = open(cr_path,'w')
        f.write(newdata)
        f.close()
        return

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

        OpaqueFunction(function=evaluate_rviz),

        # LogInfo(
        #     msg=['params: ', params]
        # ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[{'use_sim_time': use_sim_time},
                        {'topic_name': "/map"},
                        {'frame_id': "map"},
                        {'yaml_filename': map_path}],
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[configured_params],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            namespace= robot_name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            namespace= robot_name,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings = [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            namespace= robot_name,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'bond_timeout': 0.0},
                        {'node_names': lifecycle_nodes_loc}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            namespace= robot_name,
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': False},
                        {'bond_timeout': 0.0},
                        {'node_names': lifecycle_nodes_nav}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            namespace= robot_name,
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(headless),
            output='screen'
        ),
    ])

    