#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_id = LaunchConfiguration('robot_id', default='G')
    robot_name = PythonExpression(["'", 'shelfino', robot_id, "'"])

    def evaluate_rviz(context, *args, **kwargs):
        rn = 'shelfino' + LaunchConfiguration('robot_id').perform(context)
        rviz_path = os.path.join(get_package_share_directory('shelfino_description'), 'rviz', 'shelfino.rviz')
        cr_path = os.path.join(get_package_share_directory('shelfino_description'), 'rviz', 'shelfino') + LaunchConfiguration('robot_id').perform(context) + '.rviz'
        
        f = open(rviz_path,'r')
        filedata = f.read()
        f.close()

        newdata = filedata.replace("shelfinoX",rn)

        f = open(cr_path,'w')
        f.write(newdata)
        f.close()

        instances_cmds = []
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_name,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            arguments=['-d', cr_path]
        )
        instances_cmds.append(rviz_node)

        return instances_cmds

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),
        DeclareLaunchArgument(name='robot_id', default_value='G',
                        description='ID of the robot'),

        OpaqueFunction(function=evaluate_rviz),

    ])
