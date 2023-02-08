#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_file_dir = os.path.join(get_package_share_directory('shelfino_description'), 'launch')
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

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            namespace=robot_name,
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            arguments=['-d', cr_path]
        )

        return rviz_node

    return LaunchDescription([

        OpaqueFunction(function=evaluate_rviz),

    ])
