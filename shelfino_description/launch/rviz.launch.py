#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    def get_namespaces_for_rviz(context, *args, **kwargs):
        ps = subprocess.Popen(("ros2", "node", "list"), 
                                    stdout=subprocess.PIPE)
        output = subprocess.check_output(('grep', 'robot_state_publisher'), 
                                        stdin=ps.stdout, 
                                        text=True)
        namespaces = output.replace("/robot_state_publisher", "")
        namespaces = namespaces.replace("/", "").splitlines()
        print(namespaces)

        instances_cmds = []
        for robot in namespaces:
            rviz_path = os.path.join(get_package_share_directory('shelfino_description'), 'rviz', 'shelfino.rviz')
            cr_path = os.path.join(get_package_share_directory('shelfino_description'), 'rviz', '') + robot + '.rviz'
            
            f = open(rviz_path,'r')
            filedata = f.read()
            f.close()

            newdata = filedata.replace("shelfinoX",robot)

            f = open(cr_path,'w')
            f.write(newdata)
            f.close()

            instances_cmds = []
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                namespace=robot,
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
                arguments=['-d', cr_path]
            )
            instances_cmds.append(rviz_node)
        return instances_cmds


    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', choices=['true', 'false'],
                        description='Flag to toggle between real robot and simulation'),

        OpaqueFunction(function=get_namespaces_for_rviz),

    ])
