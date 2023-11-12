#!/usr/bin/env python3
#
# Authors: Placido Falqueto

import os

from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        Node(
            package='dubins_planner',
            executable='plot.py',
            namespace='shelfinoG',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
