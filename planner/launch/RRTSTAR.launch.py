import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

rviz_config_dir = get_package_share_directory('planner') + '/rviz/map.rviz'
def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('type', default_value='rrt_star'),
        Node(
            package='planner',
            executable='rrt_star',
            name='RRTSTAR',
            output='screen',
            # prefix  ="xterm -fa 'Monospace' -fs 18 -e ",
            parameters=[{'type': launch.substitutions.LaunchConfiguration('type')}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            # arguments=['-d', rviz_config_dir]
        )
    ])