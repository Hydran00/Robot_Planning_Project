import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument

rviz_config_dir = get_package_share_directory("planner") + "/rviz/map.rviz"


def generate_launch_description():
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument("show_graphics", default_value="true"),
            launch.actions.DeclareLaunchArgument("dubins_radius", default_value="0.5"),
            Node(
                package="planner",
                executable="rrt_star",
                name="rrt_star",
                output="screen",
                parameters=[
                    {
                        "planner_type": "rrt_star",
                        "show_graphics": launch.substitutions.LaunchConfiguration('show_graphics'),
                        "dubins_radius": launch.substitutions.LaunchConfiguration('dubins_radius'),
                    }
                ],
            ),
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz",
            #     output="screen",
            #     # condition=IfCondition(
            #     #     PythonExpression(["'", launch.substitutions.LaunchConfiguration("show_graphics"), "' == 'true'"])
            #     # ),
            #     arguments=["-d", rviz_config_dir],
            # ),
        ],
    )
