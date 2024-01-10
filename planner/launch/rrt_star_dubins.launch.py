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
            launch.actions.DeclareLaunchArgument("type", default_value="rrt_star"),
            launch.actions.DeclareLaunchArgument("show_graphics", default_value="true"),
            Node(
                package="planner",
                executable="rrt_star_dubins",
                name="rrt_star_dubins",
                output="screen",
                parameters=[
                    {
                        "type": launch.substitutions.LaunchConfiguration("type"),
                        "show_graphics": launch.substitutions.LaunchConfiguration('show_graphics'),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="log",
                condition=IfCondition(
                    PythonExpression(["'", launch.substitutions.LaunchConfiguration("show_graphics"), "' == 'true'"])
                ),
                arguments=["-d", rviz_config_dir],
            ),
        ],
    )
